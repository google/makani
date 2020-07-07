/*
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "avionics/firmware/drivers/adis16488.h"

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/n2het.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/drivers/adis16488_coef.h"
#include "avionics/firmware/drivers/adis16488_reg.h"
#include "common/c_math/util.h"
#include "common/macros.h"

#define IMU_SPI 3
#define IMU_CS  0x1
#define IMU_RESET_PIN kN2het1Pin20

#define ADIS16488_RESET_US            100            // > 10 us typical.
#define ADIS16488_RESET_RECOVERY_US   (1000 * 1000)  // > 500 ms typical.
#define ADIS16488_TRANSFER_TIMEOUT_US (10 * 1000)
#define ADIS16488_DATA_TIMEOUT_US     (15 * 1000)

// See Table 3. Register Specific Stall Times.
// TODO: Understand delays required for firmware 110.
#define ADIS16488_FNCTIO_CTRL_STALL_US     (10 * 1000)  // > 15 us.
#define ADIS16488_FILTR_BNK_STALL_US       (10 * 1000)  // > 10 us.
#define ADIS16488_SELF_TEST_STALL_US       (500 * 1000)  // > 12 us.
#define ADIS16488_FLASH_TEST_STALL_US      (500 * 1000)  // > 50,000 us.
#define ADIS16488_FLASH_UPDATE_STALL_US    (500 * 1000)  // > 375,000 us.
#define ADIS16488_FACTORY_RESTORE_STALL_US (500 * 1000)  // > 75,000 us.
#define ADIS16488_SOFTWARE_RESET_STALL_US  (500 * 1000)  // > 120,000 us.

// See Table 115. GLOB_CMD (Page 3, Base Address = 0x02).
#define ADIS16488_SELF_TEST_TIMEOUT_US (500 * 1000)  // > 12 ms.

// SPI parameters.
#define ADIS16488_SPI_FREQ_HZ   (14 * 1000 * 1000)  // 15 MHz max.
#define ADIS16488_SPI_WDELAY_NS 2000
#define ADIS16488_SPI_BUFFERS   ADIS16488_COEF_SET_MOSI_SIZE

// Sample rate.
#define ADIS16488_SAMPLE_RATE ADIS16488_SYNC_FREQ_HZ
#define ADIS16488_DEC_RATE (ADIS16488_SYNC_FREQ_HZ / ADIS16488_SAMPLE_RATE - 1)

COMPILE_ASSERT(0 <= ADIS16488_DEC_RATE && ADIS16488_DEC_RATE <= 2047,
               ADIS16488_DEC_RATE_invalid);

// FNCTIO_CTRL
// Bits   Description
// -----  -----------------------------------------------------------------
// 7      Sync clock enable: 1 = enabled, 0 = disabled.
// 6      Sync clock polarity: 1 = rising edge, 0 = failing edge.
// [5:4]  Sync clock selection: 00 = DIO1, 01 = DIO2, 10 = DIO3, 11 = DIO4.
// 3      Data-ready enable: 1 = enabled, 0 = disabled.
// 2      Data-ready polarity: 1 = positive, 0 = negative.
// [1:0]  Data-ready selection: 00 = DIO1, 01 = DIO2, 10 = DIO3, 11 = DIO4.
#define ADIS16488_FNCTIO_CTRL ((1 << 7) | (1 << 6) | (1 << 4) | (1 << 3))
#define ADIS16488_CONFIG      0x00C0  // Linear g-comp, point of percussion.
#define ADIS16488_NULL_CNFG   0x0000
#define ADIS16488_SLP_CNT     0x0000
#ifdef ADIS16488_USE_COEF
// Enable filter bank A for accelerometer and gyro triads (all axes).
#define ADIS16488_FILTR_BNK_0 (1 << 14 | 1 << 11 | 1 << 8 | 1 << 5 | 1 << 2)
#define ADIS16488_FILTR_BNK_1 (1 << 2)
#else
#define ADIS16488_FILTR_BNK_0 0x0000
#define ADIS16488_FILTR_BNK_1 0x0000
#endif

#define ADIS16488_GYRO_LSB    (0.02 * PI / 180.0 / (1UL << 16))       // 32-bit.
#define ADIS16488_ACCEL_LSB   (0.8 * 9.80665 / 1000.0 / (1UL << 16))  // 32-bit.
#define ADIS16488_MAG_LSB     (0.1 / 1000.0)           // 16-bit.
#define ADIS16488_DELTVEL_LSB (200.0 / (1UL << 31))    // 32-bit.
#define ADIS16488_DELTANG_LSB (720.0 / (1UL << 31))    // 32-bit.
#define ADIS16488_BARO_LSB    (0.00004 / (1UL << 16))  // 32-bit.
#define ADIS16488_TEMP_LSB    0.00565                  // 16-bit, 25 C offset.
#define ADIS16488_TEMP_OFFSET 25.0

static const uint16_t kMosiGetIdentity[] = {
  ADIS16488_TURN_PAGE(0),
  ADIS16488_READ(ADIS16488_PAGE0_PROD_ID),
  ADIS16488_TURN_PAGE(3),
  ADIS16488_READ(ADIS16488_PAGE3_FIRM_REV),
  ADIS16488_READ(ADIS16488_PAGE3_FIRM_DM),
  ADIS16488_READ(ADIS16488_PAGE3_FIRM_Y),
  ADIS16488_TURN_PAGE(4),
  ADIS16488_READ(ADIS16488_PAGE4_SERIAL_NUM),
  ADIS16488_TURN_PAGE(0)
};

static const uint16_t kMosiStartSelfTest[] = {
  ADIS16488_TURN_PAGE(3),
  ADIS16488_WRITE_L(ADIS16488_PAGE3_GLOB_CMD, 1 << 1),
  ADIS16488_WRITE_H(ADIS16488_PAGE3_GLOB_CMD, 1 << 1),
  // Stall.
};

static const uint16_t kMosiGetGlobCmd[] = {
  ADIS16488_TURN_PAGE(3),
  ADIS16488_READ(ADIS16488_PAGE3_GLOB_CMD),
  ADIS16488_TURN_PAGE(0)
};

static const uint16_t kMosiGetControl[] = {
  ADIS16488_TURN_PAGE(3),
  ADIS16488_READ(ADIS16488_PAGE3_FNCTIO_CTRL),
  ADIS16488_READ(ADIS16488_PAGE3_CONFIG),
  ADIS16488_READ(ADIS16488_PAGE3_DEC_RATE),
  ADIS16488_READ(ADIS16488_PAGE3_NULL_CNFG),
  ADIS16488_READ(ADIS16488_PAGE3_SLP_CNT),
  ADIS16488_READ(ADIS16488_PAGE3_FILTR_BNK_0),
  ADIS16488_READ(ADIS16488_PAGE3_FILTR_BNK_1),
  ADIS16488_TURN_PAGE(0)
};

static const uint16_t kMosiSetFnctioCtrl[] = {
  ADIS16488_TURN_PAGE(3),
  ADIS16488_WRITE_L(ADIS16488_PAGE3_FNCTIO_CTRL, ADIS16488_FNCTIO_CTRL),
  ADIS16488_WRITE_H(ADIS16488_PAGE3_FNCTIO_CTRL, ADIS16488_FNCTIO_CTRL),
  // Stall.
};

static const uint16_t kMosiSetFiltrBnk0[] = {
  ADIS16488_TURN_PAGE(3),
  ADIS16488_WRITE_L(ADIS16488_PAGE3_FILTR_BNK_0, ADIS16488_FILTR_BNK_0),
  ADIS16488_WRITE_H(ADIS16488_PAGE3_FILTR_BNK_0, ADIS16488_FILTR_BNK_0),
  // Stall.
};

static const uint16_t kMosiSetFiltrBnk1[] = {
  ADIS16488_TURN_PAGE(3),
  ADIS16488_WRITE_L(ADIS16488_PAGE3_FILTR_BNK_1, ADIS16488_FILTR_BNK_1),
  ADIS16488_WRITE_H(ADIS16488_PAGE3_FILTR_BNK_1, ADIS16488_FILTR_BNK_1),
  // Stall.
};

static const uint16_t kMosiSetControl[] = {
  ADIS16488_TURN_PAGE(3),
  ADIS16488_WRITE_L(ADIS16488_PAGE3_CONFIG, ADIS16488_CONFIG),
  ADIS16488_WRITE_H(ADIS16488_PAGE3_CONFIG, ADIS16488_CONFIG),
  ADIS16488_WRITE_L(ADIS16488_PAGE3_DEC_RATE, ADIS16488_DEC_RATE),
  ADIS16488_WRITE_H(ADIS16488_PAGE3_DEC_RATE, ADIS16488_DEC_RATE),
  ADIS16488_WRITE_L(ADIS16488_PAGE3_NULL_CNFG, ADIS16488_NULL_CNFG),
  ADIS16488_WRITE_H(ADIS16488_PAGE3_NULL_CNFG, ADIS16488_NULL_CNFG),
  ADIS16488_WRITE_L(ADIS16488_PAGE3_SLP_CNT, ADIS16488_SLP_CNT),
  ADIS16488_WRITE_H(ADIS16488_PAGE3_SLP_CNT, ADIS16488_SLP_CNT),
  ADIS16488_TURN_PAGE(0)
};

static const uint16_t kMosiGetOutputData[] = {
  ADIS16488_TURN_PAGE(0),
  ADIS16488_READ(ADIS16488_PAGE0_SEQ_CNT),
  ADIS16488_READ(ADIS16488_PAGE0_SYS_E_FLAG),
  ADIS16488_READ(ADIS16488_PAGE0_DIAG_STS),
  ADIS16488_READ(ADIS16488_PAGE0_TEMP_OUT),
  ADIS16488_READ(ADIS16488_PAGE0_X_MAGN_OUT),
  ADIS16488_READ(ADIS16488_PAGE0_Y_MAGN_OUT),
  ADIS16488_READ(ADIS16488_PAGE0_Z_MAGN_OUT),
  ADIS16488_READ(ADIS16488_PAGE0_BAROM_LOW),
  ADIS16488_READ(ADIS16488_PAGE0_BAROM_OUT),
  ADIS16488_READ(ADIS16488_PAGE0_X_GYRO_LOW),
  ADIS16488_READ(ADIS16488_PAGE0_X_GYRO_OUT),
  ADIS16488_READ(ADIS16488_PAGE0_Y_GYRO_LOW),
  ADIS16488_READ(ADIS16488_PAGE0_Y_GYRO_OUT),
  ADIS16488_READ(ADIS16488_PAGE0_Z_GYRO_LOW),
  ADIS16488_READ(ADIS16488_PAGE0_Z_GYRO_OUT),
  ADIS16488_READ(ADIS16488_PAGE0_X_ACCL_LOW),
  ADIS16488_READ(ADIS16488_PAGE0_X_ACCL_OUT),
  ADIS16488_READ(ADIS16488_PAGE0_Y_ACCL_LOW),
  ADIS16488_READ(ADIS16488_PAGE0_Y_ACCL_OUT),
  ADIS16488_READ(ADIS16488_PAGE0_Z_ACCL_LOW),
  ADIS16488_READ(ADIS16488_PAGE0_Z_ACCL_OUT),
  ADIS16488_TURN_PAGE(0)
};

COMPILE_ASSERT(ADIS16488_COEF_SET_MOSI_SIZE <= ADIS16488_SPI_BUFFERS,
               ADIS16488_SPI_BUFFERS_too_small_for_COEF_SET_MOSI);
COMPILE_ASSERT(ADIS16488_COEF_GET_MOSI_SIZE <= ADIS16488_SPI_BUFFERS,
               ADIS16488_SPI_BUFFERS_too_small_for_COEF_GET_MOSI);
COMPILE_ASSERT(ARRAYSIZE(kMosiGetIdentity) <= ADIS16488_SPI_BUFFERS,
               ADIS16488_SPI_BUFFERS_too_small_for_kMosiGetIdentity);
COMPILE_ASSERT(ARRAYSIZE(kMosiStartSelfTest) <= ADIS16488_SPI_BUFFERS,
               ADIS16488_SPI_BUFFERS_too_small_for_kMosiStartSelftest);
COMPILE_ASSERT(ARRAYSIZE(kMosiGetGlobCmd) <= ADIS16488_SPI_BUFFERS,
               ADIS16488_SPI_BUFFERS_too_small_for_kMosiGetGlobCmd);
COMPILE_ASSERT(ARRAYSIZE(kMosiGetControl) <= ADIS16488_SPI_BUFFERS,
               ADIS16488_SPI_BUFFERS_too_small_for_kMosiGetControl);
COMPILE_ASSERT(ARRAYSIZE(kMosiSetControl) <= ADIS16488_SPI_BUFFERS,
               ADIS16488_SPI_BUFFERS_too_small_for_kMosiSetControl);
COMPILE_ASSERT(ARRAYSIZE(kMosiGetOutputData) <= ADIS16488_SPI_BUFFERS,
               ADIS16488_SPI_BUFFERS_too_small_for_kMosiGetOutputData);

typedef enum {
  kImuReset,
  kImuResetRecovery,
  kImuGetIdentity,
  kImuStartSelfTest,
  kImuWaitForSelfTest,
  kImuGetSelfTest,
  kImuSetCoefBank,
  kImuGetCoefBank,
  kImuSetControl,
  kImuSetFunctionIoControl,
  kImuSetFilterBank0,
  kImuSetFilterBank1,
  kImuGetControl,
  kImuGetOutputData
} ImuState;

static struct {
  ImuState state;
  bool first_entry;
  int64_t timeout;
  int64_t self_test_timeout;
  Adis16488CoefBank bank;
  int32_t block;
} g_imu;

typedef struct {
  int32_t prod_id;
  int32_t firmware_rev;
  int32_t firmware_day;
  int32_t firmware_month;
  int32_t firmware_year;
  int32_t serial_number;
} Adis16488Identity;

static int32_t g_device = -1;  // Set invalid.
static uint16_t g_miso[ADIS16488_SPI_BUFFERS];
static Adis16488Identity g_identity;

static uint16_t BcdToDec(uint16_t bcd) {
  return 1000*((bcd >> 12) & 0x0F) + 100*((bcd >> 8) & 0x0F) +
      10*((bcd >> 4) & 0x0F) + 1*((bcd >> 0) & 0x0F);
}

static float Int16ToFloat(uint16_t buf, float lsb) {
  int16_t raw = (int16_t)buf;
  return raw * lsb;
}

static float Int32ToFloat(const uint16_t *buf, float lsb) {
  int32_t raw = (int32_t)buf[0] | ((int32_t)buf[1] << 16);
  return raw * lsb;
}

// See kMosiGetIdentity.
static void ReadIdentity(Adis16488Identity *out) {
  out->prod_id = g_miso[2];
  out->firmware_rev = BcdToDec(g_miso[4]);
  out->firmware_day = BcdToDec(g_miso[5] & 0xFF);
  out->firmware_month = BcdToDec(g_miso[5] >> 8);
  out->firmware_year = BcdToDec(g_miso[6]);
  out->serial_number = g_miso[8];

  if (out->prod_id != 0xFFFF) {
    printf("IMU Product Id    = %ld\n", out->prod_id);
    printf("IMU Firmware      = rev=%ld %ld/%ld/%ld\n", out->firmware_rev,
           out->firmware_month, out->firmware_day, out->firmware_year);
    printf("IMU Serial Number = %ld\n", out->serial_number);
  }
}

// See kMosiGetGlobCmd.
static uint16_t ReadGlobCmd(void) {
  return g_miso[2];
}

// See kMosiGetOutputData.
static void ReadOutputData(Adis16488OutputData *out) {
  out->seq_cnt = g_miso[2];
  out->sys_e_flag = g_miso[3];
  out->diag_sts = g_miso[4];
  out->dt = (float)(ADIS16488_DEC_RATE + 1) / (float)ADIS16488_SYNC_FREQ_HZ;
  out->temp = ADIS16488_TEMP_OFFSET + ADIS16488_TEMP_LSB * (int16_t)g_miso[5];
  out->mag[0] = Int16ToFloat(g_miso[6], ADIS16488_MAG_LSB);
  out->mag[1] = Int16ToFloat(g_miso[7], ADIS16488_MAG_LSB);
  out->mag[2] = Int16ToFloat(g_miso[8], ADIS16488_MAG_LSB);
  out->baro = Int32ToFloat(&g_miso[9], ADIS16488_BARO_LSB);
  out->gyro[0] = Int32ToFloat(&g_miso[11], ADIS16488_GYRO_LSB);
  out->gyro[1] = Int32ToFloat(&g_miso[13], ADIS16488_GYRO_LSB);
  out->gyro[2] = Int32ToFloat(&g_miso[15], ADIS16488_GYRO_LSB);
  out->accel[0] = Int32ToFloat(&g_miso[17], ADIS16488_ACCEL_LSB);
  out->accel[1] = Int32ToFloat(&g_miso[19], ADIS16488_ACCEL_LSB);
  out->accel[2] = Int32ToFloat(&g_miso[21], ADIS16488_ACCEL_LSB);
}

static void ImuReset(int64_t now, ImuState next) {
  if (g_imu.first_entry) {
    N2hetSetOutputLow(IMU_RESET_PIN);
    MibSPITriggerDisable(IMU_SPI, g_device);
    g_imu.timeout = now + ADIS16488_RESET_US;
    g_imu.bank = kAdis16488CoefBankA;
    g_imu.block = 0;
  } else if (now >= g_imu.timeout) {
    Adis16488ResetRelease();
    g_imu.state = next;
  }
}

static bool ImuSendReceive(int64_t t_stall, int32_t length,
                           const uint16_t *mosi, int64_t now) {
  assert(0 <= length && length <= ADIS16488_SPI_BUFFERS);
  assert(mosi != NULL);

  static bool transfer_complete = false;
  static int64_t transfer_timeout = 0;
  static int64_t stall_timeout = 0;

  if (g_imu.first_entry) {
    MibSPIWriteUint16(IMU_SPI, g_device, length, 1, mosi);
    MibSPITriggerBySoftware(IMU_SPI, g_device);
    transfer_complete = false;
    transfer_timeout = now + ADIS16488_TRANSFER_TIMEOUT_US;
    stall_timeout = now + t_stall;
  } else if (transfer_complete) {
    // Done.
  } else if (MibSPIReadUint16(IMU_SPI, g_device, length, 1, g_miso)) {
    transfer_complete = true;
  } else if (now >= transfer_timeout) {
    g_imu.state = kImuReset;
  }
  return transfer_complete && now >= stall_timeout;
}

static void ImuGetIdentity(int64_t now, ImuState next) {
  if (ImuSendReceive(0, ARRAYSIZE(kMosiGetIdentity), kMosiGetIdentity, now)) {
    ReadIdentity(&g_identity);
    if (g_identity.prod_id == ADIS16488_PROD_ID) {
      g_imu.state = next;
    } else {
      g_imu.state = kImuReset;
    }
  }
}

static void ImuStartSelfTest(int64_t now, ImuState next) {
  if (ImuSendReceive(ADIS16488_SELF_TEST_STALL_US,
                     ARRAYSIZE(kMosiStartSelfTest),
                     kMosiStartSelfTest, now)) {
    g_imu.self_test_timeout = now + ADIS16488_SELF_TEST_TIMEOUT_US;
    g_imu.state = next;
  }
}

static void ImuWait(int64_t now, int64_t timeout, ImuState next) {
  if (g_imu.first_entry) {
    g_imu.timeout = now + timeout;
  } else if (now >= g_imu.timeout) {
    g_imu.state = next;
  }
}

static void ImuGetSelfTest(int64_t now, ImuState next) {
  if (ImuSendReceive(0, ARRAYSIZE(kMosiGetGlobCmd), kMosiGetGlobCmd, now)) {
    if (ReadGlobCmd() == 0x0) {
      g_imu.state = next;
    } else if (now >= g_imu.self_test_timeout) {
      g_imu.state = kImuReset;
    } else {
      g_imu.state = kImuWaitForSelfTest;
    }
  }
}

static void ImuSetCoefBank(int64_t now, ImuState next) {
  if (ImuSendReceive(0, ADIS16488_COEF_SET_MOSI_SIZE,
                     Adis16488MosiSetCoef(g_imu.bank, g_imu.block), now)) {
    g_imu.state = next;
  }
}

static void ImuGetCoefBank(int64_t now, ImuState next) {
  if (ImuSendReceive(0, ADIS16488_COEF_GET_MOSI_SIZE,
                     Adis16488MosiGetCoef(g_imu.bank, g_imu.block), now)) {
    if (Adis16488ValidateCoef(g_imu.bank, g_imu.block, g_miso)) {
      ++g_imu.block;
      if (g_imu.block == ADIS16488_COEF_BLOCKS_PER_BANK) {
        g_imu.block = 0;
        ++g_imu.bank;
      }
      if (g_imu.bank == kNumAdis16488CoefBanks) {
        g_imu.state = next;
      } else {
        g_imu.state = kImuSetCoefBank;
      }
    } else {
      g_imu.state = kImuReset;
    }
  }
}

static void ImuSetControl(int64_t now, ImuState next) {
  if (ImuSendReceive(0, ARRAYSIZE(kMosiSetControl), kMosiSetControl, now)) {
    g_imu.state = next;
  }
}

static void ImuSetFunctionIoControl(int64_t now, ImuState next) {
  if (ImuSendReceive(ADIS16488_FNCTIO_CTRL_STALL_US,
                     ARRAYSIZE(kMosiSetFnctioCtrl),
                     kMosiSetFnctioCtrl, now)) {
    g_imu.state = next;
  }
}

static void ImuSetFilterBank0(int64_t now, ImuState next) {
  if (ImuSendReceive(ADIS16488_FILTR_BNK_STALL_US,
                     ARRAYSIZE(kMosiSetFiltrBnk0),
                     kMosiSetFiltrBnk0, now)) {
    g_imu.state = next;
  }
}

static void ImuSetFilterBank1(int64_t now, ImuState next) {
  if (ImuSendReceive(ADIS16488_FILTR_BNK_STALL_US,
                     ARRAYSIZE(kMosiSetFiltrBnk1),
                     kMosiSetFiltrBnk1, now)) {
    g_imu.state = next;
  }
}

static void ImuGetControl(int64_t now, ImuState next) {
  if (ImuSendReceive(0, ARRAYSIZE(kMosiGetControl), kMosiGetControl, now)) {
    printf("FNCTIO_CTRL = 0x%04X\n", g_miso[2]);
    printf("CONFIG      = 0x%04X\n", g_miso[3]);
    printf("DEC_RATE    = 0x%04X\n", g_miso[4]);
    printf("NULL_CNFG   = 0x%04X\n", g_miso[5]);
    printf("SLP_CNT     = 0x%04X\n", g_miso[6]);
    printf("FILTR_BNK_0 = 0x%04X\n", g_miso[7]);
    printf("FILTR_BNK_1 = 0x%04X\n", g_miso[8]);

    if (g_miso[2] == ADIS16488_FNCTIO_CTRL
        && g_miso[3] == ADIS16488_CONFIG
        && g_miso[4] == ADIS16488_DEC_RATE
        && g_miso[5] == ADIS16488_NULL_CNFG
        && g_miso[6] == ADIS16488_SLP_CNT
        && g_miso[7] == ADIS16488_FILTR_BNK_0
        && g_miso[8] == ADIS16488_FILTR_BNK_1) {
      g_imu.state = next;
    } else {
      g_imu.state = kImuReset;
    }
  }
}

static bool ImuGetOutputData(int64_t now, Adis16488OutputData *out) {
  assert(out != NULL);
  assert(ARRAYSIZE(kMosiGetOutputData) <= ADIS16488_SPI_BUFFERS);

  bool updated = false;
  if (g_imu.first_entry) {
    MibSPIWriteUint16(IMU_SPI, g_device, ARRAYSIZE(kMosiGetOutputData), 1,
                      kMosiGetOutputData);
    MibSPITrigger(IMU_SPI, g_device, kMibSpiSourceGioA5,
                  kMibSpiEventRisingEdge, false, true);
    g_imu.timeout = now + ADIS16488_DATA_TIMEOUT_US;
  } else {
    updated = MibSPIReadUint16(IMU_SPI, g_device, ARRAYSIZE(kMosiGetOutputData),
                               1, g_miso);
  }
  if (updated) {
    ReadOutputData(out);
    g_imu.timeout = now + ADIS16488_DATA_TIMEOUT_US;
  } else if (now >= g_imu.timeout) {
    MibSPITriggerDisable(IMU_SPI, g_device);
    g_imu.state = kImuReset;
  }
  return updated;
}

// TODO: Create an ePWM module.
static void SetupPwm(void) {
  // Synchronize all time bases (TBCLKSYNC).
  IommSetPinmux(37, 1);

  // Enable eTPWM2A clock via PINMMR37[16].
  IommSetPinmux(37, 16);

  // Disable while configuring.
  EPWM(2).TBCTL.CTRMODE = 3;

  // Disable ePWM interrupts.
  EPWM(2).ETSEL.INTEN = 0;
  EPWM(2).TZEINT.raw = 0x0;

  // Scale TBCLK relative to system clock (VCLK4) at 80 MHz.
  EPWM(2).TBCTL.CLKDIV = 0;
  EPWM(2).TBCTL.HSPCLKDIV = 0;
  EPWM(2).TBCTL.FREE_SOFT = 2;

  // Configure PWM period.
  int32_t t_sync = PeripheralGetClockPrescale(kPeripheralEpwm,
                                              ADIS16488_SYNC_FREQ_HZ);
  assert(t_sync < UINT16_MAX);
  EPWM(2).TBPRD.TBPRD = t_sync - 1;
  EPWM(2).CMPA.CMPA = EPWM(2).TBPRD.TBPRD / 2;  // 50% duty cycle.

  // Mode.
  EPWM(2).TBCTL.PHSEN = 0;     // 0 = Disabled.
  EPWM(2).TBCTL.PRDLD = 0;     // 0 = Shadow mode.
  EPWM(2).TBCTL.SYNCOSEL = 3;  // 3 = Disable.

  // Compare control.
  EPWM(2).CMPCTL.SHDWAMODE = 0;  // Shadow mode.
  EPWM(2).CMPCTL.SHDWBMODE = 0;  // Shadow mode.
  EPWM(2).CMPCTL.LOADAMODE = 0;  // load on CTR = Zero.
  EPWM(2).CMPCTL.LOADBMODE = 0;  // load on CTR = Zero.

  // Action qualifier.
  EPWM(2).AQCTLA.CAU = 1;  // Force EPWM2A output low on CMPA and incrementing.
  EPWM(2).AQCTLA.ZRO = 2;  // Force EPWM2A output high on reload.

  // Trigger interrupt/flag when time-base counter equals CMPA.
  EPWM(2).ETPS.INTPRD = 1;   // 1h = Generate interrupt on first count.
  EPWM(2).ETSEL.INTSEL = 4;  // 4h = TBCNT equals CMPA and incrementing.
  EPWM(2).ETSEL.INTEN = 1;

  // Clear spurious interrupts.
  EPWM(2).ETCLR.raw = 0xFFFF;
  EPWM(2).TZCLR.raw = 0xFFFF;

  // Enable.
  EPWM(2).TBCTR.TBCTR = 0;
  EPWM(2).TBCTL.CTRMODE = 0;  // 0 = Count up.
}

void Adis16488Init(void) {
  // ADIS16884 hardware configuration.
  //
  // TMS570 Pin  ADIS16488 Pin   Type
  // ----------  --------------  ----------
  // SPI3_CLK    IMU_SCLK        Out
  // SPI3_MISO   IMU_MISO        In
  // SPI3_MOSI   IMU_MOSI        Out
  // SPI3_NCS0   IMU_NCS         Out
  // N2HET1[20]  IMU_NRST        Open-drain
  // GIOA[5]     IMU_DIO1:READY  In
  // ETPWM2A     IMU_DIO2:SYNC   Out

  // See Table 4-21 from TMS570 Reference Manual Rev A (TI P/N SPNU515A).
  // Select GIOA5 (default function).
  IommClearPinmux(2, 24);
  // Select ETPWM2A (alternate function 2).
  IommSetPinmux(4, 2);

  // Place IMU in reset during initialization.
  N2hetConfigureAsOutputOpenDrain(IMU_RESET_PIN, false);

  // Pull GIO module out of reset.
  GIO.GCR0.RESET = 1;

  // Initialize IMU_READY (pulses high when data is ready).
  GIO.DIRA.DIR5 = 0;        // 0 = Input.
  GIO.PULDISA.PULDIS5 = 0;  // 0 = Enable pulling.
  GIO.PSLA.PSL5 = 0;        // 0 = Pull down.
  GIO.ENACLR.ENACLRA5 = 1;  // 1 = Disable interrupts.
  GIO.INTDET.INTDETA5 = 0;  // 0 = Either falling or rising edge.
  GIO.POL.POLA5 = 1;        // 1 = Trigger on rising edge.
  GIO.LVLSET.LVLSETA5 = 1;  // 1 = High-level interrupt (GIOOFF1, GIOEMU1).

  // Initialize IMU_SYNC (specifies sample rate).
  // PWM at 50% duty cycle.
  SetupPwm();

  // Initialize global variables.
  memset(&g_identity, 0, sizeof(g_identity));
  memset(&g_imu, 0, sizeof(g_imu));
  g_imu.state = kImuReset;

  // Enable SPI communications.
  g_device = MibSPIAddDevice(IMU_SPI, IMU_CS, ADIS16488_SPI_FREQ_HZ,
                             ADIS16488_SPI_WDELAY_NS, true, true, 16,
                             ADIS16488_SPI_BUFFERS);
}

// Reset line must remain low at least 10 us to assure proper reset
// initiation and recovery.
void Adis16488ResetRelease(void) {
  // Bring IMU out of reset.
  // IMU requires at least 500 ms reset recovery time.
  N2hetSetOutputHigh(IMU_RESET_PIN);
}

void Adis16488WaitForSyncPulse(void) {
  while (!EPWM(2).ETFLG.INT) {}
  EPWM(2).ETCLR.raw = EPWM_ETCLR_INT;
}

bool Adis16488PollOutputData(int64_t now, Adis16488OutputData *out) {
  assert(out != NULL);

  bool updated = false;
  ImuState current = g_imu.state;
  switch (current) {
    case kImuReset:
      ImuReset(now, kImuResetRecovery);
      break;
    case kImuResetRecovery:
      ImuWait(now, ADIS16488_RESET_RECOVERY_US, kImuGetIdentity);
      break;
    case kImuGetIdentity:
      ImuGetIdentity(now, kImuStartSelfTest);
      break;
    case kImuStartSelfTest:
      ImuStartSelfTest(now, kImuWaitForSelfTest);
      break;
    case kImuWaitForSelfTest:
      ImuWait(now, ADIS16488_SELF_TEST_STALL_US, kImuGetSelfTest);
      break;
    case kImuGetSelfTest:
      ImuGetSelfTest(now, kImuSetCoefBank);
      break;
    case kImuSetCoefBank:
      ImuSetCoefBank(now, kImuGetCoefBank);
      break;
    case kImuGetCoefBank:
      ImuGetCoefBank(now, kImuSetControl);
      break;
    case kImuSetControl:
      ImuSetControl(now, kImuSetFunctionIoControl);
      break;
    case kImuSetFunctionIoControl:
      ImuSetFunctionIoControl(now, kImuSetFilterBank0);
      break;
    case kImuSetFilterBank0:
      ImuSetFilterBank0(now, kImuSetFilterBank1);
      break;
    case kImuSetFilterBank1:
      ImuSetFilterBank1(now, kImuGetControl);
      break;
    case kImuGetControl:
      ImuGetControl(now, kImuGetOutputData);
      break;
    case kImuGetOutputData:
      updated = ImuGetOutputData(now, out);
      break;
    default:
      g_imu.state = kImuReset;
      break;
  }
  g_imu.first_entry = (current != g_imu.state);
  return updated;
}

bool Adis16488IsReady(void) {
  return g_imu.state == kImuGetOutputData;
}

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

#include "avionics/firmware/drivers/ltc6804.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/common/fast_math/fast_math.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/spi.h"
#include "avionics/firmware/cpu/spi_pin.h"
#include "common/macros.h"

// SPI settings.
#define LTC6804_SPI 3
#define LTC6804_CSMASK 1
#define LTC6804_FREQ_HZ 750000
#define LTC6804_WDELAY 0
#define LTC6804_POLARITY true
#define LTC6804_PHASE true
#define LTC6804_CHARLEN 8
#define LTC6804_SPI_TIMEOUT_US 1000

// ADC mode.
#define LTC6804_MD_FAST (1 << 7)
#define LTC6804_MD_NORMAL (2 << 7)
#define LTC6804_MD_FILTERED (3 << 7)

// Pull-up / pull-down current for open-wire conversions.
#define LTC6804_PUP_DOWN (0 << 6)
#define LTC6804_PUP_UP (1 << 6)

// Self-test mode selection.
#define LTC6804_ST_TEST1 (1 << 5)
#define LTC6804_ST_TEST2 (2 << 5)

// Time before IsoSPI returns to idle state due to inactivity.
#define LTC6804_IDLE_TIMEOUT_US 4200  // 5.5 ms nominal (range: 4.3 - 6.7 ms).

// Time before core returns to sleep due to inactivity.
#define LTC6804_SLEEP_TIMEOUT_US 1750000  // 2 s nominal (range: 1.8 - 2.2 s).

// Time for core/IsoSPI to transition to standby/ready state.
#define LTC6804_WAKE_US 300  // 100 us nominal (300 us max).

// Time for reference to turn on after conversion or refup command.
#define LTC6804_REFUP_US 5000  // 3.5 ms nominal (4.4 ms max).

// Conversion time tolerance multiplier.
#define LTC6804_TIME_TOL 1.1f

// Time for single conversion (i.e. 2 cells or single gpio).
#define LTC6804_27KHZ_1 (int64_t)(LTC6804_TIME_TOL * 201)
#define LTC6804_14KHZ_1 (int64_t)(LTC6804_TIME_TOL * 230)
#define LTC6804_7KHZ_1 (int64_t)(LTC6804_TIME_TOL * 405)
#define LTC6804_3KHZ_1 (int64_t)(LTC6804_TIME_TOL * 521)
#define LTC6804_2KHZ_1 (int64_t)(LTC6804_TIME_TOL * 754)
#define LTC6804_26HZ_1 (int64_t)(LTC6804_TIME_TOL * 33568)

// Time for four conversions (only applies for ADSTAT command).
#define LTC6804_27KHZ_4 (int64_t)(LTC6804_TIME_TOL * 748)
#define LTC6804_14KHZ_4 (int64_t)(LTC6804_TIME_TOL * 865)
#define LTC6804_7KHZ_4 (int64_t)(LTC6804_TIME_TOL * 1563)
#define LTC6804_3KHZ_4 (int64_t)(LTC6804_TIME_TOL * 2028)
#define LTC6804_2KHZ_4 (int64_t)(LTC6804_TIME_TOL * 2959)
#define LTC6804_26HZ_4 (int64_t)(LTC6804_TIME_TOL * 134218)

// Time for six conversions (i.e. 12 cells or 5 gpio + 2nd reference).
#define LTC6804_27KHZ_6 (int64_t)(LTC6804_TIME_TOL * 1113)
#define LTC6804_14KHZ_6 (int64_t)(LTC6804_TIME_TOL * 1288)
#define LTC6804_7KHZ_6 (int64_t)(LTC6804_TIME_TOL * 2335)
#define LTC6804_3KHZ_6 (int64_t)(LTC6804_TIME_TOL * 3033)
#define LTC6804_2KHZ_6 (int64_t)(LTC6804_TIME_TOL * 4430)
#define LTC6804_26HZ_6 (int64_t)(LTC6804_TIME_TOL * 201317)

// Time for eight conversions (only applies for ADCVAX command).
#define LTC6804_27KHZ_8 (int64_t)(LTC6804_TIME_TOL * 1564)
#define LTC6804_14KHZ_8 (int64_t)(LTC6804_TIME_TOL * 1736)
#define LTC6804_7KHZ_8 (int64_t)(LTC6804_TIME_TOL * 3133)
#define LTC6804_3KHZ_8 (int64_t)(LTC6804_TIME_TOL * 4064)
#define LTC6804_2KHZ_8 (int64_t)(LTC6804_TIME_TOL * 5925)
#define LTC6804_26HZ_8 (int64_t)(LTC6804_TIME_TOL * 268442)

#define LTC6804_VOLT_THRES_MIN 0.0016f  // (0 + 1) * 16 * 100 uV.
#define LTC6804_VOLT_THRES_MAX 6.5536f  // (4095 + 1) * 16 * 100 uV.

static int32_t g_device = 0;
static uint8_t g_mosi[LTC6804_BUF_SIZE(LTC6804_DEVICES)] = {0};
static uint8_t g_miso[LTC6804_BUF_SIZE(LTC6804_DEVICES)] = {0};

// Dummy byte to wakeup device.
static const uint8_t kMosiWakeup[] = {0x00};

static const uint8_t kMosiWriteConfig[] = {0x00, 0x01, 0x3D, 0x6E};
static const uint8_t kMosiReadConfig[] = {0x00, 0x02, 0x2B, 0x0A};
static const uint8_t kMosiReadCellA[] = {0x00, 0x04, 0x07, 0xC2};
static const uint8_t kMosiReadCellB[] = {0x00, 0x06, 0x9A, 0x94};
static const uint8_t kMosiReadCellC[] = {0x00, 0x08, 0x5E, 0x52};
static const uint8_t kMosiReadCellD[] = {0x00, 0x0A, 0xC3, 0x04};
static const uint8_t kMosiReadAuxA[] = {0x00, 0x0C, 0xEF, 0xCC};
static const uint8_t kMosiReadAuxB[] = {0x00, 0x0E, 0x72, 0x9A};
static const uint8_t kMosiReadStatA[] = {0x00, 0x10, 0xED, 0x72};
static const uint8_t kMosiReadStatB[] = {0x00, 0x12, 0x70, 0x24};

static const uint8_t kMosiClearCell[] = {0x07, 0x11, 0xC9, 0xC0};
static const uint8_t kMosiClearAux[] = {0x07, 0x12, 0xDF, 0xA4};
static const uint8_t kMosiClearStatus[] = {0x07, 0x13, 0x54, 0x96};

// Polling does not work with daisy-chained devices.
static const uint8_t kMosiPollAdc[] = {0x07, 0x14, 0xF3, 0x6C};

static const uint8_t kMosiDiagnoseMux[] = {0x07, 0x15, 0x78, 0x5E};

static const uint8_t kMosiWriteComm[] = {0x07, 0x21, 0x24, 0xB2};
static const uint8_t kMosiReadComm[] = {0x07, 0x22, 0x32, 0xD6};
static const uint8_t kMosiStartComm[] = {0x07, 0x23, 0xB9, 0xE4};

typedef enum {
  kLtc6804CommandCell = 0x0260,
  kLtc6804CommandOpenWire = 0x0228,
  kLtc6804CommandCellSelfTest = 0x0207,
  kLtc6804CommandAux = 0x0460,
  kLtc6804CommandAuxSelfTest = 0x0407,
  kLtc6804CommandStat = 0x0468,
  kLtc6804CommandStatSelfTest = 0x040F,
  kLtc6804CommandCellGPIO = 0x046F,
} Ltc6804Command;

typedef enum {
  kLtc6804ConvertForceSigned = -1,
  kLtc6804ConvertSingle = 0,
  kLtc6804ConvertFour = 1,
  kLtc6804ConvertSix = 2,
  kLtc6804ConvertEight = 3,
} Ltc6804ConvertNum;

typedef enum {
  kLtc6804RegisterConfig,
  kLtc6804RegisterCellA,
  kLtc6804RegisterCellB,
  kLtc6804RegisterCellC,
  kLtc6804RegisterCellD,
  kLtc6804RegisterAuxA,
  kLtc6804RegisterAuxB,
  kLtc6804RegisterStatA,
  kLtc6804RegisterStatB,
  kLtc6804RegisterComm,
} Ltc6804Register;

// Lookup table of ADC conversion times.
// Dimension 1: Ltc6804Rate. Dimension 2: Ltc6804ConvertNum.
static const int64_t kConversionTime[6][4] = {
  {LTC6804_27KHZ_1, LTC6804_27KHZ_4, LTC6804_27KHZ_6, LTC6804_27KHZ_8},
  {LTC6804_14KHZ_1, LTC6804_14KHZ_4, LTC6804_14KHZ_6, LTC6804_14KHZ_8},
  {LTC6804_7KHZ_1, LTC6804_7KHZ_4, LTC6804_7KHZ_6, LTC6804_7KHZ_8},
  {LTC6804_3KHZ_1, LTC6804_3KHZ_4, LTC6804_3KHZ_6, LTC6804_3KHZ_8},
  {LTC6804_2KHZ_1, LTC6804_2KHZ_4, LTC6804_2KHZ_6, LTC6804_2KHZ_8},
  {LTC6804_26HZ_1, LTC6804_26HZ_4, LTC6804_26HZ_6, LTC6804_26HZ_8},
};

// PEC lookup table.
static const uint16_t kCrc15PecTable[256] = {
  0x0000, 0xC599, 0xCEAB, 0x0B32, 0xD8CF, 0x1D56, 0x1664, 0xD3FD,
  0xF407, 0x319E, 0x3AAC, 0xFF35, 0x2CC8, 0xE951, 0xE263, 0x27FA,
  0xAD97, 0x680E, 0x633C, 0xA6A5, 0x7558, 0xB0C1, 0xBBF3, 0x7E6A,
  0x5990, 0x9C09, 0x973B, 0x52A2, 0x815F, 0x44C6, 0x4FF4, 0x8A6D,
  0x5B2E, 0x9EB7, 0x9585, 0x501C, 0x83E1, 0x4678, 0x4D4A, 0x88D3,
  0xAF29, 0x6AB0, 0x6182, 0xA41B, 0x77E6, 0xB27F, 0xB94D, 0x7CD4,
  0xF6B9, 0x3320, 0x3812, 0xFD8B, 0x2E76, 0xEBEF, 0xE0DD, 0x2544,
  0x02BE, 0xC727, 0xCC15, 0x098C, 0xDA71, 0x1FE8, 0x14DA, 0xD143,
  0xF3C5, 0x365C, 0x3D6E, 0xF8F7, 0x2B0A, 0xEE93, 0xE5A1, 0x2038,
  0x07C2, 0xC25B, 0xC969, 0x0CF0, 0xDF0D, 0x1A94, 0x11A6, 0xD43F,
  0x5E52, 0x9BCB, 0x90F9, 0x5560, 0x869D, 0x4304, 0x4836, 0x8DAF,
  0xAA55, 0x6FCC, 0x64FE, 0xA167, 0x729A, 0xB703, 0xBC31, 0x79A8,
  0xA8EB, 0x6D72, 0x6640, 0xA3D9, 0x7024, 0xB5BD, 0xBE8F, 0x7B16,
  0x5CEC, 0x9975, 0x9247, 0x57DE, 0x8423, 0x41BA, 0x4A88, 0x8F11,
  0x057C, 0xC0E5, 0xCBD7, 0x0E4E, 0xDDB3, 0x182A, 0x1318, 0xD681,
  0xF17B, 0x34E2, 0x3FD0, 0xFA49, 0x29B4, 0xEC2D, 0xE71F, 0x2286,
  0xA213, 0x678A, 0x6CB8, 0xA921, 0x7ADC, 0xBF45, 0xB477, 0x71EE,
  0x5614, 0x938D, 0x98BF, 0x5D26, 0x8EDB, 0x4B42, 0x4070, 0x85E9,
  0x0F84, 0xCA1D, 0xC12F, 0x04B6, 0xD74B, 0x12D2, 0x19E0, 0xDC79,
  0xFB83, 0x3E1A, 0x3528, 0xF0B1, 0x234C, 0xE6D5, 0xEDE7, 0x287E,
  0xF93D, 0x3CA4, 0x3796, 0xF20F, 0x21F2, 0xE46B, 0xEF59, 0x2AC0,
  0x0D3A, 0xC8A3, 0xC391, 0x0608, 0xD5F5, 0x106C, 0x1B5E, 0xDEC7,
  0x54AA, 0x9133, 0x9A01, 0x5F98, 0x8C65, 0x49FC, 0x42CE, 0x8757,
  0xA0AD, 0x6534, 0x6E06, 0xAB9F, 0x7862, 0xBDFB, 0xB6C9, 0x7350,
  0x51D6, 0x944F, 0x9F7D, 0x5AE4, 0x8919, 0x4C80, 0x47B2, 0x822B,
  0xA5D1, 0x6048, 0x6B7A, 0xAEE3, 0x7D1E, 0xB887, 0xB3B5, 0x762C,
  0xFC41, 0x39D8, 0x32EA, 0xF773, 0x248E, 0xE117, 0xEA25, 0x2FBC,
  0x0846, 0xCDDF, 0xC6ED, 0x0374, 0xD089, 0x1510, 0x1E22, 0xDBBB,
  0x0AF8, 0xCF61, 0xC453, 0x01CA, 0xD237, 0x17AE, 0x1C9C, 0xD905,
  0xFEFF, 0x3B66, 0x3054, 0xF5CD, 0x2630, 0xE3A9, 0xE89B, 0x2D02,
  0xA76F, 0x62F6, 0x69C4, 0xAC5D, 0x7FA0, 0xBA39, 0xB10B, 0x7492,
  0x5368, 0x96F1, 0x9DC3, 0x585A, 0x8BA7, 0x4E3E, 0x450C, 0x8095
};

// Calculate the PEC word.
static uint16_t Crc15Pec(int32_t length, const void *data) {
  assert(length >= 0);
  assert(data != NULL);

  uint16_t pec = 16, addr;  // Initial seed value is 0x0010.
  const uint8_t *buf = (const uint8_t *)data;
  for (int32_t i = 0; i < length; ++i) {
    addr = ((pec >> 7) ^ buf[i]) & 0xFF;
    pec = (pec << 8) ^ kCrc15PecTable[addr];
  }
  return pec << 1;  // Complete PEC with a 0 bit appended to its LSB.
}

static uint16_t UnderVoltFloatToRaw(float val) {
  val = Saturatef(val, LTC6804_VOLT_THRES_MIN, LTC6804_VOLT_THRES_MAX);
  // VUV = (comparison_voltage * 625) - 1.
  return (uint16_t)(lroundf(val * 625.0f) - 1);
}

static uint16_t OverVoltFloatToRaw(float val) {
  val = Saturatef(val, LTC6804_VOLT_THRES_MIN, LTC6804_VOLT_THRES_MAX);
  // VOV = (comparison_voltage *625).
  return (uint16_t)(lroundf(val * 625.0f));
}

static float VoltRawToFloat(uint16_t raw) {
  // Voltage (cell, gpio, reference) = register_value * 100uV.
  return ((float)raw) * 0.0001f;
}

static float TempRawToFloat(uint16_t raw) {
  // Temperature = (raw * 100uV / 7.5mV) - 273 degC.
  return ((float)raw * 0.0001f / 0.0075f) - 273;
}

static float VoltSOCRawToFloat(uint16_t raw) {
  // SOC (sum of cells) voltage = register_value * 100uV * 20.
  return ((float)raw) * 0.002f;
}

static uint8_t GetAdcOption(Ltc6804Rate rate) {
  switch (rate) {
    case kLtc6804Rate27kHz:
    case kLtc6804Rate7kHz:
    case kLtc6804Rate26Hz:
      return 0;
    case kLtc6804Rate14kHz:
    case kLtc6804Rate3kHz:
    case kLtc6804Rate2kHz:
      return 1;
    default:
      return 0;
  }
}

static uint16_t GetAdcMode(Ltc6804Rate rate) {
  switch (rate) {
    case kLtc6804Rate27kHz:
    case kLtc6804Rate14kHz:
      return LTC6804_MD_FAST;
    case kLtc6804Rate7kHz:
    case kLtc6804Rate3kHz:
      return LTC6804_MD_NORMAL;
    case kLtc6804Rate26Hz:
    case kLtc6804Rate2kHz:
      return LTC6804_MD_FILTERED;
    default:
      return LTC6804_MD_NORMAL;
  }
}

static int64_t GetConversionTime(Ltc6804Rate rate, Ltc6804ConvertNum num,
                                 bool reference_on) {
  assert(0 <= rate && rate < ARRAYSIZE(kConversionTime));
  assert(0 <= num && num < ARRAYSIZE(kConversionTime[0]));

  // Find conversion time using lookup table.
  int64_t timeout = kConversionTime[rate][num];

  // Add time for reference to power up if necessary.
  if (!reference_on) {
    timeout += LTC6804_REFUP_US;
  }
  return timeout;
}

// Populate MOSI buffer with write configuration command and data.
static void FillMosiConfig(const Ltc6804 *device, bool discharge_en) {
  memset(g_mosi, 0, device->buf_length);
  memcpy(g_mosi, kMosiWriteConfig, sizeof(kMosiWriteConfig));
  uint8_t buf[8];
  for (int32_t i = 0; i < device->daisy_length; ++i) {
    memset(buf, 0, sizeof(buf));
    buf[0] = (uint8_t)(device->data[i].wr_cfg.gpio << 3);
    buf[0] |= (uint8_t)(device->data[i].wr_cfg.reference_on ? 1 << 2 : 0);
    buf[0] |= (uint8_t)(device->data[i].wr_cfg.adc_option ? 1 : 0);

    buf[1] = (uint8_t)(device->data[i].wr_cfg.under_volt_thres >> 0) & 0xFF;

    buf[2] = (uint8_t)(device->data[i].wr_cfg.under_volt_thres >> 8) & 0x0F;
    buf[2] |= (uint8_t)(device->data[i].wr_cfg.over_volt_thres << 4) & 0xF0;

    buf[3] = (uint8_t)(device->data[i].wr_cfg.over_volt_thres >> 4) & 0xFF;

    if (discharge_en) {
      buf[4] = (uint8_t)(device->data[i].wr_cfg.discharge_en_flag >> 0) & 0xFF;
      buf[5] = (uint8_t)(device->data[i].wr_cfg.discharge_en_flag >> 8) & 0x0F;
    } else {
      buf[4] = 0;
      buf[5] = 0;
    }
    buf[5] |= (uint8_t)(device->data[i].wr_cfg.discharge_timeout << 4) & 0xF0;

    uint16_t pec = Crc15Pec(6, buf);
    WriteUint16Be(pec, &buf[6]);

    // When writing the configuration registers, the correct order is
    // descending from the top most device in the daisy chain.
    memcpy(&g_mosi[(8 * (device->daisy_length - 1 - i)) + 4],
           buf, sizeof(buf));
  }
}

// Decode and record data according to which register was read.
static void ParseData(Ltc6804Register reg, Ltc6804 *device) {
  uint8_t *buf = g_miso;
  for (int32_t i = 0; i < device->daisy_length; ++i) {
    int32_t base = (8 * i) + 4;
    uint16_t pec, raw;
    ReadUint16Be(&buf[base + 6], &pec);
    if (pec != Crc15Pec(6, &buf[base])) {
      device->data[i].error_count++;
    } else {
      switch (reg) {
        case kLtc6804RegisterConfig:
          device->data[i].rd_cfg.gpio = (buf[base + 0] >> 3);
          device->data[i].rd_cfg.reference_on = (buf[base + 0] & 0x04);
          device->data[i].rd_cfg.sw_timer_en = (buf[base + 0] & 0x02);
          device->data[i].rd_cfg.adc_option = (buf[base + 0] & 0x01);

          device->data[i].rd_cfg.under_volt_thres =
              (((uint16_t)buf[base + 2] & 0x0F) << 8) | buf[base + 1];
          device->data[i].rd_cfg.over_volt_thres =
              ((uint16_t)buf[base + 3] << 4) | ((buf[base + 2] & 0xF0) >> 4);

          device->data[i].rd_cfg.discharge_en_flag =
              (((uint16_t)buf[base + 5] & 0x0F) << 8) | buf[base + 4];
          device->data[i].rd_cfg.discharge_timeout =
              ((buf[base + 5] & 0xF0) >> 4);
          break;
        case kLtc6804RegisterCellA:
          ReadUint16Le(&buf[base + 0], &raw);
          device->data[i].cell_voltage[0] = VoltRawToFloat(raw);
          ReadUint16Le(&buf[base + 2], &raw);
          device->data[i].cell_voltage[1] = VoltRawToFloat(raw);
          ReadUint16Le(&buf[base + 4], &raw);
          device->data[i].cell_voltage[2] = VoltRawToFloat(raw);
          break;
        case kLtc6804RegisterCellB:
          ReadUint16Le(&buf[base + 0], &raw);
          device->data[i].cell_voltage[3] = VoltRawToFloat(raw);
          ReadUint16Le(&buf[base + 2], &raw);
          device->data[i].cell_voltage[4] = VoltRawToFloat(raw);
          ReadUint16Le(&buf[base + 4], &raw);
          device->data[i].cell_voltage[5] = VoltRawToFloat(raw);
          break;
        case kLtc6804RegisterCellC:
          ReadUint16Le(&buf[base + 0], &raw);
          device->data[i].cell_voltage[6] = VoltRawToFloat(raw);
          ReadUint16Le(&buf[base + 2], &raw);
          device->data[i].cell_voltage[7] = VoltRawToFloat(raw);
          ReadUint16Le(&buf[base + 4], &raw);
          device->data[i].cell_voltage[8] = VoltRawToFloat(raw);
          break;
        case kLtc6804RegisterCellD:
          ReadUint16Le(&buf[base + 0], &raw);
          device->data[i].cell_voltage[9] = VoltRawToFloat(raw);
          ReadUint16Le(&buf[base + 2], &raw);
          device->data[i].cell_voltage[10] = VoltRawToFloat(raw);
          ReadUint16Le(&buf[base + 4], &raw);
          device->data[i].cell_voltage[11] = VoltRawToFloat(raw);
          break;
        case kLtc6804RegisterAuxA:
          ReadUint16Le(&buf[base + 0], &raw);
          device->data[i].gpio_voltage[0] = VoltRawToFloat(raw);
          ReadUint16Le(&buf[base + 2], &raw);
          device->data[i].gpio_voltage[1] = VoltRawToFloat(raw);
          ReadUint16Le(&buf[base + 4], &raw);
          device->data[i].gpio_voltage[2] = VoltRawToFloat(raw);
          break;
        case kLtc6804RegisterAuxB:
          ReadUint16Le(&buf[base + 0], &raw);
          device->data[i].gpio_voltage[3] = VoltRawToFloat(raw);
          ReadUint16Le(&buf[base + 2], &raw);
          device->data[i].gpio_voltage[4] = VoltRawToFloat(raw);
          ReadUint16Le(&buf[base + 4], &raw);
          device->data[i].reference_voltage = VoltRawToFloat(raw);
          break;
        case kLtc6804RegisterStatA:
          ReadUint16Le(&buf[base + 0], &raw);
          device->data[i].sum_cell_voltage = VoltSOCRawToFloat(raw);
          ReadUint16Le(&buf[base + 2], &raw);
          device->data[i].internal_temperature = TempRawToFloat(raw);
          ReadUint16Le(&buf[base + 4], &raw);
          device->data[i].ana_supply_voltage = VoltRawToFloat(raw);
          break;
        case kLtc6804RegisterStatB:
          ReadUint16Le(&buf[base + 0], &raw);
          device->data[i].dig_supply_voltage = VoltRawToFloat(raw);
          uint32_t flag;
          ReadUint24Le(&buf[base + 2], &flag);
          device->data[i].uv_ov_flag = flag;
          device->data[i].revision = (buf[base + 5] & 0xF0);
          device->data[i].mux_fail = (buf[base + 5] & 0x02);
          device->data[i].thermal_shutdown = (buf[base + 5] & 0x01);
          break;
        case kLtc6804RegisterComm:
          // Do nothing for now.
          break;
        default:
          assert(false);
          break;
      }
    }
  }
}

static bool BmsTransfer(int64_t now, int32_t length, const uint8_t *mosi,
                        Ltc6804 *device) {
  assert(0 < length && length <= device->buf_length);
  assert(mosi != NULL);

  if (device->first_entry) {
    if (now >= device->isospi_idle_timeout &&
        device->state != kLtc6804StateWakeup) {
      // IsoSPI port is idle. Wake up before attempting communications.
      device->next = device->state;
      device->state = kLtc6804StateWakeup;
      return false;
    }
    MibSPIWriteUint8(LTC6804_SPI, g_device, 1, length, mosi);
    MibSPITriggerBySoftware(LTC6804_SPI, g_device);
    device->transfer_timeout = now + LTC6804_SPI_TIMEOUT_US;
    // Record time at which IsoSPI will return to idle (sleep) state.
    device->isospi_idle_timeout = now + LTC6804_IDLE_TIMEOUT_US;
  } else if (MibSPIReadUint8(LTC6804_SPI, g_device, 1, length, g_miso)) {
    // Transfer complete.
    return true;
  } else if (now >= device->transfer_timeout) {
    // Wake up IsoSPI before attempting further communications.
    device->next = device->state;
    device->state = kLtc6804StateWakeup;
  }
  return false;
}

// Read a specific register group.
static void BmsRead(int64_t now, Ltc6804Register reg, Ltc6804State next,
                    Ltc6804 *device) {
  if (device->first_entry) {
    memset(g_mosi, 0, device->buf_length);
    switch (reg) {
      case kLtc6804RegisterConfig:
        memcpy(g_mosi, kMosiReadConfig, sizeof(kMosiReadConfig));
        break;
      case kLtc6804RegisterCellA:
        memcpy(g_mosi, kMosiReadCellA, sizeof(kMosiReadCellA));
        break;
      case kLtc6804RegisterCellB:
        memcpy(g_mosi, kMosiReadCellB, sizeof(kMosiReadCellB));
        break;
      case kLtc6804RegisterCellC:
        memcpy(g_mosi, kMosiReadCellC, sizeof(kMosiReadCellC));
        break;
      case kLtc6804RegisterCellD:
        memcpy(g_mosi, kMosiReadCellD, sizeof(kMosiReadCellD));
        break;
      case kLtc6804RegisterAuxA:
        memcpy(g_mosi, kMosiReadAuxA, sizeof(kMosiReadAuxA));
        break;
      case kLtc6804RegisterAuxB:
        memcpy(g_mosi, kMosiReadAuxB, sizeof(kMosiReadAuxB));
        break;
      case kLtc6804RegisterStatA:
        memcpy(g_mosi, kMosiReadStatA, sizeof(kMosiReadStatA));
        break;
      case kLtc6804RegisterStatB:
        memcpy(g_mosi, kMosiReadStatB, sizeof(kMosiReadStatB));
        break;
      case kLtc6804RegisterComm:
        memcpy(g_mosi, kMosiReadComm, sizeof(kMosiReadComm));
        break;
      default:
        assert(false);
        break;
    }
  }
  if (BmsTransfer(now, device->buf_length, g_mosi, device)) {
    ParseData(reg, device);
    device->state = next;
  }
}

// Start status group ADC conversion.
static void BmsConvertStat(int64_t now, Ltc6804State next, Ltc6804 *device) {
  if (device->first_entry) {
    memset(g_mosi, 0, device->buf_length);
    uint16_t command = kLtc6804CommandStat;
    command |= GetAdcMode(device->ctrl.rate);
    command |= device->ctrl.stat_channels;
    WriteUint16Be(command, g_mosi);

    uint16_t pec = Crc15Pec(2, g_mosi);
    WriteUint16Be(pec, &g_mosi[2]);
  }
  if (BmsTransfer(now, 4, g_mosi, device)) {
    // Set timeout for expected completion time.
    Ltc6804ConvertNum num;
    if (device->ctrl.stat_channels == kLtc6804StatChAll) {
      num = kLtc6804ConvertFour;
    } else {
      num = kLtc6804ConvertSingle;
    }
    device->conversion_done_time =
        GetConversionTime(device->ctrl.rate, num, device->ctrl.reference_on)
        + now;
    device->state = next;
  }
}

// Start GPIO 1-5, 2nd reference ADC conversion.
static void BmsConvertAux(int64_t now, Ltc6804State next, Ltc6804 *device) {
  if (device->first_entry) {
    memset(g_mosi, 0, device->buf_length);
    uint16_t command = kLtc6804CommandAux;
    command |= GetAdcMode(device->ctrl.rate);
    command |= device->ctrl.aux_channels;
    WriteUint16Be(command, g_mosi);

    uint16_t pec = Crc15Pec(2, g_mosi);
    WriteUint16Be(pec, &g_mosi[2]);
  }
  if (BmsTransfer(now, 4, g_mosi, device)) {
    // Set timeout for expected completion time.
    Ltc6804ConvertNum num;
    if (device->ctrl.aux_channels == kLtc6804AuxChAll) {
      num = kLtc6804ConvertSix;
    } else {
      num = kLtc6804ConvertSingle;
    }
    device->conversion_done_time =
        GetConversionTime(device->ctrl.rate, num, device->ctrl.reference_on)
        + now;
    device->state = next;
  }
}

// Start cell voltage ADC conversion.
static void BmsConvertCells(int64_t now, Ltc6804State next, Ltc6804 *device) {
  if (device->first_entry) {
    memset(g_mosi, 0, device->buf_length);
    uint16_t command = kLtc6804CommandCell;
    command |= GetAdcMode(device->ctrl.rate);
    if (device->ctrl.discharge_permitted) {
      command |= LTC6804_DCP_ENABLED;
    }
    command |= device->ctrl.cell_channels;
    WriteUint16Be(command, g_mosi);

    uint16_t pec = Crc15Pec(2, g_mosi);
    WriteUint16Be(pec, &g_mosi[2]);
  }
  if (BmsTransfer(now, 4, g_mosi, device)) {
    // Set timeout for expected completion time.
    Ltc6804ConvertNum num;
    if (device->ctrl.cell_channels == kLtc6804CellChAll) {
      num = kLtc6804ConvertSix;
    } else {
      num = kLtc6804ConvertSingle;
    }
    device->conversion_done_time =
        GetConversionTime(device->ctrl.rate, num, device->ctrl.reference_on)
        + now;
    device->state = next;
  }
}

static void BmsConversionWait(int64_t now, Ltc6804State next, Ltc6804 *device) {
  if (now >= device->conversion_done_time) {
    if (now >= device->isospi_idle_timeout) {
      // IsoSPI port has timed out. Send wakeup signal.
      device->next = next;
      device->state = kLtc6804StateWakeup;
    } else {
      device->state = next;
    }
  }
}

// Write to configuration register group.
static void BmsWriteConfig(int64_t now, bool discharge_en, Ltc6804State next,
                           Ltc6804 *device) {
  if (device->first_entry) {
    FillMosiConfig(device, discharge_en);
  }
  if (BmsTransfer(now, device->buf_length, g_mosi, device)) {
    device->state = next;
  }
}

// Wait for devices to wake up.
static void BmsWakeupWait(int64_t now, Ltc6804 *device) {
  // Wait for wakeup signal to propagate to all daisy chained devices.
  if (now >= device->wakeup_timeout) {
    device->state = device->next;
  }
}

// Send dummy byte to wake up isoSPI port.
static void BmsWakeup(int64_t now, Ltc6804 *device) {
  // Transfer wakeup byte.
  if (BmsTransfer(now, ARRAYSIZE(kMosiWakeup), kMosiWakeup, device)) {
    device->wakeup_timeout = now + device->daisy_length * LTC6804_WAKE_US;
    device->state = kLtc6804StateWakeupWait;
  }
}

void Ltc6804Init(const Ltc6804Control *ctrl, int32_t num_daisy_chain,
                 Ltc6804 *device) {
  assert(ctrl != NULL);
  assert(device != NULL);
  assert(0 < num_daisy_chain && num_daisy_chain <= LTC6804_DEVICES);

  // Initialize LTC6804 object.
  device->state = kLtc6804StateWakeup;
  device->next = kLtc6804StateWriteConfigMeasure;
  device->first_entry = true;
  device->ctrl = *ctrl;
  device->daisy_length = num_daisy_chain;
  device->buf_length = LTC6804_BUF_SIZE(num_daisy_chain);
  device->isospi_idle_timeout = 0;
  device->data_ready = false;

  for (int32_t i = 0; i < device->daisy_length; ++i) {
    device->data[i].error_count = 0;

    // Update write configuration.
    device->data[i].wr_cfg.gpio = 0;
    device->data[i].wr_cfg.reference_on = device->ctrl.reference_on;
    device->data[i].wr_cfg.adc_option = GetAdcOption(device->ctrl.rate);
    device->data[i].wr_cfg.under_volt_thres =
        UnderVoltFloatToRaw(device->ctrl.under_volt_thres);
    device->data[i].wr_cfg.over_volt_thres =
        OverVoltFloatToRaw(device->ctrl.over_volt_thres);
    device->data[i].wr_cfg.discharge_en_flag = 0;
    device->data[i].wr_cfg.discharge_timeout = device->ctrl.discharge_timeout;
  }

  // Initialize SPI send/receive buffers.
  memset(g_mosi, 0, device->buf_length);
  memset(g_miso, 0, device->buf_length);

  // Enable SPI communications.
  g_device = MibSPIAddDevice(LTC6804_SPI, LTC6804_CSMASK, LTC6804_FREQ_HZ,
                             LTC6804_WDELAY, LTC6804_POLARITY, LTC6804_PHASE,
                             LTC6804_CHARLEN, device->buf_length);
}

// Update configuration of gpio outputs and discharge switches.
void Ltc6804BuildConfig(const uint8_t *gpio, const uint16_t *discharge,
                        Ltc6804 *device) {
  assert(gpio != NULL);
  assert(discharge != NULL);

  for (int32_t i = 0; i < device->daisy_length; ++i) {
    device->data[i].wr_cfg.gpio = gpio[i];
    device->data[i].wr_cfg.reference_on = device->ctrl.reference_on;
    device->data[i].wr_cfg.adc_option = GetAdcOption(device->ctrl.rate);
    device->data[i].wr_cfg.under_volt_thres =
        UnderVoltFloatToRaw(device->ctrl.under_volt_thres);
    device->data[i].wr_cfg.over_volt_thres =
        OverVoltFloatToRaw(device->ctrl.over_volt_thres);
    device->data[i].wr_cfg.discharge_en_flag = discharge[i];
    device->data[i].wr_cfg.discharge_timeout = device->ctrl.discharge_timeout;
  }
}

// Update desired state of discharge switches.
void Ltc6804SetDischarge(const uint16_t *discharge, Ltc6804 *device) {
  assert(discharge != NULL);

  for (int32_t i = 0; i < device->daisy_length; ++i) {
    device->data[i].wr_cfg.discharge_en_flag = discharge[i];
  }
}

// Main state machine.
bool Ltc6804PollData(int64_t now, Ltc6804 *device) {
  assert(device != NULL);

  device->data_ready = false;
  bool yield = false;
  Ltc6804State current = device->state;
  switch (current) {
    case kLtc6804StateWakeup:
      // This state is always followed by kLtc6804StateWakeupWait.
      BmsWakeup(now, device);
      break;
    case kLtc6804StateWakeupWait:
      // The state that runs after wakeup sequence is set by device->next.
      BmsWakeupWait(now, device);
      break;
    case kLtc6804StateWriteConfigMeasure:
      BmsWriteConfig(now, false, kLtc6804StateConvertCells, device);
      break;
    case kLtc6804StateConvertCells:
      BmsConvertCells(now, kLtc6804StateWaitCells, device);
      break;
    case kLtc6804StateWaitCells:
      BmsConversionWait(now, kLtc6804StateWriteConfigDischarge, device);
      break;
    case kLtc6804StateWriteConfigDischarge:
      BmsWriteConfig(now, true, kLtc6804StateReadConfig, device);
      break;
    case kLtc6804StateReadConfig:
      BmsRead(now, kLtc6804RegisterConfig, kLtc6804StateConvertStat, device);
      break;
    case kLtc6804StateConvertStat:
      BmsConvertStat(now, kLtc6804StateReadCellA, device);
      break;
    case kLtc6804StateReadCellA:
      BmsRead(now, kLtc6804RegisterCellA, kLtc6804StateReadCellB, device);
      break;
    case kLtc6804StateReadCellB:
      BmsRead(now, kLtc6804RegisterCellB, kLtc6804StateReadCellC, device);
      break;
    case kLtc6804StateReadCellC:
      BmsRead(now, kLtc6804RegisterCellC, kLtc6804StateReadCellD, device);
      break;
    case kLtc6804StateReadCellD:
      BmsRead(now, kLtc6804RegisterCellD, kLtc6804StateWaitStat, device);
      break;
    case kLtc6804StateWaitStat:
      BmsConversionWait(now, kLtc6804StateConvertAux, device);
      break;
    case kLtc6804StateConvertAux:
      BmsConvertAux(now, kLtc6804StateReadStatA, device);
      break;
    case kLtc6804StateReadStatA:
      BmsRead(now, kLtc6804RegisterStatA, kLtc6804StateReadStatB, device);
      break;
    case kLtc6804StateReadStatB:
      BmsRead(now, kLtc6804RegisterStatB, kLtc6804StateWaitAux, device);
      break;
    case kLtc6804StateWaitAux:
      BmsConversionWait(now, kLtc6804StateReadAuxA, device);
      break;
    case kLtc6804StateReadAuxA:
      BmsRead(now, kLtc6804RegisterAuxA, kLtc6804StateReadAuxB, device);
      break;
    case kLtc6804StateReadAuxB:
      BmsRead(now, kLtc6804RegisterAuxB, kLtc6804StateWriteConfigMeasure,
              device);
      device->data_ready = true;
      break;
    default:
      device->state = kLtc6804StateWakeup;
      device->next = kLtc6804StateWriteConfigMeasure;
      assert(false);
      break;
  }

  if (current != device->state) {
    device->first_entry = true;
    if (current != kLtc6804StateWakeupWait) {
      // Yield to other tasks when switching states, except for immediately
      // after wakeup.
      yield = true;
    }
  } else {
    device->first_entry = false;
  }

  return yield;
}

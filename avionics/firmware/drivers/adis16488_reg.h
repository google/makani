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

#ifndef AVIONICS_FIRMWARE_DRIVERS_ADIS16488_REG_H_
#define AVIONICS_FIRMWARE_DRIVERS_ADIS16488_REG_H_

// ADIS16488 PROD_ID register value (for identity verification).
#define ADIS16488_PROD_ID 16488

// ADIS16488 PAGE_ID address is common to all register pages.
#define ADIS16488_PAGE_ID 0x00

// ADIS16488 page 0 registers (output data, clock, identification).
#define ADIS16488_PAGE0_SEQ_CNT       0x06
#define ADIS16488_PAGE0_SYS_E_FLAG    0x08
#define ADIS16488_PAGE0_DIAG_STS      0x0A
#define ADIS16488_PAGE0_ALM_STS       0x0C
#define ADIS16488_PAGE0_TEMP_OUT      0x0E
#define ADIS16488_PAGE0_X_GYRO_LOW    0x10
#define ADIS16488_PAGE0_X_GYRO_OUT    0x12
#define ADIS16488_PAGE0_Y_GYRO_LOW    0x14
#define ADIS16488_PAGE0_Y_GYRO_OUT    0x16
#define ADIS16488_PAGE0_Z_GYRO_LOW    0x18
#define ADIS16488_PAGE0_Z_GYRO_OUT    0x1A
#define ADIS16488_PAGE0_X_ACCL_LOW    0x1C
#define ADIS16488_PAGE0_X_ACCL_OUT    0x1E
#define ADIS16488_PAGE0_Y_ACCL_LOW    0x20
#define ADIS16488_PAGE0_Y_ACCL_OUT    0x22
#define ADIS16488_PAGE0_Z_ACCL_LOW    0x24
#define ADIS16488_PAGE0_Z_ACCL_OUT    0x26
#define ADIS16488_PAGE0_X_MAGN_OUT    0x28
#define ADIS16488_PAGE0_Y_MAGN_OUT    0x2A
#define ADIS16488_PAGE0_Z_MAGN_OUT    0x2C
#define ADIS16488_PAGE0_BAROM_LOW     0x2E
#define ADIS16488_PAGE0_BAROM_OUT     0x30
#define ADIS16488_PAGE0_X_DELTANG_LOW 0x40
#define ADIS16488_PAGE0_X_DELTANG_OUT 0x42
#define ADIS16488_PAGE0_Y_DELTANG_LOW 0x44
#define ADIS16488_PAGE0_Y_DELTANG_OUT 0x46
#define ADIS16488_PAGE0_Z_DELTANG_LOW 0x48
#define ADIS16488_PAGE0_Z_DELTANG_OUT 0x4A
#define ADIS16488_PAGE0_X_DELTVEL_LOW 0x4C
#define ADIS16488_PAGE0_X_DELTVEL_OUT 0x4E
#define ADIS16488_PAGE0_Y_DELTVEL_LOW 0x50
#define ADIS16488_PAGE0_Y_DELTVEL_OUT 0x52
#define ADIS16488_PAGE0_Z_DELTVEL_LOW 0x54
#define ADIS16488_PAGE0_Z_DELTVEL_OUT 0x56
#define ADIS16488_PAGE0_TIME_MS_OUT   0x78
#define ADIS16488_PAGE0_TIME_DH_OUT   0x7A
#define ADIS16488_PAGE0_TIME_YM_OUT   0x7C
#define ADIS16488_PAGE0_PROD_ID       0x7E

// ADIS16488 page 2 registers (calibration).
#define ADIS16488_PAGE2_X_GYRO_SCALE  0x04
#define ADIS16488_PAGE2_Y_GYRO_SCALE  0x06
#define ADIS16488_PAGE2_Z_GYRO_SCALE  0x08
#define ADIS16488_PAGE2_X_ACCL_SCALE  0x0A
#define ADIS16488_PAGE2_Y_ACCL_SCALE  0x0C
#define ADIS16488_PAGE2_Z_ACCL_SCALE  0x0E
#define ADIS16488_PAGE2_XG_BIAS_LOW   0x10
#define ADIS16488_PAGE2_XG_BIAS_OUT   0x12
#define ADIS16488_PAGE2_YG_BIAS_LOW   0x14
#define ADIS16488_PAGE2_YG_BIAS_OUT   0x16
#define ADIS16488_PAGE2_ZG_BIAS_LOW   0x18
#define ADIS16488_PAGE2_ZG_BIAS_OUT   0x1A
#define ADIS16488_PAGE2_XA_BIAS_LOW   0x1C
#define ADIS16488_PAGE2_XA_BIAS_OUT   0x1E
#define ADIS16488_PAGE2_YA_BIAS_LOW   0x20
#define ADIS16488_PAGE2_YA_BIAS_OUT   0x22
#define ADIS16488_PAGE2_ZA_BIAS_LOW   0x24
#define ADIS16488_PAGE2_ZA_BIAS_OUT   0x26
#define ADIS16488_PAGE2_HARD_IRON_X   0x28
#define ADIS16488_PAGE2_HARD_IRON_Y   0x2A
#define ADIS16488_PAGE2_HARD_IRON_Z   0x2C
#define ADIS16488_PAGE2_SOFT_IRON_S11 0x2E
#define ADIS16488_PAGE2_SOFT_IRON_S12 0x30
#define ADIS16488_PAGE2_SOFT_IRON_S13 0x32
#define ADIS16488_PAGE2_SOFT_IRON_S21 0x34
#define ADIS16488_PAGE2_SOFT_IRON_S22 0x36
#define ADIS16488_PAGE2_SOFT_IRON_S23 0x38
#define ADIS16488_PAGE2_SOFT_IRON_S31 0x3A
#define ADIS16488_PAGE2_SOFT_IRON_S32 0x3C
#define ADIS16488_PAGE2_SOFT_IRON_S33 0x3E
#define ADIS16488_PAGE2_BR_BIAS_LOW   0x40
#define ADIS16488_PAGE2_BR_BIAS_HIGH  0x42
#define ADIS16488_PAGE2_USER_SCR_1    0x74
#define ADIS16488_PAGE2_USER_SCR_2    0x76
#define ADIS16488_PAGE2_USER_SCR_3    0x78
#define ADIS16488_PAGE2_USER_SCR_4    0x7A
#define ADIS16488_PAGE2_FLSHCNT_LOW   0x7C
#define ADIS16488_PAGE2_FLSHCNT_HIGH  0x7E

// ADIS16488 page 3 registers (control, sample rate, filtering, I/O, alarms).
#define ADIS16488_PAGE3_GLOB_CMD    0x02
#define ADIS16488_PAGE3_FNCTIO_CTRL 0x06
#define ADIS16488_PAGE3_GPIO_CTRL   0x08
#define ADIS16488_PAGE3_CONFIG      0x0A
#define ADIS16488_PAGE3_DEC_RATE    0x0C
#define ADIS16488_PAGE3_NULL_CNFG   0x0E
#define ADIS16488_PAGE3_SLP_CNT     0x10
#define ADIS16488_PAGE3_FILTR_BNK_0 0x16
#define ADIS16488_PAGE3_FILTR_BNK_1 0x18
#define ADIS16488_PAGE3_ALM_CNFG_0  0x20
#define ADIS16488_PAGE3_ALM_CNFG_1  0x22
#define ADIS16488_PAGE3_ALM_CNFG_2  0x24
#define ADIS16488_PAGE3_XG_ALM_MAGN 0x28
#define ADIS16488_PAGE3_YG_ALM_MAGN 0x2A
#define ADIS16488_PAGE3_ZG_ALM_MAGN 0x2C
#define ADIS16488_PAGE3_XA_ALM_MAGN 0x2E
#define ADIS16488_PAGE3_YA_ALM_MAGN 0x30
#define ADIS16488_PAGE3_ZA_ALM_MAGN 0x32
#define ADIS16488_PAGE3_XM_ALM_MAGN 0x34
#define ADIS16488_PAGE3_YM_ALM_MAGN 0x36
#define ADIS16488_PAGE3_ZM_ALM_MAGN 0x38
#define ADIS16488_PAGE3_BR_ALM_MAGN 0x3A
#define ADIS16488_PAGE3_FIRM_REV    0x78
#define ADIS16488_PAGE3_FIRM_DM     0x7A
#define ADIS16488_PAGE3_FIRM_Y      0x7C

// ADIS16488 page 4 registers (serial number).
#define ADIS16488_PAGE4_SERIAL_NUM 0x20

// ADIS16488 page 5 registers (FIR filter bank A, coefficients 0 ot 59).
#define ADIS16488_PAGE5_FIR_COEF_A(index) (0x08 + 2 * (index))

// ADIS16488 page 6 registers (FIR filter bank A, coefficients 60 to 119).
#define ADIS16488_PAGE6_FIR_COEF_A(index) (0x08 + 2 * ((index) - 60))

// ADIS16488 page 7 registers (FIR filter bank B, coefficients 0 to 59).
#define ADIS16488_PAGE7_FIR_COEF_B(index) (0x08 + 2 * (index))

// ADIS16488 page 8 registers (FIR filter bank B, coefficients 60 to 119).
#define ADIS16488_PAGE8_FIR_COEF_B(index) (0x08 + 2 * ((index) - 60))

// ADIS16488 page 9 registers (FIR filter bank C, coefficents 0 to 59).
#define ADIS16488_PAGE9_FIR_COEF_C(index) (0x08 + 2 * (index))

// ADIS16488 page A registers (FIR filter bank C, coefficients 60 to 119).
#define ADIS16488_PAGEA_FIR_COEF_C(index) (0x08 + 2 * ((index) - 60))

// ADIS16488 page B registers (FIR filter bank D, coefficients 0 to 59).
#define ADIS16488_PAGEB_FIR_COEF_D(index) (0x08 + 2 * (index))

// ADIS16488 page C registers (FIR filter bank D, coefficients 60 to 119).
#define ADIS16488_PAGEC_FIR_COEF_D(index) (0x08 + 2 * ((index) - 60))

// See page 10.
#define ADIS16488_WRITE_L(address, data)                \
  (0x8000 | ((address) & 0x7E) << 8 | ((data) & 0xFF))
#define ADIS16488_WRITE_H(address, data)                        \
  (0x8100 | ((address) & 0x7E) << 8 | (((data) >> 8) & 0xFF))
#define ADIS16488_READ(address) (((address) & 0x7E) << 8)
#define ADIS16488_TURN_PAGE(id) (ADIS16488_WRITE_L(ADIS16488_PAGE_ID, (id)))

#endif  // AVIONICS_FIRMWARE_DRIVERS_ADIS16488_REG_H_

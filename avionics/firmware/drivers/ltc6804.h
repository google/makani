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

#ifndef AVIONICS_FIRMWARE_DRIVERS_LTC6804_H_
#define AVIONICS_FIRMWARE_DRIVERS_LTC6804_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ltc6804_types.h"

// Discharge permitted.
#define LTC6804_DCP_DISABLED (0 << 4)
#define LTC6804_DCP_ENABLED (1 << 4)

// Spi buffer size required for a given number of daisy chained devices.
#define LTC6804_BUF_SIZE(daisy_length) (4 + 8 * daisy_length)

// Number of cell measurement channels per chip.
#define LTC6804_NUM_CHANNELS 12

// Maximum number of daisy chained devices is limited by MibSPI buffer length.
#define LTC6804_DEVICES 15

typedef enum {
  kLtc6804StateWakeup,
  kLtc6804StateWakeupWait,
  kLtc6804StateWriteConfigMeasure,  // Cell discharge disabled.
  kLtc6804StateWriteConfigDischarge,  // Cell discharge enabled.
  kLtc6804StateReadConfig,
  kLtc6804StateConvertCells,
  kLtc6804StateWaitCells,
  kLtc6804StateReadCellA,
  kLtc6804StateReadCellB,
  kLtc6804StateReadCellC,
  kLtc6804StateReadCellD,
  kLtc6804StateConvertAux,
  kLtc6804StateWaitAux,
  kLtc6804StateReadAuxA,
  kLtc6804StateReadAuxB,
  kLtc6804StateConvertStat,
  kLtc6804StateWaitStat,
  kLtc6804StateReadStatA,
  kLtc6804StateReadStatB,
} Ltc6804State;

typedef struct {
  uint8_t gpio;
  bool reference_on;
  bool sw_timer_en;
  bool adc_option;
  uint16_t under_volt_thres;
  uint16_t over_volt_thres;
  uint16_t discharge_en_flag;
  uint8_t discharge_timeout;
} Ltc6804Config;

typedef struct {
  int32_t error_count;
  Ltc6804Config wr_cfg;
  Ltc6804Config rd_cfg;
  float cell_voltage[LTC6804_NUM_CHANNELS];
  float gpio_voltage[5];
  float reference_voltage;
  float sum_cell_voltage;
  float internal_temperature;
  float ana_supply_voltage;
  float dig_supply_voltage;
  uint32_t uv_ov_flag;
  uint8_t revision;
  bool mux_fail;
  bool thermal_shutdown;
} Ltc6804Data;

typedef struct {
  Ltc6804Control ctrl;
  Ltc6804Data data[LTC6804_DEVICES];
  Ltc6804State state;
  Ltc6804State next;
  int32_t daisy_length;  // Number of daisy chained devices.
  int32_t buf_length;  // Length of MOSI and MISO buffers.
  bool first_entry;
  int64_t wakeup_timeout;
  int64_t transfer_timeout;
  int64_t isospi_idle_timeout;
  int64_t conversion_done_time;
  bool data_ready;
} Ltc6804;

void Ltc6804Init(const Ltc6804Control *ctrl, int32_t num_daisy_chain,
                 Ltc6804 *device);
void Ltc6804BuildConfig(const uint8_t *gpio, const uint16_t *discharge,
                        Ltc6804 *device);
void Ltc6804SetDischarge(const uint16_t *discharge, Ltc6804 *device);
bool Ltc6804PollData(int64_t now, Ltc6804 *device);

#endif  // AVIONICS_FIRMWARE_DRIVERS_LTC6804_H_

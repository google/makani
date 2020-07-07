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

#ifndef AVIONICS_FIRMWARE_MONITORS_LTC6804_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_LTC6804_TYPES_H_

#include <stdint.h>

#include "avionics/firmware/drivers/ltc6804_types.h"

// Max number of daisy-chained LTC6804s for any AIO carrier.
#define MAX_LTC6804_DEVICES 2

typedef struct {
  int32_t monitor;
  uint8_t stack_level;
  uint16_t input_mask;
  float v_balance_min;
  float v_balance_thres;
  float v_balance_hyst;
  int32_t num_max_simult_bal;
  int32_t num_series_cells;
  Ltc6804Control control;
} Ltc6804Monitor;

typedef struct {
  uint32_t populated;
  int32_t num_devices;
  const Ltc6804Monitor *device;
} Ltc6804Monitors;

#endif  // AVIONICS_FIRMWARE_MONITORS_LTC6804_TYPES_H_

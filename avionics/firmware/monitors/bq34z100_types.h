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

#ifndef AVIONICS_FIRMWARE_MONITORS_BQ34Z100_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_BQ34Z100_TYPES_H_

#include <stdint.h>

#include "avionics/firmware/drivers/bq34z100_types.h"

typedef enum {
  kBq34z100MonitorFlagOverCurrent   = 1 << 0,
  kBq34z100MonitorFlagOverVoltage   = 1 << 1,
  kBq34z100MonitorFlagUnderVoltage  = 1 << 2,
  kBq34z100MonitorFlagLowCharge     = 1 << 3,
} Bq34z100MonitorFlag;

#define BQ34Z100_MONITOR_WARNING_FLAGS (kBq34z100MonitorFlagOverCurrent \
                                        | kBq34z100MonitorFlagOverVoltage \
                                        | kBq34z100MonitorFlagUnderVoltage \
                                        | kBq34z100MonitorFlagLowCharge)

typedef struct {
  int32_t monitor;
  Bq34z100Config config;
  float current_max;
  float voltage_max;
  float voltage_min;
  uint16_t soc_min;
} Bq34z100Monitor;

typedef struct {
  uint32_t populated;
  int32_t num_devices;
  const Bq34z100Monitor *device;
} Bq34z100Monitors;

#endif  // AVIONICS_FIRMWARE_MONITORS_BQ34Z100_TYPES_H_

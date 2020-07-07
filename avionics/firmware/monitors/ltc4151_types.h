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

#ifndef AVIONICS_FIRMWARE_MONITORS_LTC4151_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_LTC4151_TYPES_H_

#include <stdint.h>

#include "avionics/firmware/drivers/ltc4151_types.h"

typedef enum {
  kLtc4151MonitorFlagOverCurrent  = 1 << 0,
  kLtc4151MonitorFlagOverVoltage  = 1 << 1,
  kLtc4151MonitorFlagUnderVoltage = 1 << 2,
} Ltc4151MonitorFlag;

#define LTC4151_MONITOR_WARNING_FLAGS (kLtc4151MonitorFlagOverCurrent   \
                                       | kLtc4151MonitorFlagOverVoltage \
                                       | kLtc4151MonitorFlagUnderVoltage)

typedef struct {
  int32_t monitor;
  Ltc4151Config config;
  float current_max;
  float voltage_max;
  float voltage_min;
} Ltc4151Monitor;

typedef struct {
  uint32_t populated;
  int32_t num_devices;
  const Ltc4151Monitor *device;
} Ltc4151Monitors;

#endif  // AVIONICS_FIRMWARE_MONITORS_LTC4151_TYPES_H_

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

#ifndef AVIONICS_FIRMWARE_MONITORS_INA219_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_INA219_TYPES_H_

#include <stdint.h>

#include "avionics/firmware/drivers/ina219_types.h"

typedef enum {
  kIna219MonitorFlagOverCurrent  = 1 << 0,
  kIna219MonitorFlagOverVoltage  = 1 << 1,
  kIna219MonitorFlagUnderVoltage = 1 << 2,
} Ina219MonitorFlag;

#define INA219_MONITOR_WARNING_FLAGS (kIna219MonitorFlagOverCurrent     \
                                      | kIna219MonitorFlagOverVoltage   \
                                      | kIna219MonitorFlagUnderVoltage)

typedef struct {
  Ina219Config config;
  int32_t monitor;
  float current_max;
  float voltage_max;
  float voltage_min;
  float voltage_nominal;
} Ina219Monitor;

typedef struct {
  uint32_t populated;
  int32_t num_devices;
  const Ina219Monitor *device;
} Ina219Monitors;

#endif  // AVIONICS_FIRMWARE_MONITORS_INA219_TYPES_H_

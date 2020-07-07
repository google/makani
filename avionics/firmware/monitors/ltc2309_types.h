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

#ifndef AVIONICS_FIRMWARE_MONITORS_LTC2309_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_LTC2309_TYPES_H_

#include <stdint.h>

#include "avionics/firmware/drivers/ltc2309_types.h"

typedef enum {
  kLtc2309MonitorFlagOverVoltage  = 1 << 0,
  kLtc2309MonitorFlagUnderVoltage = 1 << 1,
} Ltc2309MonitorFlag;

#define LTC2309_MONITOR_WARNING_FLAGS (kLtc2309MonitorFlagOverVoltage   \
                                       | kLtc2309MonitorFlagUnderVoltage)

typedef struct {
  int32_t monitor;
  Ltc2309Config config;
  float volts_per_count;
  float offset;
  float nominal;
  float min;
  float max;
} Ltc2309MonitorConfig;

typedef struct {
  int32_t num_configs;
  const Ltc2309MonitorConfig *config;
} Ltc2309MonitorDevice;

typedef struct {
  uint32_t populated;
  int32_t num_devices;
  const Ltc2309MonitorDevice *device;
} Ltc2309Monitors;

#endif  // AVIONICS_FIRMWARE_MONITORS_LTC2309_TYPES_H_

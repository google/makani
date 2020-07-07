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

#ifndef AVIONICS_FIRMWARE_MONITORS_ADS7828_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_ADS7828_TYPES_H_

#include <stdint.h>

#include "avionics/firmware/drivers/ads7828_types.h"

typedef enum {
  kAds7828MonitorFlagOverVoltage  = 1 << 0,
  kAds7828MonitorFlagUnderVoltage = 1 << 1,
} Ads7828MonitorFlag;

#define ADS7828_MONITOR_WARNING_FLAGS (kAds7828MonitorFlagOverVoltage   \
                                       | kAds7828MonitorFlagUnderVoltage)

typedef struct {
  int32_t monitor;
  Ads7828Config config;
  float volts_per_count;
  float offset;
  float nominal;
  float min;
  float max;
} Ads7828MonitorConfig;

typedef struct {
  int32_t num_configs;
  const Ads7828MonitorConfig *config;
} Ads7828MonitorDevice;

typedef struct {
  uint32_t populated;
  int32_t num_devices;
  const Ads7828MonitorDevice *device;
} Ads7828Monitors;

#endif  // AVIONICS_FIRMWARE_MONITORS_ADS7828_TYPES_H_

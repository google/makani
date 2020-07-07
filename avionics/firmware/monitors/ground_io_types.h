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

#ifndef AVIONICS_FIRMWARE_MONITORS_GROUND_IO_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_GROUND_IO_TYPES_H_

#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/monitors/ground_io_ads7828_types.h"
#include "avionics/firmware/monitors/ground_io_analog_types.h"

typedef enum {
  kGroundIoMonitorStatusEepromWp = 1 << 0,
} GroundIoMonitorStatus;

typedef enum {
  kGroundIoMonitorWarning12v = 1 << 0,
  kGroundIoMonitorWarningLvA = 1 << 1,
  kGroundIoMonitorWarningLvB = 1 << 2,
} GroundIoMonitorWarning;

typedef struct {
  StatusFlags flags;

  // ADS7828 voltage monitors.
  uint32_t ads7828_populated;
  float ads7828_data[kNumGroundIoAds7828Monitors];
} GroundIoMonitorData;

#endif  // AVIONICS_FIRMWARE_MONITORS_GROUND_IO_TYPES_H_

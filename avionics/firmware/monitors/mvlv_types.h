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

#ifndef AVIONICS_FIRMWARE_MONITORS_MVLV_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_MVLV_TYPES_H_

#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/monitors/mvlv_analog_types.h"
#include "avionics/firmware/monitors/mvlv_ltc2309_types.h"
#include "avionics/firmware/monitors/mvlv_mcp342x_types.h"

typedef enum {
  // Values exceeding min/max only cause warning.
  kMvlvMonitorWarning12v            = 1 << 0,
  kMvlvMonitorWarning3v3            = 1 << 1,
  kMvlvMonitorWarning5v             = 1 << 2,
  kMvlvMonitorWarningIHall          = 1 << 3,
  kMvlvMonitorWarningVExt           = 1 << 4,
  kMvlvMonitorWarningVLv            = 1 << 5,
  kMvlvMonitorWarningVLvOr          = 1 << 6,
  kMvlvMonitorWarningVLvPri         = 1 << 7,
  kMvlvMonitorWarningVLvSec         = 1 << 8,
  kMvlvMonitorWarningTempReadErrors = 1 << 9,
} MvlvMonitorWarning;

typedef enum {
  kMvlvMonitorErrorNone                = 0,
  // Over temperature shows as an error.
  kMvlvMonitorErrorSyncRectMosfetSide = 1 << 1,
  kMvlvMonitorErrorSyncRectPcb        = 1 << 2,
  kMvlvMonitorErrorFilterCap          = 1 << 3,
  kMvlvMonitorErrorOutputSwitch       = 1 << 4,
  kMvlvMonitorErrorSyncRectMosfetTop  = 1 << 5,
  kMvlvMonitorErrorHvResonantCap      = 1 << 6,
  kMvlvMonitorErrorIgbt               = 1 << 7,
  kMvlvMonitorErrorEnclosureAir       = 1 << 8,
} MvlvMonitorError;

typedef enum {
  kMvlvMonitorStatusEnabled      = 1 << 0,
  kMvlvMonitorStatusConnected    = 1 << 1,
  kMvlvMonitorStatusFaultRetry   = 1 << 2,
  kMvlvMonitorStatusCmdReceived  = 1 << 3,
  kMvlvMonitorStatusCmdProcessed = 1 << 4,
} MvlvMonitorStatus;

typedef struct {
  StatusFlags flags;

  // Tms570 ADC channels.
  uint32_t analog_populated;
  float analog_data[kNumMvlvAnalogVoltages];

  // LTC2309 voltage monitors.
  uint32_t ltc2309_populated;
  float ltc2309_data[kNumMvlvLtc2309Monitors];

  // MCP342X temperature monitors.
  uint32_t mcp342x_populated;
  float mcp342x_data[kNumMvlvMcp342xMonitors];
} MvlvMonitorData;

#endif  // AVIONICS_FIRMWARE_MONITORS_MVLV_TYPES_H_

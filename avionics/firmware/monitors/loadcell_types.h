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

#ifndef AVIONICS_FIRMWARE_MONITORS_LOADCELL_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_LOADCELL_TYPES_H_

#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/monitors/analog_types.h"
#include "avionics/firmware/monitors/loadcell_analog_types.h"

typedef enum {
  kLoadcellMonitorWarningVbattArm = 1 << 0,
  kLoadcellMonitorWarningVbattRelease = 1 << 1,
  kLoadcellMonitorWarningReleaseCurrent = 1 << 2,
  kLoadcellMonitorWarningLoadcellBias = 1 << 3,
  kLoadcellMonitorWarning5v = 1 << 4,
} LoadcellMonitorWarning;

typedef struct {
  StatusFlags flags;
  uint32_t analog_populated;
  float analog_data[kNumLoadcellAnalogVoltages];
} LoadcellMonitorData;

typedef enum {
  kBridleJuncWarningLoadPinReadTimeout = 1 << 0,
  kBridleJuncWarningEncoderReadTimeout = 1 << 1,
} BridleJuncWarning;

typedef struct {
  StatusFlags flags;
  float junc_load;
  float junc_angle;
} BridleJuncData;

#endif  // AVIONICS_FIRMWARE_MONITORS_LOADCELL_TYPES_H_

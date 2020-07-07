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

#ifndef AVIONICS_COMMON_LOADCELL_TYPES_H_
#define AVIONICS_COMMON_LOADCELL_TYPES_H_

#include <stdint.h>

// See W7 Loadcell source code.
typedef enum {
  kLoadcellCommandZeroCal = 0xA0C9,  // Perform zero-calibration.
  kLoadcellCommandStream  = 0xA13C,  // Configure for continuous output.
  kLoadcellCommandPoll    = 0xA21B   // Configured for polling (not supported).
} LoadcellCommand;

typedef enum {
  kLoadcellStatusAdcError    = (1 << 0),
  kLoadcellStatusParityError = (1 << 1),
} LoadcellStatus;

typedef enum {
  kLoadcellWarningCh0Invalid = (1 << 0),
  kLoadcellWarningCh1Invalid = (1 << 1),
} LoadcellWarning;

typedef enum {
  kLoadcellErrorLowBattery                = (1 << 0),
  kLoadcellErrorBatteryDisconnected       = (1 << 1),
  kLoadcellErrorReleaseCircuitFailedShort = (1 << 2),
  kLoadcellErrorReleaseCircuitFailedOpen  = (1 << 3),
  kLoadcellErrorReleaseDisconnected       = (1 << 4),
} LoadcellError;

typedef struct {
  uint32_t value_raw;
  float value_raw_mv_per_v;
  float value;
  uint8_t status;
  uint8_t seq_num;
} LoadcellStrain;

typedef struct {
  LoadcellStrain strain[2];
} LoadcellData;

#endif  // AVIONICS_COMMON_LOADCELL_TYPES_H_

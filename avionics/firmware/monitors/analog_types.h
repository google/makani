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

#ifndef AVIONICS_FIRMWARE_MONITORS_ANALOG_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_ANALOG_TYPES_H_

#include <stdint.h>

typedef enum {
  kAnalogTypeVoltage,
  kAnalogTypeLogicLow,
  kAnalogTypeLogicHigh,
  kAnalogTypePortDetect
} AnalogType;

typedef enum {
  kAnalogFlagAsserted     = 1 << 0,
  kAnalogFlagOverVoltage  = 1 << 1,
  kAnalogFlagUnderVoltage = 1 << 2,
} AnalogFlag;

#define ANALOG_MONITOR_STATUS_FLAGS kAnalogFlagAsserted
#define ANALOG_MONITOR_WARNING_FLAGS (kAnalogFlagAsserted               \
                                      | kAnalogFlagOverVoltage          \
                                      | kAnalogFlagUnderVoltage)
#define ANALOG_MONITOR_ERROR_FLAGS (kAnalogFlagAsserted         \
                                    | kAnalogFlagOverVoltage    \
                                    | kAnalogFlagUnderVoltage)

typedef struct {
  AnalogType type;
  int32_t input;
  int32_t voltage;
  int32_t channel;
  float volts_per_count;
  float offset;
  float nominal;
  float min;
  float max;
} AnalogMonitor;

typedef struct {
  uint32_t channel_mask;
  uint32_t populated;
  int32_t num_devices;
  const AnalogMonitor *device;
} AnalogMonitors;

#endif  // AVIONICS_FIRMWARE_MONITORS_ANALOG_TYPES_H_

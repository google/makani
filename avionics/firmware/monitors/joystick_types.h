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

#ifndef AVIONICS_FIRMWARE_MONITORS_JOYSTICK_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_JOYSTICK_TYPES_H_

#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/drivers/ina219_types.h"
#include "avionics/firmware/monitors/joystick_analog_types.h"
#include "avionics/firmware/monitors/joystick_ina219_types.h"

typedef enum {
  kJoystickMonitorStatusEepromWp = 1 << 0,
} JoystickMonitorStatus;

typedef enum {
  kJoystickMonitorWarningLvA = 1 << 0,
  kJoystickMonitorWarningLvB = 1 << 1,
  kJoystickMonitorWarning12v = 1 << 2,
} JoystickMonitorWarning;

typedef struct {
  StatusFlags flags;

  // INA219 voltage monitors.
  uint32_t ina219_populated;
  Ina219OutputData ina219_data[kNumJoystickIna219Monitors];

  // Analog voltage monitors.
  uint32_t analog_populated;
  float analog_data[kNumJoystickAnalogVoltages];
} JoystickMonitorData;

#endif  // AVIONICS_FIRMWARE_MONITORS_JOYSTICK_TYPES_H_

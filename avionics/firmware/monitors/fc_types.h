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

#ifndef AVIONICS_FIRMWARE_MONITORS_FC_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_FC_TYPES_H_

#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/drivers/ina219_types.h"
#include "avionics/firmware/monitors/fc_analog_types.h"
#include "avionics/firmware/monitors/fc_ina219_types.h"

typedef enum {
  kFcMonitorStatusHiltDetect  = 1 << 0,
  kFcMonitorStatusInstDetect  = 1 << 1,
  kFcMonitorStatusPortDetect0 = 1 << 2,
  kFcMonitorStatusPortDetect1 = 1 << 3,
} FcMonitorStatus;

typedef enum {
  kFcMonitorWarning12v     = 1 << 0,
  kFcMonitorWarning12vInst = 1 << 1,
  kFcMonitorWarning1v2     = 1 << 2,
  kFcMonitorWarning3v3     = 1 << 3,
  kFcMonitorWarning3v3Gps  = 1 << 4,
  kFcMonitorWarning3v3Imu  = 1 << 5,
  kFcMonitorWarning5v      = 1 << 6,
  kFcMonitorWarning6vLna   = 1 << 7,
  kFcMonitorWarningTemp    = 1 << 8,
  kFcMonitorWarningVAux    = 1 << 9,
  kFcMonitorWarningVIn     = 1 << 10,
} FcMonitorWarning;

typedef enum {
  kFcMonitorErrorPowerNotGood  = 1 << 0,
  kFcMonitorErrorQ7ThermalTrip = 1 << 1,
} FcMonitorError;

typedef struct {
  StatusFlags flags;
  FcHardware revision;  // Deprecated.

  // INA219 voltage monitors.
  uint32_t ina219_populated;
  Ina219OutputData ina219_data[kNumFcIna219Monitors];

  // Analog voltage monitors.
  uint32_t analog_populated;
  float analog_data[kNumFcAnalogVoltages];
} FcMonitorData;

#endif  // AVIONICS_FIRMWARE_MONITORS_FC_TYPES_H_

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

#ifndef AVIONICS_FIRMWARE_MONITORS_CS_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_CS_TYPES_H_

#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/drivers/ina219_types.h"
#include "avionics/firmware/drivers/si7021_types.h"
#include "avionics/firmware/monitors/cs_analog_types.h"
#include "avionics/firmware/monitors/cs_ina219_types.h"
#include "avionics/firmware/monitors/cs_si7021_types.h"

typedef enum {
  kCsMonitorStatusHiltDetect   = 1 << 0,
  kCsMonitorStatusRadioSignal1 = 1 << 1,
  kCsMonitorStatusRadioSignal2 = 1 << 2,
  kCsMonitorStatusRadioSignal3 = 1 << 3,
  kCsMonitorStatusRadioStatus  = 1 << 4,
  kCsMonitorStatusSfpAuxModAbs = 1 << 5,
  kCsMonitorStatusSfpModAbs    = 1 << 6,
} CsMonitorStatus;

typedef enum {
  kCsMonitorWarning12v    = 1 << 0,
  kCsMonitorWarning1v2    = 1 << 1,
  kCsMonitorWarning2v5    = 1 << 2,
  kCsMonitorWarning3v3    = 1 << 3,
  kCsMonitorWarning3v3Vrl = 1 << 4,
} CsMonitorWarning;

typedef enum {
  kCsMonitorErrorPowerNotGood1v2 = 1 << 0,
  kCsMonitorErrorPowerNotGood2v5 = 1 << 1,
  kCsMonitorErrorPowerNotGood3v3 = 1 << 2,
} CsMonitorError;

typedef struct {
  StatusFlags flags;

  // INA219 voltage monitors.
  uint32_t ina219_populated;
  Ina219OutputData ina219_data[kNumCsIna219Monitors];

  // SI7021 temperature and humidity monitors.
  uint32_t si7021_populated;
  Si7021OutputData si7021_data[kNumCsSi7021Monitors];

  // Analog voltage monitors.
  uint32_t analog_populated;
  float analog_data[kNumCsAnalogVoltages];
} CsMonitorData;

#endif  // AVIONICS_FIRMWARE_MONITORS_CS_TYPES_H_

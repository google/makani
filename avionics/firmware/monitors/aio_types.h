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

#ifndef AVIONICS_FIRMWARE_MONITORS_AIO_TYPES_H_
#define AVIONICS_FIRMWARE_MONITORS_AIO_TYPES_H_

#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/drivers/ina219_types.h"
#include "avionics/firmware/drivers/si7021_types.h"
#include "avionics/firmware/monitors/aio_analog_types.h"
#include "avionics/firmware/monitors/aio_ina219_types.h"
#include "avionics/firmware/monitors/aio_si7021_types.h"
#include "avionics/firmware/monitors/analog_types.h"
#include "avionics/firmware/monitors/ina219_types.h"
#include "avionics/firmware/monitors/si7021_types.h"

typedef enum {
  kAioMonitorStatusGtiDetect       = 1 << 0,
  kAioMonitorStatusPortDetect0     = 1 << 1,
  kAioMonitorStatusPortDetect1     = 1 << 2,
  kAioMonitorStatusPortDetect2     = 1 << 3,
  kAioMonitorStatusPortDetect3     = 1 << 4,
  kAioMonitorStatusWatchdogEnabled = 1 << 5,
} AioMonitorStatus;

typedef enum {
  kAioMonitorWarning12v = 1 << 0,
  kAioMonitorWarning1v2 = 1 << 1,
  kAioMonitorWarning2v5 = 1 << 2,
  kAioMonitorWarning3v3 = 1 << 3,
} AioMonitorWarning;

typedef struct {
  StatusFlags flags;
  AioHardware revision;  // Deprecated.

  // INA219 voltage monitors.
  uint32_t ina219_populated;
  Ina219OutputData ina219_data[kNumAioIna219Monitors];

  // SI7021 temperature and humidity monitors.
  uint32_t si7021_populated;
  Si7021OutputData si7021_data[kNumAioSi7021Monitors];

  // Analog voltage monitors.
  uint32_t analog_populated;
  float analog_data[kNumAioAnalogVoltages];
} AioModuleMonitorData;

#endif  // AVIONICS_FIRMWARE_MONITORS_AIO_TYPES_H_

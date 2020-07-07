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

#ifndef AVIONICS_MOTOR_MONITORS_TYPES_H_
#define AVIONICS_MOTOR_MONITORS_TYPES_H_

#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/drivers/ina219_types.h"
#include "avionics/firmware/drivers/si7021_types.h"
#include "avionics/motor/monitors/motor_ina219_types.h"
#include "avionics/motor/monitors/motor_si7021_types.h"

typedef enum {
  kMotorMonitorWarning12v = 1 << 0,
  kMotorMonitorWarning1v2 = 1 << 1,
  kMotorMonitorWarning3v3 = 1 << 2,
} MotorMonitorWarning;

typedef struct {
  StatusFlags flags;

  // INA219 voltage monitors.
  uint32_t ina219_populated;
  Ina219OutputData ina219_data[kNumMotorIna219Monitors];

  // Si7021 temperature/humidity monitors.
  uint32_t si7021_populated;
  Si7021OutputData si7021_data[kNumMotorSi7021Monitors];
} MotorMonitorData;

#endif  // AVIONICS_MOTOR_MONITORS_TYPES_H_

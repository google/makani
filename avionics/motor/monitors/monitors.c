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

#include "avionics/motor/monitors/monitors.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ina219.h"
#include "avionics/firmware/monitors/ina219.h"
#include "avionics/firmware/serial/motor_serial_params.h"
#include "avionics/motor/monitors/motor_ina219_types.h"
#include "common/macros.h"

// TODO: Integrate other I2C monitors.

static MotorMonitorData *g_monitors = NULL;
static Ina219 g_ina219[MAX_MOTOR_INA219_DEVICES];

static void OutputIna219(int32_t device, const Ina219OutputData *data,
                         uint32_t flags) {
  assert(g_monitors != NULL);
  assert(0 <= device && device < ARRAYSIZE(g_monitors->ina219_data));

  // Map the INA219 voltage monitoring to a common flags field output. The
  // warn_map[] array defines a bitmask to set or clear in SignalWarning.
  // Unspecified entries in the map default to zero, thus calling
  // SignalWarning does not alter the flags field output.
  const uint16_t warn_map[kNumMotorIna219Monitors] = {
    [kMotorIna219Monitor12v] = kMotorMonitorWarning12v,
    [kMotorIna219Monitor1v2] = kMotorMonitorWarning1v2,
    [kMotorIna219Monitor3v3] = kMotorMonitorWarning3v3,
  };
  SignalWarning(warn_map[device], flags & INA219_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);

  g_monitors->ina219_populated |= 1U << device;
  g_monitors->ina219_data[device] = *data;
}

static bool PollIna219(MotorHardware rev) {
  static uint32_t index = 0U;
  return Ina219MonitorPoll(MotorIna219GetConfig(rev), OutputIna219,
                           &index, g_ina219);
}

void MotorMonitorInit(void) {
  Ina219MonitorInit(ARRAYSIZE(g_ina219), g_ina219);
}

bool MotorMonitorPoll(MotorHardware rev, MotorMonitorData *monitors) {
  static bool (*const poll[])(MotorHardware rev) = {
    PollIna219,
  };
  static uint32_t index = 0U;

  // Set for output functions.
  g_monitors = monitors;

  // Make certain that device_index is always valid.
  index %= ARRAYSIZE(poll);
  if (poll[index](rev)) {
    ++index;      // Process all devices concurrently.
    return true;  // Set true when done polling. Data may not update.
  }
  return false;
}

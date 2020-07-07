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

#include "avionics/firmware/monitors/joystick.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ina219.h"
#include "avionics/firmware/monitors/analog.h"
#include "avionics/firmware/monitors/analog_types.h"
#include "avionics/firmware/monitors/ina219.h"
#include "avionics/firmware/monitors/joystick_analog_types.h"
#include "avionics/firmware/monitors/joystick_ina219_types.h"
#include "avionics/firmware/monitors/joystick_types.h"
#include "avionics/firmware/serial/joystick_serial_params.h"
#include "common/macros.h"

static JoystickMonitorData *g_monitors = NULL;
static Ina219 g_ina219[MAX_JOYSTICK_INA219_DEVICES];

static void OutputAnalog(const AnalogMonitor *config, float in,
                         uint32_t flags) {
  assert(g_monitors != NULL);

  // Map the analog input channels to a common flags field output. The
  // status_map[] and warn_map[] arrays define a bitmask to set or clear
  // in functions SetStatus and SignalWarning, respectively. Unspecified
  // entries in the map default to zero, thus calling SetStatus or
  // SignalWarning does not alter the flags field output.
  const uint16_t status_map[kNumJoystickAnalogInputs] = {
    [kJoystickAnalogInputEepromWp] = kJoystickMonitorStatusEepromWp,
  };
  const uint16_t warn_map[kNumJoystickAnalogInputs] = {
    [kJoystickAnalogInputLvA] = kJoystickMonitorWarningLvA,
    [kJoystickAnalogInputLvB] = kJoystickMonitorWarningLvB,
  };

  if (config->type == kAnalogTypeVoltage) {
    assert(0 <= config->voltage
           && config->voltage < ARRAYSIZE(g_monitors->analog_data));
    g_monitors->analog_populated |= 1U << config->voltage;
    g_monitors->analog_data[config->voltage] = in;
  }
  SetStatus(status_map[config->input], flags & ANALOG_MONITOR_STATUS_FLAGS,
            &g_monitors->flags);
  SignalWarning(warn_map[config->input], flags & ANALOG_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);
}

static bool PollAnalog(JoystickHardware rev) {
  return AnalogMonitorPoll(JoystickAnalogGetConfig(rev), OutputAnalog);
}

static void OutputIna219(int32_t device, const Ina219OutputData *data,
                         uint32_t flags) {
  assert(g_monitors != NULL);
  assert(0 <= device && device < ARRAYSIZE(g_monitors->ina219_data));

  // Map the INA219 voltage monitoring to a common flags field output. The
  // warn_map[] array defines a bitmask to set or clear in SignalWarning.
  // Unspecified entries in the map default to zero, thus calling
  // SignalWarning does not alter the flags field output.
  const uint16_t warn_map[kNumJoystickIna219Monitors] = {
    [kJoystickIna219Monitor12v] = kJoystickMonitorWarning12v,
  };
  SignalWarning(warn_map[device], flags & INA219_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);

  g_monitors->ina219_populated |= 1U << device;
  g_monitors->ina219_data[device] = *data;
}

static bool PollIna219(JoystickHardware rev) {
  static uint32_t index = 0U;
  return Ina219MonitorPoll(JoystickIna219GetConfig(rev), OutputIna219,
                           &index, g_ina219);
}

void JoystickMonitorInit(void) {
  AnalogMonitorInit();
  Ina219MonitorInit(ARRAYSIZE(g_ina219), g_ina219);
}

bool JoystickMonitorPoll(JoystickHardware rev,
                         JoystickMonitorData *monitors) {
  static bool (*const poll[])(JoystickHardware rev) = {
    PollAnalog,
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

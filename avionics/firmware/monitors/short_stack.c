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

#include "avionics/firmware/monitors/short_stack.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/faults.h"
#include "avionics/firmware/drivers/mcp342x.h"
#include "avionics/firmware/monitors/analog.h"
#include "avionics/firmware/monitors/mcp342x.h"
#include "avionics/firmware/monitors/short_stack_analog_types.h"
#include "avionics/firmware/monitors/short_stack_mcp342x_types.h"
#include "avionics/firmware/monitors/short_stack_types.h"
#include "avionics/firmware/serial/short_stack_serial_params.h"
#include "common/macros.h"

static ShortStackMonitorData *g_monitors = NULL;
static Mcp342x g_mcp342x[MAX_SHORT_STACK_MCP342X_DEVICES];

static float ThresholdRawToVolts(int32_t monitor, int32_t d_raw) {
  float lvl_bits_to_volts = 0.04696875;  // (751.5 [V/V] * 2.048V / (2^15)).
  float main_bits_to_volts = 0.12525;  // (2004 [V/V] * 2.048V / (2^15 bits)).
  if (monitor == kShortStackMcp342xMonitorMainHi ||
      monitor == kShortStackMcp342xMonitorMainLo) {
    return d_raw * main_bits_to_volts;
  } else if (monitor == kShortStackMcp342xMonitorLvlHi ||
             monitor == kShortStackMcp342xMonitorLvlLo) {
    return d_raw * lvl_bits_to_volts;
  } else {
    // Invalid mcp342x monitor index.
    return 0.0;
  }
}

static void OutputAnalog(const AnalogMonitor *config, float in,
                         uint32_t flags) {
  assert(g_monitors != NULL);

  // Map the analog input channels to a common flags field output.
  // The warn_map[] array defines a bitmask to set or clear in SignalWarning.
  const uint16_t warn_map[kNumShortStackAnalogInputs] = {
    [kShortStackAnalogInput72vfire] = kShortStackMonitorWarning72vfire,
    [kShortStackAnalogInput3v3]     = kShortStackMonitorWarning3v3,
    [kShortStackAnalogInput5v]      = kShortStackMonitorWarning5v,
  };

  const uint16_t error_map[kNumShortStackAnalogInputs] = {
    [kShortStackAnalogInput72vfire] = kShortStackMonitorErrorNone,
    [kShortStackAnalogInput3v3]     = kShortStackMonitorErrorNone,
    [kShortStackAnalogInput5v]      = kShortStackMonitorErrorNone,
  };

  if (config->type == kAnalogTypeVoltage) {
    assert(0 <= config->voltage
           && config->voltage < ARRAYSIZE(g_monitors->analog_data));
    g_monitors->analog_populated |= 1U << config->voltage;
    g_monitors->analog_data[config->voltage] = in;
  }

  SignalWarning(warn_map[config->input], flags & ANALOG_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);
  SignalError(error_map[config->input], flags & ANALOG_MONITOR_ERROR_FLAGS,
              &g_monitors->flags);
}

static void OutputMcp342x(int32_t monitor, int32_t d_raw, bool valid) {
  assert(0 <= monitor && monitor < ARRAYSIZE(g_monitors->mcp342x_data));
  // TODO: Use valid, like batt code, to monitor read errors.

  if (valid) {
    g_monitors->mcp342x_data[monitor] = ThresholdRawToVolts(monitor, d_raw);
    // TODO: Add mcp342x_populated like in batt code?
  }
}

static bool PollAnalog(ShortStackHardware rev) {
  return AnalogMonitorPoll(ShortStackAnalogGetConfig(rev), OutputAnalog);
}

static bool PollMcp342x(ShortStackHardware rev) {
  static uint32_t device_index = 0U;
  static uint32_t config_indices[MAX_SHORT_STACK_MCP342X_DEVICES] = {0U};
  return Mcp342xMonitorPoll(ShortStackMcp342xGetConfig(rev), OutputMcp342x,
                            &device_index, config_indices, g_mcp342x);
}

void ShortStackMonitorInit(void) {
  Mcp342xMonitorInit(ARRAYSIZE(g_mcp342x), g_mcp342x);
}

bool ShortStackMonitorPoll(ShortStackHardware rev,
                           ShortStackMonitorData *monitors) {
  static bool (*const poll[])(ShortStackHardware rev) = {
    PollAnalog,
    PollMcp342x,
  };
  static uint32_t index = 0U;

  // Set for output functions.
  g_monitors = monitors;

  // Make certain that device_index is always valid.
  index %= ARRAYSIZE(poll);
  if (poll[index](rev)) {
    ++index;
    return true;
  }
  return false;
}

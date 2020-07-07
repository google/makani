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

#include "avionics/firmware/monitors/loadcell.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/monitors/analog.h"
#include "avionics/firmware/monitors/loadcell_analog_types.h"
#include "avionics/firmware/serial/loadcell_serial_params.h"
#include "common/macros.h"

static LoadcellMonitorData *g_monitors = NULL;

static void OutputAnalog(const AnalogMonitor *config, float in,
                         uint32_t flags) {
  assert(g_monitors != NULL);

  // Map the analog input channels to a common flags field output. The
  // warn_map[] array defines a bitmask to set or clear in SignalWarning.
  // Unspecified entries in the map default to zero, thus calling
  // SignalWarning does not alter the flags field output.
  const uint16_t warn_map[kNumLoadcellAnalogInputs] = {
    [kLoadcellAnalogInput5v] = kLoadcellMonitorWarning5v,
    [kLoadcellAnalogInputIBatt] = kLoadcellMonitorWarningReleaseCurrent,
    [kLoadcellAnalogInputVLoadcellBias] = kLoadcellMonitorWarningLoadcellBias,
    [kLoadcellAnalogInputVArm] = kLoadcellMonitorWarningVbattArm,
    [kLoadcellAnalogInputVRelease] = kLoadcellMonitorWarningVbattRelease,
  };

  if (config->type == kAnalogTypeVoltage) {
    assert(0 <= config->voltage
           && config->voltage < ARRAYSIZE(g_monitors->analog_data));
    g_monitors->analog_populated |= 1U << config->voltage;
    g_monitors->analog_data[config->voltage] = in;
  }
  SignalWarning(warn_map[config->input], flags & ANALOG_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);
}

static bool PollAnalog(LoadcellHardware rev) {
  return AnalogMonitorPoll(LoadcellAnalogGetConfig(rev), OutputAnalog);
}

void LoadcellMonitorInit(void) {
  AnalogMonitorInit();
}

bool LoadcellMonitorPoll(LoadcellHardware rev, LoadcellMonitorData *monitors) {
  static bool (* const poll[])(LoadcellHardware rev) = {
    PollAnalog,
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

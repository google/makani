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

#include "avionics/firmware/monitors/ground_io.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ads7828.h"
#include "avionics/firmware/monitors/ads7828.h"
#include "avionics/firmware/monitors/ads7828_types.h"
#include "avionics/firmware/monitors/analog.h"
#include "avionics/firmware/monitors/analog_types.h"
#include "avionics/firmware/monitors/ground_io_ads7828_types.h"
#include "avionics/firmware/monitors/ground_io_analog_types.h"
#include "avionics/firmware/monitors/ground_io_types.h"
#include "avionics/firmware/serial/ground_io_serial_params.h"
#include "common/macros.h"

static GroundIoMonitorData *g_monitors = NULL;
static Ads7828 g_ads7828[MAX_GROUND_IO_ADS7828_DEVICES];

static void OutputAds7828(int32_t input, float data, uint32_t flags) {
  assert(g_monitors != NULL);

  // Map the ADS7828 input channels to a common flags field output. The
  // warn_map[] array defines a bitmask to set or clear in function
  // SignalWarning. Unspecified entries in the map default to zero, thus
  // calling SignalWarning does not alter the flags field output.
  const uint16_t warn_map[kNumGroundIoAds7828Monitors] = {
    [kGroundIoAds7828MonitorLvA] = kGroundIoMonitorWarningLvA,
    [kGroundIoAds7828MonitorLvB] = kGroundIoMonitorWarningLvB,
  };

  g_monitors->ads7828_populated |= 1U << input;
  g_monitors->ads7828_data[input] = data;
  SignalWarning(warn_map[input], flags & ADS7828_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);
}

static bool PollAds7828(GroundIoHardware rev) {
  static uint32_t device_index = 0U;
  static uint32_t config_indices[MAX_GROUND_IO_ADS7828_DEVICES] = {0U};
  return Ads7828MonitorPoll(GroundIoAds7828GetConfig(rev), OutputAds7828,
                            &device_index, config_indices, g_ads7828);
}

static void OutputAnalog(const AnalogMonitor *config, float in,
                         uint32_t flags) {
  assert(g_monitors != NULL);
  (void)in;

  // Map the analog input channels to a common flags field output. The
  // flags_map[] array defines a bitmask to set or clear in function
  // SetStatus. Unspecified entries in the map default to zero, thus
  // calling SetStatus does not alter the flags field output.
  const uint16_t flags_map[kNumGroundIoAnalogInputs] = {
    [kGroundIoAnalogInputEepromWp] = kGroundIoMonitorStatusEepromWp,
  };

  SetStatus(flags_map[config->input], flags & ANALOG_MONITOR_STATUS_FLAGS,
            &g_monitors->flags);
}

static bool PollAnalog(GroundIoHardware rev) {
  return AnalogMonitorPoll(GroundIoAnalogGetConfig(rev), OutputAnalog);
}

void GroundIoMonitorInit(void) {
  Ads7828MonitorInit(ARRAYSIZE(g_ads7828), g_ads7828);
  AnalogMonitorInit();
}

bool GroundIoMonitorPoll(GroundIoHardware rev,
                         GroundIoMonitorData *monitors) {
  static bool (*const poll[])(GroundIoHardware rev) = {
    PollAds7828,
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

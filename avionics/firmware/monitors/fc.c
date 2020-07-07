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

#include "avionics/firmware/monitors/fc.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ina219.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/analog.h"
#include "avionics/firmware/monitors/analog_types.h"
#include "avionics/firmware/monitors/fc_analog_types.h"
#include "avionics/firmware/monitors/fc_ina219_types.h"
#include "avionics/firmware/monitors/fc_types.h"
#include "avionics/firmware/monitors/ina219.h"
#include "avionics/firmware/serial/fc_serial_params.h"
#include "common/macros.h"

static FcMonitorData *g_monitors = NULL;
static Ina219 g_ina219[MAX_FC_INA219_DEVICES];

static bool IgnoreQ7ThermalTrip(void) {
  // The following nodes do not have a Q7 board.
  const AioNode node = AppConfigGetAioNode();
  return (node == kAioNodeGpsBaseStation ||
          node == kAioNodeLightPort ||
          node == kAioNodeLightStbd ||
          node == kAioNodeLightTailBottom ||
          node == kAioNodeLightTailTop);
}

static void OutputAnalog(const AnalogMonitor *config, float in,
                         uint32_t flags) {
  assert(g_monitors != NULL);

  // Map the analog input channels to a common flags field output. The
  // flags_map[], warn_map[], and error_map[] arrays define a bitmask to
  // set or clear in functions SetStatus, SignalWarning, and SignalError,
  // respectively. Unspecified entries in the map default to zero, thus
  // calling SetStatus, SignalWarning, or SignalError does not alter the
  // flags field output.
  const uint16_t flags_map[kNumFcAnalogInputs] = {
    [kFcAnalogInputHiltDetect] = kFcMonitorStatusHiltDetect,
    [kFcAnalogInputInstDetect] = kFcMonitorStatusInstDetect,
    [kFcAnalogInputPortDetect0] = kFcMonitorStatusPortDetect0,
    [kFcAnalogInputPortDetect1] = kFcMonitorStatusPortDetect1,
  };
  const uint16_t warn_map[kNumFcAnalogInputs] = {
    [kFcAnalogInput3v3Gps] = kFcMonitorWarning3v3Gps,
    [kFcAnalogInput3v3Imu] = kFcMonitorWarning3v3Imu,
    [kFcAnalogInput6vLna] = kFcMonitorWarning6vLna,
    [kFcAnalogInputVAux] = kFcMonitorWarningVAux,
    [kFcAnalogInputVIn] = kFcMonitorWarningVIn,
  };
  const uint16_t error_map[kNumFcAnalogInputs] = {
    [kFcAnalogInputPowerNotGood] = kFcMonitorErrorPowerNotGood,
    [kFcAnalogInputQ7ThermalTrip] = kFcMonitorErrorQ7ThermalTrip,
  };

  if (config->type == kAnalogTypeVoltage) {
    assert(0 <= config->voltage
           && config->voltage < ARRAYSIZE(g_monitors->analog_data));
    g_monitors->analog_populated |= 1U << config->voltage;
    g_monitors->analog_data[config->voltage] = in;
  }
  SetStatus(flags_map[config->input], flags & ANALOG_MONITOR_STATUS_FLAGS,
            &g_monitors->flags);
  SignalWarning(warn_map[config->input], flags & ANALOG_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);
  SignalError(error_map[config->input], flags & ANALOG_MONITOR_ERROR_FLAGS,
              &g_monitors->flags);
  if (IgnoreQ7ThermalTrip()) {
    g_monitors->flags.error &= ~kFcMonitorErrorQ7ThermalTrip;
  }
}

static bool PollAnalog(FcHardware rev) {
  return AnalogMonitorPoll(FcAnalogGetConfig(rev), OutputAnalog);
}

static void OutputIna219(int32_t device, const Ina219OutputData *data,
                         uint32_t flags) {
  assert(g_monitors != NULL);
  assert(0 <= device && device < ARRAYSIZE(g_monitors->ina219_data));

  // Map the INA219 voltage monitoring to a common flags field output. The
  // warn_map[] array defines a bitmask to set or clear in SignalWarning.
  // Unspecified entries in the map default to zero, thus calling
  // SignalWarning does not alter the flags field output.
  const uint16_t warn_map[kNumFcIna219Monitors] = {
    [kFcIna219Monitor12v] = kFcMonitorWarning12v,
    [kFcIna219Monitor12vInst] = kFcMonitorWarning12vInst,
    [kFcIna219Monitor1v2] = kFcMonitorWarning1v2,
    [kFcIna219Monitor3v3] = kFcMonitorWarning3v3,
    [kFcIna219Monitor5v] = kFcMonitorWarning5v,
  };
  SignalWarning(warn_map[device], flags & INA219_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);

  g_monitors->ina219_populated |= 1U << device;
  g_monitors->ina219_data[device] = *data;
}

static bool PollIna219(FcHardware rev) {
  static uint32_t index = 0U;
  return Ina219MonitorPoll(FcIna219GetConfig(rev), OutputIna219,
                           &index, g_ina219);
}

void FcMonitorInit(void) {
  AnalogMonitorInit();
  Ina219MonitorInit(ARRAYSIZE(g_ina219), g_ina219);
}

bool FcMonitorPoll(FcHardware rev, FcMonitorData *monitors) {
  static bool (*const poll[])(FcHardware rev) = {
    PollAnalog,
    PollIna219,
  };
  static uint32_t index = 0U;

  // Set for output functions.
  g_monitors = monitors;
  g_monitors->revision = rev;

  // Make certain that device_index is always valid.
  index %= ARRAYSIZE(poll);
  if (poll[index](rev)) {
    ++index;      // Process all devices concurrently.
    return true;  // Set true when done polling. Data may not update.
  }
  return false;
}

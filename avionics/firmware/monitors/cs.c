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

#include "avionics/firmware/monitors/cs.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ina219.h"
#include "avionics/firmware/drivers/si7021.h"
#include "avionics/firmware/monitors/analog.h"
#include "avionics/firmware/monitors/analog_types.h"
#include "avionics/firmware/monitors/cs_analog_types.h"
#include "avionics/firmware/monitors/ina219.h"
#include "avionics/firmware/monitors/si7021.h"
#include "avionics/firmware/monitors/cs_types.h"
#include "avionics/firmware/monitors/cs_ina219_types.h"
#include "avionics/firmware/monitors/cs_si7021_types.h"
#include "avionics/firmware/serial/cs_serial_params.h"
#include "common/macros.h"

static CsMonitorData *g_monitors = NULL;
static Ina219 g_ina219[MAX_CS_INA219_DEVICES];
static Si7021 g_si7021[MAX_CS_SI7021_DEVICES];

static void OutputAnalog(const AnalogMonitor *config, float in,
                         uint32_t flags) {
  assert(g_monitors != NULL);

  // Map the analog input channels to a common flags field output. The
  // flags_map[] and error_map[] arrays define a bitmask to set or clear
  // in functions SetStatus and SignalError, respectively. Unspecified
  // entries in the map default to zero, thus calling SetStatus or
  // SignalError does not alter the flags field output.
  const uint16_t flags_map[kNumCsAnalogInputs] = {
    [kCsAnalogInputHiltDetect] = kCsMonitorStatusHiltDetect,
    [kCsAnalogInputSfpAuxModAbs] = kCsMonitorStatusSfpAuxModAbs,
    [kCsAnalogInputSfpModAbs] = kCsMonitorStatusSfpModAbs,
    [kCsAnalogInputRadioSignal1] = kCsMonitorStatusRadioSignal1,
    [kCsAnalogInputRadioSignal2] = kCsMonitorStatusRadioSignal2,
    [kCsAnalogInputRadioSignal3] = kCsMonitorStatusRadioSignal3,
    [kCsAnalogInputRadioStatus] = kCsMonitorStatusRadioStatus,
  };
  const uint16_t error_map[kNumCsAnalogInputs] = {
    [kCsAnalogInputPowerNotGood1v2] = kCsMonitorErrorPowerNotGood1v2,
    [kCsAnalogInputPowerNotGood2v5] = kCsMonitorErrorPowerNotGood2v5,
    [kCsAnalogInputPowerNotGood3v3] = kCsMonitorErrorPowerNotGood3v3,
  };

  if (config->type == kAnalogTypeVoltage) {
    assert(0 <= config->voltage
           && config->voltage < ARRAYSIZE(g_monitors->analog_data));
    g_monitors->analog_populated |= 1U << config->voltage;
    g_monitors->analog_data[config->voltage] = in;
  }
  SetStatus(flags_map[config->input], flags & ANALOG_MONITOR_STATUS_FLAGS,
            &g_monitors->flags);
  SignalError(error_map[config->input], flags & ANALOG_MONITOR_ERROR_FLAGS,
              &g_monitors->flags);
}

static bool PollAnalog(CsHardware rev) {
  return AnalogMonitorPoll(CsAnalogGetConfig(rev), OutputAnalog);
}

static void OutputIna219(int32_t device, const Ina219OutputData *data,
                         uint32_t flags) {
  assert(g_monitors != NULL);
  assert(0 <= device && device < ARRAYSIZE(g_monitors->ina219_data));

  // Map the INA219 voltage monitoring to a common flags field output. The
  // warn_map[] array defines a bitmask to set or clear in SignalWarning.
  // Unspecified entries in the map default to zero, thus calling
  // SignalWarning does not alter the flags field output.
  const uint16_t warn_map[kNumCsIna219Monitors] = {
    [kCsIna219Monitor12v] = kCsMonitorWarning12v,
    [kCsIna219Monitor1v2] = kCsMonitorWarning1v2,
    [kCsIna219Monitor2v5] = kCsMonitorWarning2v5,
    [kCsIna219Monitor3v3] = kCsMonitorWarning3v3,
    [kCsIna219Monitor3v3Vrl] = kCsMonitorWarning3v3Vrl,
  };
  SignalWarning(warn_map[device], flags & INA219_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);

  g_monitors->ina219_populated |= 1U << device;
  g_monitors->ina219_data[device] = *data;
}

static bool PollIna219(CsHardware rev) {
  static uint32_t index = 0U;
  return Ina219MonitorPoll(CsIna219GetConfig(rev), OutputIna219,
                           &index, g_ina219);
}

static void OutputSi7021(int32_t device, const Si7021OutputData *data) {
  assert(g_monitors != NULL);
  assert(0 <= device && device < ARRAYSIZE(g_monitors->si7021_data));

  g_monitors->si7021_populated |= 1U << device;
  g_monitors->si7021_data[device] = *data;
}

static bool PollSi7021(CsHardware rev) {
  static uint32_t index = 0U;
  return Si7021MonitorPoll(CsSi7021GetConfig(rev), OutputSi7021,
                           &index, g_si7021);
}

void CsMonitorInit(void) {
  AnalogMonitorInit();
  Ina219MonitorInit(ARRAYSIZE(g_ina219), g_ina219);
  Si7021MonitorInit(ARRAYSIZE(g_si7021), g_si7021);
}

bool CsMonitorPoll(CsHardware rev, CsMonitorData *monitors) {
  static bool (*const poll[])(CsHardware rev) = {
    PollAnalog,
    PollIna219,
    PollSi7021,
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

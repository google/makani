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

#include "avionics/firmware/monitors/aio.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/ina219.h"
#include "avionics/firmware/drivers/si7021.h"
#include "avionics/firmware/monitors/aio_analog_types.h"
#include "avionics/firmware/monitors/aio_ina219_types.h"
#include "avionics/firmware/monitors/aio_si7021_types.h"
#include "avionics/firmware/monitors/analog.h"
#include "avionics/firmware/monitors/analog_types.h"
#include "avionics/firmware/monitors/ina219.h"
#include "avionics/firmware/monitors/si7021.h"
#include "avionics/firmware/serial/aio_serial_params.h"
#include "common/macros.h"

static AioModuleMonitorData *g_monitors = NULL;
static Ina219 g_ina219[MAX_AIO_INA219_DEVICES];
static Si7021 g_si7021[MAX_AIO_SI7021_DEVICES];

static void OutputAnalog(const AnalogMonitor *config, float in,
                         uint32_t flags) {
  assert(g_monitors != NULL);

  // Map the analog input channels to a common flags field output. The
  // flags_map[] and warn_map[] arrays define a bitmask to set or clear
  // in functions SetStatus and SignalWarning, respectively. Unspecified
  // entries in the map default to zero, thus calling SetStatus or
  // SignalWarning does not alter the flags field output.
  const uint16_t flags_map[kNumAioAnalogInputs] = {
    [kAioAnalogInputWatchdogEnabled] = kAioMonitorStatusWatchdogEnabled,
    [kAioAnalogInputPortDetect0] = kAioMonitorStatusPortDetect0,
    [kAioAnalogInputPortDetect1] = kAioMonitorStatusPortDetect1,
    [kAioAnalogInputPortDetect2] = kAioMonitorStatusPortDetect2,
    [kAioAnalogInputPortDetect3] = kAioMonitorStatusPortDetect3,
    [kAioAnalogInputGtiDetect] = kAioMonitorStatusGtiDetect,
  };
  const uint16_t warn_map[kNumAioAnalogInputs] = {
    [kAioAnalogInput2v5] = kAioMonitorWarning2v5
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
}

static bool PollAnalog(AioHardware rev) {
  return AnalogMonitorPoll(AioAnalogGetConfig(rev), OutputAnalog);
}

static void OutputIna219(int32_t device, const Ina219OutputData *data,
                         uint32_t flags) {
  assert(g_monitors != NULL);
  assert(0 <= device && device < ARRAYSIZE(g_monitors->ina219_data));

  // Map the INA219 voltage monitoring to a common flags field output. The
  // warn_map[] array defines a bitmask to set or clear in SignalWarning.
  // Unspecified entries in the map default to zero, thus calling
  // SignalWarning does not alter the flags field output.
  const uint16_t warn_map[kNumAioIna219Monitors] = {
    [kAioIna219Monitor12v] = kAioMonitorWarning12v,
    [kAioIna219Monitor1v2] = kAioMonitorWarning1v2,
    [kAioIna219Monitor3v3] = kAioMonitorWarning3v3,
  };
  SignalWarning(warn_map[device], flags & INA219_MONITOR_WARNING_FLAGS,
                &g_monitors->flags);

  g_monitors->ina219_populated |= 1U << device;
  g_monitors->ina219_data[device] = *data;
}

static bool PollIna219(AioHardware rev) {
  static uint32_t index = 0U;
  return Ina219MonitorPoll(AioIna219GetConfig(rev), OutputIna219,
                           &index, g_ina219);
}

static void OutputSi7021(int32_t device, const Si7021OutputData *data) {
  assert(g_monitors != NULL);
  assert(0 <= device && device < ARRAYSIZE(g_monitors->si7021_data));

  g_monitors->si7021_populated |= 1U << device;
  g_monitors->si7021_data[device] = *data;
}

static bool PollSi7021(AioHardware rev) {
  static uint32_t index = 0U;
  return Si7021MonitorPoll(AioSi7021GetConfig(rev), OutputSi7021,
                           &index, g_si7021);
}

void AioMonitorInit(void) {
  AnalogMonitorInit();
  Ina219MonitorInit(ARRAYSIZE(g_ina219), g_ina219);
  Si7021MonitorInit(ARRAYSIZE(g_si7021), g_si7021);
}

bool AioMonitorPoll(AioHardware rev, AioModuleMonitorData *monitors) {
  static bool (* const poll[])(AioHardware rev) = {
    PollAnalog,
    PollIna219,
    PollSi7021,
  };
  static uint32_t index = 0U;

  // Set for output functions.
  g_monitors = monitors;
  g_monitors->revision = rev;

  // Make certain that index is always valid.
  index %= ARRAYSIZE(poll);
  if (poll[index](rev)) {
    ++index;      // Process all devices concurrently.
    return true;  // Set true when done polling. Data may not update.
  }
  return false;
}

// TODO: Rework to read carrier board revision from I2C EEPROM when
// available.
bool AioMonitorPollStack(AioHardware aio_hardware_revision,
                         AioModuleMonitorData *aio_monitors,
                         bool (* const carrier_poll_function)(void)) {
  static bool poll_aio = true;
  bool complete;
  if (poll_aio) {
    complete = AioMonitorPoll(aio_hardware_revision, aio_monitors);
  } else {
    complete = carrier_poll_function();
  }
  if (complete) {
    poll_aio = !poll_aio;
  }
  return complete;
}

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

#include "avionics/motor/firmware/mon.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/firmware/drivers/mcp342x.h"
#include "avionics/firmware/drivers/mcp9800.h"
#include "avionics/firmware/monitors/mcp342x.h"
#include "avionics/firmware/monitors/mcp9800.h"
#include "avionics/firmware/monitors/si7021.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/motor/firmware/config_params.h"
#include "avionics/motor/firmware/io.h"
#include "avionics/motor/firmware/thermal.h"
#include "avionics/motor/monitors/monitors.h"
#include "avionics/motor/monitors/motor_mcp342x_types.h"
#include "avionics/motor/monitors/motor_mcp9800_types.h"
#include "avionics/motor/monitors/motor_si7021_types.h"
#include "common/macros.h"

// This file defines the motor hardware monitoring configuration.

static Mcp342x g_mcp342x[MAX_MOTOR_MCP342X_DEVICES];
static Mcp9800 g_mcp9800[MAX_MOTOR_MCP9800_DEVICES];
static Si7021 g_si7021[MAX_MOTOR_SI7021_DEVICES];

// Polling functions.

static bool PollMotorMonitor(void) {
  return MotorMonitorPoll(GetBoardHardwareRevision(),
                          MotorIoGetMotorMonitors());
}

static bool PollMcp342x(void) {
  static uint32_t device_index = 0U;
  static uint32_t config_indices[MAX_MOTOR_MCP342X_DEVICES] = {0U};
  const Mcp342xMonitors *monitors = MotorMcp342xGetConfig(
      (MotorType)kMotorConfigParams->motor_type,
      GetBoardHardwareRevision());
  return Mcp342xMonitorPoll(monitors, MotorThermalUpdateMcp342x, &device_index,
                            config_indices, g_mcp342x);
}

static bool PollMcp9800(void) {
  static uint32_t index = 0U;
  return Mcp9800MonitorPoll(MotorMcp9800GetConfig(GetBoardHardwareRevision()),
                            MotorThermalUpdateMcp9800, &index, g_mcp9800);
}

static void OutputSi7021(int32_t device, const Si7021OutputData *data) {
  MotorMonitorData *monitors = MotorIoGetMotorMonitors();

  assert(monitors != NULL);
  assert(0 <= device && device < ARRAYSIZE(monitors->si7021_data));

  monitors->si7021_populated |= 1U << device;
  monitors->si7021_data[device] = *data;

  MotorThermalUpdateSi7021(device, data);
}

static bool PollSi7021(void) {
  static uint32_t index = 0U;
  return Si7021MonitorPoll(
      MotorSi7021GetConfig(GetBoardHardwareRevision()),
      OutputSi7021, &index, g_si7021);
}

static bool PollHt3000(void) {
  // Only available in Ozone controllers.
  if (GetBoardHardwareRevision() == kMotorHardwareOzoneA1) {
    MotorThermalUpdateHt3000();
  }
  return true;
}

void MotorMonInit(void) {
  MotorMonitorInit();
  Mcp342xMonitorInit(ARRAYSIZE(g_mcp342x), g_mcp342x);
  Mcp9800MonitorInit(ARRAYSIZE(g_mcp9800), g_mcp9800);
  Si7021MonitorInit(ARRAYSIZE(g_si7021), g_si7021);
}

void MotorMonPoll(void) {
  static bool (* const kPollFunction[])(void) = {
    PollMotorMonitor,
    PollMcp342x,
    PollMcp9800,
    PollSi7021,
    PollHt3000
  };
  static uint32_t index = 0U;

  index %= ARRAYSIZE(kPollFunction);
  if (kPollFunction[index]()) {
    ++index;
  }
}

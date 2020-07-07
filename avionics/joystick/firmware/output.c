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

#include "avionics/joystick/firmware/output.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/debounce.h"
#include "avionics/common/faults.h"
#include "avionics/firmware/monitors/joystick.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/network/net_mon.h"
#include "common/macros.h"
#include "system/labels.h"

static JoystickStatusMessage g_status;
static JoystickMonitorStatusMessage g_mon_status;

static bool g_momentary_value = false;
static DebounceState g_momentary_state;

void JoystickOutputInit(void) {
  memset(&g_status, 0, sizeof(g_status));
  memset(&g_mon_status, 0, sizeof(g_mon_status));
  DebounceStateInit(false, &g_momentary_state);
}

AioModuleMonitorData *JoystickOutputGetAioModuleMonitors(void) {
  return &g_mon_status.aio_mon;
}

JoystickMonitorData *JoystickOutputGetJoystickMonitors(void) {
  return &g_mon_status.joystick_mon;
}

GroundIoMonitorData *JoystickOutputGetGroundIoMonitors(void) {
  return &g_mon_status.ground_io_mon;
}

void JoystickOutputPresence(bool present) {
  SignalWarning(kJoystickWarningNotPresent, !present, &g_status.status);
}

void JoystickOutputRoll(float value) {
  g_status.roll = value;
}

void JoystickOutputPitch(float value) {
  g_status.pitch = value;
}

void JoystickOutputYaw(float value) {
  g_status.yaw = value;
}

void JoystickOutputThrottle(float value) {
  g_status.throttle = value;
}

void JoystickOutputTriSwitch(JoystickSwitchPositionLabel value) {
  g_status.tri_switch = (uint8_t)value;
}

void JoystickOutputMomentarySwitch(JoystickSwitchPositionLabel value) {
  g_momentary_value = (value == kJoystickSwitchPositionUp);
}

void JoystickOutputSetTetherReleaseInterlockCode(uint32_t code) {
  g_status.tether_release_interlock_code = code;
}

void JoystickOutputSetScuttleCode(uint32_t code) {
  g_status.scuttle_code = code;
}

void JoystickOutputMicrohardStatus(MicrohardStatus status) {
  g_mon_status.microhard_status = status;
}

void JoystickOutputSendStatusMessage(void) {
  // Ensure that tether release (momentary_switch) is high
  // for 100ms (10 cycles at 10ms) before transitioning to active.
  if (Debounce(g_momentary_value, 10, 0, &g_momentary_state)) {
    g_status.momentary_switch = kJoystickSwitchPositionUp;
  } else {
    g_status.momentary_switch = kJoystickSwitchPositionDown;
  }

  NetSendAioJoystickStatusMessage(&g_status);
}

void JoystickOutputSendMonitorStatusMessage(void) {
  NetSendAioJoystickMonitorStatusMessage(&g_mon_status);
}

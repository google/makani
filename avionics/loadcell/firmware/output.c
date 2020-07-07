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

#include "avionics/loadcell/firmware/output.h"

#include <assert.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/fast_math/fast_math.h"
#include "avionics/common/safety_codes.h"
#include "avionics/loadcell/firmware/loadcell.h"
#include "avionics/firmware/network/net_send.h"

static LoadcellMessage g_status;

void LoadcellOutputInit(void) {
  memset(&g_status, 0, sizeof(g_status));
}

AioModuleMonitorData *LoadcellOutputGetAioModuleMonitors(void) {
  return &g_status.aio_mon;
}

LoadcellMonitorData *LoadcellOutputGetLoadcellMonitors(void) {
  return &g_status.loadcell_mon;
}

BridleJuncData *LoadcellOutputGetBridleJuncData(void) {
  return &g_status.bridle_junc;
}

void LoadcellOutputTetherReleaseState(ActuatorState state) {
  g_status.tether_release_state = state;
}

void LoadcellOutputTetherReleaseFullyArmed(bool armed) {
  g_status.tether_release_fully_armed = armed;
}

// This is latching on success, as any attempt after a successful release will
// appear to be a failure.
void LoadcellOutputTetherReleaseSuccess(bool success) {
  if (success) {
    g_status.tether_released = true;
    g_status.tether_released_safety_code = TETHER_RELEASE_SAFETY_CODE;
  }
}

void LoadcellOutputTetherReleaseSelftest(TetherReleaseSelftestResult result) {
  ClearErrors(&g_status.status);
  switch (result) {
    case kTetherReleaseSelftestPassed:
      break;
    case kTetherReleaseSelftestLowBattery:
      SignalError(kLoadcellErrorLowBattery, true, &g_status.status);
      break;
    case kTetherReleaseSelftestBatteryDisconnected:
      SignalError(kLoadcellErrorBatteryDisconnected, true, &g_status.status);
      break;
    case kTetherReleaseSelftestReleaseCircuitFailedShort:
      SignalError(kLoadcellErrorReleaseCircuitFailedShort, true,
                  &g_status.status);
      break;
    case kTetherReleaseSelftestReleaseCircuitFailedOpen:
      SignalError(kLoadcellErrorReleaseCircuitFailedOpen, true,
                  &g_status.status);
      break;
    case kTetherReleaseSelftestReleaseDisconnected:
      SignalError(kLoadcellErrorReleaseDisconnected, true, &g_status.status);
      break;
    default:
      assert(false);
      break;
  }
}

void LoadcellOutputLoadcellInvalidWarning(bool ch_0_invalid,
                                          bool ch_1_invalid) {
  SignalWarning(kLoadcellWarningCh0Invalid, ch_0_invalid, &g_status.status);
  SignalWarning(kLoadcellWarningCh1Invalid, ch_1_invalid, &g_status.status);
}

static void LoadcellOutputAngleOfAttack(void) {
  // Convert AoA and AoS.
  // TODO: Check positive angle orientation.
  // TODO: Add config param for zero position?
  g_status.angle_alpha = Crossfadef(
      -PI_F, PI_F,
      g_status.loadcell_mon.analog_data[kLoadcellAnalogVoltageVAoa1],
      0.0f, 3.0f);
  g_status.angle_beta = Crossfadef(
      -PI_F, PI_F,
      g_status.loadcell_mon.analog_data[kLoadcellAnalogVoltageVAoa2],
      0.0f, 3.0f);
}

void LoadcellOutputSendStatusMessage(void) {
  g_status.loadcell_data = *GetLoadcellData();
  LoadcellOutputAngleOfAttack();
  NetSendAioLoadcellMessage(&g_status);
}

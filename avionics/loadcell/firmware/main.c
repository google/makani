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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/safety_codes.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/dcan.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/net_diag.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/network/switch_config.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/loadcell/firmware/bridle_junc_can.h"
#include "avionics/loadcell/firmware/input.h"
#include "avionics/loadcell/firmware/loadcell.h"
#include "avionics/loadcell/firmware/mon.h"
#include "avionics/loadcell/firmware/output.h"
#include "avionics/loadcell/firmware/selftest.h"
#include "avionics/loadcell/firmware/tether_release.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_stats.h"

#define LOADCELL_PERIOD_CYCLES CLOCK32_MSEC_TO_CYCLES(LOADCELL_PERIOD_US / 1000)
#define RELEASE_CURRENT_MIN 1.0f  // [A]

static ActuatorState g_actuator_state;
static TetherReleaseCommand g_tether_cmd;
static TetherReleaseCommand g_tether_cmd_latched;
static bool g_interlock_switched;
static bool g_release_firing;
static bool g_release_success;

static void TetherInit(void) {
  g_actuator_state = kActuatorStateInit;
  g_interlock_switched = false;
  memset(&g_tether_cmd, 0, sizeof(g_tether_cmd));
  memset(&g_tether_cmd_latched, 0, sizeof(g_tether_cmd_latched));
  TetherReleaseInit();
}

static void TetherPoll(void) {
  const float *monitors = LoadcellOutputGetLoadcellMonitors()->analog_data;
  if (g_actuator_state == kActuatorStateInit) {
    TetherReleaseSelftestResult result = TetherReleaseSelftest(
        monitors[kLoadcellAnalogVoltageVBattTest],
        monitors[kLoadcellAnalogVoltageVArm],
        monitors[kLoadcellAnalogVoltageVRelease]);

    if (result != kTetherReleaseSelftestRunning) {
      g_actuator_state = kActuatorStateReady;
      LoadcellOutputTetherReleaseSelftest(result);
    }
  } else {
    // Check for state command updates.
    const TetherReleaseSetStateMessage *set_state_msg =
        LoadcellInputQueryTetherReleaseSetState();
    if (set_state_msg != NULL) {
      switch (set_state_msg->state_command) {
        case kActuatorStateCommandArm:
          if (g_actuator_state != kActuatorStateRunning) {
            g_actuator_state = kActuatorStateArmed;
          }
          break;
        case kActuatorStateCommandDisarm:
          g_actuator_state = kActuatorStateReady;
          break;
        default:
          break;
      }
    }
  }

  // Check interlock and fire commands.
  const TetherJoystick *joystick_status_msg =
      LoadcellInputQueryTetherJoystick();
  if (joystick_status_msg) {
    g_interlock_switched = (joystick_status_msg->tether_release_interlock_code
                            == TETHER_RELEASE_INTERLOCK_SAFETY_CODE);
  }
  LoadcellInputQueryTetherReleaseCommand(&g_tether_cmd);
  if (g_tether_cmd.fire_tether_release) {
    g_tether_cmd_latched = g_tether_cmd;
  }

  // Set arming relay and send fire signal if appropriate.
  if (g_actuator_state == kActuatorStateArmed
      || g_actuator_state == kActuatorStateRunning) {
    if (g_interlock_switched) {
      TetherReleaseArm();
      LoadcellOutputTetherReleaseSelftest(TetherReleaseArmStatusCheck(
          monitors[kLoadcellAnalogVoltageVArm],
          monitors[kLoadcellAnalogVoltageVRelease]));
    } else {
      TetherReleaseDisarm();
    }
    LoadcellOutputTetherReleaseFullyArmed(g_interlock_switched);

    if (g_tether_cmd_latched.fire_tether_release) {
      if (g_actuator_state != kActuatorStateRunning) {
        g_actuator_state = kActuatorStateRunning;
        g_release_firing = false;
        g_release_success = false;
      }
      // Verify that the battery current exceeds RELEASE_CURRENT_MIN and then
      // drops below it.  This indicates that the release was fired and then
      // disconnected when the release mechanism came apart.
      if (monitors[kLoadcellAnalogVoltageIBatt] > RELEASE_CURRENT_MIN) {
        g_release_firing = true;
        g_release_success = false;
      } else if (g_release_firing) {
        g_release_success = true;
      }
      LoadcellOutputTetherReleaseSuccess(g_release_success);
      if (TetherReleaseFire(g_tether_cmd_latched.safety_code)) {
        g_tether_cmd_latched.fire_tether_release = false;
        g_tether_cmd_latched.safety_code = 0;
        g_actuator_state = kActuatorStateArmed;
      }
    }
  } else {
    TetherReleaseDisarm();
    LoadcellOutputTetherReleaseFullyArmed(false);
  }

  LoadcellOutputTetherReleaseState(g_actuator_state);
}

int main(void) {
  ExtWatchdogInit();
  IoInit();
  MibSPIInit(1, kSpiPinmuxAll);
  SwitchConfigInit();
  Bcm53101Init(true);
  NetInit(AppConfigGetAioNode());
  // Perform self test as soon as possible and immediately after NetInit()
  // such that we validate parameters before calling dependent code.
  SelfTest();
  NetMonInit(GetSwitchConfig()->info);
  NetDiagInit(GetSwitchConfig()->info);
  I2cInit(400e3);
  LoadcellOutputInit();
  LoadcellMonInit();
  MibSPIInit(3, kSpiPinmuxAll);
  LoadcellInit();
  TetherInit();
  BridleJuncCanInit();

  // Slow status.
  OutputInitSlowStatusMessage();

  VimEnableIrq();

  uint32_t now = Clock32GetCycles();
  uint32_t loadcell_wakeup = now;
  uint32_t slow_status_wakeup = now + LOADCELL_PERIOD_CYCLES / 2;
  while (true) {
    now = Clock32GetCycles();
    if (CLOCK32_GE(now, loadcell_wakeup)) {
      loadcell_wakeup += LOADCELL_PERIOD_CYCLES;
      LoadcellOutputSendStatusMessage();
    } else if (CLOCK32_GE(now, slow_status_wakeup)) {
      slow_status_wakeup += SLOW_STATUS_PERIOD_CYCLES;
      OutputSendSlowStatusMessage(ClockGetUs());
    }
    NetPoll();
    NetMonPoll();
    NetDiagPoll();
    Bcm53101Poll(GetSwitchConfig());
    ExtWatchdogPoll();
    LoadcellMonPoll();
    LoadcellPoll();
    TetherPoll();
    BridleJuncCanPoll();
  }
  return 0;
}

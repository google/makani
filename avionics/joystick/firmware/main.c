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

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avionics/common/build_info.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/drivers/microhard.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/monitors/aio.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/net_diag.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/network/switch_config.h"
#include "avionics/firmware/params/param_server.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/joystick/firmware/config_params.h"
#include "avionics/joystick/firmware/monitor.h"
#include "avionics/joystick/firmware/output.h"
#include "avionics/joystick/firmware/ppm.h"
#include "avionics/joystick/firmware/selftest.h"
#include "avionics/joystick/firmware/switches.h"
#include "common/macros.h"

#define STATUS_PERIOD_CYCLES      CLOCK32_MSEC_TO_CYCLES(10)

static MicrohardData g_microhard_data;
static const Microhard kMicrohard = {
  .data = &g_microhard_data,
  .sci = &kSci2Interrupt,
  .baud_rate = 115200,
  .set_status = JoystickOutputMicrohardStatus,
};

int main(void) {
  ExtWatchdogInit();
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
  JoystickMonInit();
  PpmInit();
  SwitchesInit();

  // Initialize outputs.
  JoystickOutputInit();
  OutputInitSlowStatusMessage();

  MicrohardInit(&kMicrohard, &kJoystickConfigParams->microhard_params);

  PpmRun();

  // Enable interrupts for SCI driver.
  VimEnableIrq();

  uint32_t now = Clock32GetCycles();
  uint32_t wakeup_status = now + STATUS_PERIOD_CYCLES / 2U;
  uint32_t wakeup_mon_status = now + STATUS_PERIOD_CYCLES;
  uint32_t wakeup_slow_status = now + SLOW_STATUS_PERIOD_CYCLES
      + STATUS_PERIOD_CYCLES * 3U / 4U;
  while (true) {
    now = Clock32GetCycles();
    if (CLOCK32_GE(now, wakeup_status)) {
      wakeup_status += STATUS_PERIOD_CYCLES;
      JoystickOutputSendStatusMessage();
    } else if (CLOCK32_GE(now, wakeup_mon_status)) {
      wakeup_mon_status += STATUS_PERIOD_CYCLES;
      SwitchesPoll();
      JoystickOutputSendMonitorStatusMessage();
    } else if (CLOCK32_GE(now, wakeup_slow_status)) {
      wakeup_slow_status += SLOW_STATUS_PERIOD_CYCLES;
      OutputSendSlowStatusMessage(ClockGetUs());
    } else {
      NetPoll();
      NetMonPoll();
      Bcm53101Poll(GetSwitchConfig());
      NetDiagPoll();
      ExtWatchdogPoll();
      JoystickMonPoll();
      ParamServerPoll(kParamServerCalib
                      | kParamServerConfig
                      | kParamServerSerial
                      | kParamServerCarrierSerial);

      JoystickCommandMessage command;
      if (CvtGetJoystickCommandMessage(kAioNodeControllerA, &command,
                                       NULL, NULL)) {
        PpmEnableRawStatus(command.enable_raw);
      }

      MicrohardPoll(&kMicrohard);
      PpmPoll();
    }
  }

  return 0;
}

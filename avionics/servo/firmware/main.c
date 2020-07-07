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
#include <stdint.h>
#include <stdio.h>

#include "avionics/common/network_config.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/dcan.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/drivers/led.h"
#include "avionics/firmware/drivers/log.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/net_diag.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/switch_config.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/network/aio_node.h"
#include "avionics/servo/firmware/def.h"
#include "avionics/servo/firmware/mon.h"
#include "avionics/servo/firmware/output.h"
#include "avionics/servo/firmware/r22_can.h"
#include "avionics/servo/firmware/selftest.h"
#include "avionics/servo/firmware/state.h"

int main(void) {
  ExtWatchdogInit();
  LedInit();
  MibSPIInit(1, kSpiPinmuxAll);
  SwitchConfigInit();
  Bcm53101Init(true);
  NetInit(AppConfigGetAioNode());
  I2cInit(400000);
  // Perform self test as soon as possible and immediately after NetInit()
  // such that we validate parameters before calling dependent code.
  SelfTest();
  NetMonInit(GetSwitchConfig()->info);
  NetDiagInit(GetSwitchConfig()->info);
  DcanInit(kDcanBus1, kDcanBitRate1000kbps);  // DCAN1 = Diagnostic CAN bus.
  ServoStateInit();

  int64_t now = ClockGetUs();

  ServoInputInit(now);
  ServoOutputInit();
  OutputInitSlowStatusMessage();
  ServoMonInit();

  LogInit();
  VimEnableIrq();

  int64_t slow_status_wakeup = now + SLOW_STATUS_PERIOD_US;
  while (true) {
    now = ClockGetUs();
    if (now >= slow_status_wakeup) {
      slow_status_wakeup += SLOW_STATUS_PERIOD_US;
      OutputSendSlowStatusMessage(now);
    } else {
      R22CanPoll();
      Bcm53101Poll(GetSwitchConfig());
      ServoStatePoll(now);
      ServoMonPoll();
      NetMonPoll();
      NetDiagPoll();
      ExtWatchdogPoll();
    }
    NetPoll();
#ifndef NDEBUG
    LogToStdout();
#endif  // NDEBUG
  }
  return 0;
}

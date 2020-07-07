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

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/drivers/q7_watchdog.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/net_diag.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/switch_config.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/network/aio_node.h"
#include "avionics/recorder/firmware/monitor.h"
#include "avionics/recorder/firmware/output.h"
#include "avionics/recorder/firmware/selftest.h"
#include "common/macros.h"

// Status message frequency: 10ms.
#define STATUS_MESSAGE_US 10000

int main(void) {
  // Initialize TMS570 hardware.
  MibSPIInit(1, kSpiPinmuxAll);  // For access switch.
  I2cInit(400000);  // For voltage and temperature monitoring.

  // Initialize watchdog.
  ExtWatchdogInit();
  Q7WatchdogRecorderInit();

  // Initialize network.
  SwitchConfigInit();
  Bcm53101Init(true);
  NetInit(AppConfigGetAioNode());
  // Perform self test as soon as possible and immediately after NetInit()
  // such that we validate parameters before calling dependent code.
  SelfTest();
  NetMonInit(GetSwitchConfig()->info);
  NetDiagInit(GetSwitchConfig()->info);

  // Initialize hardware monitors.
  RecorderMonInit();

  // Initialize recorder output.
  RecorderOutputInit();
  OutputInitSlowStatusMessage();

  int64_t now = ClockGetUs();
  int64_t status_timeout = now + STATUS_MESSAGE_US;
  int64_t slow_status_timeout = now + SLOW_STATUS_PERIOD_US;

  while (true) {
    now = ClockGetUs();
    if (status_timeout <= now) {
      status_timeout += STATUS_MESSAGE_US;
      RecorderOutputSendStatusMessage();
    } else if (slow_status_timeout <= now) {
      slow_status_timeout += SLOW_STATUS_PERIOD_US;
      OutputSendSlowStatusMessage(now);
    } else {
      RecorderMonPoll();
      NetMonPoll();
      NetDiagPoll();
      NetPoll();
      Bcm53101Poll(GetSwitchConfig());
      Q7WatchdogRecorderPoll();
      ExtWatchdogPoll();
    }
  }
  return 0;
}

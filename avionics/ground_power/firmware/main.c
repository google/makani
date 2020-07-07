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
#include <stdint.h>

#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/network/net_diag.h"
#include "avionics/firmware/network/net_mon.h"
#include "avionics/firmware/network/switch_config.h"
#include "avionics/firmware/output/slow_status.h"
#include "avionics/ground_power/firmware/selftest.h"

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

  // Initialize outputs.
  OutputInitSlowStatusMessage();

  uint32_t now = Clock32GetCycles();
  uint32_t wakeup_slow_status = now + SLOW_STATUS_PERIOD_CYCLES;
  while (true) {
    now = Clock32GetCycles();
    if (CLOCK32_GE(now, wakeup_slow_status)) {
      wakeup_slow_status += SLOW_STATUS_PERIOD_CYCLES;
      OutputSendSlowStatusMessage(ClockGetUs());
    } else {
      NetPoll();
      NetMonPoll();
      Bcm53101Poll(GetSwitchConfig());
      NetDiagPoll();
      ExtWatchdogPoll();
    }
  }
  return 0;
}

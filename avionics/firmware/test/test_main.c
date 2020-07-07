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

#include "avionics/firmware/test/test_main.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/rti.h"
#include "avionics/firmware/cpu/vim.h"
#include "avionics/firmware/drivers/bcm_unified.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/test/poll.h"
#include "avionics/firmware/test/test.h"
#include "avionics/network/switch_types.h"

// Increments for each watchdog kick. Used to align test execution to watchdog.
static volatile uint32_t g_watchdog_count = 0U;

static void KickWatchdogInterrupt(void) {
  ExtWatchdogKick();
  ++g_watchdog_count;
}

int TestMain(int32_t num_suites, const TestSuite **suites) {
  ExtWatchdogInit();
  TestInit();
  MibSPIInit(1, kSpiPinmuxAll);
  BcmUnifiedInit(true);
  NetInit(AppConfigGetAioNode());

  // Use RTI module to kick watchdog in an interrupt function.
  float period_usec_f = CLOCK32_CYCLES_TO_USEC_F(EXT_WATCHDOG_KICK_CYCLES);
  uint32_t period_usec = (uint32_t)period_usec_f;
  VimRegisterFiq(kVimChannelRtiCompare0, KickWatchdogInterrupt);
  VimEnableInterrupt(kVimChannelRtiCompare0);
  RtiEnablePeriodicInterrupt0(RtiGetPeriodicTimer() + period_usec, period_usec);
  RtiEnablePeriodicTimer();

  // Enable interrupts last.
  VimEnableFiq();

  // Idle loop.
  uint32_t next_count = 0U;
  while (true) {
    // Align test execution to watchdog interval.
    if ((int32_t)(next_count - g_watchdog_count) <= 0) {
      TestPoll(num_suites, suites);
      next_count = g_watchdog_count + 1U;
    } else {
      NetPoll();
      BcmUnifiedPoll(NULL);
    }
  }
  return 0;
}

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

#include "avionics/firmware/drivers/ext_watchdog.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/registers.h"

static uint32_t g_kick_cycles = 0;
static bool g_kick_high = false;

void ExtWatchdogInit(void) {
  SYS.PC1.ECPCLKFUN = 0;  // 0 = GIO mode.
  SYS.PC2.ECPCLKDIR = 1;  // 1 = Output.

  // Here, we assume that the bootloader calls this function immediately
  // upon initialization and EXT_WATCHDOG_KICK_CYCLES plus the startup time
  // is less than the maximum watchdog kick interval. We also assume that
  // the bootloader kicks the watchdog immediately before launching the
  // application. We set g_kick_cycles such that ExtWatchdogPoll() will
  // not kick the watchdog for at least EXT_WATCHDOG_KICK_CYCLES after
  // initialization.
  g_kick_cycles = Clock32GetCycles();
  g_kick_high = SYS.PC4.ECPCLKDOUT;  // Set to current state.
}

void ExtWatchdogKick(void) {
  g_kick_cycles = Clock32GetCycles();
  g_kick_high = !g_kick_high;
  SYS.PC4.ECPCLKDOUT = g_kick_high;
}

bool ExtWatchdogPoll(void) {
  // Do not kick watchdog on interval [0, EXT_WATCHDOG_KICK_CYCLES).
  int32_t no_kick_cycles = CLOCK32_SUBTRACT(Clock32GetCycles(), g_kick_cycles);
  if (!(0 <= no_kick_cycles && no_kick_cycles < EXT_WATCHDOG_KICK_CYCLES)) {
    ExtWatchdogKick();
    return !g_kick_high;  // Kicked on falling edge.
  }
  return false;
}

void ExtWatchdogWaitCycles(uint32_t cycles) {
  uint32_t now = Clock32GetCycles();
  uint32_t timeout = now + cycles;
  while (CLOCK32_GT(timeout, now)) {
    ExtWatchdogPoll();
    now = Clock32GetCycles();
  }
}

void ExtWatchdogWaitUsec(uint32_t usec) {
  ExtWatchdogWaitCycles(CLOCK32_USEC_TO_CYCLES(usec));
}

void ExtWatchdogWaitMsec(uint32_t msec) {
  ExtWatchdogWaitCycles(CLOCK32_MSEC_TO_CYCLES(msec));
}

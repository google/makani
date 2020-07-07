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

#include "avionics/firmware/drivers/q7_watchdog.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/n2het.h"
#include "avionics/firmware/cpu/registers.h"

// Specify the Q7 watchdog timeout. This timeout specifies the amount of time
// the TMS570 will wait for the Q7 to toggle the watchdog kick pin. After the
// Q7 toggles the watchdog kick pin, the TMS570 resets the timer. If the Q7
// fails to toggle the watchdog pin and the timeout elapses, the TMS570 will
// reset the Q7. Here, we assume that the TMS570 polls the watchdog kick pin
// at least twice as fast as the Q7 toggles the pin.
#define WATCHDOG_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(1000)

// Specify the amount of time to hold the Q7 in reset.
#define Q7_RESET_CYCLES CLOCK32_MSEC_TO_CYCLES(100)

#define FC_N2HET_KICK kN2het1Pin22
#define FC_N2HET_RESET kN2het1Pin18

#define RECORDER_SPI3_KICK (1U << 4)
#define RECORDER_SPI1_RESET (1U << 4)

static bool g_enabled = false;
static bool g_kick_state = false;
static bool g_in_reset = false;
static uint32_t g_timeout_cycles = 0U;

static void Q7WatchdogInit(void) {
  g_enabled = false;
  g_kick_state = false;
  g_in_reset = false;
  g_timeout_cycles = 0U;
}

static void Q7WatchdogPoll(bool (* const get_kick)(void),
                           void (* const assert_reset)(bool reset)) {
  if (get_kick() != g_kick_state) {
    g_enabled = true;
    g_kick_state = !g_kick_state;
    g_timeout_cycles = Clock32GetCycles() + WATCHDOG_TIMEOUT_CYCLES;
  } else if (g_enabled) {
    if (g_in_reset && CLOCK32_GE(Clock32GetCycles(), g_timeout_cycles)) {
      // Release Q7 reset button. Disable watchdog until Q7 reboots and begins
      // kicking the watchdog again.
      assert_reset(false);
      g_in_reset = false;
      g_enabled = false;
    } else if (CLOCK32_GE(Clock32GetCycles(), g_timeout_cycles)) {
      // Assert Q7 reset button.
      assert_reset(true);
      g_in_reset = true;
      g_timeout_cycles = Clock32GetCycles() + Q7_RESET_CYCLES;
    }
  }
}

static bool FcGetKickState(void) {
  return N2hetGetValue(FC_N2HET_KICK);
}

static void FcAssertReset(bool reset) {
  N2hetSetValue(FC_N2HET_RESET, !reset);  // Reset when low.
}

static void FcInit(void) {
  // Configure Q7_SIG0 as input.
  N2hetConfigureAsInputPullDown(FC_N2HET_KICK);
  // Configure Q7_nRSTBTN as open drain.
  N2hetConfigureAsOutputOpenDrain(FC_N2HET_RESET, true);  // High impedance.
  // Disable watchdog by default.
  g_kick_state = FcGetKickState();
}

static bool RecorderGetKickState(void) {
  return (SPI(3).PC2.SCSDIN & RECORDER_SPI3_KICK) != 0x0;
}

static void RecorderAssertReset(bool reset) {
  if (reset) {
    SPI(1).PC5.raw = RECORDER_SPI1_RESET;  // Low.
  } else {
    SPI(1).PC4.raw = RECORDER_SPI1_RESET;  // High impedance.
  }
}

static void RecorderInit(void) {
  // Configure Q7_SIG0 as input.
  SPI(3).GCR0.NRESET = 1;
  SPI(3).PC8.SCSPSEL &= ~RECORDER_SPI3_KICK;  // Configure as pull-down.
  SPI(3).PC7.SCSPDIS &= ~RECORDER_SPI3_KICK;  // Enable internal pull.
  SPI(3).PC1.SCSDIR &= ~RECORDER_SPI3_KICK;   // Configure as input.
  SPI(3).PC0.SCSFUN &= ~RECORDER_SPI3_KICK;   // Configure as GPIO.

  // Configure Q7_nRSTBTN as open drain.
  SPI(1).GCR0.NRESET = 1;
  RecorderAssertReset(false);
  SPI(1).PC6.SCSPDR |= RECORDER_SPI1_RESET;   // Configure as open drain.
  SPI(1).PC1.SCSDIR |= RECORDER_SPI1_RESET;   // Configure as output.
  SPI(1).PC0.SCSFUN &= ~RECORDER_SPI1_RESET;  // Configure as GPIO.

  // Disable watchdog by default.
  g_kick_state = RecorderGetKickState();
}

void Q7WatchdogFcInit(void) {
  Q7WatchdogInit();
  FcInit();
}

void Q7WatchdogFcPoll(void) {
  Q7WatchdogPoll(FcGetKickState, FcAssertReset);
}

void Q7WatchdogRecorderInit(void) {
  Q7WatchdogInit();
  RecorderInit();
}

void Q7WatchdogRecorderPoll(void) {
  Q7WatchdogPoll(RecorderGetKickState, RecorderAssertReset);
}

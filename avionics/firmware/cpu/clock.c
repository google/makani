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

#include "avionics/firmware/cpu/clock.h"

#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/identity/board_hardware.h"
#include "avionics/firmware/identity/identity_types.h"
#include "avionics/firmware/serial/aio_serial_params.h"
#include "avionics/firmware/serial/board_serial.h"
#include "avionics/firmware/serial/cs_serial_params.h"
#include "avionics/firmware/serial/motor_serial_params.h"
#include "avionics/firmware/startup/clock_tms570.h"
#include "common/macros.h"

#define CPU_CYCLES_PER_US (CLOCK_GCLK_KHZ / 1000)

// See Cortex R4/R4F TRM "c9, Performance Monitor Control Register".
#define PMCR_E (1 << 0)  // Enable cycle counter.
#define PMCR_C (1 << 2)  // Cycle counter reset.
#define PMCR_D (1 << 3)  // Cycle counter divide by 64.

COMPILE_ASSERT(PMCR_CYCLE_DIV == 1 || PMCR_CYCLE_DIV == 64,
               PMCR_CYCLE_DIV_must_be_1_or_64);
COMPILE_ASSERT(CLOCK32_KHZ * PMCR_CYCLE_DIV == CLOCK_GCLK_KHZ,
               CLOCK_GCLK_KHZ_must_divide_evenly_by_PMCR_CYCLE_DIV);

static int64_t g_cycles = 0;
static int64_t g_us = 0;

static bool Has8MhzClock(void) {
  return
      (GetBoardHardwareType() == kHardwareTypeAio &&
       GetBoardHardwareRevision() == kAioHardwareRevAd) ||
      (GetBoardHardwareType() == kHardwareTypeCs &&
       GetBoardHardwareRevision() == kCsHardwareRevAdClk8) ||
      (GetBoardHardwareType() == kHardwareTypeMotor &&
       GetBoardHardwareRevision() == kMotorHardwareGinA4Clk8);
}

static void EnableCycleCounter(void) {
  // Global counter enable.
  // [ARM Cortex-R4 TRM 6.3.1]
  uint32_t pmcr;
  __asm__ __volatile__("mrc p15, 0, %0, c9, c12, 0" : "=r"(pmcr));
  pmcr |= PMCR_E | PMCR_C;
  if (PMCR_CYCLE_DIV == 64) {
    pmcr |= PMCR_D;
  } else {
    pmcr &= ~PMCR_D;
  }
  __asm__ __volatile__("mcr p15, 0, %0, c9, c12, 0" : : "r"(pmcr));

  // Cycle counter enable.
  // [ARM Cortex-R4 TRM 6.3.2]
  uint32_t pmcntenset = 1 << 31;
  __asm__ __volatile__("mcr p15, 0, %0, c9, c12, 1" : : "r"(pmcntenset));
}

static void Sample(void) {
  static uint32_t pmccntr_prev = 0U;
  static uint32_t us_remainder = 0U;

  // Sample cycle counter.
  // [ARM Cortex-R4 TRM 6.3.7]
  uint32_t pmccntr = Clock32GetCycles();
  uint32_t delta = (pmccntr - pmccntr_prev) * PMCR_CYCLE_DIV;
  pmccntr_prev = pmccntr;

  // Accumulate cycles.
  g_cycles += delta;

  // Accumulate microseconds.
  g_us += (us_remainder + delta) / CPU_CYCLES_PER_US;
  us_remainder = (us_remainder + delta) % CPU_CYCLES_PER_US;
}

void StartupClockInit(void) {
  if (Has8MhzClock()) {
    // Configure PLL1 for 160 MHz given f_OSCIN = 8 MHz.
    // f_INTCLK = f_OSCIN / NR = 2.667 MHz.
    // f_VCOCLK = f_INTCLK * NF = 320 MHz (target 350 MHz).
    // f_post_ODCLK = f_VCOCLK / OD = 160 MHz.
    // f_PLLCLK = f_post_ODCLK / R = 160 MHz.
    StartupClockSelectOscIn();
    StartupClockSetPll1NrNf(3, 120);
    StartupClockSetPll1Od(2);
    StartupClockSetPll1R(1);
    StartupClockSelectPll1();

    // Configure PLL2 for 100 MHz (EMAC requires multiple of 25 MHz).
    // f_INTCLK2 = f_OSCIN / NR2 = 4.000 MHz.
    // f_VCOCLK2 = f_INTCLK2 * NF2 = 300 MHz (target 350 MHz).
    // f_post_ODCLK2 = f_VCOCLK2 / OD2 = 100 MHz.
    // f_PLLCLK2 = f_post_ODCLK2 / R2 = 100 MHz.
    StartupClockSetPll2NrNf(2, 75);
    StartupClockSetPll2Od(3);
    StartupClockSetPll2R(1);

    // Configure RTI clock for 2 MHz given f_OSCIN = 8 MHz.
    // f_RTI = f_OSCIN / (1 << RTI1DIV) = 2 MHz.
    StartupClockSetRtidiv(2);
  }
  EnableCycleCounter();
  assert(ClockDomainGetFreq(kClockDomainGclk) == CLOCK_GCLK_KHZ * 1000);
}

int32_t ClockGetOscInFreq(void) {
  if (Has8MhzClock()) {
    return 8 * 1000 * 1000;  // 8 MHz.
  } else {
    return 16 * 1000 * 1000;  // 16 MHz.
  }
}

int32_t ClockGetPll1Freq(void) {
  int32_t freq = ClockGetOscInFreq();
  int32_t nr = SYS.PLLCTL1.REFCLKDIV + 1;
  int32_t nf = SYS.PLLCTL1.PLLMUL / 256 + 1;
  int32_t od = SYS.PLLCTL2.ODPLL + 1;
  int32_t r = SYS.PLLCTL1.PLLDIV + 1;

  return (freq * nf) / (nr * od * r);
}

int32_t ClockGetPll2Freq(void) {
  int32_t freq = ClockGetOscInFreq();
  int32_t nr = SYS2.PLLCTL3.REFCLKDIV2 + 1;
  int32_t nf = SYS2.PLLCTL3.PLLMUL2 / 256 + 1;
  int32_t od = SYS2.PLLCTL3.ODPLL2 + 1;
  int32_t r = SYS2.PLLCTL3.PLLDIV2 + 1;

  return (freq * nf) / (nr * od * r);
}

int32_t ClockGetRticlkFreq(void) {
  int32_t rti1div = 1 << SYS.RCLKSRC.RTI1DIV;
  if (SYS.RCLKSRC.RTI1SRC <= 7) {
    return ClockSourceGetFreq(SYS.RCLKSRC.RTI1SRC) / rti1div;
  } else {
    return ClockDomainGetFreq(kClockDomainVclk) / rti1div;
  }
}

int32_t ClockGetCycleCounterFreq(void) {
  uint32_t pmcr;
  __asm__ __volatile__("mrc p15, 0, %0, c9, c12, 0" : "=r"(pmcr));
  if (pmcr & PMCR_D) {
    return ClockDomainGetFreq(kClockDomainGclk) / 64;
  } else {
    return ClockDomainGetFreq(kClockDomainGclk);
  }
}

int32_t ClockSourceGetFreq(ClockSource source) {
  switch (source) {
    case kClockSourceOscIn:
      return ClockGetOscInFreq();
    case kClockSourcePll1:
      return ClockGetPll1Freq();
    case kClockSourceExtClkIn:
      return 0;
    case kClockSourceLfLpo:
      return 85 * 1000;  // Varies significantly.
    case kClockSourceHfLpo:
      return 9600 * 1000;  // Varies significantly.
    case kClockSourceExtClkIn2:
      return 0;
    default:
      return 0;
  }
}

int32_t ClockDomainGetFreq(ClockDomain domain) {
  switch (domain) {
    case kClockDomainHclk:
      return CLOCK_HCLK_KHZ * 1000;
    case kClockDomainGclk:
      return CLOCK_GCLK_KHZ * 1000;
    case kClockDomainVclk:
      return CLOCK_VCLK_KHZ * 1000;
    case kClockDomainVclk2:
      return CLOCK_VCLK2_KHZ * 1000;
    case kClockDomainVclk3:
      return CLOCK_VCLK3_KHZ * 1000;
    case kClockDomainVclk4:
      return CLOCK_VCLK3_KHZ * 1000;  // Assumed equivalent to f_vclk3.
    case kClockDomainVclka1:
      return CLOCK_VCLKA1_KHZ * 1000;
    case kClockDomainVclka2:
      return CLOCK_VCLKA2_KHZ * 1000;
    case kClockDomainVclka3divr:
      return CLOCK_VCLKA3_DIVR_KHZ * 1000;
    case kClockDomainVclka4divr:
      return CLOCK_VCLKA4_DIVR_KHZ * 1000;
    case kClockDomainRticlk:
      return ClockGetRticlkFreq();
    default:
      assert(false);
      return 0;
  }
}

int64_t ClockGetUs(void) {
  Sample();
  return g_us;
}

void Clock32WaitCycles(int32_t cycles) {
  int32_t wakeup = Clock32GetCycles() + cycles;
  while (CLOCK32_LT(Clock32GetCycles(), wakeup)) {}
}

int32_t SaturateLatency(int64_t latency) {
  assert(latency >= 0);
  return (latency > INT32_MAX || latency < 0) ? INT32_MAX : (int32_t)latency;
}

bool PollPeriodicCycles(int32_t period, uint32_t *next) {
  uint32_t now = Clock32GetCycles();
  if (CLOCK32_LT(*next, now - period)) {
    *next = now;
  }
  if (CLOCK32_GE(now, *next)) {
    *next += period;
    if (CLOCK32_LT(*next, now)) {
      *next = now + period;
    }
    return true;
  }
  return false;
}

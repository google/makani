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

// System clock based on the ARM Performance Monitoring Unit (PMU) cycle
// counter, a 32-bit hardware counter that increments every CPU cycle. System
// time is stored as 64-bit software counters and maintained by accumulating the
// deltas between samples of the cycle counter. The cycle counter periodically
// overflows and must be sampled at least once every 26.84 seconds on the
// TMS570LS1227 to ensure correct timekeeping. Calling any of these functions
// samples the cycle counter.

#ifndef AVIONICS_FIRMWARE_CPU_CLOCK_H_
#define AVIONICS_FIRMWARE_CPU_CLOCK_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/startup/clock_tms570_config.h"

// Performance Monitor Control Register (PMCR) divide configuration. We can
// specify a value of 1 or 64. A value of 1 requires all time comparisons
// to be within 2^31 / 160e6 = 13.422 seconds, assuming a 160 MHz clock
// frequency. A value of 64 requires all time comparisons to be within
// 2^31 * 64 / 160e6 = 858.99 seconds.
#define PMCR_CYCLE_DIV 64
#define CLOCK32_KHZ   (CLOCK_GCLK_KHZ / PMCR_CYCLE_DIV)
#define CLOCK32_HZ    (CLOCK32_KHZ * 1000)
#define CLOCK32_KHZ_F ((float)CLOCK32_KHZ)
#define CLOCK32_MHZ_F (CLOCK32_KHZ_F / 1000.0f)

// Helper macros for handling unsigned 32-bit timestamps.
#define CLOCK32_SUBTRACT(a, b) ((int32_t)((uint32_t)(a) - (uint32_t)(b)))
#define CLOCK32_EQ(a, b) (CLOCK32_SUBTRACT(a, b) == 0)
#define CLOCK32_NE(a, b) (CLOCK32_SUBTRACT(a, b) != 0)
#define CLOCK32_LE(a, b) (CLOCK32_SUBTRACT(a, b) <= 0)
#define CLOCK32_LT(a, b) (CLOCK32_SUBTRACT(a, b) < 0)
#define CLOCK32_GE(a, b) (CLOCK32_SUBTRACT(a, b) >= 0)
#define CLOCK32_GT(a, b) (CLOCK32_SUBTRACT(a, b) > 0)

// Helper macros to convert to/from cycles.
#define CLOCK32_USEC_TO_CYCLES(usec) (((usec) * CLOCK32_KHZ + 500) / 1000)
#define CLOCK32_MSEC_TO_CYCLES(msec) ((msec) * CLOCK32_KHZ)
#define CLOCK32_SEC_TO_CYCLES(msec) CLOCK32_MSEC_TO_CYCLES((msec) * 1000)
#define CLOCK32_CYCLES_TO_MSEC_F(cycles) ((cycles) / CLOCK32_KHZ_F)
#define CLOCK32_CYCLES_TO_USEC_F(cycles) ((cycles) / CLOCK32_MHZ_F)

// See TMS570LS1227 Table 6-8 "Available Clock Sources".
typedef enum {
  kClockSourceOscIn     = 0,
  kClockSourcePll1      = 1,
  kClockSourceExtClkIn  = 3,
  kClockSourceLfLpo     = 4,
  kClockSourceHfLpo     = 5,
  kClockSourcePll2      = 6,
  kClockSourceExtClkIn2 = 7
} ClockSource;

// See TMS570LS1227 Section 6.6.2 "Clock Domains".
typedef enum {
  kClockDomainHclk,
  kClockDomainGclk,
  kClockDomainVclk,
  kClockDomainVclk2,
  kClockDomainVclk3,
  kClockDomainVclk4,
  kClockDomainVclka1,
  kClockDomainVclka2,
  kClockDomainVclka3divr,
  kClockDomainVclka4divr,
  kClockDomainRticlk
} ClockDomain;

// Starts the PMU cycle counter. Called from startup code.
void StartupClockInit(void);

// Get clock frequencies.
int32_t ClockGetOscInFreq(void);
int32_t ClockGetPll1Freq(void);
int32_t ClockGetPll2Freq(void);
int32_t ClockGetRticlkFreq(void);
int32_t ClockGetCycleCounterFreq(void);
int32_t ClockSourceGetFreq(ClockSource source);
int32_t ClockDomainGetFreq(ClockDomain domain);

// Sample cycle counter.
static inline uint32_t Clock32GetCycles(void) {
  // [ARM Cortex-R4 TRM 6.3.7]
  uint32_t clock_count;
  __asm__ __volatile__("mrc p15, 0, %0, c9, c13, 0" : "=r"(clock_count));
  return clock_count;
}

// Returns the number of microseconds since ClockInit.
int64_t ClockGetUs(void);

// Waits for at least the given number of cycles.
void Clock32WaitCycles(int32_t cycles);

// Saturate latency measurement to 32-bit signed.
int32_t SaturateLatency(int64_t latency);

// Helper function for polling periodic tasks.
bool PollPeriodicCycles(int32_t period, uint32_t *next);

#endif  // AVIONICS_FIRMWARE_CPU_CLOCK_H_

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

#include "avionics/firmware/cpu/rti.h"

#include <assert.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/registers.h"

// This module implements two timers using the real-time interrupt (RTI)
// module. Timer 0 provides a 63-bit microsecond up-time counter, while
// timer 1 provides a 32-bit microsecond timer for periodic functions.

void StartupRtiInit(void) {
  // Disable.
  RTI.GCTRL.raw = 0x0;
  RTI.CLEARINTENA.raw = 0xFFFFFFFF;

  // This code assumes RTICLK operates at 2 MHz such that we can use both
  // the up counter (RTICU0) and the free running counter (RTIFRC0) to create
  // a 63-bit microsecond counter in hardware.
  assert(ClockGetRticlkFreq() == 2000000);

  // Set free running counter prescales.
  RTI.CPUC0.raw = 0xFFFFFFFF;  // Up counter used to create a 63-bit timer.
  RTI.CPUC1.raw = 1;           // Prescale to microseconds.

  // Increment free running counter via up counter.
  RTI.TBCTRL.raw = 0x0;  // TBEXT=0: RTIUC0 clocks RTIFRC0.

  // Zero counters.
  RTI.FRC0.raw = 0U;
  RTI.UC0.raw = 0U;
  RTI.FRC1.raw = 0U;
  RTI.UC1.raw = 0U;

  // Enable timers.
  RTI.GCTRL.raw |= RTI_GCTRL_CNT0EN | RTI_GCTRL_CNT1EN;
}

int64_t RtiGetUptime(void) {
  // See TMS570 TRM 13.2.1.1 "Counter and Capture Read Consistency".
  uint32_t hi = RTI.FRC0.raw;  // Read RTIFRCx first.
  uint32_t lo = RTI.UC0.raw >> 1;
  return (int64_t)hi << 31 | (int64_t)lo;
}

void RtiEnablePeriodicTimer(void) {
  RTI.GCTRL.CNT1EN = 1;
}

void RtiDisablePeriodicTimer(void) {
  RTI.GCTRL.CNT1EN = 0;
}

void RtiSetPeriodicTimer(uint32_t time_usec) {
  // RTI clock must be disabled to ensure RTIFRC1/RTIUC1 consistency.
  RtiDisablePeriodicTimer();
  RTI.FRC1.raw = time_usec;
  RTI.UC1.raw = 0;
}

uint32_t RtiGetPeriodicTimer(void) {
  return RTI.FRC1.raw;
}

void RtiEnablePeriodicInterrupt0(uint32_t offset_usec, uint32_t period_usec) {
  RTI.COMPCTRL.COMPSEL0 = 1;                  // Select RTIRFC1 clock source.
  RTI.CMP0CLR.raw = offset_usec + 1;          // Compare to clear interrupt.
  RTI.COMP0.raw = offset_usec;                // Compare to set interrupt.
  RTI.UDCP0.raw = period_usec;                // Compare counter increment.
  RTI.INTFLAG.raw = RTI_INTFLAG_INT0;         // Clear pending interrupt.
  RTI.INTCLRENABLE.INTCLRENABLE0 = 0x0A;      // Enable interrupt auto-clear.
  RTI.SETINTENA.raw = RTI_SETINTENA_SETINT0;  // Enable interrupt.
}

void RtiEnablePeriodicInterrupt1(uint32_t offset_usec, uint32_t period_usec) {
  RTI.COMPCTRL.COMPSEL1 = 1;                  // Select RTIRFC1 clock source.
  RTI.CMP1CLR.raw = offset_usec + 1;          // Compare to clear interrupt.
  RTI.COMP1.raw = offset_usec;                // Compare to set interrupt.
  RTI.UDCP1.raw = period_usec;                // Compare counter increment.
  RTI.INTFLAG.raw = RTI_INTFLAG_INT1;         // Clear pending interrupt.
  RTI.INTCLRENABLE.INTCLRENABLE1 = 0x0A;      // Enable interrupt auto-clear.
  RTI.SETINTENA.raw = RTI_SETINTENA_SETINT1;  // Enable interrupt.
}

void RtiEnablePeriodicInterrupt2(uint32_t offset_usec, uint32_t period_usec) {
  RTI.COMPCTRL.COMPSEL2 = 1;                  // Select RTIRFC1 clock source.
  RTI.CMP2CLR.raw = offset_usec + 1;          // Compare to clear interrupt.
  RTI.COMP2.raw = offset_usec;                // Compare to set interrupt.
  RTI.UDCP2.raw = period_usec;                // Compare counter increment.
  RTI.INTFLAG.raw = RTI_INTFLAG_INT2;         // Clear pending interrupt.
  RTI.INTCLRENABLE.INTCLRENABLE2 = 0x0A;      // Enable interrupt auto-clear.
  RTI.SETINTENA.raw = RTI_SETINTENA_SETINT2;  // Enable interrupt.
}

void RtiEnablePeriodicInterrupt3(uint32_t offset_usec, uint32_t period_usec) {
  RTI.COMPCTRL.COMPSEL3 = 1;                  // Select RTIRFC1 clock source.
  RTI.CMP3CLR.raw = offset_usec + 1;          // Compare to clear interrupt.
  RTI.COMP3.raw = offset_usec;                // Compare to set interrupt.
  RTI.UDCP3.raw = period_usec;                // Compare counter increment.
  RTI.INTFLAG.raw = RTI_INTFLAG_INT3;         // Clear pending interrupt.
  RTI.INTCLRENABLE.INTCLRENABLE3 = 0x0A;      // Enable interrupt auto-clear.
  RTI.SETINTENA.raw = RTI_SETINTENA_SETINT3;  // Enable interrupt.
}

void RtiDisablePeriodicInterrupt0(void) {
  RTI.CLEARINTENA.raw = RTI_CLEARINTENA_CLEARINT0;
}

void RtiDisablePeriodicInterrupt1(void) {
  RTI.CLEARINTENA.raw = RTI_CLEARINTENA_CLEARINT1;
}

void RtiDisablePeriodicInterrupt2(void) {
  RTI.CLEARINTENA.raw = RTI_CLEARINTENA_CLEARINT2;
}

void RtiDisablePeriodicInterrupt3(void) {
  RTI.CLEARINTENA.raw = RTI_CLEARINTENA_CLEARINT3;
}

void RtiClearPeriodicFlag0(void) {
  RTI.INTFLAG.raw = RTI_INTFLAG_INT0;
}

void RtiClearPeriodicFlag1(void) {
  RTI.INTFLAG.raw = RTI_INTFLAG_INT1;
}

void RtiClearPeriodicFlag2(void) {
  RTI.INTFLAG.raw = RTI_INTFLAG_INT2;
}

void RtiClearPeriodicFlag3(void) {
  RTI.INTFLAG.raw = RTI_INTFLAG_INT3;
}

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

#include "avionics/firmware/cpu/peripherals.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/registers.h"

// See TMS570LS1227 Table 4-21 "Device Memory Map". The Frame Chip Select
// column corresponds to the peripheral select PS[] and peripheral memory
// clock select PCS[].

// See TMS570LS1227 Figure 4-7 "Device Clock Domains".

typedef struct {
  ClockDomain domain;
  int32_t ps;   // PS[] peripheral select number, -1 to disable.
  int32_t pcs;  // PCS[] Peripheral clock select number, -1 to disable.
} PeripheralSelect;

static const PeripheralSelect kPeripheralMap[kNumPeripherals] = {
  [kPeripheralDcan1]     = {kClockDomainVclka1, 8, 15},
  [kPeripheralDcan2]     = {kClockDomainVclka1, 8, 14},
  [kPeripheralDcan3]     = {kClockDomainVclka1, 7, 13},
  [kPeripheralEcap]      = {kClockDomainVclk4, -1, -1},  // See TRM.
  [kPeripheralEpwm]      = {kClockDomainVclk4, -1, -1},  // See TRM.
  [kPeripheralGio]       = {kClockDomainVclk, 16, -1},
  [kPeripheralI2c]       = {kClockDomainVclk, 10, -1},
  [kPeripheralMibAdc1]   = {kClockDomainVclk, 15, 31},
  [kPeripheralMibAdc2]   = {kClockDomainVclk, 15, 29},
  [kPeripheralMibSpi1]   = {kClockDomainVclk, 2, 7},
  [kPeripheralMibSpi3]   = {kClockDomainVclk, 1, 6},
  [kPeripheralMibSpi5]   = {kClockDomainVclk, 0, 5},
  [kPeripheralN2Het1]    = {kClockDomainVclk2, 17, 35},
  [kPeripheralN2Het1Tu1] = {kClockDomainVclk, -1, 39},
  [kPeripheralN2Het2]    = {kClockDomainVclk2, 17, 34},
  [kPeripheralN2Het2Tu2] = {kClockDomainVclk, -1, 38},
  [kPeripheralSci1]      = {kClockDomainVclk, 6, -1},  // We use LIN as SCI1.
  [kPeripheralSci2]      = {kClockDomainVclk, 6, -1},
  [kPeripheralSpi2]      = {kClockDomainVclk, 2, -1},
  [kPeripheralSpi4]      = {kClockDomainVclk, 1, -1},
};

int32_t PeripheralGetClockFreq(Peripheral per) {
  assert(0 <= per && per < kNumPeripherals);
  return ClockDomainGetFreq(kPeripheralMap[per].domain);
}

int32_t PeripheralGetClockPrescale(Peripheral per, int32_t output_freq) {
  assert(output_freq > 0);
  float f_pclk = (float)PeripheralGetClockFreq(per);
  float f_out = (float)output_freq;
  return lroundf(f_pclk / f_out);
}

void PeripheralEnable(Peripheral per) {
  assert(0 <= per && per < kNumPeripherals);
  const PeripheralSelect *p = &kPeripheralMap[per];
  if (p->ps >= 0) {
    PCR.PSPWRDWNCLR[p->ps / 32].raw = 1U << (p->ps % 32);
  }
  if (p->pcs >= 0) {
    PCR.PCSPWRDWNCLR[p->pcs / 32].raw = 1U << (p->pcs % 32);
  }
}

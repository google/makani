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

#ifndef AVIONICS_FIRMWARE_CPU_GIO_H_
#define AVIONICS_FIRMWARE_CPU_GIO_H_

#include <stdbool.h>

// Ordered to ensure mathematical relationship with registers.
typedef enum {
  kGioPinForceSigned = -1,
  kGioPinA0,
  kGioPinA1,
  kGioPinA2,
  kGioPinA3,
  kGioPinA4,
  kGioPinA5,
  kGioPinA6,
  kGioPinA7,
  kGioPinB0,
  kGioPinB1,
  kGioPinB2,
  kGioPinB3,
  kGioPinB4,
  kGioPinB5,
  kGioPinB6,
  kGioPinB7,
  kNumGioPins
} GioPin;

typedef enum {
  kGioInterruptDetectFallingEdge,
  kGioInterruptDetectRisingEdge,
  kGioInterruptDetectBothEdges,
} GioInterruptDetect;

void GioInit(void);

// Use these functions to initialize a GIO pin to a known state.
void GioConfigureAsInputPullDown(GioPin pin);
void GioConfigureAsInputPullUp(GioPin pin);
void GioConfigureAsInput(GioPin pin);
void GioConfigureAsOutputPushPull(GioPin pin, bool value);
void GioConfigureAsOutputOpenDrain(GioPin pin, bool value);

// The functions below set individual bits of a GIO pin.

// Interrupt bits.
void GioSetInterruptDetect(GioPin pin, GioInterruptDetect detect);
void GioEnableInterrupt(GioPin pin);
void GioDisableInterrupt(GioPin pin);
void GioSetInterruptPriorityAsHigh(GioPin pin);
void GioSetInterruptPriorityAsLow(GioPin pin);
void GioClearInterruptFlag(GioPin pin);
bool GioGetInterruptFlag(GioPin pin);

// Direction bits.
void GioSetDirectionAsOutput(GioPin pin);
void GioSetDirectionAsInput(GioPin pin);

// Get/Set value bits.
void GioSetValue(GioPin pin, bool value);
bool GioGetValue(GioPin pin);
void GioSetOutputHigh(GioPin pin);
void GioSetOutputLow(GioPin pin);

// Output type bits.
void GioSetAsPushPull(GioPin pin);
void GioSetAsOpenDrain(GioPin pin);

// Input type bits.
void GioDisablePull(GioPin pin);
void GioEnablePull(GioPin pin);
void GioSetAsPullDown(GioPin pin);
void GioSetAsPullUp(GioPin pin);

#endif  // AVIONICS_FIRMWARE_CPU_GIO_H_

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

#ifndef AVIONICS_FIRMWARE_CPU_SCI_PIN_H_
#define AVIONICS_FIRMWARE_CPU_SCI_PIN_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  /* Sci1 vs. Sci2: bus = (pin / 2) + 1, for value 1 or 2.
     Rx vs. Tx: register values are Rx=1 and Tx=2, so reg = (pin % 2) + 1.
     See tms570 TRM pages 496-497. */
  kSci1PinRx,
  kSci1PinTx,
  kSci2PinRx,
  kSci2PinTx,
  kNumSciPins
} SciPin;

// Use these functions to initialize a GIO pin to a known state.
void SciPinConfigureAsInputPullDown(SciPin pin);
void SciPinConfigureAsInputPullUp(SciPin pin);
void SciPinConfigureAsInput(SciPin pin);
void SciPinConfigureAsOutputPushPull(SciPin pin, bool value);
void SciPinConfigureAsOutputOpenDrain(SciPin pin, bool value);

// The functions below set individual bits of a GIO pin.

// Direction bits.
void SciPinSetDirectionAsOutput(SciPin pin);
void SciPinSetDirectionAsInput(SciPin pin);

// Get/Set value bits.
void SciPinSetValue(SciPin pin, bool value);
bool SciPinGetValue(SciPin pin);
void SciPinSetOutputHigh(SciPin pin);
void SciPinSetOutputLow(SciPin pin);

// Output type bits.
void SciPinSetAsPushPull(SciPin pin);
void SciPinSetAsOpenDrain(SciPin pin);

// Input type bits.
void SciPinDisablePull(SciPin pin);
void SciPinEnablePull(SciPin pin);
void SciPinSetAsPullDown(SciPin pin);
void SciPinSetAsPullUp(SciPin pin);

#endif  // AVIONICS_FIRMWARE_CPU_SCI_PIN_H_

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

#ifndef AVIONICS_FIRMWARE_CPU_DCAN_PIN_H_
#define AVIONICS_FIRMWARE_CPU_DCAN_PIN_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  kDcanPinForceSigned = -1,
  kDcan1PinRx,
  kDcan1PinTx,
  kDcan2PinRx,
  kDcan2PinTx,
  kDcan3PinRx,
  kDcan3PinTx,
  kNumDcanPins
} DcanPin;

// Use these functions to initialize a GIO pin to a known state.
void DcanPinConfigureAsInputPullDown(DcanPin pin);
void DcanPinConfigureAsInputPullUp(DcanPin pin);
void DcanPinConfigureAsInput(DcanPin pin);
void DcanPinConfigureAsOutputPushPull(DcanPin pin, bool value);
void DcanPinConfigureAsOutputOpenDrain(DcanPin pin, bool value);

// The functions below set individual bits of a GIO pin.

// Direction bits.
void DcanPinSetDirectionAsOutput(DcanPin pin);
void DcanPinSetDirectionAsInput(DcanPin pin);

// Get/Set value bits.
void DcanPinSetValue(DcanPin pin, bool value);
bool DcanPinGetValue(DcanPin pin);
void DcanPinSetOutputHigh(DcanPin pin);
void DcanPinSetOutputLow(DcanPin pin);

// Output type bits.
void DcanPinSetAsPushPull(DcanPin pin);
void DcanPinSetAsOpenDrain(DcanPin pin);

// Input type bits.
void DcanPinDisablePull(DcanPin pin);
void DcanPinEnablePull(DcanPin pin);
void DcanPinSetAsPullDown(DcanPin pin);
void DcanPinSetAsPullUp(DcanPin pin);

#endif  // AVIONICS_FIRMWARE_CPU_DCAN_PIN_H_

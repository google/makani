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

#ifndef AVIONICS_FIRMWARE_CPU_N2HET_H_
#define AVIONICS_FIRMWARE_CPU_N2HET_H_

#include <stdbool.h>

// Ordered to ensure mathematical relationship with registers.
typedef enum {
  kN2hetPinForceSigned = -1,
  kN2het1Pin0,
  kN2het1Pin1,
  kN2het1Pin2,
  kN2het1Pin3,
  kN2het1Pin4,
  kN2het1Pin5,
  kN2het1Pin6,
  kN2het1Pin7,
  kN2het1Pin8,
  kN2het1Pin9,
  kN2het1Pin10,
  kN2het1Pin11,
  kN2het1Pin12,
  kN2het1Pin13,
  kN2het1Pin14,
  kN2het1Pin15,
  kN2het1Pin16,
  kN2het1Pin17,
  kN2het1Pin18,
  kN2het1Pin19,
  kN2het1Pin20,
  kN2het1Pin21,
  kN2het1Pin22,
  kN2het1Pin23,
  kN2het1Pin24,
  kN2het1Pin25,
  kN2het1Pin26,
  kN2het1Pin27,
  kN2het1Pin28,
  kN2het1Pin29,
  kN2het1Pin30,
  kN2het1Pin31,
  kN2het2Pin0,
  kN2het2Pin1,
  kN2het2Pin2,
  kN2het2Pin3,
  kN2het2Pin4,
  kN2het2Pin5,
  kN2het2Pin6,
  kN2het2Pin7,
  kN2het2Pin8,
  kN2het2Pin9,
  kN2het2Pin10,
  kN2het2Pin11,
  kN2het2Pin12,
  kN2het2Pin13,
  kN2het2Pin14,
  kN2het2Pin15,
  kN2het2Pin16,
  kN2het2Pin17,
  kN2het2Pin18,
  kN2het2Pin19,
  kN2het2Pin20,
  kN2het2Pin21,
  kN2het2Pin22,
  kN2het2Pin23,
  kN2het2Pin24,
  kN2het2Pin25,
  kN2het2Pin26,
  kN2het2Pin27,
  kN2het2Pin28,
  kN2het2Pin29,
  kN2het2Pin30,
  kN2het2Pin31,
  kNumN2hetPins
} N2hetPin;

void N2hetInit(void);

// Use these functions to initialize a GIO pin to a known state.
void N2hetConfigureAsInputPullDown(N2hetPin pin);
void N2hetConfigureAsInputPullUp(N2hetPin pin);
void N2hetConfigureAsInput(N2hetPin pin);
void N2hetConfigureAsOutputPushPull(N2hetPin pin, bool value);
void N2hetConfigureAsOutputOpenDrain(N2hetPin pin, bool value);

// The functions below set individual bits of a GIO pin.

// Direction bits.
void N2hetSetDirectionAsOutput(N2hetPin pin);
void N2hetSetDirectionAsInput(N2hetPin pin);

// Get/Set value bits.
void N2hetSetValue(N2hetPin pin, bool value);
bool N2hetGetValue(N2hetPin pin);
void N2hetSetOutputHigh(N2hetPin pin);
void N2hetSetOutputLow(N2hetPin pin);

// Output type bits.
void N2hetSetAsPushPull(N2hetPin pin);
void N2hetSetAsOpenDrain(N2hetPin pin);

// Input type bits.
void N2hetDisablePull(N2hetPin pin);
void N2hetEnablePull(N2hetPin pin);
void N2hetSetAsPullDown(N2hetPin pin);
void N2hetSetAsPullUp(N2hetPin pin);

#endif  // AVIONICS_FIRMWARE_CPU_N2HET_H_

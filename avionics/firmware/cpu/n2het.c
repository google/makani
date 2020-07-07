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

#include "avionics/firmware/cpu/n2het.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "common/macros.h"


// N2HET pin enumeration must adhere to mathematical relationship between pin
// number and device/offset.
COMPILE_ASSERT(kN2het1Pin0 == 0, kN2het1Pin0_must_be_0);
COMPILE_ASSERT(kN2het1Pin31 == 31, kN2het1Pin31_must_be_31);
COMPILE_ASSERT(kN2het2Pin0 == 32, kN2het2Pin0_must_be_32);
COMPILE_ASSERT(kN2het2Pin31 == 63, kN2het2Pin31_must_be_63);

static bool GetDeviceMask(N2hetPin pin, int32_t *device, int32_t *mask) {
  *device = pin / 32 + 1;
  *mask = 1U << (pin % 32);
  assert(1 <= *device && *device <= 2);
  return 1 <= *device && *device <= 2;
}

static void EnablePinmux(N2hetPin pin) {
  // See TMS570 TRM "Output Multiplexing and Control".
  static const IommPinmux kPinmux[kNumN2hetPins] = {
    [kN2het1Pin0] = {5, 0},
    [kN2het1Pin1] = {4, 16},
    [kN2het1Pin2] = {5, 8},
    [kN2het1Pin3] = {4, 24},
    [kN2het1Pin4] = {33, 0},
    [kN2het1Pin5] = {5, 16},
    [kN2het1Pin6] = {7, 16},
    [kN2het1Pin7] = {6, 0},
    [kN2het1Pin8] = {14, 0},
    [kN2het1Pin9] = {6, 16},
    [kN2het1Pin10] = {17, 0},
    [kN2het1Pin11] = {1, 8},
    [kN2het1Pin12] = {17, 16},
    [kN2het1Pin13] = {8, 0},
    // kN2het1Pin14 does not have a pinmux.
    [kN2het1Pin15] = {8, 16},
    [kN2het1Pin16] = {34, 0},
    [kN2het1Pin17] = {20, 17},
    [kN2het1Pin18] = {34, 8},
    [kN2het1Pin19] = {8, 9},
    [kN2het1Pin20] = {34, 16},
    [kN2het1Pin21] = {9, 25},
    // kN2het1Pin22 does not have a pinmux.
    [kN2het1Pin23] = {12, 17},
    [kN2het1Pin24] = {11, 24},
    [kN2het1Pin25] = {7, 9},
    [kN2het1Pin26] = {12, 0},
    [kN2het1Pin27] = {0, 26},
    [kN2het1Pin28] = {14, 8},
    [kN2het1Pin29] = {0, 18},
    [kN2het1Pin30] = {19, 8},
    [kN2het1Pin31] = {9, 10},
    [kN2het2Pin0] = {2, 3},
    [kN2het2Pin1] = {22, 1},
    [kN2het2Pin2] = {2, 17},
    [kN2het2Pin3] = {21, 1},
    [kN2het2Pin4] = {3, 17},
    [kN2het2Pin5] = {14, 25},
    [kN2het2Pin6] = {4, 1},
    [kN2het2Pin7] = {10, 18},
    [kN2het2Pin8] = {4, 20},
    [kN2het2Pin9] = {11, 2},
    [kN2het2Pin10] = {4, 28},
    [kN2het2Pin11] = {22, 18},
    [kN2het2Pin12] = {5, 18},
    [kN2het2Pin13] = {22, 10},
    [kN2het2Pin14] = {6, 3},
    [kN2het2Pin15] = {22, 26},
    [kN2het2Pin16] = {6, 17},
    // kN2het2Pin17 does not have a pinmux.
    [kN2het2Pin18] = {1, 10},
    // kN2het2Pin19--kN2het2Pin31 do not have pinmuxes.
  };
  IommSetPinmuxByIndex(kPinmux, ARRAYSIZE(kPinmux), pin);
}

static void EnablePeripheralForPin(N2hetPin pin) {
  if (kN2het1Pin0 <= pin && pin <= kN2het1Pin31) {
    PeripheralEnable(kPeripheralN2Het1);
  } else if (kN2het2Pin0 <= pin && pin <= kN2het2Pin31) {
    PeripheralEnable(kPeripheralN2Het2);
  }
}

void N2hetInit(void) {
  // TODO: Evaluate N2HET initialization.
}

void N2hetConfigureAsInputPullDown(N2hetPin pin) {
  EnablePeripheralForPin(pin);
  N2hetSetDirectionAsInput(pin);
  N2hetSetAsPullDown(pin);
  N2hetEnablePull(pin);
  EnablePinmux(pin);
}

void N2hetConfigureAsInputPullUp(N2hetPin pin) {
  EnablePeripheralForPin(pin);
  N2hetSetDirectionAsInput(pin);
  N2hetSetAsPullUp(pin);
  N2hetEnablePull(pin);
  EnablePinmux(pin);
}

void N2hetConfigureAsInput(N2hetPin pin) {
  EnablePeripheralForPin(pin);
  N2hetSetDirectionAsInput(pin);
  N2hetDisablePull(pin);
  EnablePinmux(pin);
}

void N2hetConfigureAsOutputPushPull(N2hetPin pin, bool value) {
  EnablePeripheralForPin(pin);
  N2hetSetValue(pin, value);
  N2hetSetAsPushPull(pin);
  N2hetSetDirectionAsOutput(pin);
  EnablePinmux(pin);
}

void N2hetConfigureAsOutputOpenDrain(N2hetPin pin, bool value) {
  EnablePeripheralForPin(pin);
  N2hetSetValue(pin, value);
  N2hetSetAsOpenDrain(pin);
  N2hetSetDirectionAsOutput(pin);
  EnablePinmux(pin);
}

void N2hetSetDirectionAsOutput(N2hetPin pin) {
  int32_t device, mask;
  if (GetDeviceMask(pin, &device, &mask)) {
    N2HET(device).HETDIR.raw |= mask;
  }
}

void N2hetSetDirectionAsInput(N2hetPin pin) {
  int32_t device, mask;
  if (GetDeviceMask(pin, &device, &mask)) {
    N2HET(device).HETDIR.raw &= ~mask;
  }
}

void N2hetSetValue(N2hetPin pin, bool value) {
  if (value) {
    N2hetSetOutputHigh(pin);
  } else {
    N2hetSetOutputLow(pin);
  }
}

bool N2hetGetValue(N2hetPin pin) {
  int32_t device, mask;
  if (GetDeviceMask(pin, &device, &mask)) {
    return (N2HET(device).HETDIN.raw & mask) != 0;
  }
  return false;
}

void N2hetSetOutputHigh(N2hetPin pin) {
  int32_t device, mask;
  if (GetDeviceMask(pin, &device, &mask)) {
    N2HET(device).HETDSET.raw = mask;
  }
}

void N2hetSetOutputLow(N2hetPin pin) {
  int32_t device, mask;
  if (GetDeviceMask(pin, &device, &mask)) {
    N2HET(device).HETDCLR.raw = mask;
  }
}

void N2hetSetAsPushPull(N2hetPin pin) {
  int32_t device, mask;
  if (GetDeviceMask(pin, &device, &mask)) {
    N2HET(device).HETPDR.raw &= ~mask;
  }
}

void N2hetSetAsOpenDrain(N2hetPin pin) {
  int32_t device, mask;
  if (GetDeviceMask(pin, &device, &mask)) {
    N2HET(device).HETPDR.raw |= mask;
  }
}

void N2hetDisablePull(N2hetPin pin) {
  int32_t device, mask;
  if (GetDeviceMask(pin, &device, &mask)) {
    N2HET(device).HETPULDIS.raw |= mask;
  }
}

void N2hetEnablePull(N2hetPin pin) {
  int32_t device, mask;
  if (GetDeviceMask(pin, &device, &mask)) {
    N2HET(device).HETPULDIS.raw &= ~mask;
  }
}

void N2hetSetAsPullDown(N2hetPin pin) {
  int32_t device, mask;
  if (GetDeviceMask(pin, &device, &mask)) {
    N2HET(device).HETPSL.raw &= ~mask;
  }
}

void N2hetSetAsPullUp(N2hetPin pin) {
  int32_t device, mask;
  if (GetDeviceMask(pin, &device, &mask)) {
    N2HET(device).HETPSL.raw |= mask;
  }
}

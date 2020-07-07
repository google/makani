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

#include "avionics/firmware/drivers/led.h"

#include <stdbool.h>
#include <stddef.h>

#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"

void LedInit(void) {
  // Set pinmux.
  IommSetPinmux(6, 3);   // N2HET2[14]
  IommSetPinmux(6, 17);  // N2HET2[16]

  // Initialize N2HET2 for GPIO functionality on N2HET2[14] and N2HET2[16],
  // which is used for the LEDs.

  // Enable clocks.
  PeripheralEnable(kPeripheralN2Het2);

  // Set pins to output.
  N2HET(2).HETDIR.HETDIR14 = 1;
  N2HET(2).HETDIR.HETDIR16 = 1;
}

void LedSet(bool led1, bool led2) {
  N2HET(2).HETDOUT.HETDOUT14 = led1;
  N2HET(2).HETDOUT.HETDOUT16 = led2;
}

void LedGet(bool *led1, bool *led2) {
  if (led1 != NULL) {
    *led1 = N2HET(2).HETDOUT.HETDOUT14;
  }
  if (led2 != NULL) {
    *led2 = N2HET(2).HETDOUT.HETDOUT16;
  }
}

void LedOn(bool led1, bool led2) {
  N2HET(2).HETDSET.raw = led1 << 14 | led2 << 16;
}

void LedOff(bool led1, bool led2) {
  N2HET(2).HETDCLR.raw = led1 << 14 | led2 << 16;
}

void LedToggle(bool led1, bool led2) {
  N2HET(2).HETDOUT.HETDOUT14 ^= led1;
  N2HET(2).HETDOUT.HETDOUT16 ^= led2;
}

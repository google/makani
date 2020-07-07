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

#ifndef AVIONICS_MOTOR_FIRMWARE_UTIL_H_
#define AVIONICS_MOTOR_FIRMWARE_UTIL_H_

#include "avionics/firmware/cpu/registers.h"

void GateDriverInit(void);

static inline void GateDriverEnable(void) {
  SCI(2).PIO4.TXSET = 1;
}

static inline void GateDriverDisable(void) {
  // The CLR registers read the current state of the pins, so if you
  // do a read modify write (which a bitwise struct access does) it
  // clears all of the GPIO!  The .raw forces a single write to the
  // register which clears only the selected GPIO.
  SCI(2).PIO5.raw = 1 << 2;  // TXCLR.
}

#endif  // AVIONICS_MOTOR_FIRMWARE_UTIL_H_

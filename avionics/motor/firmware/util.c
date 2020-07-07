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

#include "avionics/motor/firmware/util.h"

#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/registers.h"

void GateDriverInit(void) {
  // Set pinmux to SCITX.
  IommSetPinmux(8, 1);

  // Enable SCI.
  SCI(2).GCR0.RESET = 1;

  // Clear and set TX as output.
  SCI(2).PIO5.raw = 1 << 2;  // TXCLR (see comment in header).
  SCI(2).PIO1.TXDIR = 1;
}

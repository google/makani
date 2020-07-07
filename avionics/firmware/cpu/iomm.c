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

#include "avionics/firmware/cpu/iomm.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/registers.h"

#define IOMM_REG_MAX 48
#define IOMM_BIT_MAX 32

static void Unlock(void) {
  IOMM.KICK_REG0.KICK0 = 0x83E70B13;
  IOMM.KICK_REG1.KICK1 = 0x95A4F1E0;
}

static void Lock(void) {
  IOMM.KICK_REG0.KICK0 = 0;
  IOMM.KICK_REG1.KICK1 = 0;
}

static bool IsValidPinmux(int32_t reg, int32_t bit) {
  return 0 <= reg && reg < IOMM_REG_MAX && 0 <= bit && bit < IOMM_BIT_MAX;
}

void IommSetPinmux(int32_t reg, int32_t bit) {
  assert(IsValidPinmux(reg, bit));
  if (IsValidPinmux(reg, bit)) {
    Unlock();
    ((volatile uint8_t *)&IOMM.PINMMR[reg])[3 - bit / 8] = 1 << (bit % 8);
    Lock();
  }
}

void IommSetPinmuxByIndex(const IommPinmux *map, int32_t length,
                          int32_t index) {
  // We ignore PINMMR0[0] (reg == 0 and bit == 0) to handle the designated
  // initializer default for values not specified in the map. TI does not
  // use PINMMR0[0], so we do not foresee any issues with this approach.
  const IommPinmux *pinmux = map + index;
  if (0 <= index && index < length && !(pinmux->reg <= 0 && pinmux->bit <= 0)) {
    IommSetPinmux(pinmux->reg, pinmux->bit);
  }
}

void IommClearPinmux(int32_t reg, int32_t bit) {
  assert(IsValidPinmux(reg, bit));
  if (IsValidPinmux(reg, bit)) {
    Unlock();
    ((volatile uint8_t *)&IOMM.PINMMR[reg])[3 - bit / 8] = 0;
    Lock();
  }
}

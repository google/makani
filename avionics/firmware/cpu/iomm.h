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

#ifndef AVIONICS_FIRMWARE_CPU_IOMM_H_
#define AVIONICS_FIRMWARE_CPU_IOMM_H_

#include <stdint.h>

typedef struct {
  int32_t reg;  // PINMMR register number x in PINMMRx[y].
  int32_t bit;  // PINMMR register bit number y in PINMMRx[y].
} IommPinmux;

void IommSetPinmux(int32_t reg, int32_t bit);
void IommSetPinmuxByIndex(const IommPinmux *map, int32_t length, int32_t index);
void IommClearPinmux(int32_t reg, int32_t bit);

#endif  // AVIONICS_FIRMWARE_CPU_IOMM_H_

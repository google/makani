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

#include "avionics/firmware/drivers/ltc2309_types.h"

#include <assert.h>
#include <stdint.h>

uint8_t Ltc2309BuildCommand(Ltc2309Select select, Ltc2309ConversionMode convert,
                            Ltc2309PowerSavingMode sleep) {
  assert((select & 0xF0) == select);
  assert((convert & 0x08) == convert);
  assert((sleep & 0x04) == sleep);

  return (uint8_t)((select & 0xF0) | (convert & 0x08) | (sleep & 0x04));
}

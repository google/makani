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

#include "avionics/firmware/drivers/mcp9800_types.h"

#include <assert.h>
#include <stdint.h>


uint8_t Mcp9800BuildConfig(Mcp9800Resolution res) {
  assert((res & 0x03) == res);

  return (uint8_t)((res & 0x03) << 5);
}

float Mcp9800TempRawToC(uint16_t ta_raw) {
  // The format of the ambient temperature [C] measurement
  // is in sign-magnitude format as follows:
  //
  // MSB: sign | 2^6  | 2^5  | 2^4  | 2^3 | 2^2 | 2^1 | 2^0
  // LSB: 2^-1 | 2^-2 | 2^-3 | 2^-4 | 0   | 0   | 0   | 0
  float ta_c = (float)((ta_raw & 0x7FFF) >> 4) / 16.0f;
  if ((ta_raw & 0x8000) != 0) {
    ta_c = -ta_c;
  }
  return ta_c;
}

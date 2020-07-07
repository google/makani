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

#ifndef AVIONICS_COMMON_GILL_BINARY_H_
#define AVIONICS_COMMON_GILL_BINARY_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/gill_types.h"

typedef enum {
  kGillBinaryIdWindmasterMode7  = 0xB1,
  kGillBinaryIdWindmasterMode8  = 0xB2,
  kGillBinaryIdWindmasterMode9  = 0xB3,
  kGillBinaryIdWindmasterMode10 = 0xB4,
} GillBinaryId;

typedef struct {
  GillBinaryId id;
  uint8_t checksum;
  int32_t length;
} GillBinary;

int32_t GillBinaryGetLength(GillBinaryId id);
bool GillBinaryDecodeWindmaster(const GillBinary *bin, const uint8_t *data,
                                GillData *out);

#endif  // AVIONICS_COMMON_GILL_BINARY_H_

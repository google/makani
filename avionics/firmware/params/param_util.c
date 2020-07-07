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

#include "avionics/firmware/params/param_util.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define MAC_BYTES 6
#define MAC_HEX_STRING_LEN (MAC_BYTES * 2)
#define MAC_MAX_VALUE ((1ULL << (MAC_BYTES * 8)) - 1)

bool ParamStringToEthernetAddress(const char *str, EthernetAddress *addr) {
  const uint32_t len = (uint32_t)strlen(str);
  if (len != MAC_HEX_STRING_LEN) {
    return false;
  }
  for (uint32_t i = 0; i < len; ++i) {
    if (!isxdigit((unsigned char)str[i])) {
      return false;
    }
  }
  uint64_t intval = strtoull(str, NULL, 16);
  if (intval > MAC_MAX_VALUE) {
    return false;
  }
  if (addr != NULL) {
    *addr = (EthernetAddress) {
      (uint8_t)((intval >> 40) & 0xff),
      (uint8_t)((intval >> 32) & 0xff),
      (uint8_t)((intval >> 24) & 0xff),
      (uint8_t)((intval >> 16) & 0xff),
      (uint8_t)((intval >> 8) & 0xff),
      (uint8_t)((intval >> 0) & 0xff)
    };
  }
  return true;
}

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

#include "avionics/firmware/startup/ldscript.h"

#include <stdint.h>

// WARNING: These functions are called from early startup code.
// Functions that run before StartupCopyData() must not call outside
// .text.startup!  Be aware of what has and hasn't been initialized --
// see startup_*.s.

void StartupCopyData(void) {
  // Initialize .text.ram (possibly including memcpy).
  for (int32_t i = 0; ldscript_copy_table[i].size >= 0; ++i) {
    const uint8_t *from = ldscript_copy_table[i].from;
    const uint8_t *from_end = from + ldscript_copy_table[i].size;
    uint8_t *to = ldscript_copy_table[i].to;
    while (from < from_end) {
      *to = *from;
      ++to;
      ++from;
    }
  }
}

void StartupClearBss(void) {
  // Initialize .bss.
  uint8_t *begin = ldscript_bss_begin;
  uint8_t *end = ldscript_bss_end;
  while (begin < end) {
    *begin = 0x0;
    ++begin;
  }
}

bool IsRunningFromBootSegment(void) {
  return (uint32_t)ldscript_copy_table < (uint32_t)ldscript_app_flash_begin;
}

int32_t LdscriptReadInt32(int32_t *address) {
  int32_t value;
  __asm__ __volatile__ ("ldr %0, [%1]"
                        : "=r"(value)
                        : "r"(address));
  return value;
}

int16_t LdscriptReadInt16(int16_t *address) {
  int32_t value;
  __asm__ __volatile__ ("ldrh %0, [%1]"
                        : "=r"(value)
                        : "r"(address));
  return (int16_t)value;
}

uint8_t LdscriptReadUint8(uint8_t *address) {
  int32_t value;
  __asm__ __volatile__ ("ldrb %0, [%1]"
                        : "=r"(value)
                        : "r"(address));
  return (uint8_t)value;
}

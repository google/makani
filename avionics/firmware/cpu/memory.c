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

#include "avionics/firmware/cpu/memory.h"

#include <stdint.h>

#include "avionics/firmware/cpu/registers.h"

void MemoryHardwareInit(uint32_t memory_bitmask) {
  // Disable global memory self-test mode.
  SYS.MSTGCR.MSTGENA = 0x05;

  // Enable global hardware memory initialization mode.
  SYS.MINITGCR.MINITGENA = 0x0A;

  // See Table 4-27: Memory Initialization of TMS570LS1227 datasheet.
  SYS.MSIENA.MSIENA |= memory_bitmask;
  while (!SYS.MSTCGSTAT.MINIDONE) {}

  // Disable global hardware memory initialization mode.
  SYS.MINITGCR.MINITGENA = 0x05;
}

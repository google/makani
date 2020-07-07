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

#ifndef AVIONICS_FIRMWARE_CPU_MEMORY_H_
#define AVIONICS_FIRMWARE_CPU_MEMORY_H_

#include <stdint.h>

// See Table 4-27 from TMS570LS1227 Datasheet.
typedef enum {
  kMemoryInitRam          = (1 << 0),
  kMemoryInitMibSpi5Ram   = (1 << 12),
  kMemoryInitMibSpi3Ram   = (1 << 11),
  kMemoryInitMibSpi1Ram   = (1 << 7),
  kMemoryInitDcan3Ram     = (1 << 10),
  kMemoryInitDcan2Ram     = (1 << 6),
  kMemoryInitDcan1Ram     = (1 << 5),
  kMemoryInitMibAdc2Ram   = (1 << 14),
  kMemoryInitMibAdc1Ram   = (1 << 8),
  kMemoryInitN2Het2Ram    = (1 << 15),
  kMemoryInitN2Het1Ram    = (1 << 3),
  kMemoryInitHetTu2Ram    = (1 << 16),
  kMemoryInitHetTu1Ram    = (1 << 4),
  kMemoryInitDmaRam       = (1 << 1),
  kMemoryInitVimRam       = (1 << 2),
  kMemoryInitFlexRayTuRam = (1 << 13)
} MemoryInitBit;

void MemoryHardwareInit(uint32_t memory_bitmask);

#endif  // AVIONICS_FIRMWARE_CPU_MEMORY_H_

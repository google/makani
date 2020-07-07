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

#ifndef AVIONICS_FIRMWARE_CPU_MODE_H_
#define AVIONICS_FIRMWARE_CPU_MODE_H_

#include <stdbool.h>
#include <stdint.h>

// See ARM Cortex R4 TRM, Section 3.6: Program status registers.
typedef enum {
  kCpuModeUser                 = 0x10,
  kCpuModeFiq                  = 0x11,
  kCpuModeIrq                  = 0x12,
  kCpuModeSupervisor           = 0x13,
  kCpuModeAbort                = 0x17,
  kCpuModeUndefinedInstruction = 0x1B,
  kCpuModeSystem               = 0x1F
} CpuMode;

static inline CpuMode GetCpuMode(void) {
  uint32_t cpsr;
  __asm__ __volatile__("mrs %0, cpsr" : "=r"(cpsr));
  return (CpuMode)(cpsr & 0x1F);
}

static inline bool IsFiqMode(void) {
  return GetCpuMode() == kCpuModeFiq;
}

static inline bool IsIrqMode(void) {
  return GetCpuMode() == kCpuModeIrq;
}

static inline bool IsInterruptMode(void) {
  return IsFiqMode() || IsIrqMode();
}

#endif  // AVIONICS_FIRMWARE_CPU_MODE_H_

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

#ifndef AVIONICS_FIRMWARE_CPU_VIM_H_
#define AVIONICS_FIRMWARE_CPU_VIM_H_

#include <stdint.h>

// See ARMv7-A/ARMv7-R TRM B1.3.3 "Program Status Registers (PSRs)".
#define CPSR_F_BIT (1U << 6)
#define CPSR_I_BIT (1U << 7)

#define VIM_CHANNELS 96

// Assume straight mapping between interrupt sources and VIM channels.
// See TMS570LS1227 Table 6-32 "Interrupt Request Assignments". Add as needed.
typedef enum {
  kVimChannelRtiCompare0 = 2,
  kVimChannelRtiCompare1 = 3,
  kVimChannelRtiCompare2 = 4,
  kVimChannelRtiCompare3 = 5,
  kVimChannelGioHighLevel = 9,
  kVimChannelSci1Level0 = 13,  // We use LIN as SCI1.
  kVimChannelAdc1SwGroup = 15,
  kVimChannelSoftware = 21,
  kVimChannelGioLowLevel = 23,
  kVimChannelSci1Level1 = 27,  // We use LIN as SCI1.
  kVimChannelAdc1MagnitudeCompare = 31,
  kVimChannelDmaBtca = 40,
  kVimChannelAdc2MagnitudeCompare = 59,
  kVimChannelSci2Level0 = 64,
  kVimChannelSci2Level1 = 74,
  kVimChannelEPwm2 = 92
} VimChannel;

typedef struct {
  int32_t count;
  int32_t total_cycles;
  int32_t max_cycles;
} VimAccounting;

typedef void (* const VimInterruptFunc)(void);

// Initialize and shutdown Vim (called from startup code).
void StartupVimInit(void);
void ShutdownVim(void);

// Enable IRQ interrupts.
// See TMS570 TRM Example 15-2 "Enable/Disable IRQ/FIQ through CPSR".
static inline void VimEnableIrq(void) {
  uint32_t old_cpsr;
  __asm__ __volatile__("mrs %0, cpsr" : "=r"(old_cpsr));
  uint32_t new_cpsr = old_cpsr & ~CPSR_I_BIT;  // Clear I bit to enable IRQs.
  __asm__ __volatile__("msr cpsr_c, %0" : : "r"(new_cpsr));
}

// Enable FIQ interrupts. Once enabled, FIQ interrupts cannot be disabled.
static inline void VimEnableFiq(void) {
  uint32_t old_cpsr;
  __asm__ __volatile__("mrs %0, cpsr" : "=r"(old_cpsr));
  // Clear F bit to enable FIQs. Cannot disable once enabled.
  uint32_t new_cpsr = old_cpsr & ~CPSR_F_BIT;
  __asm__ __volatile__("msr cpsr_c, %0" : : "r"(new_cpsr));
}

// Disable all IRQ interrupts (FIQs may still execute). Use returned CPSR
// register to restore IRQ interrupts using VimRestoreIrq().
static inline uint32_t VimDisableIrq(void) {
  uint32_t old_cpsr;
  __asm__ __volatile__("mrs %0, cpsr" : "=r"(old_cpsr));
  uint32_t new_cpsr = old_cpsr | CPSR_I_BIT;  // Set I bit to disable IRQs.
  __asm__ __volatile__("msr cpsr_c, %0" : : "r"(new_cpsr));
  return old_cpsr;
}

// Restore IRQ interrupt enable state. Call with CPSR register returned from
// VimDisableIrq().
static inline void VimRestoreIrq(uint32_t old_cpsr) {
  if (!(old_cpsr & CPSR_I_BIT)) {
    __asm__ __volatile__("msr cpsr_c, %0" : : "r"(old_cpsr));
  }
}

// Register an IRQ interrupt handler. Do not apply __attribute__((isr("IRQ")))
// to function func.
void VimRegisterIrq(VimChannel ch, VimInterruptFunc func);

// Register an FIQ interrupt handler. Do not apply __attribute__((isr("FIQ")))
// to function func.
void VimRegisterFiq(VimChannel ch, VimInterruptFunc func);

// Unregister an interrupt handler.
void VimUnregister(VimChannel ch);

// Enable a specific interrupt channel.
void VimEnableInterrupt(VimChannel ch);

// Disable a specific interrupt channel. Use return value to call
// VimRestoreInterrupt().
uint32_t VimDisableInterrupt(VimChannel ch);

// Restore an enable state for a specific interrupt channel. Call with return
// value of VimDisableInterrupt().
void VimRestoreInterrupt(VimChannel ch, uint32_t old_enable);

// Wrap your IRQ/FIQ function using this call to measure execution time.
// Argument func should point to a function without an IRQ/FIQ attribute.
void VimAccountingWrapper(VimInterruptFunc func, VimAccounting *account);

// Applications must define these symbols.
void OnFatalException(void);

#endif  // AVIONICS_FIRMWARE_CPU_VIM_H_

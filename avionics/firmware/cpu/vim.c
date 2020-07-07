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

#include "avionics/firmware/cpu/vim.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "avionics/firmware/cpu/registers.h"
#include "common/macros.h"

// See Cortex-R4/R4F 4.3.15 "c1, System Control Register".
#define SYSCTL_TE_BIT (1U << 30)
#define SYSCTL_VE_BIT (1U << 24)

// The LSB sets TE in the system control register on BX instruction.
static uint32_t ArmToThumbTrampoline(VimInterruptFunc func) {
  assert(((uint32_t)func & 0x01) == 0x01);
  return (uint32_t)func | 0x01;
}

// Define fatal exception handlers.
// Args:
//   description: Human-readable description of exception.
//   lr: Load Register (LR) at exception.
//   spsr: Saved Program Status Register (SPSR) at exception.
//   arm_offset, thumb_offset: How far LR will be past interrupted instruction.
//       See: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0433b/ch06s02s01.html

static void FatalExceptionHandler(const char *desc, uint32_t lr, uint32_t spsr,
                                  uint32_t arm_offset, uint32_t thumb_offset) {
  bool thumb = spsr & (1U << 5);
  printf("%s at PC=0x%08lX", desc, lr - (thumb ? thumb_offset : arm_offset));
  OnFatalException();
  while (true) {}
}

void InterruptUndefInstructionHandler(void) __attribute__((isr("UNDEF")));
void InterruptUndefInstructionHandler(void) {
  uint32_t lr, spsr;
  __asm__("mov %0, lr" : "=r"(lr));
  __asm__("mrs %0, spsr" : "=r"(spsr));
  // See ARMv7 TRM B1.9.2 "Undefined Instruction exception" for LR offsets.
  FatalExceptionHandler("Undefined instruction", lr, spsr, 4, 2);
}

void InterruptSoftwareInterruptHandler(void) __attribute__((isr("SWI")));
void InterruptSoftwareInterruptHandler(void) {
  uint32_t lr, spsr;
  __asm__("mov %0, lr" : "=r"(lr));
  __asm__("mrs %0, spsr" : "=r"(spsr));
  // See ARMv7 TRM B1.9.4 "Supervisor Call exception" for LR offsets.
  FatalExceptionHandler("Software interrupt", lr, spsr, 4, 2);
}

void InterruptPrefetchAbortHandler(void) __attribute__((isr("ABORT")));
void InterruptPrefetchAbortHandler(void) {
  uint32_t lr, spsr;
  __asm__("mov %0, lr" : "=r"(lr));
  __asm__("mrs %0, spsr" : "=r"(spsr));
  // See ARMv7 TRM B1.9.7 "Prefetch Abort exception" for LR offsets.
  FatalExceptionHandler("Instruction prefetch abort", lr, spsr, 4, 4);
}

void InterruptDataAbortHandler(void) __attribute__((isr("ABORT")));
void InterruptDataAbortHandler(void) {
  uint32_t lr, spsr;
  __asm__("mov %0, lr" : "=r"(lr));
  __asm__("mrs %0, spsr" : "=r"(spsr));
  // See ARMv7 TRM B1.9.8 "Data Abort exception" for LR offsets.
  FatalExceptionHandler("Data access abort", lr, spsr, 8, 8);
}

void InterruptIrqHandler(void) __attribute__((isr("IRQ")));
void InterruptIrqHandler(void) {
  uint32_t lr, spsr;
  __asm__("mov %0, lr" : "=r"(lr));
  __asm__("mrs %0, spsr" : "=r"(spsr));
  // See ARMv7 TRM B1.9.10 "IRQ exception" for LR offsets.
  FatalExceptionHandler("IRQ", lr, spsr, 4, 4);
}

void InterruptFiqHandler(void) __attribute__((isr("FIQ")));
void InterruptFiqHandler(void) {
  uint32_t lr, spsr;
  __asm__("mov %0, lr" : "=r"(lr));
  __asm__("mrs %0, spsr" : "=r"(spsr));
  // See ARMv7 TRM B1.9.12 "FIQ exception" for LR offsets.
  FatalExceptionHandler("FIQ", lr, spsr, 4, 4);
}

// TODO: This function should restore the VIM table.
void InterruptVimParityHandler(void) __attribute__((isr("IRQ")));
void InterruptVimParityHandler(void) {
  uint32_t lr, spsr;
  __asm__("mov %0, lr" : "=r"(lr));
  __asm__("mrs %0, spsr" : "=r"(spsr));
  // See ARMv7 TRM B1.9.10 "IRQ exception" for LR offsets.
  FatalExceptionHandler("TMS570 VIM RAM parity error", lr, spsr, 4, 4);
}

void InterruptUnhandledError(void) __attribute__((isr("IRQ")));
void InterruptUnhandledError(void) {
  uint32_t lr;
  __asm__("mov %0, lr" : "=r"(lr));
  printf("Unhandled IRQ %ld at PC=0x%08lX", VIM.IRQINDEX.raw, lr - 4);
  OnFatalException();
  while (true) {}
}

void StartupVimInit(void) {
  // Ensure interrupts are disabled!
  VimDisableIrq();

  // Disable vectored interrupt handling. Vectored interrupt handling handles
  // only IRQ interrupts and requires that the handler execute in the same
  // processing mode as specified by the TE bit. We found that this behavior
  // prevents exception handling (including the FIQ) from functioning. Instead,
  // we use VIM's IRQVECREQ/FIQVECREQ register in the boot loader's interrupt
  // handler.

  // See TRM 15.7.1 "Enable hardware vector interrupt".
  // See Cortex-R4/R4F 4.3.15 "c1, System Control Register".
  uint32_t sysctl;
  __asm__("mrc p15, 0, %0, c1, c0, 0" : "=r"(sysctl));
  sysctl &= ~SYSCTL_TE_BIT;  // Disable TE thumb interrupt handling.
  sysctl &= ~SYSCTL_VE_BIT;  // Disable VE vectored interrupt handling.
  __asm__ __volatile__("mcr p15, 0, %0, c1, c0, 0" : : "r"(sysctl));

  // Initialize VIM module.
  // See TRM 15.8 "Registers".
  VIM.FBPARERR.raw = ArmToThumbTrampoline(InterruptVimParityHandler);
  VIM.PARFLG.raw = 0x01;  // Clear pending parity error.
  SYS.DEVCR1.raw = 0x0A;  // Use odd parity (default).
  VIM.PARCTL.raw = 0x0A;  // Enable VIM parity checking (0x0A recommended).
  for (int32_t i = 0; i < ARRAYSIZE(VIM.FIRQPR); ++i) {
    VIM.FIRQPR[i].raw = 0x0;             // Map interrupts to IRQ, not FIQ.
    VIM.INTREQ[i].raw = 0xFFFFFFFF;      // Clear pending interrupts.
    VIM.REQENACLR[i].raw = 0xFFFFFFFF;   // Disable interrupts.
    VIM.WAKEENACLR[i].raw = 0xFFFFFFFF;  // Disable wake-up interrupts.
  }

  // Initialize interrupt vector table to ensure parity bits are correct.
  for (int32_t i = 0; i < ARRAYSIZE(VIM_RAM.VECTOR); ++i) {
    // Initialize channel map to straight map (default).
    int32_t shift = (3 - (i % 4)) * 8;
    uint32_t mask = ~(0x0F << shift);
    VIM.CHANCTRL[i / 4].raw = (VIM.CHANCTRL[i / 4].raw & mask) | (i << shift);
    VIM_RAM.VECTOR[i].raw = ArmToThumbTrampoline(InterruptUnhandledError);
  }
}

void ShutdownVim(void) {
  // Disable interrupts.
  // See TRM 15.8 "Registers".
  VimDisableIrq();
  for (int32_t i = 0; i < ARRAYSIZE(VIM.FIRQPR); ++i) {
    VIM.REQENACLR[i].raw = 0xFFFFFFFF;   // Disable interrupts.
    VIM.WAKEENACLR[i].raw = 0xFFFFFFFF;  // Disable wake-up interrupts.
    VIM.FIRQPR[i].raw = 0x0;             // Map interrupts to IRQ, not FIQ.
    VIM.INTREQ[i].raw = 0xFFFFFFFF;      // Clear pending interrupts.
  }
}

void VimRegisterIrq(VimChannel ch, VimInterruptFunc func) {
  // VIM_RAM vector table contains a phantom vector at index 0, so offset by 1.
  // See Figure 15-7 "VIM Interrupt Address Memory Map".
  uint32_t reg = ch / 32;
  uint32_t mask = 1U << (ch % 32);
  uint32_t old_enable = VIM.REQENASET[reg].raw;  // Store previous enable.
  VIM.REQENACLR[reg].raw = mask;                 // Disable interrupt.
  VIM_RAM.VECTOR[1 + ch].raw = ArmToThumbTrampoline(func);
  VIM.FIRQPR[reg].raw &= ~mask;                  // Set IRQ priority.
  VIM.REQENASET[reg].raw = old_enable & mask;    // Restore interrupt enable.
}

void VimRegisterFiq(VimChannel ch, VimInterruptFunc func) {
  // VIM_RAM vector table contains a phantom vector at index 0, so offset by 1.
  // See Figure 15-7 "VIM Interrupt Address Memory Map".
  uint32_t reg = ch / 32;
  uint32_t mask = 1U << (ch % 32);
  uint32_t old_enable = VIM.REQENASET[reg].raw;  // Store previous enable.
  VIM.REQENACLR[reg].raw = mask;                 // Disable interrupt.
  VIM_RAM.VECTOR[1 + ch].raw = ArmToThumbTrampoline(func);
  VIM.FIRQPR[reg].raw |= mask;                   // Set FIQ priority.
  VIM.REQENASET[reg].raw = old_enable & mask;    // Restore interrupt enable.
}

void VimUnregister(VimChannel ch) {
  VimDisableInterrupt(ch);
  VimRegisterIrq(ch, InterruptUnhandledError);
}

void VimEnableInterrupt(VimChannel ch) {
  uint32_t reg = ch / 32;
  uint32_t mask = 1U << (ch % 32);
  VIM.INTREQ[reg].raw = mask;     // Clear pending interrupt.
  VIM.REQENASET[reg].raw = mask;  // Enable interrupt.
}

uint32_t VimDisableInterrupt(VimChannel ch) {
  uint32_t reg = ch / 32;
  uint32_t mask = 1U << (ch % 32);
  uint32_t old_enable = VIM.REQENASET[reg].raw;
  VIM.REQENACLR[reg].raw = mask;  // Disable interrupt.
  return old_enable;
}

void VimRestoreInterrupt(VimChannel ch, uint32_t old_enable) {
  uint32_t reg = ch / 32;
  uint32_t mask = 1U << (ch % 32);
  VIM.REQENASET[reg].raw = old_enable & mask;  // Restore interrupt enable.
}

void VimAccountingWrapper(VimInterruptFunc func, VimAccounting *account) {
  // Sample cycle counter.
  // [ARM Cortex-R4 TRM 6.3.7]
  uint32_t start, stop;
  __asm__ __volatile__("mrc p15, 0, %0, c9, c13, 0" : "=r"(start));
  func();
  __asm__ __volatile__("mrc p15, 0, %0, c9, c13, 0" : "=r"(stop));
  int32_t delta = (int32_t)(stop - start);
  if (account->max_cycles < delta) {
    account->max_cycles = delta;
  }
  account->total_cycles += delta;
  ++account->count;
}

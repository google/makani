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

/*
 * TMS570 interrupt vector table for applications.
 */

#include "avionics/firmware/cpu/registers_def.h"

    .section ".text.vectors", "xa"
    .syntax unified
    .arm

    .global vector_table
vector_table:
    ldr     pc, [pc,#24]  /* Reset handler. */
    ldr     pc, [pc,#24]  /* Undefined instruction handler. */
    ldr     pc, [pc,#24]  /* Service/SWI handler. */
    ldr     pc, [pc,#24]  /* Prefetch abort handler. */
    ldr     pc, [pc,#24]  /* Data abort handler. */
    .long   0             /* Reserved. */
    ldr     pc, [pc,#24]  /* IRQ handler. */
    ldr     pc, [pc,#24]  /* FIQ handler. */

    .long   App_ResetHandler
    .long   InterruptUndefInstructionHandler
    .long   InterruptSoftwareInterruptHandler
    .long   InterruptPrefetchAbortHandler
    .long   InterruptDataAbortHandler
    .long   0
    .long   InterruptIrqHandler
    .long   InterruptFiqHandler


    .section ".text.startup", "ax"
    .thumb_func
App_ResetHandler:
    /* Initialize the prefetch unit to a known state. This code ensures that
     * the lockstep cores have an identical prefetch state. Execute this code
     * before calling other functions. */
    bl      . + 4  /* Branch to next instruction and push PC. */
    bl      . + 4  /* Branch to next instruction and push PC. */
    bl      . + 4  /* Branch to next instruction and push PC. */
    bl      . + 4  /* Branch to next instruction and push PC. */

    /* Initialize lockstep cores to a known state. */
    bl      StartupCpuInit

    /* The Logic Built-In Self-Test (LBIST) resets the CPU core on completion
     * with ESR.CPURST=1. Detect this condition and resume the application
     * immediately. */
    ldr     r2, =STC_GSTAT_ADDR
    ldr     r0, [r2]
    tst     r0, #STC_GSTAT_TEST_DONE
    bne     normal_startup
    ldr     r2, =SYS_ESR_ADDR
    ldr     r0, [r2]
    cmp     r0, #SYS_ESR_CPURST  /* Check all reset reasons. */
    bne     normal_startup
    mov     r0, #SYS_ESR_CPURST  /* Clear CPURST. */
    str     r0, [r2]
    bl      StartupCpuPopContext
normal_startup:

    /* Initialize flash ECC. */
    bl      StartupFlashEnableEcc

    /* Select OSCIN while configuring PLLs (important for software reset). */
    bl      StartupClockDisableRticlk
    bl      StartupClockDisableVclka
    bl      StartupClockDisableVclk
    bl      StartupClockEnableOscIn
    bl      StartupClockWaitForOscIn
    bl      StartupClockSelectOscIn

    /* Stage PLL bring up. Start slow. */
    bl      StartupClockSetPll1Slow
    bl      StartupClockEnablePll1
    bl      StartupClockSetPll2Slow
    bl      StartupClockEnablePll2

    /* Perform other tasks while waiting for PLL stabilization. */
    bl      StartupClockEnableVclk
    bl      StartupPeripheralsEnableClocks
    bl      StartupLedInit
    bl      StartupFlashSetSpeed
    bl      StartupClockSetTrim
    bl      StartupClockSetRticlk
    bl      StartupClockWaitForPll1
    bl      StartupClockWaitForPll2
    bl      StartupClockEnableDomains

    /* Bring up PLLs. */
    bl      StartupClockSetPll1Fast
    bl      StartupClockSetPll2Fast
    bl      StartupClockSelectPll1
    bl      StartupClockEnableVclka
    bl      StartupClockEnableRticlk

    /* Initialize RAM (destructive). */
    bl      StartupRamDisablePbist
    bl      StartupRamEnableEcc
    bl      StartupRamInitialize

    /* Initialize context stack pointer after initializing RAM. */
    ldr     r0, =ldscript_context_stack
    ldr     r2, =ldscript_context_sp
    str     r0, [r2]

    /* Prepare to run general C code. */
    bl      StartupCopyData
    bl      StartupClearBss

    /* Initialize common driver code. */
    bl      StartupClockInit
    bl      StartupRtiInit
    bl      StartupVimInit

    /* Continue to main. */
    bl      Led0Off
    bl      Led1Off
    bl      main
main_returned:
    b       main_returned

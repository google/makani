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
 * TMS570 interrupt vector table for the bootloader.
 * These trampoline to the app vector table once a ldscript_boot_stage is set.
 */

#include "avionics/firmware/cpu/registers_def.h"

    .syntax unified

    BOOTING_FLAG = 0xBA  /* Magic value to switch vectors. */

    .section ".text.vectors", "xa"
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

    .long   Boot_ResetHandler
    .long   Boot_UndefInstructionHandler
    .long   Boot_SoftwareInterruptHandler
    .long   Boot_PrefetchAbortHandler
    .long   Boot_DataAbortHandler
    .long   0
    .long   Boot_IrqHandler
    .long   Boot_FiqHandler


    .section ".text.startup", "ax"
    .thumb_func
Boot_ResetHandler:
    /* See "Initialization of Hercules™ ARM® Cortex™-r4F Microcontrollers",
     * TI document number SPNA106D. */

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
     * with ESR.CPURST=1. We assume if ESR.PORST=0 and ESR.SWRST=0, then the
     * application requested the LBIST. We therefore jump immediately to the
     * application. This assumption eliminates potential versioning issues
     * between the application's context store and the boot loader's context
     * restore. */
    ldr     r2, =STC_GSTAT_ADDR
    ldr     r0, [r2]
    tst     r0, #STC_GSTAT_TEST_DONE
    bne     normal_startup
    ldr     r2, =SYS_ESR_ADDR
    ldr     r0, [r2]
    cmp     r0, #SYS_ESR_CPURST  /* Check all reset reasons. */
    bne     normal_startup
    ldr     r0, =ldscript_app_flash_begin
    bx      r0
normal_startup:

    /* Initialize flash ECC. */
    bl      StartupFlashEnableEcc

    /* Check for critical hardware failures. */
    bl      StartupEsmCheckGroup3

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
    bl      StartupEFuseStartSelfTest
    bl      StartupClockEnableVclk
    bl      StartupPeripheralsEnableClocks
    bl      StartupLedInit
    bl      Led0On
    bl      StartupEFuseWaitForSelfTestOrDie
    bl      StartupFlashSetSpeed
    bl      Led0Off
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

    /* Run CPU self tests. Do not run in Debug Mode (e.g., via JTAG in CCS). */
    bl      StartupCpuRunAndResumeSelfTestDiagOrDie  /* CPU reset. */
    bl      Led0On
    bl      StartupCpuRunAndResumeSelfTestOrDie  /* CPU reset. */
    bl      StartupCpuStartCompareSelfTest
    bl      StartupCpuWaitForCompareSelfTestOrDie
    bl      StartupCpuRunCompareForceErrorTestOrDie
    bl      StartupCpuRunCompareSelfTestForceErrorOrDie
    bl      Led0Off

    /* Run RAM self tests (destructive). */
    /* The PBIST self test accesses flash as if it were RAM to produce a self
     * test failure, and thus creates a ESM FMC uncorrectable ECC error
     * (group 3). We therefore disable flash ECC prior to the test and
     * reenable afterwards. */
    bl      StartupRamEnablePbist
    bl      StartupFlashDisableEcc
    bl      StartupRamRunPbistSelfTestOrDie
    bl      StartupFlashEnableEcc
    bl      StartupRamEnablePbist
    bl      StartupRamRunPbistOnRomOrDie
    bl      StartupRamEnablePbist
    bl      StartupRamRunPbistOnAllOrDie
    bl      StartupRamDisablePbist

    /* Initialize RAM (destructive). */
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

    /* Set ldscript_boot_stage to BOOTING. This flag indicates we're in the
     * boot loader and our exception/interrupt trampolines should not jump to
     * the application vectors. */
    ldr     r0, =ldscript_boot_stage
    mov     r1, #BOOTING_FLAG
    str     r1, [r0]

    /* Continue to main. */
    bl      Led0On
    bl      Led1On
    bl      main
main_returned:
    b       main_returned


/* For each vector, this macro chooses between the bootloader's handler and
 * the app's handler, depending on the state of the ldscript_boot_stage
 * flag. */
.macro MAYBE_TRAMPOLINE boot_handler app_vector
    push    {r0}
    ldr     r0, =ldscript_boot_stage
    ldr     r0, [R0]
    cmp     r0, #BOOTING_FLAG
    pop     {r0}
    bne     1f
    .align 2  /* Align instruction so the relative load address is aligned. */
    ldr     pc, [pc,#4]
1:
    .align 2  /* Align just in case previous instruction wasn't 4 bytes. */
    ldr     pc, [pc,#4]
    .align 2  /* Align just in case previous instruction wasn't 4 bytes. */
    .long   \boot_handler
    .long   \app_vector
.endm


    .section ".text.vectors", "ax"
    .thumb_func
Boot_UndefInstructionHandler:
    MAYBE_TRAMPOLINE InterruptUndefInstructionHandler, \
        ldscript_app_flash_begin + 4


    .section ".text.vectors", "ax"
    .thumb_func
Boot_SoftwareInterruptHandler:
    MAYBE_TRAMPOLINE InterruptSoftwareInterruptHandler, \
        ldscript_app_flash_begin + 8


    .section ".text.vectors", "ax"
    .thumb_func
Boot_PrefetchAbortHandler:
    MAYBE_TRAMPOLINE InterruptPrefetchAbortHandler, \
        ldscript_app_flash_begin + 12


    .section ".text.vectors", "ax"
    .thumb_func
Boot_DataAbortHandler:
    MAYBE_TRAMPOLINE InterruptDataAbortHandler, \
        ldscript_app_flash_begin + 16


    .section ".text.vectors", "ax"
    .thumb_func
Boot_IrqHandler:
    /* Use VIM module to handle trampoline. */
    push {r0-r4,ip,lr}
    /* Save FPU scratch registers (d0-d7 overlap s0-s15). */
    vmrs    r4, fpscr
    fstmdbs sp!, {s0-s15}
    /* Branch to function pointer contained in IRQVECREG. */
    ldr     r0, =VIM_IRQVECREG_ADDR
    ldr     r1, [r0]
    blx     r1
    /* Restore FPU scratch registers (d0-d7 overlap s0-s15). */
    fldmias sp!, {s0-s15}
    vmsr    fpscr, r4
    /* Return. */
    pop {r0-r4,ip,lr}
    subs    pc, lr, #4


    .section ".text.vectors", "ax"
    .thumb_func
Boot_FiqHandler:
    /* Use VIM module to handle trampoline. */
    push {r0-r4,ip,lr}
    /* Save FPU scratch registers (d0-d7 overlap s0-s15). */
    vmrs    r4, fpscr
    fstmdbs sp!, {s0-s15}
    /* Branch to function pointer contained in FIQVECREG. */
    ldr     r0, =VIM_FIQVECREG_ADDR
    ldr     r1, [r0]
    blx     r1
    /* Restore FPU scratch registers (d0-d7 overlap s0-s15). */
    fldmias sp!, {s0-s15}
    vmsr    fpscr, r4
    /* Return. */
    pop {r0-r4,ip,lr}
    subs    pc, lr, #4

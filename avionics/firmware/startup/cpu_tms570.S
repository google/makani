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

/* Initial processor startup for the TMS570 family. */

#include "avionics/firmware/cpu/registers_def.h"
#include "avionics/firmware/startup/return_codes.h"

    .section ".text.startup", "xa"
    .syntax unified

    /* Current Program Status Register (CPSR) bits. */
    MODE_BITS        = 0x1F   /* Bit mask for mode bits in cpsr. */
    USR_MODE         = 0x10   /* User mode. */
    FIQ_MODE         = 0x11   /* Fast Interrupt Request mode. */
    IRQ_MODE         = 0x12   /* Interrupt Request mode. */
    SVC_MODE         = 0x13   /* Supervisor mode. */
    ABT_MODE         = 0x17   /* Abort mode. */
    UND_MODE         = 0x1B   /* Undefined Instruction mode. */
    SYS_MODE         = 0x1F   /* System mode. */
    THUMB_BIT        = 0x20   /* Thumb mode. */
    FIQ_BIT          = 0x40   /* Mask FIQ interrupts. */
    IRQ_BIT          = 0x80   /* Mask normal interrupts. */
    ABORT_BIT        = 0x100  /* Mask asynchronous aborts. */
    BIG_ENDIAN_BIT   = 0x200  /* Big-endian mode. */

    /* CPU self test controller register settings. */
    STC_SCSCR_DIAG_BITS    = (STC_SCSCR_FAULT_INS \
                              | 0x0A << STC_SCSCR_SELF_CHECK_KEY_SHIFT)

    /* ESM failure bits. See TMS570LS1227 Table 4-36. */
    ESM_SR1_CCMR4F_ST_BIT  = 1 << 31
    ESM_SR1_CPU_ST_BIT     = 1 << 27
    ESM_SR2_CCMR4F_CMP_BIT = 1 << 2

    /* CCM-R4F MKEY values. See TMS570 TRM 9.4.2. */
    CCM_KEYR_MKEY_LOCKSTEP                = 0x0
    CCM_KEYR_MKEY_SELF_TEST               = 0x6
    CCM_KEYR_MKEY_ERROR_FORCING           = 0x9
    CCM_KEYR_MKEY_SELF_TEST_ERROR_FORCING = 0xF


    .global StartupCpuInit
    .thumb_func
StartupCpuInit:
    /* Initialize all common registers to known values, so that lockstep cores
     * have an identical state. */
    mov     r0, #0
    mov     r1, #0
    mov     r2, #0
    mov     r3, #0
    mov     r4, #0
    mov     r5, #0
    mov     r6, #0
    mov     r7, #0
    mov     r8, #0
    /* r9 through r14 have per-mode values. */
    /* r15 = PC (Program Counter). */

    /* Save Link Register in r3 to return to calling function (we haven't
     * initialized the stack yet). */
    mov     r3, lr

    /* Initialize FIQ mode. */
    mov     r0, #(THUMB_BIT + FIQ_BIT + IRQ_BIT + ABORT_BIT + BIG_ENDIAN_BIT)
    orr     r1, r0, #FIQ_MODE
    msr     cpsr_cxsf, r1
    msr     spsr_cxsf, r1
    mov     r8, #0
    mov     r9, #0
    mov     r10, #0
    mov     r11, #0
    mov     r12, #0
    ldr     sp, =ldscript_fiq_stack  /* r13 = sp (Stack Pointer). */
    mov     lr, #0  /* r14 = lr (Link Register). */

    /* Initialize IRQ mode. */
    orr     r1, r0, #IRQ_MODE
    msr     cpsr_cxsf, r1
    msr     spsr_cxsf, r1
    ldr     sp, =ldscript_irq_stack  /* r13 = sp (Stack Pointer). */
    mov     lr, #0  /* r14 = lr (Link Register). */

    /* Initialize Abort mode. */
    orr     r1, r0, #ABT_MODE
    msr     cpsr_cxsf, r1
    msr     spsr_cxsf, r1
    ldr     sp, =ldscript_abt_stack  /* r13 = sp (Stack Pointer). */
    mov     lr, #0  /* r14 = lr (Link Register). */

    /* Initialize Undefined mode */
    orr     r1, r0, #UND_MODE
    msr     cpsr_cxsf, r1
    msr     spsr_cxsf, r1
    ldr     sp, =ldscript_und_stack  /* r13 = sp (Stack Pointer). */
    mov     lr, #0  /* r14 = lr (Link Register). */

    /* Initialize Service mode. */
    orr     r1, r0, #SVC_MODE
    msr     cpsr_cxsf, r1
    msr     spsr_cxsf, r1
    ldr     sp, =ldscript_svc_stack  /* r13 = sp (Stack Pointer). */
    mov     lr, #0  /* r14 = lr (Link Register). */

    /* Initialize System mode (and equivalently, User mode). */
    orr     r1, r0, #SYS_MODE
    msr     cpsr_cxsf, r1
    msr     spsr_cxsf, r1
    mov     r9, #0
    mov     r10, #0
    mov     r11, #0
    mov     r12, #0
    ldr     sp, =ldscript_main_stack  /* r13 = sp (Stack Pointer). */
    mov     lr, #0  /* r14 = lr (Link Register). */

    /* Initialize prefetch unit call stack. */
    bl      . + 4  /* Branch to next instruction and push PC. */
    bl      . + 4  /* Branch to next instruction and push PC. */
    bl      . + 4  /* Branch to next instruction and push PC. */
    bl      . + 4  /* Branch to next instruction and push PC. */
    ldr     sp, =ldscript_main_stack

    /* Enable FPU. */
    mrc     p15, 0, r0, c1, c0, 2  /* Read coprocessor access register bit. */
    mov     r1, #0x00F00000  /* Enable coprocessors cp10 & cp11. */
    orr     r0, r0, r1
    mcr     p15, 0, r0, c1, c0, 2
    mov     r0, #0x40000000  /* Set FPEXC.EN (bit 30) to enable Neon/VFP. */
    fmxr    fpexc, r0

    /* Initialize FPU registers. */
    vmov    D0, r9, r9
    vmov    D1, r9, r9
    vmov    D2, r9, r9
    vmov    D3, r9, r9
    vmov    D4, r9, r9
    vmov    D5, r9, r9
    vmov    D6, r9, r9
    vmov    D7, r9, r9
    vmov    D8, r9, r9
    vmov    D9, r9, r9
    vmov    D10, r9, r9
    vmov    D11, r9, r9
    vmov    D12, r9, r9
    vmov    D13, r9, r9
    vmov    D14, r9, r9
    vmov    D15, r9, r9

    /* Return to Boot/App_ResetHandler. */
    bx      r3


    /* This function stores the CPU context without building a stack, such
     * that we can call this function repeatedly, and then only restore when
     * necessary (e.g., for unit testing). */
    .global StartupCpuSaveContext
    .thumb_func
StartupCpuSaveContext:
    /* Re-initialize stack pointer. */
    ldr     r0, =ldscript_context_stack
    ldr     r2, =ldscript_context_sp
    str     r0, [r2]
    b       StartupCpuPushContext


    /* Call this function after StartupCpuSaveContext to restore from
     * last saved context. */
    .global StartupCpuRestoreContext
    .thumb_func
StartupCpuRestoreContext:
    b       StartupCpuPopContext


    /* This function stores the CPU context in the reverse order of
     * StartupCpuPopContext such that StartupCpuPopContext can restore
     * the CPU context in a desirable order. */
    .global StartupCpuPushContext
    .thumb_func
StartupCpuPushContext:
    /* Increment context stack pointer in R3. */
    ldr     r2, =ldscript_context_sp
    ldr     r3, [r2]

    /* Set restore return value. */
    mov     r0, #1

    /* (11) Push all standard registers for the current mode. */
    mov     r1, sp
    stmdb   r3!, {r0,r1,r4-r12,lr}

    /* (10) Store current mode. */
    push    {r4,r5,lr}
    mrs     r4, cpsr
    mrs     r5, spsr
    stmdb   r3!, {r4-r5}

    /* (9) Switch to System mode. */
    mov     r0, #(THUMB_BIT + FIQ_BIT + IRQ_BIT + ABORT_BIT + BIG_ENDIAN_BIT)
    orr     r1, r0, #SYS_MODE
    msr     cpsr_cxsf, r1

    /* (8) Push coprocessor registers. */
    ldr     r2, =ldscript_context_sp
    str     r3, [r2]
    bl      StartupCpuPushCp15Registers
    bl      StartupCpuPushFpuRegisters
    ldr     r2, =ldscript_context_sp
    ldr     r3, [r2]

    /* (7) Push FIQ mode registers. */
    mov     r0, #(THUMB_BIT + FIQ_BIT + IRQ_BIT + ABORT_BIT + BIG_ENDIAN_BIT)
    orr     r1, r0, #FIQ_MODE
    msr     cpsr_cxsf, r1
    stmdb   r3!, {r8-r12}
    mrs     r1, spsr
    mov     r2, r13
    stmdb   r3!, {r1,r2,r14}

    /* (6) Push IRQ mode registers. */
    orr     r1, r0, #IRQ_MODE
    msr     cpsr_cxsf, r1
    mrs     r1, spsr
    mov     r2, r13
    stmdb   r3!, {r1,r2,r14}

    /* (5) Push Abort mode registers. */
    orr     r1, r0, #ABT_MODE
    msr     cpsr_cxsf, r1
    mrs     r1, spsr
    mov     r2, r13
    stmdb   r3!, {r1,r2,r14}

    /* (4) Push Undefined mode registers. */
    orr     r1, r0, #UND_MODE
    msr     cpsr_cxsf, r1
    mrs     r1, spsr
    mov     r2, r13
    stmdb   r3!, {r1,r2,r14}

    /* (3) Push Service mode registers. */
    orr     r1, r0, #SVC_MODE
    msr     cpsr_cxsf, r1
    mrs     r1, spsr
    mov     r2, r13
    stmdb   r3!, {r1,r2,r14}

    /* (2) Push System mode registers. */
    orr     r1, r0, #SYS_MODE
    msr     cpsr_cxsf, r1
    mrs     r1, spsr
    mov     r2, r13
    stmdb   r3!, {r1,r2,r9-r12,r14}

    /* (1) Save context stack pointer to known location in RAM. */
    ldr     r2, =ldscript_context_sp
    str     r3, [r2]

    /* Restore previous mode. */
    msr     cpsr_cxsf, r4
    msr     spsr_cxsf, r5
    pop     {r4,r5,lr}

    /* Set save return value. */
    mov     r0, #0
    bx      lr


    /* Pop CPU registers. */
    .global StartupCpuPopContext
    .thumb_func
StartupCpuPopContext:
    /* (1) Restore context stack pointer. Increment in R3. */
    ldr     r2, =ldscript_context_sp
    ldr     r3, [r2]

    /* Validate stack pointer. The stack pointer should be between
     * ldscript_context_stack_top <= sp < ldscript_context_stack. */
    ldr     r0, =RETURN_FAIL_CPU_CONTEXT_FAILURE
    ldr     r2, =ldscript_context_stack_top
    cmp     r2, r3
    bgt     StartupEmitFailureCodeInR0AndDie
    ldr     r2, =ldscript_context_stack
    cmp     r2, r3
    ble     StartupEmitFailureCodeInR0AndDie

    /* (2) Pop System mode registers. */
    mov     r0, #(THUMB_BIT + FIQ_BIT + IRQ_BIT + ABORT_BIT + BIG_ENDIAN_BIT)
    orr     r1, r0, #SYS_MODE
    ldmia   r3!, {r1,r2,r9-r12,r14}
    msr     spsr_cxsf, r1
    mov     r13, r2

    /* (3) Pop Service mode registers. */
    orr     r1, r0, #SVC_MODE
    msr     cpsr_cxsf, r1
    ldmia   r3!, {r1,r2,r14}
    msr     spsr_cxsf, r1
    mov     r13, r2

    /* (4) Pop Undefined mode registers. */
    orr     r1, r0, #UND_MODE
    msr     cpsr_cxsf, r1
    ldmia   r3!, {r1,r2,r14}
    msr     spsr_cxsf, r1
    mov     r13, r2

    /* (5) Pop Abort mode registers. */
    orr     r1, r0, #ABT_MODE
    msr     cpsr_cxsf, r1
    ldmia   r3!, {r1,r2,r14}
    msr     spsr_cxsf, r1
    mov     r13, r2

    /* (6) Pop IRQ mode registers. */
    orr     r1, r0, #IRQ_MODE
    msr     cpsr_cxsf, r1
    ldmia   r3!, {r1,r2,r14}
    msr     spsr_cxsf, r1
    mov     r13, r2

    /* (7) Pop FIQ mode registers. */
    orr     r1, r0, #FIQ_MODE
    msr     cpsr_cxsf, r1
    ldmia   r3!, {r1,r2,r14}
    msr     spsr_cxsf, r1
    mov     r13, r2
    ldmia   r3!, {r8-r12}

    /* (8) Switch to System mode. */
    orr     r1, r0, #SYS_MODE
    msr     cpsr_cxsf, r1

    /* (9) Pop coprocessor registers. */
    ldr     r2, =ldscript_context_sp
    str     r3, [r2]
    bl      StartupCpuPopFpuRegisters
    bl      StartupCpuPopCp15Registers
    ldr     r2, =ldscript_context_sp
    ldr     r3, [r2]

    /* (10) Pop current mode. */
    ldmia   r3!, {r0-r1}
    msr     cpsr_cxsf, r0  /* Potentially enables interrupts. */
    msr     spsr_cxsf, r1

    /* (11) Pop all standard registers for the current mode. */
    ldmia   r3!, {r0,r1,r4-r12,lr}
    mov     sp, r1

    /* Save context stack pointer. */
    ldr     r2, =ldscript_context_sp
    str     r3, [r2]
    dmb
    bx      lr


    /* This function stores the CP15 System Control Coprocessor registers in
     * reverse order of StartupCpuPopCp15Registers such that
     * StartupCpuPopCp15Registers can restore the registers in the desired
     * order. */
    .global StartupCpuPushCp15Registers
    .thumb_func
StartupCpuPushCp15Registers:
    /* Decrement context stack pointer in R3. */
    ldr     r2, =ldscript_context_sp
    ldr     r3, [r2]

    /* See Cortex-R4/R4F TRM 4.1.1 "System identification control and
     * configuration". */

    /* (4) Push c1 registers. */
    mrc     p15, 0, r0, c1, c0, 0  /* System Control Register. */
    mrc     p15, 0, r1, c1, c0, 1  /* Auxiliary Control Register. */
    mrc     p15, 0, r2, c1, c0, 2  /* Coprocessor Access Register. */
    stmdb   r3!, {r0-r2}

    /* (3) Push c13, c15 registers. */
    mrc     p15, 0, r0, c13, c0, 1  /* Context ID register. */
    mrc     p15, 0, r1, c15, c0, 0  /* Secondary Auxiliary Control Register. */
    stmdb   r3!, {r0-r1}

    /* See Cortex-R4/R4F TRM 4.1.2 "MPU control and configuration". */
    /* TODO: Implement save/restore. */

    /* See Cortex-R4/R4F TRM 4.1.4 "Interface control and configuration". */

    /* (2) Push c0 registers. */
    mrc     p15, 0, r0, c9, c1, 0   /* BTCM Region Register. */
    mrc     p15, 0, r1, c9, c1, 1   /* ATCM Region Register. */
    mrc     p15, 0, r2, c11, c0, 0  /* Slave Port Control Register. */
    stmdb   r3!, {r0-r2}

    /* See Cortex-R4/R4F TRM 4.1.5 "System performance monitor". */

    /* (1) Push c9 registers. */
    mrc     p15, 0, r0, c9, c14, 0  /* User Enable Register. */
    mrc     p15, 0, r1, c9, c14, 1  /* Interrupt Enable Set Register. */
    mrc     p15, 0, r2, c9, c12, 1  /* Count Enable Set Register. */
    stmdb   r3!, {r0-r2}
    mrc     p15, 0, r0, c9, c13, 0  /* Cycle Count Register. */
    mrc     p15, 0, r1, c9, c13, 2  /* Performance Count Register. */
    stmdb   r3!, {r0-r1}
    mrc     p15, 0, r0, c9, c12, 0  /* Performance Monitor Control Register. */
    mrc     p15, 0, r1, c9, c12, 5  /* Performance Counter Selection Reg. */
    mrc     p15, 0, r2, c9, c13, 1  /* Event Select Register. */
    stmdb   r3!, {r0-r2}

    /* Save context stack pointer. */
    ldr     r2, =ldscript_context_sp
    str     r3, [r2]
    bx      lr


    /* Pop CP15 System Control Coprocessor registers. */
    .global StartupCpuPopCp15Registers
    .thumb_func
StartupCpuPopCp15Registers:
    /* Increment context stack pointer in R3. */
    ldr     r2, =ldscript_context_sp
    ldr     r3, [r2]

    /* See Cortex-R4/R4F TRM 4.1.5 "System performance monitor". */
    /* Clear all enables first. */
    ldr     r0, =0xFFFFFFFF
    mrc     p15, 0, r0, c9, c12, 2  /* Count Enable Clear Register. */
    mrc     p15, 0, r0, c9, c14, 2  /* Interrupt Enable Clear Register. */

    /* (1) Pop c9 registers. */
    ldmia   r3!, {r0-r2}
    mcr     p15, 0, r0, c9, c12, 0  /* Performance Monitor Control Register. */
    mcr     p15, 0, r1, c9, c12, 5  /* Performance Counter Selection Reg. */
    mcr     p15, 0, r2, c9, c13, 1  /* Event Select Register. */
    ldmia   r3!, {r0-r1}
    mcr     p15, 0, r0, c9, c13, 0  /* Cycle Count Register. */
    mcr     p15, 0, r1, c9, c13, 2  /* Performance Count Register. */
    ldmia   r3!, {r0-r2}
    mcr     p15, 0, r0, c9, c14, 0  /* User Enable Register. */
    mcr     p15, 0, r1, c9, c14, 1  /* Interrupt Enable Set Register. */
    mcr     p15, 0, r2, c9, c12, 1  /* Count Enable Set Register. */

    /* See Cortex-R4/R4F TRM 4.1.4 "Interface control and configuration". */

    /* (2) Pop c0 registers. */
    ldmia   r3!, {r0-r2}
    mcr     p15, 0, r0, c9, c1, 0   /* BTCM Region Register. */
    mcr     p15, 0, r1, c9, c1, 1   /* ATCM Region Register. */
    mcr     p15, 0, r2, c11, c0, 0  /* Slave Port Control Register. */

    /* See Cortex-R4/R4F TRM 4.1.2 "MPU control and configuration". */
    /* TODO: Implement save/restore. */

    /* See Cortex-R4/R4F TRM 4.1.1 "System identification control and
     * configuration". */

    /* (3) Pop c13, c15 registers. */
    ldmia   r3!, {r0-r1}
    mcr     p15, 0, r0, c13, c0, 1  /* Context ID register. */
    mcr     p15, 0, r1, c15, c0, 0  /* Secondary Auxiliary Control Register. */

    /* (4) Pop c1 registers. */
    ldmia   r3!, {r0-r2}
    mcr     p15, 0, r0, c1, c0, 0  /* System Control Register. */
    mcr     p15, 0, r1, c1, c0, 1  /* Auxiliary Control Register. */
    mcr     p15, 0, r2, c1, c0, 2  /* Coprocessor Access Register. */

    /* Save context stack pointer. */
    ldr     r2, =ldscript_context_sp
    str     r3, [r2]
    dmb
    bx      lr


    /* This function stores the FPU registers in reverse order of
     * StartupCpuPopFpuRegisters such that StartupCpuPopFpuRegisters can
     * restore the registers in the desired order. */
    .global StartupCpuPushFpuRegisters
    .thumb_func
StartupCpuPushFpuRegisters:
    /* Decrement context stack pointer in R3. */
    ldr     r2, =ldscript_context_sp
    ldr     r3, [r2]

    /* (2) Push FPU computation registers. */
    fstmdbs r3!, {s0-s31}

    /* (1) Push FPU control registers. */
    mrc     p15, 0, r0, c1, c0, 2  /* Coprocessor Access Control Register. */
    fmrx    r1, fpscr  /* Floating-point System and Control Register. */
    fmrx    r2, fpexc  /* Floating-point Exception Register. */
    stmdb   r3!, {r0-r2}

    /* Save context stack pointer. */
    ldr     r2, =ldscript_context_sp
    str     r3, [r2]
    bx      lr


    /* Pop FPU registers. */
    .global StartupCpuPopFpuRegisters
    .thumb_func
StartupCpuPopFpuRegisters:
    /* Increment context stack pointer in R3. */
    ldr     r2, =ldscript_context_sp
    ldr     r3, [r2]

    /* (1) Pop FPU control registers. */
    ldmia   r3!, {r0-r2}
    mcr     p15, 0, r0, c1, c0, 2  /* Coprocessor Access Control Register. */
    fmxr    fpscr, r1  /* Floating-point System and Control Register. */
    fmxr    fpexc, r2  /* Floating-point Exception Register. */

    /* (2) Pop FPU computation registers. */
    fldmias r3!, {s0-s31}

    /* Save context stack pointer. */
    ldr     r2, =ldscript_context_sp
    str     r3, [r2]
    dmb
    bx      lr


    /* This function handles execution and resume of the CPU self-test
     * diagnostic routine. The first execution of this function runs the
     * CPU LBIST diagnostic test. Upon completion of the self-test, the CPU
     * reboots with ESR.CPURST=1. This flag indicates completion of the
     * self-test and we should check the results. Since the standard CPU
     * self-test (non-diagnostic mode) also reboots with ESR.CPURST=1, we
     * check SCSCR for the diagnostic settings. If both conditions are
     * true, we evaluate the results here. */
    .global StartupCpuRunAndResumeSelfTestDiagOrDie
    .thumb_func
StartupCpuRunAndResumeSelfTestDiagOrDie:
    /* See TMS570 TRM 8.5.1 "Example 1: Self Test Run for 24 Interval". */
    /* If SYSESR.CPURST not set, then run LBIST diagnostic. */
    ldr     r2, =SYS_ESR_ADDR
    ldr     r0, [r2]
    tst     r0, #SYS_ESR_CPURST
    beq     StartupCpuRunSelfTestDiag
    /* Check if we're resuming the LBIST diagnostic test. */
    ldr     r2, =STC_SCSCR_ADDR
    ldr     r1, [r2]
    ldr     r0, =STC_SCSCR_DIAG_BITS
    cmp     r0, r1
    beq     lbist_diag_resume
    mov     r0, #RETURN_SUCCESS  /* Not for us. */
    bx      lr
lbist_diag_resume:
    /* Check completion status of LBIST. See TMS570 TRM Table 8-9. */
    ldr     r2, =STC_GSTAT_ADDR
    ldr     r0, [r2]
    tst     r0, #STC_GSTAT_TEST_DONE  /* True when complete. */
    beq     lbist_resume
    tst     r0, #STC_GSTAT_TEST_FAIL  /* True upon failure (expected!). */
    beq     lbist_failure
    /* Check ESM CPU self test error. */
    ldr     r2, =ESM_SR1_ADDR
    ldr     r0, [r2]
    tst     r0, #ESM_SR1_CPU_ST_BIT  /* Error expected, should be true. */
    beq     lbist_esm_failure
    mov     r0, #ESM_SR1_CPU_ST_BIT  /* Clear. */
    str     r0, [r2]
    /* Clear SYSESR.CPURST bit for next test. */
    ldr     r2, =SYS_ESR_ADDR
    mov     r0, #SYS_ESR_CPURST
    str     r0, [r2]
    mov     r0, #RETURN_SUCCESS  /* Test complete and successful. */
    bx      lr


    /* Execute CPU self-test diagnostic routine. The CPU reboots upon
     * completion with ESR.CPURST=1. */
    .global StartupCpuRunSelfTestDiag
    .thumb_func
StartupCpuRunSelfTestDiag:
    /* Clear ESM CPU self test error. */
    ldr     r2, =ESM_SR1_ADDR
    mov     r0, #ESM_SR1_CPU_ST_BIT
    str     r0, [r2]
    /* Enable self test fault injection. */
    ldr     r2, =STC_SCSCR_ADDR
    ldr     r0, =STC_SCSCR_DIAG_BITS
    str     r0, [r2]
    /* Configure interval count (up to 24 supported). */
    ldr     r2, =STC_GCR0_ADDR
    /* 1 interval for diagnostic test. */
    ldr     r0, =1 << STC_GCR0_INTCOUNT_SHIFT | STC_GCR0_RS_CNT
    str     r0, [r2]
    b       lbist_start


    /* This function handles execution and resume of the CPU self-test
     * routine. The first execution of this function runs the CPU LBIST
     * test. Upon completion of the self-test, the CPU reboots with
     * ESR.CPURST=1. This flag indicates completion of the self-test and
     * we should check the results. Since the diagnostic CPU self-test also
     * reboots with ESR.CPURST=1, we check SCSCR for the diagnostic settings
     * unset. If both conditions are true, we evaluate the results here. */
    .global StartupCpuRunAndResumeSelfTestOrDie
    .thumb_func
StartupCpuRunAndResumeSelfTestOrDie:
    /* See TMS570 TRM 8.5.1 "Example 1: Self Test Run for 24 Interval". */
    /* If SYSESR.CPURST not set, then run LBIST. */
    ldr     r2, =SYS_ESR_ADDR
    ldr     r0, [r2]
    tst     r0, #SYS_ESR_CPURST
    beq     StartupCpuRunSelfTest
    /* Check if we're resuming the LBIST diagnostic test. */
    ldr     r2, =STC_SCSCR_ADDR
    ldr     r1, [r2]
    ldr     r0, =STC_SCSCR_DIAG_BITS
    cmp     r0, r1
    bne     lbist_full_resume
    mov     r0, #RETURN_SUCCESS  /* Not for us. */
    bx      lr
lbist_full_resume:
    /* Check completion status of LBIST. See TMS570 TRM Table 8-9. */
    ldr     r2, =STC_GSTAT_ADDR
    ldr     r0, [r2]
    tst     r0, #STC_GSTAT_TEST_DONE  /* True when complete. */
    beq     lbist_resume
    tst     r0, #STC_GSTAT_TEST_FAIL  /* True upon failure (not expected). */
    bne     lbist_failure
    /* Clear SYSESR.CPURST bit for next test. */
    ldr     r2, =SYS_ESR_ADDR
    mov     r0, #SYS_ESR_CPURST
    str     r0, [r2]
    mov     r0, #RETURN_SUCCESS  /* Test complete and successful. */
    bx      lr


    /* Execute CPU self-test routine. The CPU reboots upon completion with
     * ESR.CPURST=1. */
    .global StartupCpuRunSelfTest
    .thumb_func
StartupCpuRunSelfTest:
    /* Disable self test fault injection. */
    ldr     r2, =STC_SCSCR_ADDR
    ldr     r0, =0x05 << STC_SCSCR_SELF_CHECK_KEY_SHIFT
    str     r0, [r2]
    /* Configure interval count (up to 24 supported). */
    ldr     r2, =STC_GCR0_ADDR
    ldr     r0, =24 << STC_GCR0_INTCOUNT_SHIFT | STC_GCR0_RS_CNT
    str     r0, [r2]
    b       lbist_start


    /* Common code for StartupCpuRunSelfTestDiag and StartupCpuRunSelfTest. */
lbist_start:
    /* Disable IRQ interrupts. */
    mrs     r0, cpsr
    orr     r0, #IRQ_BIT  /* Set I bit to disable IRQs. */
    msr     cpsr_c, r0
    /* Clear SYSESR.CPURST. */
    ldr     r2, =SYS_ESR_ADDR
    mov     r0, #SYS_ESR_CPURST
    str     r0, [r2]
    /* Enable STC clock divider: f_STCLK = f_HCLK / prescale (90 MHz max). */
    ldr     r2, =SYS2_STCLKDIV_ADDR
    ldr     r0, =(2 - 1) << SYS2_STCLKDIV_CLKDIV_SHIFT
    str     r0, [r2]
    /* Configure timeout (VCLK cycles). */
    ldr     r2, =STC_TPR_ADDR
    ldr     r0, =STC_TPR_RTOD_MASK
    str     r0, [r2]
    dmb
lbist_resume:
    /* Enable. */
    ldr     r2, =STC_GCR1_ADDR
    mov     r0, #0x0A << STC_GCR1_STC_ENA_SHIFT
    str     r0, [r2]
    dsb
    isb
lbist_wait:
    /* Enter idle mode to start self test. CPU should reset upon completion. */
    wfi
    b       lbist_wait
lbist_failure:
    ldr     r0, =RETURN_FAIL_CPU_LBIST
    b       StartupEmitFailureCodeInR0AndDie
lbist_esm_failure:
    ldr     r0, =RETURN_FAIL_CPU_LBIST_ESM
    b       StartupEmitFailureCodeInR0AndDie


    /* This function starts the CPU lockstep compare test. During this test,
     * the CPU compare logic will not check the CPU signals. Call function
     * StartupCpuWaitForCompareSelfTest to check the results. */
    /* See TMS570 TRM 9.3.2 "Self Test Mode". */
    .global StartupCpuStartCompareSelfTest
    .thumb_func
StartupCpuStartCompareSelfTest:
    /* Switch to lock step mode. */
    ldr     r2, =CCM_KEYR_ADDR
    mov     r0, #CCM_KEYR_MKEY_LOCKSTEP << CCM_KEYR_MKEY_SHIFT
    str     r0, [r2]
    dmb
    /* Clear status registers. */
    ldr     r2, =CCM_SR_ADDR
    mov     r0, #0xFFFFFFFF
    str     r0, [r2]
    dmb
    /* Switch to self test mode. */
    ldr     r2, =CCM_KEYR_ADDR
    mov     r0, #CCM_KEYR_MKEY_SELF_TEST << CCM_KEYR_MKEY_SHIFT
    str     r0, [r2]
    dmb
    /* Self test takes 3615 cycles to complete. We can perform other tasks
     * while in progress. Compare logic will not be checking CPU signals. */
    bx      lr


    .global StartupCpuWaitForCompareSelfTestOrDie
    .thumb_func
StartupCpuWaitForCompareSelfTestOrDie:
    mov     r3, lr  /* No push; code runs before RAM initialization. */
    bl      StartupCpuWaitForCompareSelfTest  /* Stores failure code in R0. */
    cmp     r0, #0
    bne     StartupEmitFailureCodeInR0AndDie
    bx      r3


    /* This function blocks until the completion of the CPU lockstep compare
     * test. Call StartupCpuStartCompareSelfTest prior to calling this function.
     *
     * Returns failure code, or zero upon success, in R0.
     */
    .global StartupCpuWaitForCompareSelfTest
    .thumb_func
StartupCpuWaitForCompareSelfTest:
    ldr     r2, =CCM_SR_ADDR
    ldr     r0, [r2]
    tst     r0, #CCM_SR_STC  /* Self test complete (STC) when true. */
    beq     StartupCpuWaitForCompareSelfTest
    tst     r0, #CCM_SR_STE  /* Self test error (STE) when true. */
    bne     compare_st_failure
    /* Switch to lock step mode. */
    ldr     r2, =CCM_KEYR_ADDR
    mov     r0, #CCM_KEYR_MKEY_LOCKSTEP << CCM_KEYR_MKEY_SHIFT
    str     r0, [r2]
    dmb
    mov     r0, #RETURN_SUCCESS
    bx      lr
compare_st_failure:
    ldr     r0, =RETURN_FAIL_CPU_COMPARE_SELF_TEST
    bx      lr


    .global StartupCpuRunCompareForceErrorTestOrDie
    .thumb_func
StartupCpuRunCompareForceErrorTestOrDie:
    mov     r3, lr  /* No push; code runs before RAM initialization. */
    bl      StartupCpuRunCompareForceErrorTest  /* Stores failure code in R0. */
    cmp     r0, #0
    bne     StartupEmitFailureCodeInR0AndDie
    bx      r3


    /* This function forces a CPU compare mismatch and tests if the error
     * propagates to the ESM module. Note that this test asserts nERROR.
     *
     * Returns failure code, or zero upon success, in R0.
     */
    .global StartupCpuRunCompareForceErrorTest
    .thumb_func
StartupCpuRunCompareForceErrorTest:
    /* Switch to lock step mode. */
    ldr     r2, =CCM_KEYR_ADDR
    mov     r0, #CCM_KEYR_MKEY_LOCKSTEP << CCM_KEYR_MKEY_SHIFT
    str     r0, [r2]
    /* Clear status registers. */
    ldr     r2, =CCM_SR_ADDR
    mov     r0, #0xFFFFFFFF
    str     r0, [r2]
    /* Clear ESM CCM-R4F Compare Error. */
    ldr     r2, =ESM_SR2_ADDR
    mov     r0, #ESM_SR2_CCMR4F_CMP_BIT
    str     r0, [r2]
    ldr     r2, =ESM_SSR2_ADDR
    mov     r0, #ESM_SR2_CCMR4F_CMP_BIT
    str     r0, [r2]
    dmb
    /* Switch to error forcing mode. */
    ldr     r2, =CCM_KEYR_ADDR
    mov     r0, #CCM_KEYR_MKEY_ERROR_FORCING << CCM_KEYR_MKEY_SHIFT
    str     r0, [r2]
    dmb
    /* Test requires one cycle to complete. */
    nop
    /* Verify ESM CCM-R4F Compare Error. */
    ldr     r2, =ESM_SR1_ADDR
    ldr     r0, [r2]
    tst     r0, #ESM_SR1_CCMR4F_ST_BIT  /* Error expected, should be true. */
    beq     compare_force_failure
    ldr     r2, =ESM_SR2_ADDR
    ldr     r0, [r2]
    tst     r0, #ESM_SR2_CCMR4F_CMP_BIT  /* Error expected, should be true. */
    beq     compare_force_failure
    ldr     r2, =ESM_SSR2_ADDR
    ldr     r0, [r2]
    tst     r0, #ESM_SR2_CCMR4F_CMP_BIT  /* Error expected, should be true. */
    beq     compare_force_failure
    /* Clear expected ESM errors. */
    ldr     r2, =ESM_SR2_ADDR
    mov     r0, #ESM_SR2_CCMR4F_CMP_BIT
    str     r0, [r2]
    ldr     r2, =ESM_SSR2_ADDR
    mov     r0, #ESM_SR2_CCMR4F_CMP_BIT
    str     r0, [r2]
    ldr     r2, =ESM_SR1_ADDR
    mov     r0, #ESM_SR1_CCMR4F_ST_BIT  /* Byproduct of compare test. */
    str     r0, [r2]
    ldr     r2, =ESM_EKR_ADDR
    mov     r0, #5  /* Clear nERROR pin. */
    str     r0, [r2]
    mov     r0, #RETURN_SUCCESS
    bx      lr
compare_force_failure:
    ldr     r0, =RETURN_FAIL_CPU_COMPARE_FORCE
    bx      lr


    .global StartupCpuRunCompareSelfTestForceErrorOrDie
    .thumb_func
StartupCpuRunCompareSelfTestForceErrorOrDie:
    mov     r3, lr  /* No push; code runs before RAM initialization. */
    bl      StartupCpuRunCompareSelfTestForceError  /* Stores failure in R0. */
    cmp     r0, #0
    bne     StartupEmitFailureCodeInR0AndDie
    bx      r3


    /* This function tests the error forcing function in the CPU compare force
     * error test.
     *
     * Returns failure code, or zero upon success, in R0.
     */
    .global StartupCpuRunCompareSelfTestForceError
    .thumb_func
StartupCpuRunCompareSelfTestForceError:
    /* Switch to lock step mode. */
    ldr     r2, =CCM_KEYR_ADDR
    mov     r0, #CCM_KEYR_MKEY_LOCKSTEP << CCM_KEYR_MKEY_SHIFT
    str     r0, [r2]
    /* Clear status registers. */
    ldr     r2, =CCM_SR_ADDR
    mov     r0, #0xFFFFFFFF
    str     r0, [r2]
    /* Clear ESM CCM-R4F Compare Error. */
    ldr     r2, =ESM_SR1_ADDR
    mov     r0, #ESM_SR1_CCMR4F_ST_BIT
    str     r0, [r2]
    dmb
    /* Switch to self test error forcing mode. */
    ldr     r2, =CCM_KEYR_ADDR
    mov     r0, #CCM_KEYR_MKEY_SELF_TEST_ERROR_FORCING << CCM_KEYR_MKEY_SHIFT
    str     r0, [r2]
    dmb
    /* Test requires one cycle to complete. */
    nop
    /* Verify ESM CCM-R4F Self Test Error. */
    ldr     r2, =ESM_SR1_ADDR
    ldr     r0, [r2]
    tst     r0, #ESM_SR1_CCMR4F_ST_BIT  /* Error expected, should be true. */
    beq     compare_st_force_failure
    /* Clear expected ESM errors. */
    ldr     r2, =ESM_SR1_ADDR
    mov     r0, #ESM_SR1_CCMR4F_ST_BIT
    str     r0, [r2]
    mov     r0, #RETURN_SUCCESS
    bx      lr
compare_st_force_failure:
    ldr     r0, =RETURN_FAIL_CPU_COMPARE_SELF_TEST_FORCE
    bx      lr

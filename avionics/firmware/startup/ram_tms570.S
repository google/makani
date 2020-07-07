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

/* Processor startup subroutines for the TMS570LS1227 and similar chips. */

#include "avionics/firmware/cpu/registers_def.h"
#include "avionics/firmware/startup/return_codes.h"

    .section ".text.startup", "xa"
    .syntax unified


    .global StartupRamEnablePbist
    .thumb_func
StartupRamEnablePbist:
    /* Disable memory hardware initialization global key (MINITGENA). */
    ldr     r2, =SYS_MINITGCR_ADDR
    mov     r0, #0x05 << SYS_MINITGCR_MINITGENA_SHIFT
    str     r0, [r2]
    dmb
    /* Set PBIST ROM_CLK prescale (90 MHz maximum frequency). */
    ldr     r2, =SYS_MSTGCR_ADDR
    ldr     r0, =(0x01 << SYS_MSTGCR_ROM_DIV_SHIFT \
                  | 0x05 << SYS_MSTGCR_MSTGENA_SHIFT)
    str     r0, [r2]
    dmb
    /* Enable PBIST controller. */
    ldr     r2, =SYS_MSIENA_ADDR
    mov     r0, #0x01  /* See TMS570LS1227 TRM Table 4-27 for controllers. */
    str     r0, [r2]
    dmb
    /* Enable PBIST self test. */
    ldr     r2, =SYS_MSTGCR_ADDR
    ldr     r0, [r2]
    mov     r1, #0x0A  /* MSTGENA. */
    bfi     r0, r1, #SYS_MSTGCR_MSTGENA_SHIFT, #SYS_MSTGCR_MSTGENA_WIDTH
    mov     r1, #0x02  /* MBIST_ALGSEL. */
    bfi     r0, r1, #SYS_MSTGCR_MBIST_ALGSEL_SHIFT, \
                    #SYS_MSTGCR_MBIST_ALGSEL_WIDTH
    str     r0, [r2]
    dmb
    /* Wait at least 32 VCLK cycles for PBIST. */
    mov     r0, #64
wait_for_pbist_enable:
    subs    r0, #1
    bne     wait_for_pbist_enable
    /* Enable PBIST internal clocks. */
    ldr     r2, =PBIST_PACT_ADDR
    mov     r0, #PBIST_PACT_PACT0 | PBIST_PACT_PACT1
    str     r0, [r2]
    dmb
    /* We found that OVER must be set true while setting RINFOx. Clear OVER
     * after setting RINFOx to latch RINFOx. */
    ldr     r2, =PBIST_OVER_ADDR
    mov     r0, #PBIST_OVER_OVER0
    str     r0, [r2]
    dmb
    bx      lr


    .global StartupRamDisablePbist
    .thumb_func
StartupRamDisablePbist:
    /* Disable PBIST internal clocks. */
    ldr     r2, =PBIST_PACT_ADDR
    mov     r0, #0x00
    str     r0, [r2]
    dmb
    /* Disable PBIST. */
    ldr     r2, =SYS_MSTGCR_ADDR
    mov     r0, #0x05
    str     r0, [r2]
    dmb
    bx      lr


    .global StartupRamRunPbistSelfTestOrDie
    .thumb_func
StartupRamRunPbistSelfTestOrDie:
    /* Select algorithm for RAM test on ROM (expect failure). */
    ldr     r2, =PBIST_ALGO_ADDR
    mov     r0, #1 << 3  /* See TMS570 TRM Table 2-6. */
    str     r0, [r2]
    /* Select PBIST and STC ROM groups (groups 1 and 2). */
    ldr     r2, =PBIST_RINFOL_ADDR
    mov     r0, #1 << 1 | 1 << 0
    str     r0, [r2]
    ldr     r2, =PBIST_RINFOU_ADDR
    mov     r0, #0
    str     r0, [r2]
    dmb
    /* Do not override RINFOx with ROM-based configuration. */
    ldr     r2, =PBIST_OVER_ADDR
    mov     r0, #0
    str     r0, [r2]
    /* Select ROM access mode. */
    ldr     r2, =PBIST_ROM_ADDR
    mov     r0, #0x03
    str     r0, [r2]
    dmb
    /* Start PBIST. */
    ldr     r2, =PBIST_DLR_ADDR
    mov     r0, #0x14
    str     r0, [r2]
    dmb
    /* Wait for test completion. */
wait_for_pbist_st:
    ldr     r2, =SYS_MSTCGSTAT_ADDR
    ldr     r0, [r2]
    tst     r0, #0x01
    beq     wait_for_pbist_st
    /* Check results. */
    ldr     r2, =PBIST_FSRF0_ADDR
    ldr     r0, [r2]
    tst     r0, #0x01  /* True indicates failure (expected). */
    beq     pbist_st_failure
    bx      lr
pbist_st_failure:
    ldr     r0, =RETURN_FAIL_RAM_PBIST_SELF_TEST
    b       StartupEmitFailureCodeInR0AndDie


    .global StartupRamRunPbistOnRomOrDie
    .thumb_func
StartupRamRunPbistOnRomOrDie:
    /* Select algorithm for ROM tests on ROM. */
    ldr     r2, =PBIST_ALGO_ADDR
    mov     r0, #0x02  /* See TMS570 TRM Table 2-6. */
    str     r0, [r2]
    /* Select PBIST and STC ROM groups (groups 1 and 2). */
    ldr     r2, =PBIST_RINFOL_ADDR
    mov     r0, #0x03
    str     r0, [r2]
    ldr     r2, =PBIST_RINFOU_ADDR
    mov     r0, #0
    str     r0, [r2]
    dmb
    /* Do not override RINFOx with ROM-based configuration. */
    ldr     r2, =PBIST_OVER_ADDR
    mov     r0, #0
    str     r0, [r2]
    b       start_pbist


    .global StartupRamRunPbistOnAllOrDie
    .thumb_func
StartupRamRunPbistOnAllOrDie:
    /* Select algorithm (March 13N recommended for applications by TI). */
    ldr     r2, =PBIST_ALGO_ADDR
    mov     r0, #0x0C  /* See TMS570 TRM Table 2-6. */
    str     r0, [r2]
    /* Override with ROM-based configuration. */
    ldr     r2, =PBIST_OVER_ADDR
    mov     r0, #1
    str     r0, [r2]
    b       start_pbist


start_pbist:
    /* Select ROM access mode. */
    ldr     r2, =PBIST_ROM_ADDR
    mov     r0, #0x03
    str     r0, [r2]
    dmb
    /* Start PBIST. */
    ldr     r2, =PBIST_DLR_ADDR
    mov     r0, #0x14
    str     r0, [r2]
    dmb
    /* Wait for test completion. */
wait_for_pbist:
    ldr     r2, =SYS_MSTCGSTAT_ADDR
    ldr     r0, [r2]
    tst     r0, #0x01  /* Wait for MSTDONE true. */
    beq     wait_for_pbist
    /* Check results. */
    ldr     r2, =PBIST_FSRF0_ADDR
    ldr     r0, [r2]
    tst     r0, #0x01  /* True indicates failure. */
    bne     pbist_failure
    bx      lr
pbist_failure:
    ldr     r0, =RETURN_FAIL_RAM_PBIST_FAILURE
    b       StartupEmitFailureCodeInR0AndDie


    .global StartupRamEnableEcc
    .thumb_func
StartupRamEnableEcc:
    /* Enable ECC in the RAM wrapper for both SRAM banks.
     * This must be done before the RAM is initialized so that the ECC
     * data can be properly setup during RAM initialization. */
    /* Bank 1. */
    ldr     r2, =TCRAM1_RAMCTRL_ADDR
    ldr     r0, [r2]
    ldr     r1, =0xA
    bfi     r0, r1, #TCRAM_RAMCTRL_ECC_DETECT_EN_SHIFT, \
                    #TCRAM_RAMCTRL_ECC_DETECT_EN_WIDTH
    str     r0, [r2]
    /* Bank 2. */
    ldr     r2, =TCRAM2_RAMCTRL_ADDR
    ldr     r0, [r2]
    bfi     r0, r1, #TCRAM_RAMCTRL_ECC_DETECT_EN_SHIFT, \
                    #TCRAM_RAMCTRL_ECC_DETECT_EN_WIDTH
    str     r0, [r2]

    /* Enable ECC in the ARM memory controller. */
    mrc     p15, #0, r0, c1, c0, #1  /* ACR: Auxiliary control register. */
    orr     r0, r0, #0x0C000000      /* Set B0TCMPCEN, B1TCMPCEN. */
    dmb
    mcr     p15, #0, r0, c1, c0, #1
    isb
    bx      lr


    .global StartupRamInitialize
    .thumb_func
StartupRamInitialize:
    /* Now that ECC is enabled, use the memory initializer to erase RAM. */
    ldr     r2, =SYS_MINITGCR_ADDR  /* Memory init global control register. */
    str     r1, [r2]                /* MINITGENA = 0xA: Unlock initializer. */

    ldr     r3, =SYS_MSIENA_ADDR  /* Memory test/init enable register. */
    ldr     r0, =0x1              /* Bit 0 initializes all SRAM blocks. */
    str     r0, [r3]

    /* Memory self test controller global status register. */
    ldr     r3, =SYS_MSTCGSTAT_ADDR
minit_loop:
    ldr     r0, [r3]
    ands    r0, #SYS_MSTCGSTAT_MINIDONE
    beq     minit_loop  /* Wait for MINIDONE to be set. */

    mov     r1, #0x5
    str     r1, [r2]  /* MINITGENA = 0x5: Re-lock initializer. */

    mrc     p15, #0, r0, c9, c12, #0  /* PMNC: Performance monitor control. */
    orr     r0, r0, #0x10             /* X = 1: Export ECC events to wrapper. */
    mcr     p15, #0, r0, c9, c12, #0

    mov     r0, #0x1  /* THRESHOLD = 1: Detect ECC errors. */
    ldr     r2, =TCRAM1_RAMTHRESHOLD_ADDR
    str     r0, [r2]
    ldr     r2, =TCRAM2_RAMTHRESHOLD_ADDR
    str     r0, [r2]
    bx      lr

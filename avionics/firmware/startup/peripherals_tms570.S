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

/* TMS570 peripherals startup sequence. */

#include "avionics/firmware/cpu/registers_def.h"

    .section ".text.startup", "xa"
    .syntax unified

    LED0_BIT = 1 << 14
    LED1_BIT = 1 << 16


    .global StartupPeripheralsEnableClocks
    .thumb_func
StartupPeripheralsEnableClocks:
    /* Initialize Power Mamagement Module (PMM) */

    /* Enable all clocks to logic power domains. See TMS570 TRM Table 3-4 */
    ldr     r2, =PMM_PDCLKDISCLR_ADDR
    ldr     r0, =0xFFFFFFFF
    str     r0, [r2]
    dmb

    /* Power up all logic power domains. See TMS570 TRM Table 3-2. */
    ldr     r2, =PMM_LOGICPDPWRCTRL0_ADDR
    ldr     r0, [r2]
    ldr     r1, =0xF0F0F0F0
    and     r0, r1
    ldr     r1, =0x05050505
    orr     r0, r1
    str     r0, [r2]
    dmb

    /* Wait for all logic power domains to change state. */
wait_for_logic_on:
    ldr     r2, =PMM_LOGICPDPWRCTRL0_ADDR
    ldr     r0, [r2]
    ldr     r1, =0x0F0F0F0F
    and     r0, r1
    ldr     r1, =0x05050505
    cmp     r0, r1
    bne     wait_for_logic_on

    /* Power up all memory power domains. See TMS570 TRM Table 3-3. */
    ldr     r2, =PMM_MEMPDPWRCTRL0_ADDR
    ldr     r0, [r2]
    ldr     r1, =0xF0F0FFFF
    and     r0, r1
    ldr     r1, =0x05050000
    orr     r0, r1
    str     r0, [r2]
    dmb

    /* Wait for all memory power domains to change state. */
wait_for_memory_on:
    ldr     r2, =PMM_MEMPDPWRCTRL0_ADDR
    ldr     r0, [r2]
    ldr     r1, =0x0F0F0000
    and     r0, r1
    ldr     r1, =0x05050000
    cmp     r0, r1
    bne     wait_for_memory_on

    /* Initialize Peripheral Central Resource (PCR).
     * See TMS570LS1227 Table 4-21 "Device Memory Map". The Frame Chip Select
     * column corresponds to the peripheral select PS[] and peripheral memory
     * clock select PCS[]. */

    /* Power up all peripheral memory selects. */
    ldr   r2, =PCR_PCSPWRDWNCLR0_ADDR
    ldr   r0, =0xFFFFFFFF  /* See TMS570 TRM Table 2-99. */
    str   r0, [r2]

    ldr   r2, =PCR_PCSPWRDWNCLR1_ADDR
    ldr   r0, =0xFFFFFFFF  /* See TMS570 TRM Table 2-100. */
    str   r0, [r2]
    dmb

    /* Power up all peripheral selects. */
    ldr   r2, =PCR_PSPWRDWNCLR0_ADDR
    ldr   r0, =0xFFFFFFFF  /* See TMS570 TRM Table 2-105. */
    str   r0, [r2]

    ldr   r2, =PCR_PSPWRDWNCLR1_ADDR
    ldr   r0, =0xFFFFFFFF  /* See TMS570 TRM Table 2-106. */
    str   r0, [r2]

    ldr   r2, =PCR_PSPWRDWNCLR2_ADDR
    ldr   r0, =0xFFFFFFFF  /* See TMS570 TRM Table 2-107. */
    str   r0, [r2]

    ldr   r2, =PCR_PSPWRDWNCLR3_ADDR
    ldr   r0, =0xFFFFFFFF  /* See TMS570 TRM Table 2-108. */
    str   r0, [r2]
    dmb

    bx      lr


    .global StartupLedInit
    .thumb_func
StartupLedInit:
    /* LED0 = N2HET2[14], LED1 = N2HET2[16]. */

    /* Unlock IOMM. */
    ldr     r2, =IOMM_KICK_REG0_ADDR
    ldr     r0, =0x83E70B13
    str     r0, [r2]
    dmb
    ldr     r2, =IOMM_KICK_REG1_ADDR
    ldr     r0, =0x95A4F1E0
    str     r0, [r2]
    dmb

    /* Select LED pins. */
    ldr     r2, =IOMM_PINMMR6_ADDR
    ldr     r0, [r2]
    mov     r1, #1 << 1
    bfi     r0, r1, #16, #8
    mov     r1, #1 << 3
    bfi     r0, r1, #0, #8
    str     r0, [r2]
    dmb

    /* Lock IOMM. */
    ldr     r2, =IOMM_KICK_REG0_ADDR
    mov     r0, #0
    str     r0, [r2]
    dmb
    ldr     r2, =IOMM_KICK_REG1_ADDR
    mov     r0, #0
    str     r0, [r2]
    dmb

    /* Set direction to output. */
    ldr     r2, =N2HET2_HETDIR_ADDR
    ldr     r0, [r2]
    orr     r0, #LED0_BIT | LED1_BIT
    str     r0, [r2]
    bx      lr


    .global StartupEmitFailureCodeInR0
    .thumb_func
StartupEmitFailureCodeInR0:
    mov     r3, r0
    mov     r1, #LED1_BIT
    /* Emit preamble synchronization pulse. */
    ldr     r2, =N2HET2_HETDCLR_ADDR
    str     r1, [r2]
    mov     r0, #64
led_preamble_lo0:
    subs    r0, #1
    bne     led_preamble_lo0
    ldr     r2, =N2HET2_HETDSET_ADDR
    str     r1, [r2]
    mov     r0, #64
led_preamble_hi:
    subs    r0, #1
    bne     led_preamble_hi
    ldr     r2, =N2HET2_HETDCLR_ADDR
    str     r1, [r2]
    mov     r0, #64
led_preamble_lo1:
    subs    r0, #1
    bne     led_preamble_lo1
    /* Clock out 16 bits, MSB first. */
    mov     r0, #1 << 15
led_loop:
    cmp     r0, #0
    beq     led_return
    tst     r0, r3
    lsr     r0, #1
    beq     led_clear
    ldr     r2, =N2HET2_HETDSET_ADDR
    str     r1, [r2]
    b       led_loop
led_clear:
    ldr     r2, =N2HET2_HETDCLR_ADDR
    str     r1, [r2]
    b       led_loop
led_return:
    ldr     r2, =N2HET2_HETDCLR_ADDR
    str     r1, [r2]
    bx      lr


    .global StartupEmitFailureCodeInR0AndDie
    .thumb_func
StartupEmitFailureCodeInR0AndDie:
    bl      StartupEmitFailureCodeInR0
    wfi
    b       StartupEmitFailureCodeInR0AndDie


    .global Led0On
    .thumb_func
Led0On:
    ldr     r2, =N2HET2_HETDSET_ADDR
    mov     r0, #LED0_BIT
    str     r0, [r2]
    bx      lr


    .global Led1On
    .thumb_func
Led1On:
    ldr     r2, =N2HET2_HETDSET_ADDR
    mov     r0, #LED1_BIT
    str     r0, [r2]
    bx      lr


    .global Led0Off
    .thumb_func
Led0Off:
    ldr     r2, =N2HET2_HETDCLR_ADDR
    mov     r0, #LED0_BIT
    str     r0, [r2]
    bx      lr


    .global Led1Off
    .thumb_func
Led1Off:
    ldr     r2, =N2HET2_HETDCLR_ADDR
    mov     r0, #LED1_BIT
    str     r0, [r2]
    bx      lr

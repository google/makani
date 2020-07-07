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

/* TMS570 clock startup sequence. */

#include "avionics/firmware/cpu/registers_def.h"
#include "avionics/firmware/startup/clock_tms570_config.h"

    .section ".text.startup", "xa"
    .syntax unified

    /* See TMS570LS1227 4.6.1 "Clock Sources". */
    /* See TMS570LS1227 Table 4-8 "Available Clock Sources". */
    CLOCK_SOURCE_OSCIN = 0
    CLOCK_SOURCE_PLL1 = 1
    CLOCK_SOURCE_PLL2 = 6

    /* See TMS570LS1227 6.6.1.3.2 "PLL Timing Specifications". We configure
     * the PLL to have an output frequency near half way between the minimum
     * and maximum frequencies. Table 6-11 indicates 150 MHz -- 550 MHz for
     * both PLL1 and PLL2, thus we have a target frequency of 350 MHz. */

    /* Configure PLL1 for 160 MHz given f_OSCIN = 16 MHz. */
    PLL1_NR = 6    /* f_INTCLK = f_OSCIN / NR = 2.667 MHz. */
    PLL1_NF = 120  /* f_VCOCLK = f_INTCLK * NF = 320 MHz (target 350 MHz). */
    PLL1_OD = 2    /* f_post_ODCLK = f_VCOCLK / OD = 160 MHz. */
    PLL1_R = 1     /* f_PLLCLK = f_post_ODCLK / R = 160 MHz. */

    /* Configure PLL2 for 100 MHz (EMAC requires multiple of 25 MHz). */
    PLL2_NR = 4    /* f_INTCLK2 = f_OSCIN / NR2 = 4.000 MHz. */
    PLL2_NF = 75   /* f_VCOCLK2 = f_INTCLK2 * NF2 = 300 MHz (target 350 MHz). */
    PLL2_OD = 3    /* f_post_ODCLK2 = f_VCOCLK2 / OD2 = 100 MHz. */
    PLL2_R = 1     /* f_PLLCLK2 = f_post_ODCLK2 / R2 = 100 MHz. */

    /* Configure RTI clock for 2 MHz given f_OSCIN = 16 MHz. */
    RTI1DIV = 3  /* f_RTI = f_OSCIN / (1 << RTI1DIV) = 2 MHz. */


    .global StartupClockSetTrim
    .thumb_func
StartupClockSetTrim:
    /* See TMS570 TRM 10.2 "Quick Start". Initial oscillator trim values stored
     * in OTP. Bits 31-16 of OTP address 0xF00801B4 contain oscillator trim
     * values for LPOMONCTL bits 15-0. */
    ldr     r2, =0xF00801B4
    ldr     r0, [r2]
    lsr     r0, #16
    orr     r0, #SYS_LPOMONCTL_BIAS_ENABLE
    ldr     r2, =SYS_LPOMONCTL_ADDR
    str     r0, [r2]
    dmb
    bx      lr


    .global StartupClockEnableOscIn
    .thumb_func
StartupClockEnableOscIn:
    /* Set CSDISCLR to clear clock disables. */
    ldr     r2, =SYS_CSDISCLR_ADDR
    mov     r0, #1 << CLOCK_SOURCE_OSCIN
    str     r0, [r2]
    bx      lr


    .global StartupClockEnablePll1
    .thumb_func
StartupClockEnablePll1:
    /* Set CSDISCLR to clear clock disables. */
    ldr     r2, =SYS_CSDISCLR_ADDR
    mov     r0, #1 << CLOCK_SOURCE_PLL1
    str     r0, [r2]
    bx      lr


    .global StartupClockEnablePll2
    .thumb_func
StartupClockEnablePll2:
    /* Set CSDISCLR to clear clock disables. */
    ldr     r2, =SYS_CSDISCLR_ADDR
    mov     r0, #1 << CLOCK_SOURCE_PLL2
    str     r0, [r2]
    bx      lr


    .global StartupClockWaitForOscIn
    .thumb_func
StartupClockWaitForOscIn:
    /* Poll CSVSTAT until source becomes valid. See TMS570 TRM Table 2-37. */
    ldr     r2, =SYS_CSVSTAT_ADDR
wait_for_oscin_valid:
    ldr     r0, [r2]
    ands    r0, #1 << CLOCK_SOURCE_OSCIN  /* Set when valid. */
    beq     wait_for_oscin_valid
    bx      lr


    .global StartupClockWaitForPll1
    .thumb_func
StartupClockWaitForPll1:
    /* Poll CSVSTAT until source becomes valid. See TMS570 TRM Table 2-37. */
    ldr     r2, =SYS_CSVSTAT_ADDR
wait_for_pll1_valid:
    ldr     r0, [r2]
    ands    r0, #1 << CLOCK_SOURCE_PLL1  /* Set when valid. */
    beq     wait_for_pll1_valid
    bx      lr


    .global StartupClockWaitForPll2
    .thumb_func
StartupClockWaitForPll2:
    /* Poll CSVSTAT until source becomes valid. See TMS570 TRM Table 2-37. */
    ldr     r2, =SYS_CSVSTAT_ADDR
wait_for_pll2_valid:
    ldr     r0, [r2]
    ands    r0, #1 << CLOCK_SOURCE_PLL2  /* Set when valid. */
    beq     wait_for_pll2_valid
    bx      lr


    .global StartupClockSelectOscIn
    .thumb_func
StartupClockSelectOscIn:
    mov     r3, lr
    ldr     r0, =CLOCK_SOURCE_OSCIN
    bl      StartupClockSelectSource
    bx      r3


    .global StartupClockSelectPll1
    .thumb_func
StartupClockSelectPll1:
    mov     r3, lr
    ldr     r0, =CLOCK_SOURCE_PLL1
    bl      StartupClockSelectSource
    bx      r3


    /* StartupClockSelectSource(int32_t source);
     * StartupClockSelectSource R0=source
     * This function sets the GCLK (global clock) source using registers
     * R0, R1, R2 only. */
    .global StartupClockSelectSource
    .thumb_func
StartupClockSelectSource:
    /* Set GHVSRC to select GCLK (global clock) source. */
    /* See TMS570 TRM Table 2-34. */
    ldr     r1, =0
    bfi     r1, r0, SYS_GHVSRC_GHVWAKE_SHIFT, SYS_GHVSRC_GHVWAKE_WIDTH
    bfi     r1, r0, SYS_GHVSRC_HVLPM_SHIFT, SYS_GHVSRC_HVLPM_WIDTH
    bfi     r1, r0, SYS_GHVSRC_GHVSRC_SHIFT, SYS_GHVSRC_GHVSRC_WIDTH
    ldr     r2, =SYS_GHVSRC_ADDR
    str     r1, [r2]
    /* Prevent ESM FMC uncorrectable ECC error. */
    dmb
    isb
    bx      lr


    .global StartupClockSetPll1Slow
    .thumb_func
StartupClockSetPll1Slow:
    /* Set PLLCTL1 to maximum output dividers within timing specifications. */
    mov     r3, lr
    ldr     r0, =PLL1_NR
    ldr     r1, =PLL1_NF
    bl      StartupClockSetPll1NrNf
    bx      r3


    /* StartupClockSetPll1NrNf(int32_t nr, int32_t nf);
     * StartupClockSetPll1NrNf R0=nr R1=nf
     * This function sets the REFCLKDIV and PLLMUL parameters given arguments
     * nr and nf, and dividers ODPLL and PLLDIV to their maximum value. This
     * function uses registers R0, R1, R2 only. */
    .global StartupClockSetPll1NrNf
    .thumb_func
StartupClockSetPll1NrNf:
    /* See TMS570LS1227 Table 4-11 "PLL Timing Specifications". */
    /* See TMS570 TRM Table 2-44. */
    /* TODO: Evaluate reset on slip (ROS) and reset on failure (ROF). */
    ldr     r2, =(0 * SYS_PLLCTL1_ROS \
                  | 2 << SYS_PLLCTL1_MASK_SLIP_SHIFT \
                  | SYS_PLLCTL1_PLLDIV_MASK \
                  | 0 * SYS_PLLCTL1_ROF)
    /* Handle PLL1_NR. */
    sub     r0, r0, #1
    bfi     r2, r0, SYS_PLLCTL1_REFCLKDIV_SHIFT, SYS_PLLCTL1_REFCLKDIV_WIDTH
    /* Handle PLL1_NF. */
    sub     r1, r1, #1
    lsl     r1, r1, 8
    bfi     r2, r1, SYS_PLLCTL1_PLLMUL_SHIFT, SYS_PLLCTL1_PLLMUL_WIDTH
    /* Set PLLCTL1 register. */
    ldr     r1, =SYS_PLLCTL1_ADDR
    str     r2, [r1]
    /* Set PLLCTL2 to maximum output dividers. */
    ldr     r1, =SYS_PLLCTL2_ADDR
    ldr     r0, =SYS_PLLCTL2_ODPLL_MASK  /* See TMS570 TRM Table 2-45. */
    str     r0, [r1]
    dmb
    bx      lr


    .global StartupClockSetPll2Slow
    .thumb_func
StartupClockSetPll2Slow:
    /* Set PLLCTL3 to maximum output dividers. */
    mov     r3, lr
    ldr     r0, =PLL2_NR
    ldr     r1, =PLL2_NF
    bl      StartupClockSetPll2NrNf
    bx      r3


    /* StartupClockSetPll2NrNf(int32_t nr, int32_t nf);
     * StartupClockSetPll2NrNf R0=nr R1=nf
     * This function sets the REFCLKDIV2 and PLLMUL2 parameters given arguments
     * nr and nf, and dividers ODPLL2 and PLLDIV2 to their maximum value. This
     * function uses registers R0, R1, R2 only. */
    .global StartupClockSetPll2NrNf
    .thumb_func
StartupClockSetPll2NrNf:
    /* See TMS570 TRM Table 2-74. */
    ldr     r2, =SYS2_PLLCTL3_ODPLL2_MASK | SYS2_PLLCTL3_PLLDIV2_MASK
    /* Handle PLL2_NR. */
    sub     r0, r0, #1
    bfi     r2, r0, SYS2_PLLCTL3_REFCLKDIV2_SHIFT, SYS2_PLLCTL3_REFCLKDIV2_WIDTH
    /* Handle PLL2_NF. */
    sub     r1, r1, #1
    lsl     r1, r1, 8
    bfi     r2, r1, SYS2_PLLCTL3_PLLMUL2_SHIFT, SYS2_PLLCTL3_PLLMUL2_WIDTH
    /* Set PLLCTL3 register. */
    ldr     r1, =SYS2_PLLCTL3_ADDR
    str     r2, [r1]
    dmb
    bx      lr


    .global StartupClockSetPll1Fast
    .thumb_func
StartupClockSetPll1Fast:
    /* Setup PLLs for operational speed. */
    /* See TMS570 TRM 2.4 "Clocks".
     * See TMS570LS1227 3.3 "Switching Characteristics over Recommended
     * Operating Conditions for Clock Domains".
     * See TMS570LS1227 4.6.1.3.1 "Block Diagram" for PLL block diagram.
     * See TMS570LS1227 4.6.2.2 "Mapping of Clock Domains to Device Modules". */
    mov     r3, lr
    ldr     r0, =PLL1_OD
    bl      StartupClockSetPll1Od  /* Should not clobber R3. */
    ldr     r0, =PLL1_R
    bl      StartupClockSetPll1R  /* Should not clobber R3. */
    bx      r3


    /* StartupClockSetPll1Od(int32_t od);
     * StartupClockSetPll1Od R0=od
     * This function sets the ODPLL divider in steps until it reaches the
     * desired value using registers R0, R1, R2 only. */
    .global StartupClockSetPll1Od
    .thumb_func
StartupClockSetPll1Od:
    sub     r2, r0, #1  /* R2 now stores PLL1_OD - 1. */

    /* Relax ODPLL divider in steps until we reach our desired value. */
pll1_od_dec:
    ldr     r1, =SYS_PLLCTL2_ADDR
    ldr     r0, [r1]
    and     r0, #SYS_PLLCTL2_ODPLL_MASK
    lsr     r0, #SYS_PLLCTL2_ODPLL_SHIFT  /* R0 now stores ODPLL. */
    cmp     r0, r2
    ble     pll1_od_done
    sub     r0, #1
    lsl     r0, #SYS_PLLCTL2_ODPLL_SHIFT
    str     r0, [r1]
    dmb
    /* Poll CSVSTAT until source becomes valid. See TMS570 TRM Table 2-37. */
    ldr     r1, =SYS_CSVSTAT_ADDR
pll1_od_valid:
    ldr     r0, [r1]
    ands    r0, #1 << CLOCK_SOURCE_PLL1  /* Set when valid. */
    beq     pll1_od_valid
    b       pll1_od_dec
pll1_od_done:
    bx      lr


    /* StartupClockSetPll1R(int32_t od);
     * StartupClockSetPll1R R0=r
     * This function sets the PLLDIV divider in steps until it reaches the
     * desired value using registers R0, R1, R2 only. */
    .global StartupClockSetPll1R
    .thumb_func
StartupClockSetPll1R:
    sub     r2, r0, #1  /* R2 now stores PLL1_R - 1. */

    /* Relax PLLDIV divider in steps until we reach our desired value. */
pll1_r_dec:
    ldr     r1, =SYS_PLLCTL1_ADDR
    ldr     r1, [r1]
    mov     r0, r1
    and     r1, #SYS_PLLCTL1_PLLDIV_MASK
    lsr     r1, #SYS_PLLCTL1_PLLDIV_SHIFT  /* R1 now stores PLLDIV. */
    cmp     r1, r2
    ble     pll1_r_done
    sub     r1, #1
    bfi     r0, r1, #SYS_PLLCTL1_PLLDIV_SHIFT, #SYS_PLLCTL1_PLLDIV_WIDTH
    ldr     r1, =SYS_PLLCTL1_ADDR
    str     r0, [r1]
    dmb
    /* Poll CSVSTAT until source becomes valid. See TMS570 TRM Table 2-37. */
    ldr     r1, =SYS_CSVSTAT_ADDR
pll1_r_valid:
    ldr     r0, [r1]
    ands    r0, #1 << CLOCK_SOURCE_PLL1  /* Set when valid. */
    beq     pll1_r_valid
    b       pll1_r_dec
pll1_r_done:
    bx      lr


    .global StartupClockSetPll2Fast
    .thumb_func
StartupClockSetPll2Fast:
    /* Setup PLLs for operational speed. */
    /* See TMS570 TRM 2.4 "Clocks".
     * See TMS570LS1227 3.3 "Switching Characteristics over Recommended
     * Operating Conditions for Clock Domains".
     * See TMS570LS1227 4.6.1.3.1 "Block Diagram" for PLL block diagram.
     * See TMS570LS1227 4.6.2.2 "Mapping of Clock Domains to Device Modules". */
    mov     r3, lr
    ldr     r0, =PLL2_OD
    bl      StartupClockSetPll2Od  /* Should not clobber R3. */
    ldr     r0, =PLL2_R
    bl      StartupClockSetPll2R  /* Should not clobber R3. */
    bx      r3


    /* StartupClockSetPll2Od(int32_t od);
     * StartupClockSetPll2Od R0=od
     * This function sets the ODPLL2 divider in steps until it reaches the
     * desired value using registers R0, R1, R2 only. */
    .global StartupClockSetPll2Od
    .thumb_func
StartupClockSetPll2Od:
    sub     r2, r0, #1  /* R2 now stores PLL2_OD - 1. */

    /* Relax ODPLL2 divider in steps until we reach our desired value. */
pll2_od_dec:
    ldr     r1, =SYS2_PLLCTL3_ADDR
    ldr     r1, [r1]
    mov     r0, r1
    and     r1, #SYS2_PLLCTL3_ODPLL2_MASK
    lsr     r1, #SYS2_PLLCTL3_ODPLL2_SHIFT  /* R1 now stores ODPLL2. */
    cmp     r1, r2
    ble     pll2_od_done
    sub     r1, #1
    bfi     r0, r1, #SYS2_PLLCTL3_ODPLL2_SHIFT, #SYS2_PLLCTL3_ODPLL2_WIDTH
    ldr     r1, =SYS2_PLLCTL3_ADDR
    str     r0, [r1]
    dmb
    /* Poll CSVSTAT until source becomes valid. See TMS570 TRM Table 2-37. */
    ldr     r1, =SYS_CSVSTAT_ADDR
pll2_od_valid:
    ldr     r0, [r1]
    ands    r0, #1 << CLOCK_SOURCE_PLL2  /* Set when valid. */
    beq     pll2_od_valid
    b       pll2_od_dec
pll2_od_done:
    bx      lr


    /* StartupClockSetPll2R(int32_t od);
     * StartupClockSetPll2R R0=r
     * This function sets the PLLDIV2 divider in steps until it reaches the
     * desired value using registers R0, R1, R2 only. */
    .global StartupClockSetPll2R
    .thumb_func
StartupClockSetPll2R:
    sub     r2, r0, #1  /* R2 now stores PLL2_R - 1. */

    /* Relax PLLDIV2 divider in steps until we reach our desired value. */
pll2_r_dec:
    ldr     r1, =SYS2_PLLCTL3_ADDR
    ldr     r1, [r1]
    mov     r0, r1
    and     r1, #SYS2_PLLCTL3_PLLDIV2_MASK
    lsr     r1, #SYS2_PLLCTL3_PLLDIV2_SHIFT  /* R1 now stores PLLDIV2. */
    cmp     r1, r2
    ble     pll2_r_done
    sub     r1, #1
    bfi     r0, r1, #SYS2_PLLCTL3_PLLDIV2_SHIFT, #SYS2_PLLCTL3_PLLDIV2_WIDTH
    ldr     r1, =SYS2_PLLCTL3_ADDR
    str     r0, [r1]
    dmb
    /* Poll CSVSTAT until source becomes valid. See TMS570 TRM Table 2-37. */
    ldr     r1, =SYS_CSVSTAT_ADDR
pll2_r_valid:
    ldr     r0, [r1]
    ands    r0, #1 << CLOCK_SOURCE_PLL2  /* Set when valid. */
    beq     pll2_r_valid
    b       pll2_r_dec
pll2_r_done:
    bx      lr


    .global StartupClockEnableDomains
    .thumb_func
StartupClockEnableDomains:
    /* Enable all clock domains. */
    ldr     r2, =SYS_CDDISCLR_ADDR
    ldr     r0, =0xFFFFFFFF
    str     r0, [r2]
    dmb
    bx      lr


    .global StartupClockDisableVclk
    .thumb_func
StartupClockDisableVclk:
    /* Disable VCLK to peripherals while configuring. */
    ldr     r2, =SYS_CLKCNTL_ADDR
    ldr     r0, [r2]
    bic     r0, #SYS_CLKCNTL_PENA
    str     r0, [r2]
    dmb
    bx      lr


    .global StartupClockEnableVclk
    .thumb_func
StartupClockEnableVclk:
    /* Set VCLK ratio to maximum ratio to ensure VCLK2 is greater. TRM states
     * that VCLK2 must always be greater than or equal to VCLK, and we should
     * not change both clocks simultaneously. See TMS570 2.5.1.45. */
    ldr     r2, =SYS_CLKCNTL_ADDR
    ldr     r0, [r2]
    orr     r0, #SYS_CLKCNTL_VCLKR_MASK  /* Maximum VCLKR ratio. */
    str     r0, [r2]
    dmb

    /* Set VCLK2 ratio to desired. Must be an integer multiple of VCLK.
     * f_VCLK2 = f_HCLK / (VCLK2R + 1) <= 100 MHz max. */
    ldr     r0, [r2]
    bic     r0, #SYS_CLKCNTL_VCLK2R_MASK  /* Clear VCLK2R ratio. */
    orr     r0, #(CLOCK_VCLK2_DIVR - 1) << SYS_CLKCNTL_VCLK2R_SHIFT
    str     r0, [r2]
    dmb

    /* Set VCLK ratio to desired.
     * f_VCLK = f_HCLK / (VCLKR + 1) <= 100 MHz max. */
    ldr     r0, [r2]
    bic     r0, #SYS_CLKCNTL_VCLKR_MASK  /* Clear VCLKR ratio. */
    orr     r0, #(CLOCK_VCLK_DIVR - 1) << SYS_CLKCNTL_VCLKR_SHIFT
    str     r0, [r2]
    dmb

    /* Enable VCLK to peripherals. */
    ldr     r0, [r2]
    orr     r0, #SYS_CLKCNTL_PENA
    str     r0, [r2]
    dmb

    /* Set VCLK3.
     * f_VCLK3 = f_HCLK / (VCLK3R + 1) <= 100 MHz max. */
    ldr     r2, =SYS2_CLK2CNTRL_ADDR
    ldr     r0, [r2]
    bic     r0, #SYS2_CLK2CNTRL_VCLK3R_MASK  /* Clear VCLK3R ratio. */
    orr     r0, #(CLOCK_VCLK3_DIVR - 1) << SYS2_CLK2CNTRL_VCLK3R_SHIFT
    str     r0, [r2]
    dmb
    bx      lr


    .global StartupClockEnableVclka
    .thumb_func
StartupClockEnableVclka:
    /* Configure source selection for asynchronous peripheral clocks. */
    ldr     r2, =SYS_VCLKASRC_ADDR
    ldr     r0, =(CLOCK_SOURCE_PLL2 << SYS_VCLKASRC_VCLKA2S_SHIFT \
                  | CLOCK_SOURCE_PLL2 << SYS_VCLKASRC_VCLKA1S_SHIFT)
    str     r0, [r2]

    /* See TMS570LS1227 4.6.2.3 "Special Clock Source Selection Scheme for
     * VCLKA4_DIVR_EMAC". MII interface requires 25 MHz. */
    /* Disable VCLKA3 since it does not exist on the TMS570LS1227. */
    ldr     r2, =SYS2_VCLKACON1_ADDR
    ldr     r0, =((CLOCK_VCLKA4_DIVR - 1) << SYS2_VCLKACON1_VCLKA4R_SHIFT \
                  | 0 * SYS2_VCLKACON1_VCLKA4_DIV_CDDIS \
                  | CLOCK_SOURCE_PLL2 << SYS2_VCLKACON1_VCLKA4S_SHIFT \
                  | (CLOCK_VCLKA3_DIVR - 1) << SYS2_VCLKACON1_VCLKA3R_SHIFT \
                  | SYS2_VCLKACON1_VCLKA3_DIV_CDDIS \
                  | CLOCK_SOURCE_PLL2 << SYS2_VCLKACON1_VCLKA3S_SHIFT)
    str     r0, [r2]
    dmb
    bx      lr


    .global StartupClockDisableVclka
    .thumb_func
StartupClockDisableVclka:
    /* Disable VCLKA to peripherals while configuring. */
    ldr     r2, =SYS2_VCLKACON1_ADDR
    ldr     r0, =((CLOCK_VCLKA4_DIVR - 1) << SYS2_VCLKACON1_VCLKA4R_SHIFT \
                  | SYS2_VCLKACON1_VCLKA4_DIV_CDDIS \
                  | CLOCK_SOURCE_OSCIN << SYS2_VCLKACON1_VCLKA4S_SHIFT \
                  | (CLOCK_VCLKA3_DIVR - 1) << SYS2_VCLKACON1_VCLKA3R_SHIFT \
                  | SYS2_VCLKACON1_VCLKA3_DIV_CDDIS \
                  | CLOCK_SOURCE_OSCIN << SYS2_VCLKACON1_VCLKA3S_SHIFT)
    str     r0, [r2]

    /* Configure source selection for asynchronous peripheral clocks. */
    ldr     r2, =SYS_VCLKASRC_ADDR
    ldr     r0, =(CLOCK_SOURCE_OSCIN << SYS_VCLKASRC_VCLKA2S_SHIFT \
                  | CLOCK_SOURCE_OSCIN << SYS_VCLKASRC_VCLKA1S_SHIFT)
    str     r0, [r2]
    dmb
    bx      lr


    .global StartupClockSetRticlk
    .thumb_func
StartupClockSetRticlk:
    mov     r3, lr
    ldr     r0, =RTI1DIV
    bl      StartupClockSetRtidiv
    bx      r3


    /* StartupClockSetRtidiv(int32_t rtidiv);
     * StartupClockSetRtidiv R0=rtidiv */
    .global StartupClockSetRtidiv
    .thumb_func
StartupClockSetRtidiv:
    /* RTI clock reconfigured in clock.c. */
    ldr     r1, =CLOCK_SOURCE_OSCIN << SYS_RCLKSRC_RTI1SRC_SHIFT
    bfi     r1, r0, SYS_RCLKSRC_RTI1DIV_SHIFT, SYS_RCLKSRC_RTI1DIV_WIDTH
    ldr     r2, =SYS_RCLKSRC_ADDR
    str     r1, [r2]
    dmb
    bx      lr


    .global StartupClockEnableRticlk
    .thumb_func
StartupClockEnableRticlk:
    ldr     r2, =SYS_CDDISCLR_ADDR
    ldr     r0, =SYS_CDDISCLR_CLRRTICLK1OFF
    str     r0, [r2]
    dmb
    bx      lr


    .global StartupClockDisableRticlk
    .thumb_func
StartupClockDisableRticlk:
    ldr     r2, =SYS_CDDISSET_ADDR
    ldr     r0, =SYS_CDDISSET_SETRTICLK1OFF
    str     r0, [r2]
    dmb
    bx      lr

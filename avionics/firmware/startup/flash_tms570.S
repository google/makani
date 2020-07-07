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

/* TMS570 F021 Flash Module Controller (FMC) startup sequence. */

#include "avionics/firmware/cpu/registers_def.h"

    .section ".text.startup", "xa"
    .syntax unified

    /* See Cortex-R4/R4F TRM 4.3.16 "c1, Auxiliary Control Register". */
    ACTLR_ATCMPCEN = 1 << 25

    /* See Cortex-R4/R4F TRM 6.3.1 "c9, Performance Monitor Control Reg." */
    PMCR_X = 1 << 4


    .global StartupFlashEnableEcc
    .thumb_func
StartupFlashEnableEcc:
    /* Enable Single-Error-Correction-Double-Error-Detection (SECDED) logic. */
    dmb
    isb

    /* Set FEDACCTRL1 to enable error detection and correction on bank 0. */
    /* See TMS570 TRM Table 5-15. */
    ldr     r2, =FLASH_FEDACCTRL1_ADDR
    ldr     r0, =(FLASH_FEDACCTRL1_SUSP_IGNR \
                  | 0x0A << FLASH_FEDACCTRL1_EDACMODE_SHIFT \
                  | FLASH_FEDACCTRL1_EOFEN \
                  | FLASH_FEDACCTRL1_EZFEN \
                  | FLASH_FEDACCTRL1_EPEN \
                  | 0x0A << FLASH_FEDACCTRL1_EDACEN_SHIFT)
    str     r0, [r2]

    /* Set FEDACCTRL2 error correction threshold (0=disabled). */
    ldr     r2, =FLASH_FEDACCTRL2_ADDR
    ldr     r0, =0 << FLASH_FEDACCTRL2_SEC_THRESHOLD_SHIFT
    str     r0, [r2]

    /* Set EE_CTRL1 to enable error detection and correction on bank 7. */
    /* See TMS570 TRM Table 5-48. */
    ldr     r2, =FLASH_EE_CTRL1_ADDR
    ldr     r0, =(0x0A << FLASH_EE_CTRL1_EE_EDACMODE_SHIFT \
                  | FLASH_EE_CTRL1_EE_EOFEN \
                  | FLASH_EE_CTRL1_EE_EZFEN \
                  | FLASH_EE_CTRL1_EE_EPEN \
                  | 0 * FLASH_EE_CTRL1_EE_ALL1_OK \
                  | 0 * FLASH_EE_CTRL1_EE_ALL0_OK \
                  | 0x0A << FLASH_EE_CTRL1_EE_EDACEN_SHIFT)
    str     r0, [r2]

    /* Set EE_CTRL2 EE error correction threshold (0=disabled). */
    ldr     r2, =FLASH_EE_CTRL2_ADDR
    ldr     r0, =0 << FLASH_EE_CTRL2_EE_SEC_THRESHOLD_SHIFT
    str     r0, [r2]
    dmb

    /* See TMS570LS1227 datasheet 4.10.3 "ECC Protection for Flash Accesses". */
    mrc     p15, #0, r1, c9, c12, #0
    orr     r1, r1, #PMCR_X  /* Set X (bit 4) to enable SECDED. */
    dmb
    mcr     p15, #0, r1, c9, c12, #0

    /* See TMS570LS1227 datasheet 4.10.3 "ECC Protection for Flash Accesses".
     * See TMS570 TRM 2.2.3.2 "ECC Protection for Flash Accesses".
     * See Cortex-R4/R4F TRM 4.3.16 "c1, Auxiliary Control Register". */
    mrc     p15, #0, r1, c1, c0, #1
    orr     r1, r1, #ACTLR_ATCMPCEN  /* Set ATCMPCEN (bit 25) to enable ATCM. */
    dmb
    mcr     p15, #0, r1, c1, c0, #1
    dmb
    isb
    bx      lr


    .global StartupFlashDisableEcc
    .thumb_func
StartupFlashDisableEcc:
    /* Disable Single-Error-Correction-Double-Error-Detection (SECDED) logic. */
    dmb
    isb

    /* Set FEDACCTRL1 to disable error detection and correction on bank 0. */
    /* See TMS570 TRM Table 5-15. */
    ldr     r2, =FLASH_FEDACCTRL1_ADDR
    ldr     r0, =(FLASH_FEDACCTRL1_SUSP_IGNR \
                  | 0x05 << FLASH_FEDACCTRL1_EDACMODE_SHIFT \
                  | 0 * FLASH_FEDACCTRL1_EOFEN \
                  | 0 * FLASH_FEDACCTRL1_EZFEN \
                  | 0 * FLASH_FEDACCTRL1_EPEN \
                  | 0x05 << FLASH_FEDACCTRL1_EDACEN_SHIFT)
    str     r0, [r2]

    /* Set EE_CTRL1 to disable error detection and correction on bank 7. */
    /* See TMS570 TRM Table 5-48. */
    ldr     r2, =FLASH_EE_CTRL1_ADDR
    ldr     r0, =(0x05 << FLASH_EE_CTRL1_EE_EDACMODE_SHIFT \
                  | 0 * FLASH_EE_CTRL1_EE_EOFEN \
                  | 0 * FLASH_EE_CTRL1_EE_EZFEN \
                  | 0 * FLASH_EE_CTRL1_EE_EPEN \
                  | FLASH_EE_CTRL1_EE_ALL1_OK \
                  | FLASH_EE_CTRL1_EE_ALL0_OK \
                  | 0x05 << FLASH_EE_CTRL1_EE_EDACEN_SHIFT)
    str     r0, [r2]
    dmb

    /* See TMS570LS1227 datasheet 4.10.3 "ECC Protection for Flash Accesses". */
    mrc     p15, #0, r1, c9, c12, #0
    bic     r1, #PMCR_X  /* Clear X (bit 4) to disable SECDED. */
    dmb
    mcr     p15, #0, r1, c9, c12, #0

    /* See TMS570LS1227 datasheet 4.10.3 "ECC Protection for Flash Accesses".
     * See TMS570 TRM 2.2.3.2 "ECC Protection for Flash Accesses".
     * See Cortex-R4/R4F TRM 4.3.16 "c1, Auxiliary Control Register". */
    mrc     p15, #0, r1, c1, c0, #1
    bic     r1, #ACTLR_ATCMPCEN  /* Clear ATCMPCEN (bit 25) to disable ATCM. */
    dmb
    mcr     p15, #0, r1, c1, c0, #1
    dmb
    isb
    bx      lr


    .global StartupFlashSetSpeed
    .thumb_func
StartupFlashSetSpeed:
    /* Call this function before configuring the system clock to avoid
     * overclocking the flash memory. */

    /* Set timing for main flash bank. */
    /* See TMS570LS1227 Figure 5-1 "Wait State Scheme" for required settings.
     * For 160MHz HCLK, we require RWAIT=3 and ASWSTEN=1. */
    ldr     r2, =FLASH_FRDCNTL_ADDR
    ldr     r0, =(0x03 << FLASH_FRDCNTL_RWAIT_SHIFT \
                  | FLASH_FRDCNTL_ASWSTEN \
                  | FLASH_FRDCNTL_ENPIPE)  /* See TMS570 TRM Table 5-14. */
    str     r0, [r2]

    /* Enable writes to registers 0xFFF87200 to 0xFFF872FF (EEPROM). */
    ldr     r2, =FLASH_FSM_WR_ENA_ADDR
    mov     r0, #0x05 << FLASH_FSM_WR_ENA_WR_ENA_SHIFT
    str     r0, [r2]
    dmb

    /* Set timing for EEPROM emulation flash bank (bank 7). */
    /* See TMS570 TRM Table 5-46. */
    ldr     r2, =FLASH_EEPROM_CONFIG_ADDR
    ldr     r0, =(0x03 << FLASH_EEPROM_CONFIG_EWAIT_SHIFT \
                  | 0 * FLASH_EEPROM_CONFIG_AUTOSUSP_EN \
                  | FLASH_EEPROM_CONFIG_AUTOSTART_GRACE_MASK)
    str     r0, [r2]
    dmb
    bx      lr

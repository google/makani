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

/* TMS570 eFuse Controller (EFC) startup sequence. */

#include "avionics/firmware/cpu/registers_def.h"
#include "avionics/firmware/startup/return_codes.h"

    .section ".text.startup", "xa"
    .syntax unified


    .global StartupEFuseStartSelfTest
    .thumb_func
StartupEFuseStartSelfTest:
    /* See TMS570 TRM 33.3.2.5 "eFuse ECC Logic Self Test". */
    /* Set EFCSTCY to number self test of cycles. */
    ldr     r2, =EFC_STCY_ADDR
    ldr     r0, =600  /* Recommended value, see TMS570 TRM Table 33-6. */
    str     r0, [r2]
    /* Set EFCSTSIG to signature value. */
    ldr     r2, = EFC_STSIG_ADDR
    ldr     r0, =0x5362F97F  /* Signature value, see TMS570 TRM Table 33-7. */
    str     r0, [r2]
    dmb
    /* Set EFCBOUND to trigger self test. See TMS570 Table 33-3. */
    ldr     r2, =EFC_BOUND_ADDR
    ldr     r0, =EFC_BOUND_ECC_SELF_TEST_ENABLE | EFC_BOUND_INPUT_ENABLE_MASK
    str     r0, [r2]
    /* Self test takes 610 VCLK cycles to complete. We will poll EFCPINS later
     * to check the results. */
    bx      lr


    .global StartupEFuseWaitForSelfTest
    .thumb_func
StartupEFuseWaitForSelfTest:
    /* See TMS570 TRM Table 33-4 "EFC Pins Register". */
    ldr     r2, =EFC_PINS_ADDR
    ldr     r0, [r2]
    ands    r0, #EFC_PINS_SELF_TEST_DONE  /* True when complete. */
    beq     StartupEFuseWaitForSelfTest
    /* Determine if it is safe to continue. */
    ldr     r2, =EFC_PINS_ADDR
    ldr     r0, [r2]
    ands    r0, #EFC_PINS_SELF_TEST_ERROR  /* True indicates failure. */
    bne     selftest_failure
    mov     r0, #RETURN_SUCCESS
    bx      lr
selftest_failure:
    mov     r0, #RETURN_FAIL_EFUSE_SELF_TEST
    bx      lr


    .global StartupEFuseWaitForSelfTestOrDie
    .thumb_func
StartupEFuseWaitForSelfTestOrDie:
    mov     r3, lr
    bl      StartupEFuseWaitForSelfTest
    cbnz    r0, selftest_die
    bx      r3
selftest_die:
    b       StartupEmitFailureCodeInR0AndDie

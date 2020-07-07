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

/* TMS570 Error Signalling Module (ESM) startup sequence. */

#include "avionics/firmware/cpu/registers_def.h"

    .section ".text.startup", "xa"
    .syntax unified


    .global StartupEsmCheckGroup3
    .thumb_func
StartupEsmCheckGroup3:
    /* Group 3 errors are critical and it is therefore not safe to continue. */
    ldr     r2, =ESM_SR3_ADDR
    ldr     r0, [r2]
    cbnz    r0, esm3_failure
    bx      lr
esm3_failure:
    wfi
    b       esm3_failure

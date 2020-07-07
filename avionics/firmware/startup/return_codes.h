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

#ifndef AVIONICS_FIRMWARE_STARTUP_RETURN_CODES_H_
#define AVIONICS_FIRMWARE_STARTUP_RETURN_CODES_H_

/* The startup code clocks out the return code upon failure. */

#define RETURN_SUCCESS 0x0

/* See cpu_tms570.S. */
#define RETURN_FAIL_CPU_LBIST                   0x1000
#define RETURN_FAIL_CPU_LBIST_ESM               0x1001
#define RETURN_FAIL_CPU_COMPARE_SELF_TEST       0x1002
#define RETURN_FAIL_CPU_COMPARE_FORCE           0x1003
#define RETURN_FAIL_CPU_COMPARE_SELF_TEST_FORCE 0x1004
#define RETURN_FAIL_CPU_CONTEXT_FAILURE         0xDEAD

/* See efuse_tms570.S. */
#define RETURN_FAIL_EFUSE_SELF_TEST 0x4000

/* See ram_tms570.S. */
#define RETURN_FAIL_RAM_PBIST_SELF_TEST 0x5000
#define RETURN_FAIL_RAM_PBIST_FAILURE   0x5001

#endif  // AVIONICS_FIRMWARE_STARTUP_RETURN_CODES_H_

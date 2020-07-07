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

#ifndef AVIONICS_FIRMWARE_STARTUP_CPU_TMS570_H_
#define AVIONICS_FIRMWARE_STARTUP_CPU_TMS570_H_

#include "avionics/firmware/startup/return_codes.h"

typedef enum {
  kStartupReturnSuccess                     = RETURN_SUCCESS,
  kStartupReturnFailCpuLbist                = RETURN_FAIL_CPU_LBIST,
  kStartupReturnFailCpuLbistEsm             = RETURN_FAIL_CPU_LBIST_ESM,
  kStartupReturnFailCpuCompareSelfTest      = RETURN_FAIL_CPU_COMPARE_SELF_TEST,
  kStartupReturnFailCpuCompareForce         = RETURN_FAIL_CPU_COMPARE_FORCE,
  kStartupReturnFailCpuCompareSelfTestForce =
  RETURN_FAIL_CPU_COMPARE_SELF_TEST_FORCE,
  kStartupReturnFailCpuContextFailure       = RETURN_FAIL_CPU_CONTEXT_FAILURE,
  kStartupReturnFailEfuseSelfTest           = RETURN_FAIL_EFUSE_SELF_TEST,
} StartupReturn;

bool StartupCpuSaveContext(void);  // Returns true on restore.
void StartupCpuRestoreContext(void);

bool StartupCpuPushContext(void);  // Returns true on pop.
void StartupCpuPopContext(void);

// CPU self-tests reboot upon completion. Call StartupCpuPushContext before
// this function to store the context and resume properly.
void StartupCpuRunSelfTestDiag(void);
void StartupCpuRunSelfTest(void);

// Start CPU lockstep compare test. Call StartupCpuWaitForCompareSelfTest to
// block and wait for completion.
void StartupCpuStartCompareSelfTest(void);
StartupReturn StartupCpuWaitForCompareSelfTest(void);

StartupReturn StartupCpuRunCompareForceErrorTest(void);
StartupReturn StartupCpuRunCompareSelfTestForceError(void);

#endif  // AVIONICS_FIRMWARE_STARTUP_CPU_TMS570_H_

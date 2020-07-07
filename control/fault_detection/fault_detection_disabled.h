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

// RunFaultDetectionDisabled is responsible for kFaultTypeDisabled.
//
// These faults indicate sensors which are not installed for a given
// wing or otherwise should not be used in the controller due to
// missing calibration or hardware.  They do not change during flight.

#ifndef CONTROL_FAULT_DETECTION_FAULT_DETECTION_DISABLED_H_
#define CONTROL_FAULT_DETECTION_FAULT_DETECTION_DISABLED_H_

#include "control/fault_detection/fault_detection_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void SetDisabledFaults(const FaultDetectionDisabledParams *params,
                       FaultMask faults[]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_FAULT_DETECTION_FAULT_DETECTION_DISABLED_H_

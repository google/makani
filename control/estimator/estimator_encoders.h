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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_ENCODERS_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_ENCODERS_H_

#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the encoder estimate states.
void EstimatorEncodersInit(EstimatorEncodersState *state);

// Populate encoder estimates on the wing.
//
// If a fault is declared, the last valid angle is held.
void EstimatorEncodersStep(
    const GsgData gsg[], const FaultMask *gsg_a_faults,
    const FaultMask *gsg_b_faults, const double levelwind_ele[],
    const FaultMask *levelwind_ele_a_fault,
    const FaultMask *levelwind_ele_b_fault, const double perch_azi[],
    const FaultMask *perch_azi_a_fault, const FaultMask *perch_azi_b_fault,
    EstimatorEncodersState *state, EncodersEstimate *encoders);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_ENCODERS_H_

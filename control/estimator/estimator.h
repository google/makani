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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_H_

#include <stdbool.h>

#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the state of the estimator.
void EstimatorInit(const SystemParams *system_params,
                   const EstimatorParams *params, EstimatorState *state);

// Test valid messages are simultaneously available for a critical
// list of sensors.  This must return true before the control system will
// advance out of the kInitializationStateWaitForValidData.
bool EstimatorIsDataReady(FlightPlan flight_plan, const FaultMask faults[]);

// Run a single step of the estimator.
void EstimatorStep(const FlightStatus *flight_status,
                   const ControlInput *control_input,
                   const SystemParams *system_params, const FaultMask faults[],
                   const EstimatorParams *params, EstimatorState *state,
                   StateEstimate *state_est);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_H_

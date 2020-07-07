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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_WINCH_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_WINCH_H_

#include <stdbool.h>

#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the winch state.
void EstimatorWinchInit(FlightPlan flight_plan, GroundStationModel gs_model,
                        double drum_radius, EstimatorWinchState *state);

// Set the payout value to be associated with the last valid winch
// position estimate.
void EstimatorWinchSetPayout(double payout, const WinchParams *winch_params,
                             EstimatorWinchState *state);

// Estimate the winch position, payout, and proximity sensor state.
//
// If any fault occurs the winch position holds the last valid value.
void EstimatorWinchStep(double winch_pos, const FaultMask *winch_fault,
                        bool proximity, const FaultMask *proximity_fault,
                        bool perched, GroundStationModel gs_model,
                        const WinchParams *winch_params,
                        EstimatorWinchState *state, WinchEstimate *winch);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_WINCH_H_

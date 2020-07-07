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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_TETHER_GROUND_ANGLES_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_TETHER_GROUND_ANGLES_H_

#include "avionics/common/plc_messages.h"
#include "common/c_math/util.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize tether elevation state.
void EstimatorTetherGroundAnglesInit(EstimatorTetherGroundAnglesState *state);

// Estimate the tether elevation angle near the GS anchor point.
void EstimatorTetherGroundAnglesStep(
    bool initializing, GroundStationModel gs_model,
    const GroundStationEstimate *ground_station, const WinchEstimate *winch,
    const VesselEstimate *vessel, const WinchParams *winch_params,
    const EstimatorTetherGroundAnglesParams *params,
    const EncodersEstimate *encoders, double detwist_ele,
    EstimatorTetherGroundAnglesState *state,
    TetherGroundAnglesEstimate *tether_angles);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_TETHER_GROUND_ANGLES_H_

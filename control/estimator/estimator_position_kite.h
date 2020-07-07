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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_POSITION_KITE_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_POSITION_KITE_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool EstimatorPositionKiteValidateParams(const EstimatorPositionParams *params);

// Initialize the position estimator state.
void EstimatorPositionKiteInit(const EstimatorPositionParams *params,
                               EstimatorPositionState *state);

// Run the position estimator.
void EstimatorPositionKiteStep(
    const Vec3 *acc, const Vec3 *pqr, const Mat3 *dcm_g2b,
    const GpsData wing_gps[], const GsGpsData *gs_gps,
    const GroundStationPoseEstimate *ground_station,
    const EncodersEstimate *encoders, const PerchAziEstimate *perch_azi,
    const TetherForceEstimate *tether_force_b, const WinchEstimate *winch,
    const PitotData pitots[], FlightMode flight_mode, const FaultMask faults[],
    const SystemParams *system_params, const EstimatorPositionParams *params,
    EstimatorPositionState *state, Vec3 *Xg, Vec3 *Vg,
    EstimatorVelocitySolutionType *vel_type);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_POSITION_KITE_H_

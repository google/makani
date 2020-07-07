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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_POSITION_GROUND_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_POSITION_GROUND_H_

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

// Validate the position estimator parameters.
bool EstimatorPositionGroundValidateParams(
    const EstimatorPositionParams *params);

// Populate the ground frame using the origin in ecef.
void EstimatorPositionGroundInitializeGroundFrame(
    const Vec3 *origin_ecef, GroundStationPoseEstimate *ground_frame);

// Initialize the position estimator state.
void EstimatorPositionGroundInit(const SystemParams *system_params,
                                 const EstimatorPositionParams *params,
                                 EstimatorPositionGroundState *state);

// Run the position estimator.
void EstimatorPositionGroundStep(const Vec3 *acc, const Vec3 *pqr,
                                 const Mat3 *dcm_g2p, const GpsData *gps_data,
                                 const FaultMask faults[],
                                 const SystemParams *system_params,
                                 const EstimatorPositionParams *params,
                                 EstimatorPositionGroundState *state, Vec3 *Xg,
                                 Vec3 *Vg,
                                 EstimatorVelocitySolutionType *vel_type);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_POSITION_GROUND_H_

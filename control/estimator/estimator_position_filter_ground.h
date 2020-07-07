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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_POSITION_FILTER_GROUND_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_POSITION_FILTER_GROUND_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/control_types.h"
#include "control/estimator/estimator_position_filter.h"
#include "control/estimator/estimator_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Run one step of the position filter.
//
// Args:
//   dcm_g2b: Estimated rotation matrix from g-coordinates to p-coordinates.
//   acc: Specific force measurement [m/s^2] compensated for lever-arm.
//   gps_estimate: GPS position and velocity estimates
//   params: Parameters.
//   state: State.
//   correct: The filter corrections.
//   Xg: Output kite position [m] estimate.
//   Vg: Output kite velocity [m/s] estimate.
//   vel_type: Current velocity estimate type.
void EstimatorPositionFilterGroundStep(
    const Mat3 *dcm_g2b, const Vec3 *acc,
    const EstimatorPositionGpsEstimate *gps_estimate,
    const EstimatorPositionFilterParams *params,
    EstimatorPositionFilterState *state, EstimatorPositionCorrections *correct,
    Vec3 *Xg, Vec3 *Vg, EstimatorVelocitySolutionType *vel_type);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_POSITION_FILTER_GROUND_H_

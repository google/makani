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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_POSITION_FILTER_KITE_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_POSITION_FILTER_KITE_H_

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
//   dcm_g2b: Estimated rotation matrix from g-coordinates to b-coordinates.
//   acc: Specific force measurement [m/s^2] compensated for lever-arm.
//   baro: Barometric altitude estimate.
//   glas: Ground-line-angle sensor position estimate.
//   gps_center: GPS position and velocity estimates from the center antenna.
//   gps_port: GPS position and velocity estimates from the port antenna.
//   gps_star: GPS position and velocity estimates from the starboard antenna.
//   flight_mode: Flight mode.
//   params: Parameters.
//   state: State.
//   correct: The filter corrections.
//   Xg: Output kite position [m] estimate.
//   Vg: Output kite velocity [m/s] estimate.
//   vel_type: Current velocity estimate type.
void EstimatorPositionFilterKiteStep(
    const Mat3 *dcm_g2b, const Vec3 *acc,
    const EstimatorPositionBaroEstimate *baro,
    const EstimatorPositionGlasEstimate *glas,
    const EstimatorPositionGpsEstimate *gps_center,
    const EstimatorPositionGpsEstimate *gps_port,
    const EstimatorPositionGpsEstimate *gps_star, FlightMode flight_mode,
    const EstimatorPositionFilterParams *params,
    EstimatorPositionFilterState *state, EstimatorPositionCorrections *correct,
    Vec3 *Xg, Vec3 *Vg, EstimatorVelocitySolutionType *vel_type);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_POSITION_FILTER_KITE_H_

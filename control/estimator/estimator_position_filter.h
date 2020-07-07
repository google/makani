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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_POSITION_FILTER_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_POSITION_FILTER_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Apply the error from x_hat to the state variables.
void ApplyPositionErrorState(const Vec *x_hat,
                             EstimatorPositionFilterState *state);

// Apply gps velocity correction to filter.
bool ApplyGpsVelocityCorrection(const EstimatorPositionGpsEstimate *gps,
                                const EstimatorPositionFilterParams *params,
                                Vec *x_hat, EstimatorPositionFilterState *state,
                                EstimatorPositionCorrection3 *correct);

// Apply gps position correction to filter.
bool ApplyGpsPositionCorrection(const EstimatorPositionGpsEstimate *gps,
                                const EstimatorPositionFilterParams *params,
                                Vec *x_hat, EstimatorPositionFilterState *state,
                                EstimatorPositionCorrection3 *correct);

void CorrectPositionVector(const Vec3 *pos_meas_g, const Vec3 *sigma_meas_g,
                           Vec *x_hat, EstimatorPositionFilterState *state,
                           EstimatorPositionCorrection3 *correct);

// Validate position filter parameters.
bool EstimatorPositionFilterValidateParams(
    const EstimatorPositionFilterParams *params);

// Initialize the position filter.
void EstimatorPositionFilterInit(const EstimatorPositionFilterParams *params,
                                 EstimatorPositionFilterState *state);

void EstimatorPositionFilterPropagate(
    const Mat3 *dcm_g2b, const Vec3 *acc, bool hover_flight_mode,
    const EstimatorPositionFilterParams *params,
    EstimatorPositionFilterState *state);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_POSITION_FILTER_H_

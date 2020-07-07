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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_POSITION_GLAS_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_POSITION_GLAS_H_

#include <stdbool.h>

#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/sensor_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Validate the GLAS estimator parameters.
bool EstimatorPositionGlasValidateParams(
    const EstimatorPositionGlasParams *params);

// Initialize the GLAS estimator.
void EstimatorPositionGlasInit(EstimatorPositionGlasState *state);

// Update biases on the GSG angles based on a trusted position measurement.
void EstimatorPositionGlasUpdateGsgBias(
    const Vec3 *Xg, const EncodersEstimate *encoders,
    const PerchAziEstimate *perch_azi,
    const TetherForceEstimate *tether_force_b, const WinchEstimate *winch,
    const GsgParams *gsg_params, const WinchParams *winch_params,
    const EstimatorPositionGlasParams *params,
    EstimatorPositionGlasState *state);

// Run one step of the GLAS estimation.
void EstimatorPositionGlasStep(const EncodersEstimate *encoders,
                               const PerchAziEstimate *perch_azi,
                               const TetherForceEstimate *tether_force_b,
                               const WinchEstimate *winch,
                               const SystemParams *system_params,
                               const EstimatorPositionGlasParams *params,
                               EstimatorPositionGlasState *state,
                               EstimatorPositionGlasEstimate *glas);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_POSITION_GLAS_H_

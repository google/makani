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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_APPARENT_WIND_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_APPARENT_WIND_H_

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void EstimatorApparentWindInit(EstimatorApparentWindState *state);

void EstimatorApparentWindStep(
    const PitotData pitots[], const TetherForceEstimate *tether_force_b,
    const WindEstimate *wind_g, const Mat3 *dcm_g2b, const Vec3 *pqr,
    const Vec3 *acc, const Vec3 *Ab_f, const Vec3 *Vg, const FaultMask faults[],
    const SystemParams *system_params,
    const EstimatorApparentWindParams *params,
    EstimatorApparentWindState *state, ApparentWindEstimate *apparent_wind,
    ApparentWindSph *apparent_wind_pitot_out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_APPARENT_WIND_H_

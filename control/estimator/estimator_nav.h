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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_NAV_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_NAV_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the (common) navigation estimator state.
void EstimatorNavInit(const EstimatorNavParams *params, double g_heading,
                      EstimatorNavState *state);

bool IsImuValid(const FaultMask imu_faults[]);

void FilterInertialMeasurements(const Vec3 *pqr, const Vec3 *acc_b,
                                const EstimatorNavParams *params,
                                EstimatorNavState *state, Vec3 *pqr_f,
                                Vec3 *acc_b_f, double *acc_norm_f);

void FilterVelocity(const Mat3 *dcm_g2b, const Vec3 *Vb,
                    const EstimatorNavParams *params, EstimatorNavState *state,
                    Vec3 *Vb_f, Vec3 *Vg_f);

// Calculate representation of the inertial state in alternative coordinates.
void CalcAlternativeCoordinates(const Mat3 *dcm_g2b, const Vec3 *acc_b,
                                const Vec3 *acc_b_f, const Vec3 *Vg,
                                const SystemParams *system_params, Vec3 *Vb,
                                Vec3 *Ag, Vec3 *Ab_f);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_NAV_H_

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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_POSITION_BARO_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_POSITION_BARO_H_

#include <stdbool.h>

#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Validate the barometric altitude parameters.
bool EstimatorPositionBaroValidateParams(
    const EstimatorPositionBaroParams *params);

// Initialize the barometric altitude state.
void EstimatorPositionBaroInit(const EstimatorPositionBaroParams *params,
                               EstimatorPositionBaroState *state);

// Update the barometric altitude's bias based on another position
// estimate.
//
// Args:
//   Xg_z: Measured g-frame z coordinate to update the bias with.
//   sigma_Xg_z: Standard deviation [m] for the uncertainty in Xg_z.
//   baro: Most recent output of EstimatorPositionBaroStep.
//   params: Parameters.
//   state: State.
void EstimatorPositionBaroUpdateBias(double Xg_z, double sigma_Xg_z,
                                     const EstimatorPositionBaroEstimate *baro,
                                     const EstimatorPositionBaroParams *params,
                                     EstimatorPositionBaroState *state);

void EstimatorPositionBaroStep(const Mat3 *dcm_g2b, const PitotData pitots[],
                               const FaultMask *pitot_high_speed_static_fault,
                               const FaultMask *pitot_low_speed_static_fault,
                               const PitotParams *pitot_params,
                               const EstimatorPositionBaroParams *params,
                               EstimatorPositionBaroState *state,
                               EstimatorPositionBaroEstimate *baro);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_POSITION_BARO_H_

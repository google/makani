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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_TETHER_FORCE_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_TETHER_FORCE_H_

#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize tether force estimate state.
void EstimatorTetherForceInit(EstimatorTetherForceState *state);

// Estimate tether force and tension.
//
// Args:
//   loadcells: Array of kNumLoadcellSensors load cell values.
//   loadcell_faults: Array of kNumLoadcell faults.
//   wing_params: Wing parameters, to provide bridle information.
//   loadcell_params: Array of kNumLoadcell parameters.
//   params: Parameters.
//   state: State of the tether force estimator.
//   tether_force_b: Output tether force information.
void EstimatorTetherForceStep(const double loadcells[],
                              const FaultMask loadcell_faults[],
                              const WingParams *wing_params,
                              const LoadcellParams loadcell_params[],
                              const EstimatorTetherForceParams *params,
                              EstimatorTetherForceState *state,
                              TetherForceEstimate *tether_force_b);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_TETHER_FORCE_H_

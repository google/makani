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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_EXPERIMENT_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_EXPERIMENT_H_

#include <stdbool.h>

#include "control/experiments/experiment_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void EstimatorExperimentInit(ExperimentState *state);

void EstimatorExperimentStep(double joystick_pitch,
                             ExperimentType experiment_type,
                             uint8_t experiment_case_id,
                             ExperimentState *experiment_state,
                             ExperimentState *experiment_state_est);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_EXPERIMENT_H_

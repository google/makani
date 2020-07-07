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

#ifndef CONTROL_EXPERIMENTS_EXPERIMENT_UTIL_H_
#define CONTROL_EXPERIMENTS_EXPERIMENT_UTIL_H_

#include <stdbool.h>
#include <stdint.h>

#include "control/control_params.h"
#include "control/experiments/experiment_types.h"

#ifdef __cplusplus
extern "C" {
#endif

int32_t GetNumberOfExperimentCases(ExperimentType experiment_type,
                                   const ControlParams *control_params);

bool IsExperimentValid(ExperimentType experiment_type, uint8_t case_id,
                       const ControlParams *control_params);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_EXPERIMENTS_EXPERIMENT_UTIL_H_

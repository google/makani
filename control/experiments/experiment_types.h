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

#ifndef CONTROL_EXPERIMENTS_EXPERIMENT_TYPES_H_
#define CONTROL_EXPERIMENTS_EXPERIMENT_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// List of experiments. There should be a one-to-one mapping between this list
// and all the experiments under //config/m600/control/experiments/
typedef enum {
  kExperimentTypeNoTest,
  kExperimentTypeHoverElevator,
  kExperimentTypeCrosswindSpoiler,
  kNumExperimentTypes,
} ExperimentType;

typedef struct {
  // active_type: The experiment that is being flown.
  // staged_type: The experiment that is staged but not flown yet.
  ExperimentType active_type, staged_type;

  // case_id: The experiment case number being (or to be) flown.
  uint8_t case_id;
} ExperimentState;

const char *ExperimentTypeToString(ExperimentType experiment_type);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_EXPERIMENTS_EXPERIMENT_TYPES_H_

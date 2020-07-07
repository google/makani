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

#include "control/experiments/experiment_util.h"

int32_t GetNumberOfExperimentCases(ExperimentType experiment_type,
                                   const ControlParams *control_params) {
  switch (experiment_type) {
    case kExperimentTypeHoverElevator:
      return ARRAYSIZE(control_params->hover.experiments.hover_elevator);
    case kExperimentTypeCrosswindSpoiler:
      return ARRAYSIZE(control_params->crosswind.experiments.crosswind_spoiler);
    default:
    case kExperimentTypeNoTest:
    case kNumExperimentTypes:
      assert(false);
      return 0;
  }
}

bool IsExperimentValid(ExperimentType experiment_type, uint8_t case_id,
                       const ControlParams *control_params) {
  if (experiment_type >= kNumExperimentTypes) {
    return false;
  }
  if (case_id >= GetNumberOfExperimentCases(experiment_type, control_params)) {
    return false;
  }
  return true;
}

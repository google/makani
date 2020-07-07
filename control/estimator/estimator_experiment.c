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

#include "control/estimator/estimator_experiment.h"

#include <assert.h>

#include "control/experiments/experiment_util.h"

void EstimatorExperimentInit(ExperimentState *state) {
  state->active_type = kExperimentTypeNoTest;
  state->staged_type = kExperimentTypeNoTest;
  state->case_id = 0U;
}

static bool IsReadyToActivateExperiment(double joystick_pitch) {
  return joystick_pitch < -0.7;
}

static bool IsReadyToDeactivateExperiment(double joystick_pitch) {
  return joystick_pitch > -0.3;
}

void EstimatorExperimentStep(double joystick_pitch,
                             ExperimentType experiment_type,
                             uint8_t experiment_case_id,
                             ExperimentState *experiment_state,
                             ExperimentState *experiment_state_est) {
  // A kExperimentTypeNoTest command is regarded as a no-op.
  // We solely rely on joystick pitch to go back to nominal flight.
  if (experiment_type != kExperimentTypeNoTest &&
      IsExperimentValid(experiment_type, experiment_case_id,
                        GetControlParams()) &&
      (experiment_state->active_type == kExperimentTypeNoTest ||
       experiment_state->active_type == experiment_type)) {
    // Cannot change experiment type when an experiment is running.
    experiment_state->staged_type = experiment_type;
    experiment_state->case_id = experiment_case_id;
  }

  // Activate/Deactivate the experiment.
  if (IsReadyToActivateExperiment(joystick_pitch) &&
      experiment_state->active_type == kExperimentTypeNoTest) {
    // Activate a new experiment only after the flight is back to default.
    experiment_state->active_type = experiment_state->staged_type;
  } else if (IsReadyToDeactivateExperiment(joystick_pitch) &&
             experiment_state->active_type != kExperimentTypeNoTest) {
    // Deactivate a new experiment to the default flight.
    experiment_state->active_type = kExperimentTypeNoTest;
    // Do not reset the staged experiment type, so that the operator can
    // use the joystick to redo the same test again.
  }
  // Otherwise, keep flying in the same experiment (or default flight).
  // Note that the experiment_case_id can still be changed and become
  // effective immediately during the same experiment.

  assert(experiment_state->active_type == experiment_state->staged_type ||
         experiment_state->active_type == kExperimentTypeNoTest);

  *experiment_state_est = *experiment_state;
}

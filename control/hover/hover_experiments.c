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

#include "control/hover/hover_experiments.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

#include "control/control_types.h"
#include "control/experiments/hover_experiment_types.h"
#include "control/system_params.h"

void HoverExperimentsInit(const HoverExperiments *experiments,
                          HoverExperimentState *state) {
  assert(experiments != NULL);
  assert(experiments->elevator_rate_limit >= 0.0);
  state->elevator_cmd_z1 = 0.0;
}

bool HoverElevatorExperimentIsEnabled(const FlightStatus *flight_status,
                                      const ExperimentState *experiment_state) {
  return (experiment_state->active_type == kExperimentTypeHoverElevator) &&
         (flight_status->flight_mode == kFlightModeHoverTransformGsUp ||
          flight_status->flight_mode == kFlightModeHoverTransformGsDown);
}

void HoverElevatorExperimentsOutputSignal(const FlightStatus *flight_status,
                                          const ControlOutput *control_output,
                                          const StateEstimate *state_est,
                                          const HoverExperiments *experiments,
                                          HoverExperimentState *state,
                                          ControlOutput *control_output_out) {
  *control_output_out = *control_output;
  if (HoverElevatorExperimentIsEnabled(flight_status, &state_est->experiment)) {
    const HoverElevatorExperiment exp_config =
        experiments->hover_elevator[state_est->experiment.case_id];

    RateLimit(exp_config.elevator, -experiments->elevator_rate_limit,
              experiments->elevator_rate_limit, *g_sys.ts,
              &state->elevator_cmd_z1);
    control_output_out->flaps[kFlapEle] = state->elevator_cmd_z1;
  } else {
    state->elevator_cmd_z1 = control_output->flaps[kFlapEle];
  }
}

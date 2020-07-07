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

#ifndef CONTROL_HOVER_HOVER_EXPERIMENTS_H_
#define CONTROL_HOVER_HOVER_EXPERIMENTS_H_

#include <stdbool.h>

#include "control/control_types.h"
#include "control/experiments/hover_experiment_types.h"
#include "control/hover/hover_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void HoverExperimentsInit(const HoverExperiments *experiments,
                          HoverExperimentState *state);

bool HoverElevatorExperimentIsEnabled(const FlightStatus *flight_status,
                                      const ExperimentState *experiment_state);

void HoverElevatorExperimentsOutputSignal(const FlightStatus *flight_status,
                                          const ControlOutput *control_output,
                                          const StateEstimate *state_est,
                                          const HoverExperiments *experiments,
                                          HoverExperimentState *state,
                                          ControlOutput *control_output_out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_EXPERIMENTS_H_

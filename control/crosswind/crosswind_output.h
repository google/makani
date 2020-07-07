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

#ifndef CONTROL_CROSSWIND_CROSSWIND_OUTPUT_H_
#define CONTROL_CROSSWIND_CROSSWIND_OUTPUT_H_

#include "control/actuator_util.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_types.h"
#include "control/estimator/estimator_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Validate the parameters of the output stage.
bool CrosswindOutputValidateParams(const CrosswindOutputParams *params);

// Initializes the state of the output stage.
void CrosswindOutputInit(const CrosswindOutputParams *params,
                         const HoverOutputState *hover_output_state,
                         CrosswindOutputState *state,
                         double previous_detwist_loop_angle,
                         int32_t previous_detwist_rev_count);

// Updates the state of the output stage by a single time step.
// Returns the final control output.
void CrosswindOutputStep(LoopDirection loop_dir, double loop_angle,
                         bool allow_flare, const ThrustMoment *thrust_moment,
                         const Deltas *deltas, const StateEstimate *state_est,
                         const Vec3 *path_center_g,
                         const CrosswindOutputParams *params,
                         CrosswindOutputState *state,
                         ControlOutput *control_output,
                         Deltas *deltas_available);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_OUTPUT_H_

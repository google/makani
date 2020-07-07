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

//                                   POSITION
//                PATH    TENSION    ALTITUDE     ANGLES
//   ---------------|--------|----------|------------|----------
//    winch_speed   | Xg_cmd |          | eulers_cmd | moment_i
//    wind          | Vg_cmd |          |            |
//    position      |        | pitch_ff |            |
//    horz. tension |        |          |            |
//                  |        |          | thrust     |
//
//             OUTPUT
// --------------|---------
//  moment_i     | rotor_i
//  int_moment_y |
//  thrust       | flap_i

#ifndef CONTROL_HOVER_HOVER_H_
#define CONTROL_HOVER_HOVER_H_

#include <stdbool.h>

#include "control/control_types.h"
#include "control/crosswind/crosswind_playbook_types.h"
#include "control/estimator/estimator_types.h"
#include "control/hover/hover_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the state of the hover controller.
void HoverInit(const StateEstimate *state_est,
               const Vec3 *previous_perched_pos_g,
               double previous_int_yaw_angle, double previous_thrust_cmd,
               const double *flaps_z1, const Vec3 *previous_loop_center_v,
               const HoverParams *params, HoverState *state);

// Determines whether the hover controller is ready to switch into any
// of the hover modes.
bool HoverIsReadyForMode(FlightMode flight_mode,
                         const FlightStatus *flight_status,
                         const StateEstimate *state_est,
                         const HoverParams *params, const HoverState *state);

// Retrieves the last thrust moment command issued by the hover controller.
const ThrustMoment *HoverGetLastThrustMomentCmd(const HoverState *state);

// Updates the state of the hover controller by a single time step.
// Returns the output commands to the actuators.
void HoverStep(const FlightStatus *flight_status,
               const StateEstimate *state_est, const Playbook *playbook,
               const HoverParams *params, HoverState *state,
               ControlOutput *control_output);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_H_

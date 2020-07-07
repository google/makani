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

#ifndef CONTROL_HOVER_HOVER_OUTPUT_H_
#define CONTROL_HOVER_HOVER_OUTPUT_H_

#include <stdbool.h>

#include "control/actuator_util.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/hover/hover_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the state of the output stage.
void HoverOutputInit(double joystick_throttle, double delta_elevator,
                     const Vec3 *cw_loop_center_v,
                     const HoverOutputParams *params, HoverOutputState *state);

// Returns true if the output parameters are valid.
bool HoverOutputValidateParams(const HoverOutputParams *params);

// Returns true if the output stage is done with the slow ramp in
// gains that is used to start the motors softly.
bool HoverOutputIsGainRampDone(const HoverOutputState *state);

// Returns the thrust output on the last step.
double HoverOutputGetLastThrust(const HoverOutputState *state);

// Updates the state of the output stage by a single time step.
// Returns the final control output.
void HoverOutputStep(const FlightStatus *flight_status,
                     const StateEstimate *state_est,
                     const Vec3 crosswind_loop_center_g,
                     const ThrustMoment *thrust_moment,
                     double blown_flaps_roll_moment,
                     double elevator_pitch_moment, double rudder_yaw_moment,
                     double winch_vel_cmd, const HoverOutputParams *params,
                     HoverOutputState *state, ControlOutput *control_output);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_OUTPUT_H_

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

#ifndef CONTROL_HOVER_HOVER_ANGLES_H_
#define CONTROL_HOVER_HOVER_ANGLES_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/experiments/experiment_types.h"
#include "control/hover/hover_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the angle loop state.
void HoverAnglesInit(const HoverAnglesParams *params, HoverAnglesState *state);

// Returns true if the hover angles parameters are valid.
bool HoverAnglesValidateParams(const HoverAnglesParams *params);

// Finds the axis-angle rotation (in the body frame) that restores the
// wing to the reference attitude.  The reference attitude is
// body-x-axis pointing up and body-z-axis parallel to the tether
// projected to the NED frame's xy-plane.
void HoverAnglesGetAngles(const Vec3 *wing_pos_g, const Vec3 *hover_origin_g,
                          const Mat3 *dcm_g2b, Vec3 *angles);

// Returns the axis-angle error between the commanded and measured
// orientation.
void HoverAnglesGetAnglesError(const HoverAnglesState *state,
                               Vec3 *angles_error);

// Runs the attitude control loop.  This takes an attitude command and
// an angular rate command and attempts to meet these commands with a
// moment command.
void HoverAnglesStep(const Vec3 *angles_cmd, const Vec3 *angles,
                     const Vec3 *pqr_cmd, const Vec3 *pqr,
                     const WinchEstimate *winch, const VesselEstimate *vessel,
                     double thrust_z1, const FlightStatus *flight_status,
                     const HoverAnglesParams *params,
                     const ExperimentState *experiment_state,
                     HoverAnglesState *state, Vec3 *moment_cmd,
                     double *blown_flaps_roll_moment_cmd,
                     double *elevator_pitch_moment_cmd,
                     double *rudder_yaw_moment_cmd);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_ANGLES_H_

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

#ifndef CONTROL_HOVER_HOVER_TENSION_H_
#define CONTROL_HOVER_HOVER_TENSION_H_

#include <stdbool.h>

#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/hover/hover_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the tension loop state.
void HoverTensionInit(const HoverTensionParams *params,
                      HoverTensionState *state);

// Returns true if the hover tension parameters are valid.
bool HoverTensionValidateParams(const HoverTensionParams *params);

// Sets the horizontal tension set-point.  This is chosen such that
// there is a minimum allowed tension, but above that point the rotors
// should not put any extra tension into the tether over what occurs
// naturally from drag.
double HoverTensionSetHorizontalTension(const Vec3 *wing_pos_v_g,
                                        const Vec3 *wind_g, double winch_vel,
                                        bool hold_horizontal_tension_cmd,
                                        const HoverTensionParams *params,
                                        HoverTensionState *state);

// Controls tension by modifying pitch angle using integration
// control.  When appropriate, pitch angle is limited to values
// acceptable for perching.  Care is taken to reset and saturate the
// integrator when necessary.
double HoverTensionStep(
    double horizontal_tension_cmd, double tether_elevation_cmd,
    const TetherForceEstimate *tether_force, const Vec3 *wing_pos_g,
    const Vec3 *hover_origin_g, const Vec3 *wind_g, double payout,
    double winch_vel, bool below_half_thrust, const FlightStatus *flight_status,
    const JoystickEstimate *joystick, const HoverTensionParams *params,
    HoverTensionState *state);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_TENSION_H_

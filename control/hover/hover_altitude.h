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

#ifndef CONTROL_HOVER_HOVER_ALTITUDE_H_
#define CONTROL_HOVER_HOVER_ALTITUDE_H_

#include <stdbool.h>

#include "control/control_types.h"
#include "control/hover/hover_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the altitude loop state.
void HoverAltitudeInit(double previous_thrust_cmd,
                       const HoverAltitudeParams *params,
                       HoverAltitudeState *state);

// Returns true if the hover altitude loop parameters are valid.
bool HoverAltitudeValidateParams(const HoverAltitudeParams *params);

// Runs the altitude control loop.  This takes a vertical position
// command and estimate, a vertical velocity command and estimate, and
// a joystick throttle as inputs and outputs a thrust command.  The
// thrust command is based on a feed-forward term, which attempts to
// cancel gravity during normal autonomous hovers, and a feedback
// term, which attempts to meet the altitude command.
double HoverAltitudeStep(double wing_pos_z_g_cmd, double wing_pos_z_g,
                         double wing_vel_z_g_cmd, double wing_vel_z_g,
                         double joystick_throttle, double payout,
                         const FlightStatus *flight_status, bool gain_ramp_done,
                         double thrust_z1, const HoverAltitudeParams *params,
                         HoverAltitudeState *state);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_ALTITUDE_H_

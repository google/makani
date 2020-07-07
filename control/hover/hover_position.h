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

#ifndef CONTROL_HOVER_HOVER_POSITION_H_
#define CONTROL_HOVER_HOVER_POSITION_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/hover/hover_types.h"
#include "control/sensor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the position loop state.
void HoverPositionInit(double previous_int_yaw_angle,
                       const HoverPositionParams *params,
                       HoverPositionState *state);

// Returns true if the hover position parameters are valid.
bool HoverPositionValidateParams(const HoverPositionParams *params);

// Runs position control loop.  This takes a position command,
// velocity command, and pitch command and attempts to meet these
// commands by steering the thrust force vector with an attitude
// command.
void HoverPositionStep(const Vec3 *wing_pos_g_cmd, const Vec3 *wing_pos_g,
                       const Vec3 *wing_vel_g_cmd, const Vec3 *wing_vel_g,
                       const Vec3 *hover_origin_g, const Vec3 *pqr,
                       const Vec3 *angles, double pitch_cmd, double wind_speed,
                       const JoystickData *joystick, const Mat3 *dcm_g2b,
                       double payout, const FlightStatus *flight_status,
                       double wing_offset_from_perched_g_z,
                       const HoverPositionParams *params,
                       HoverPositionState *state, Vec3 *pqr_cmd,
                       Vec3 *angles_cmd);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_POSITION_H_

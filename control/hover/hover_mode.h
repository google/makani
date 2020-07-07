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

#ifndef CONTROL_HOVER_HOVER_MODE_H_
#define CONTROL_HOVER_HOVER_MODE_H_

#include <stdbool.h>

#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/hover/hover_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Determines whether the hover controller is ready to switch into any
// of the hover modes.
bool HoverModeIsReadyFor(
    FlightMode proposed_flight_mode, const FlightStatus *flight_status,
    const StateEstimate *state_est, bool ascent_complete, bool gain_ramp_done,
    bool forcing_detwist_turn, double good_tether_elevation_since,
    double tether_elevation_error, const Vec3 *angles_error,
    const Vec3 *accel_start_pos_g, double waypoint_azi_error,
    const Vec3 *wing_pos_g_cmd, const HoverModeParams *params);

void HoverModeCheckTetherElevationForGsTransform(
    const HoverPathParams *path_params, const HoverModeParams *params,
    const TetherGroundAnglesEstimate *tether_ground_angles,
    const FlightStatus *flight_status, HoverTetherElevationState *state);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_MODE_H_

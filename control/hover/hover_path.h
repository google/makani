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

#ifndef CONTROL_HOVER_HOVER_PATH_H_
#define CONTROL_HOVER_HOVER_PATH_H_

#include <stdbool.h>

#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/hover/hover_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes hover path loop state.
void HoverPathInit(const Vec3 *wing_pos_g, const Vec3 *previous_perched_pos_g,
                   const HoverPathParams *params, HoverPathState *state);

// Returns true if the path parameters are valid.
bool HoverPathValidateParams(const HoverPathParams *params);

// Returns true once the raw position command has reached the final
// ascent position.
bool HoverPathIsAscentComplete(FlightMode flight_mode,
                               const HoverPathParams *params,
                               const StateEstimate *state_est);

// Returns the z-offset, in ground coordinates, of the wing from the
// perched position.
double HoverPathGetZOffsetFromPerchedPosition(const Vec3 *wing_pos_g,
                                              const HoverPathState *state);

// Returns the error between the current position and waypoint it is
// heading to.
void HoverPathGetWaypointError(const Vec3 *wing_pos_g,
                               const HoverPathState *state,
                               Vec3 *waypoint_g_error);

// Returns the error in azimuth [rad] between the current position and the
// waypoint it is heading to.
double HoverPathGetWaypointAzimuthError(const Vec3 *wing_pos_g,
                                        const HoverPathState *state);

// Returns the position the wing will head to before accelerating into
// transition-in.  This is set to be a fixed offset to the left of
// downwind at a set elevation angle and at full tether length.
void HoverPathGetAccelStartPosition(double wind_dir_f,
                                    const PlaybookEntry *playbook_entry,
                                    const Vec3 *perched_pos_g,
                                    const HoverPathParams *params,
                                    Vec3 *accel_start_pos_g);

// Returns the limits on the elevation command as a function of the
// distance of the wing from the origin.  This enforces a set
// elevation during launching and perching and prevents excessively
// low or high elevation requests for the rest of the reeling
// sequence.
void HoverPathCalcElevationLimits(double norm_wing_pos_g,
                                  const HoverPathParams *params,
                                  double *elevation_min, double *elevation_max);

double HoverPathCalcTetherElevationCommand(const FlightStatus *flight_status,
                                           double payout,
                                           const HoverPathParams *params);

// Determines the position set point through all hover flight modes.
void HoverPathStep(const Vec3 *tether_anchor_g, const VesselEstimate *vessel,
                   double horizontal_tension_cmd, double tether_elevation_cmd,
                   const TetherGroundAnglesEstimate *tether_ground_angles,
                   double payout, const Vec3 *wing_pos_g,
                   const Vec3 *wing_vel_g, const WindEstimate *wind_g,
                   const Playbook *playbook, double winch_pos, double winch_vel,
                   const FlightStatus *flight_status,
                   const HoverPathParams *params, HoverPathState *state,
                   Vec3 *wing_pos_g_cmd, Vec3 *wing_vel_g_cmd,
                   Vec3 *crosswind_loop_center_pos_g);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_PATH_H_

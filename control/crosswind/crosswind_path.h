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

// Controls the wing to follow a path defined by a path type and a
// path center.  Outputs curvatures, both geometric and aerodynamic,
// to follow based on current position and orientation.  Geometric
// curvature is the actual curvature of the path the wing is following
// in an inertial reference frame.  Aerodynamic curvature is the
// curvature in a frame that moves with the wind.

#ifndef CONTROL_CROSSWIND_CROSSWIND_PATH_H_
#define CONTROL_CROSSWIND_CROSSWIND_PATH_H_

#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_playbook_types.h"
#include "control/crosswind/crosswind_types.h"
#include "control/estimator/estimator_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the state of the path loop.
void CrosswindPathInit(const StateEstimate *state_est,
                       const CrosswindPathParams *params,
                       double initial_path_radius, CrosswindPathState *state);

// For any point in the crosswind flight plane, calculates the ideal
// heading to follow the specified path.  This is essentially
// calculating the vector field that describes the flight path.
void CrosswindPathCalcBestHeading(const Vec3 *target_pos_cw,
                                  double path_radius_cmd,
                                  CrosswindPathType path_type,
                                  const Vec3 *raw_path_center_cw,
                                  const CrosswindPathParams *params,
                                  Vec3 *best_vel_cw);

// Updates the state of the path loop by a single time step.  Returns
// the curvatures for the curvature and inner loops to follow.
//
// Args:
//
//   flight_mode: Flight mode.
//   path_center_g: Center of the crosswind flight plane in ground coordinates.
//   raw_path_center_g: The raw path center location to target when slewing.
//   path_type: Path type.
//   state_est: State estimate.
//   params: Path loop parameters.
//   path_radius_cmd: The commanded path radius.
//   state: Path loop state.
//   k_aero_cmd: Commanded aerodynamic curvature.
//   k_geom_cmd: Commanded geometric curvature.
//   k_aero_curr: Current aerodynamic curvature with filtering.
//   k_geom_curr: Current geometric curvature with filtering.
void CrosswindPathStep(FlightMode flight_mode, const Vec3 *path_center_g,
                       const Vec3 *raw_path_center_g,
                       CrosswindPathType path_type,
                       const StateEstimate *state_est,
                       const PlaybookEntry *playbook_entry,
                       const CrosswindPathParams *params,
                       CrosswindPathState *state, double *k_aero_cmd,
                       double *k_geom_cmd, double *k_aero_curr,
                       double *k_geom_curr);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_PATH_H_

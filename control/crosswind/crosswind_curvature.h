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

// Controls curvature by adjusting alpha, beta, tether roll, and the
// angular rate commands.  This loop is actually closed inside the
// path controller because the current measured curvature is used to
// set the curvature command.
//                       _____________
//                      |             |--> alpha_cmd, dCL_cmd
//        k_aero_cmd -->|  SetAngles  |--> beta_cmd
//                      |_____________|--> tether_roll_cmd
//
//                       ____________
//        k_geom_cmd -->|            |
//                      |  SetRates  |--> pqr_cmd
//   tether_roll_cmd -->|____________|
//

#ifndef CONTROL_CROSSWIND_CROSSWIND_CURVATURE_H_
#define CONTROL_CROSSWIND_CROSSWIND_CURVATURE_H_

#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the state of the curvature loop.
void CrosswindCurvatureInit(double tension, double alpha, double beta,
                            const CrosswindCurvatureParams *params,
                            CrosswindCurvatureState *state);

// Return the time since the beginning of the trans-out flare or 0.0 if the
// flare hasn't started yet.
double CrosswindCurvatureFlareTimer(const CrosswindCurvatureState *state);

// Updates the state of the curvature loop by a single time step.
// Returns the commands to the inner loops.
void CrosswindCurvatureStep(double k_aero_cmd, double k_geom_cmd,
                            double tension, double airspeed, double alpha_nom,
                            double beta_nom, double loop_angle,
                            CrosswindPathType path_type, const Vec3 *Vg,
                            const FlightStatus *flight_status,
                            const CrosswindFlags *flags,
                            const CrosswindCurvatureParams *params,
                            CrosswindCurvatureState *state, Vec3 *pqr_cmd,
                            bool *flaring, double *alpha_cmd, double *dCL_cmd,
                            double *beta_cmd, double *tether_roll_cmd);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_CURVATURE_H_

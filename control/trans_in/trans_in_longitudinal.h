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

#ifndef CONTROL_TRANS_IN_TRANS_IN_LONGITUDINAL_H_
#define CONTROL_TRANS_IN_TRANS_IN_LONGITUDINAL_H_

#include <stdbool.h>

#include "control/estimator/estimator_types.h"
#include "control/trans_in/trans_in_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Validate the longitudinal controller parameters.
bool TransInLongitudinalValidateParams(const TransInLongitudinalParams *params);

// Runs one step of the longitudinal controller.
//
// Args:
//   radial_pos_ti: Radial position [m] in the trans-in coordinates.
//   elevation_angle_ti: Elevation angle [rad] in the trans-in coordinates.
//   radial_vel_ti: Estimated velocity [m/s] given by d/dt radial_pos_ti.
//   tangential_vel_ti: Estimated velocity [m/s] give by:
//       radial_pos_ti * d/dt elevation_angle_ti.
//   airspeed: Current airspeed [m/s].
//   aero_climb_angle: Estimated aerodynamic climb angle [rad].
//   tether_force_b: Current tether force estimate.
//   params: Parameter structure.
//   angle_of_attack_cmd: Commanded angle-of-attack [rad].
//   pitch_rate_b_cmd: Commanded pitch rate [rad].
//   thrust_cmd: Thrust [N] to command to the motors.
//   delta_flap_cmd: Angle [rad] to command to the center flaps.
void TransInLongitudinalStep(double radial_pos_ti, double elevation_angle_ti,
                             double radial_vel_ti, double tangential_vel_ti,
                             double airspeed, double aero_climb_angle,
                             const TetherForceEstimate *tether_force_b,
                             const TransInLongitudinalParams *params,
                             double *angle_of_attack_cmd,
                             double *pitch_rate_b_cmd, double *thrust_cmd,
                             double *delta_flap_cmd);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_TRANS_IN_TRANS_IN_LONGITUDINAL_H_

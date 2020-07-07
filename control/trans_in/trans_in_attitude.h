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

#ifndef CONTROL_TRANS_IN_TRANS_IN_ATTITUDE_H_
#define CONTROL_TRANS_IN_TRANS_IN_ATTITUDE_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/actuator_util.h"
#include "control/estimator/estimator_types.h"
#include "control/trans_in/trans_in_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Validates the parameters of the attitude controller.
bool TransInAttitudeValidateParams(const TransInAttitudeParams *params);

// Initialize the attitude controller.
void TransInAttitudeInit(double pitch_ti_cmd, const Mat3 *dcm_ti2b,
                         const Vec3 *motor_moment_z1,
                         const TransInAttitudeParams *params,
                         TransInAttitudeState *state);

// Runs one step of the attitude controller.
//
// Args:
//   flight_mode_time: Time [s] since the start of trans-in.
//   pitch_trim_ti: Pitch angle at which zero angle-of-attack is expected [rad].
//   angle_of_attack_cmd: Commanded angle-of-attack [rad].
//   angle_of_attack: Estimated angle-of-attack [rad].
//   angle_of_sideslip_cmd: Commanded angle-of-sideslip [rad].
//   angle_of_sideslip: Estimated angle-of-sideslip [rad].
//   roll_ti_cmd: Roll attitude command [rad].
//   yaw_ti_cmd: Yaw attitude command [rad].
//   dcm_ti2b: Rotation matrix from ti-coordinate frame to body coordinates.
//   pqr_cmd: Commanded body rates [rad/s] resolved in body coordinates.
//   pqr: Measured body rates [rad/s] resolved in body coordinates.
//   thrust_cmd: Thrust [N] commanded by outer loops.
//   delta_flap_cmd: Flap deflection [rad] commanded by outer loops.
//   airspeed: Airspeed [m/s].
//   tether_force_b: Current tether force estimate.
//   params: Parameters.
//   state: State.
//   thrust_moment: Commanded thrust and moments.
//   deltas: Commanded control surface deltas.
void TransInAttitudeStep(double flight_mode_time, double pitch_trim_ti,
                         double angle_of_attack_cmd, double angle_of_attack,
                         double angle_of_sideslip_cmd, double angle_of_sideslip,
                         double roll_ti_cmd, double yaw_ti_cmd,
                         const Mat3 *dcm_ti2b, const Vec3 *pqr_cmd,
                         const Vec3 *pqr, double thrust_cmd,
                         double delta_flap_cmd, double airspeed,
                         const TetherForceEstimate *tether_force_b,
                         const TransInAttitudeParams *params,
                         TransInAttitudeState *state,
                         ThrustMoment *thrust_moment, Deltas *deltas);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_TRANS_IN_TRANS_IN_ATTITUDE_H_

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

// The inner loops control the most directly measurable and
// controllable aspects of the wing: angle-of-attack, sideslip, tether
// roll, and airspeed.
//
//                      ________________
//        alpha_cmd -->|                |--> delta_elevator
//          dCL_cmd -->|  ControlAlpha  |
//            q_cmd -->|________________|--> delta_flap
//
//                      __________________
//  tether_roll_cmd -->|                  |--> delta_aileron
//         beta_cmd -->|  ControlLateral  |
//          pqr_cmd -->|__________________|--> delta_rudder
//
//                      ___________________
//                     |                   |
//     airspeed_cmd -->|  ControlAirspeed  |--> thrust
//                     |___________________|
//

#ifndef CONTROL_CROSSWIND_CROSSWIND_INNER_H_
#define CONTROL_CROSSWIND_CROSSWIND_INNER_H_

#include <stdbool.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/actuator_util.h"
#include "control/crosswind/crosswind_types.h"
#include "control/estimator/estimator_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the state of the inner loops.
void CrosswindInnerInit(const StateEstimate *state_est, const Deltas *deltas_0,
                        double alpha_cmd_0, double loop_angle_0,
                        double previous_accumulated_loop_angle,
                        const CrosswindInnerParams *params,
                        CrosswindInnerState *state);

// Returns true if the crosswind inner loop parameters are valid.
bool CrosswindInnerValidateParams(const CrosswindInnerParams *params);

// Updates the state of the inner loops by a single time step.
// Returns the generic control variables, i.e. deltas, moments, and
// thrusts, which are fed to the output stage to be converted to
// actuator commands.
void CrosswindInnerStep(
    double tether_roll_cmd, double tether_roll, double alpha_cmd, double alpha,
    double dCL_cmd, double beta_cmd, double beta, double airspeed_cmd,
    double airspeed, const Vec3 *pqr_cmd, const Vec3 *pqr, double loop_angle,
    const Mat3 *dcm_g2b, double kite_accel_ff, double flight_mode,
    const CrosswindFlags *flags, const CrosswindInnerParams *params,
    const CrosswindExperiments *experiments,
    const ExperimentState *experiment_state, CrosswindInnerState *state,
    double lateral_gains[][kNumCrosswindLateralStates], Deltas *deltas,
    ThrustMoment *thrust_moment);

// Suppose that, with the given lateral_gains and integrator states
// int_tether_roll_error and int_beta_error, control outputs of
// delta_rudder_initial and delta_aileron_initial are computed. This
// function modifies the value of the integrator states such that
// control outputs of delta_aileron_target and double
// delta_rudder_target are computed instead.
void BacksolveLateralIntegrators(
    double delta_aileron_target, double delta_aileron_initial,
    double delta_rudder_target, double delta_rudder_initial,
    double lateral_gains[][kNumCrosswindLateralStates],
    const CrosswindInnerParams *params, double *int_tether_roll_error,
    double *int_beta_error);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_INNER_H_

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

#include "control/crosswind/crosswind_inner.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/mat2.h"
#include "common/c_math/util.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/actuator_util.h"
#include "control/control_telemetry.h"
#include "control/crosswind/crosswind_util.h"
#include "control/experiments/crosswind_experiment.h"
#include "control/system_params.h"
#include "control/system_types.h"

// Schedules the longitudinal gains based on airspeed.
static void ScheduleLongitudinalGains(
    double airspeed, const CrosswindInnerParams *params,
    double longitudinal_gains[][kNumCrosswindLongitudinalStates]) {
  assert(airspeed >= 0.0);
  assert(params != NULL && longitudinal_gains != NULL);

  if (airspeed < params->airspeed_table[kCrosswindInnerNominalAirspeed]) {
    CrossfadeArray(
        &params->longitudinal_gains_min_airspeed[0][0],
        &params->longitudinal_gains_nominal_airspeed[0][0],
        kNumCrosswindLongitudinalInputs * kNumCrosswindLongitudinalStates,
        airspeed, params->airspeed_table[kCrosswindInnerMinAirspeed],
        params->airspeed_table[kCrosswindInnerNominalAirspeed],
        &longitudinal_gains[0][0]);
  } else {
    CrossfadeArray(
        &params->longitudinal_gains_nominal_airspeed[0][0],
        &params->longitudinal_gains_max_airspeed[0][0],
        kNumCrosswindLongitudinalInputs * kNumCrosswindLongitudinalStates,
        airspeed, params->airspeed_table[kCrosswindInnerNominalAirspeed],
        params->airspeed_table[kCrosswindInnerMaxAirspeed],
        &longitudinal_gains[0][0]);
  }
}

// Returns the feed-forward elevator and flap deflection for a given
// angle-of-attack and lift coefficient change command.
static double CalcLongitudinalFeedForward(double alpha_cmd, double dCL_cmd,
                                          const CrosswindInnerParams *params,
                                          double *delta_flap) {
  assert(-PI <= alpha_cmd && alpha_cmd <= PI);
  assert(-1.0 < dCL_cmd && dCL_cmd < 1.0);
  assert(params != NULL && delta_flap != NULL);

  *delta_flap = params->kp_flap * dCL_cmd;

  // The first feed-forward term is used to counteract the pitching
  // moment from the flaps.
  return params->elevator_flap_ratio * *delta_flap +
         params->delevator_dalpha * alpha_cmd;
}

// Returns the feedback elevator deflection and motor pitching moment
// based on the angle-of-attack error, pitch rate error, and the
// angle-of-attack integrator.
static double CalcLongitudinalFeedback(
    double alpha_cmd, double alpha, double q_cmd, double q,
    double int_alpha_error,
    double longitudinal_gains[][kNumCrosswindLongitudinalStates],
    double *moment_y) {
  assert(-PI <= alpha_cmd && alpha_cmd <= PI);
  assert(-PI <= alpha && alpha <= PI);
  assert(moment_y != NULL);

  // Calculate state error vector.
  // clang-format off
  double state_error[] = {
      [kCrosswindLongitudinalStatePositionGroundZ]         = 0.0,  // Unused.
      [kCrosswindLongitudinalStateVelocityGroundZ]         = 0.0,  // Unused.
      [kCrosswindLongitudinalStateAngleOfAttack]           = alpha_cmd - alpha,
      [kCrosswindLongitudinalStatePitchRate]               = q_cmd - q,
      [kCrosswindLongitudinalStateIntegratedAngleOfAttack] = -int_alpha_error
  };
  // clang-format on
  assert(ARRAYSIZE(state_error) == kNumCrosswindLongitudinalStates);

  // Apply matrix gains.
  double delta_elevator_fb = 0.0;
  *moment_y = 0.0;
  for (int32_t i = 0; i < kNumCrosswindLongitudinalStates; ++i) {
    delta_elevator_fb +=
        longitudinal_gains[kCrosswindLongitudinalInputElevator][i] *
        state_error[i];
    *moment_y += longitudinal_gains[kCrosswindLongitudinalInputMotorPitch][i] *
                 state_error[i];
  }

  return delta_elevator_fb;
}

// Initializes the angle-of-attack integrator to approximately match
// the initial elevator deflection.
static double InitAlphaErrorIntegrator(double delta_elevator_0, double airspeed,
                                       double alpha_cmd, double alpha,
                                       double q_cmd, double q,
                                       const CrosswindInnerParams *params) {
  assert(-PI < delta_elevator_0 && delta_elevator_0 < PI);
  assert(airspeed >= 0.0);
  assert(-PI <= alpha_cmd && alpha_cmd <= PI);
  assert(-PI <= alpha && alpha <= PI);
  assert(params != NULL);

  double longitudinal_gains[kNumCrosswindLongitudinalInputs]
                           [kNumCrosswindLongitudinalStates];
  ScheduleLongitudinalGains(airspeed, params, longitudinal_gains);

  // Calculate the zero-integrator value for the elevator.
  double unused_delta_flap;
  double delta_elevator_ff =
      CalcLongitudinalFeedForward(alpha_cmd, 0.0, params, &unused_delta_flap);
  double unused_moment_y;
  double delta_elevator_fb = CalcLongitudinalFeedback(
      alpha_cmd, alpha, q_cmd, q, 0.0, longitudinal_gains, &unused_moment_y);

  // Bound integrated angle-of-attack to elevator gain away from zero.
  double int_alpha_elevator_gain = fmax(
      longitudinal_gains[kCrosswindLongitudinalInputElevator]
                        [kCrosswindLongitudinalStateIntegratedAngleOfAttack],
      1e-3);

  // Return value of integrator that matches the initial elevator deflection.
  return Saturate((delta_elevator_0 - delta_elevator_ff - delta_elevator_fb) /
                      -int_alpha_elevator_gain,
                  params->int_alpha_min, params->int_alpha_max);
}

// Schedules the lateral gains based on airspeed.
static void ScheduleLateralGains(
    double airspeed, const CrosswindInnerParams *params,
    double lateral_gains[][kNumCrosswindLateralStates]) {
  assert(airspeed >= 0.0);
  assert(params != NULL && lateral_gains != NULL);

  if (airspeed < params->airspeed_table[kCrosswindInnerNominalAirspeed]) {
    CrossfadeArray(&params->lateral_gains_min_airspeed[0][0],
                   &params->lateral_gains_nominal_airspeed[0][0],
                   kNumCrosswindLateralInputs * kNumCrosswindLateralStates,
                   airspeed, params->airspeed_table[kCrosswindInnerMinAirspeed],
                   params->airspeed_table[kCrosswindInnerNominalAirspeed],
                   &lateral_gains[0][0]);
  } else {
    CrossfadeArray(&params->lateral_gains_nominal_airspeed[0][0],
                   &params->lateral_gains_max_airspeed[0][0],
                   kNumCrosswindLateralInputs * kNumCrosswindLateralStates,
                   airspeed,
                   params->airspeed_table[kCrosswindInnerNominalAirspeed],
                   params->airspeed_table[kCrosswindInnerMaxAirspeed],
                   &lateral_gains[0][0]);
  }
}

// Returns the feedback aileron and rudder deflections and motor
// yawing moment based on the tether roll error, sideslip error, roll
// and yaw rate errors, and the tether roll and sideslip integrators.
static void CalcLateralFeedback(
    double tether_roll_cmd, double tether_roll, double beta_cmd, double beta,
    const Vec3 *pqr_cmd, const Vec3 *pqr, double int_tether_roll_error,
    double int_beta_error, double lateral_gains[][kNumCrosswindLateralStates],
    double *delta_aileron, double *delta_rudder, double *moment_z) {
  assert(-PI / 2.0 <= tether_roll_cmd && tether_roll_cmd <= PI / 2.0);
  assert(-PI / 2.0 <= tether_roll && tether_roll <= PI / 2.0);
  assert(-PI / 2.0 <= beta_cmd && beta_cmd <= PI / 2.0);
  assert(-PI / 2.0 <= beta && beta <= PI / 2.0);
  assert(delta_aileron != NULL && delta_rudder != NULL && moment_z != NULL);

  // Calculate state error vector.
  // clang-format off
  double state_error[] = {
      [kCrosswindLateralStateTetherRoll] = tether_roll_cmd - tether_roll,
      [kCrosswindLateralStateSideslip]   = beta_cmd - beta,
      [kCrosswindLateralStateRollRate]   = pqr_cmd->x - pqr->x,
      [kCrosswindLateralStateYawRate]    = pqr_cmd->z - pqr->z,
      [kCrosswindLateralStateIntegratedTetherRoll] = -int_tether_roll_error,
      [kCrosswindLateralStateIntegratedSideslip]   = -int_beta_error
  };
  // clang-format on
  assert(ARRAYSIZE(state_error) == kNumCrosswindLateralStates);

  // Apply matrix gains.
  *delta_aileron = 0.0;
  *delta_rudder = 0.0;
  *moment_z = 0.0;
  for (int32_t i = 0; i < kNumCrosswindLateralStates; ++i) {
    *delta_aileron +=
        lateral_gains[kCrosswindLateralInputAileron][i] * state_error[i];
    *delta_rudder +=
        lateral_gains[kCrosswindLateralInputRudder][i] * state_error[i];
    *moment_z +=
        lateral_gains[kCrosswindLateralInputMotorYaw][i] * state_error[i];
  }
}

void BacksolveLateralIntegrators(
    double delta_aileron_target, double delta_aileron_initial,
    double delta_rudder_target, double delta_rudder_initial,
    double lateral_gains[][kNumCrosswindLateralStates],
    const CrosswindInnerParams *params, double *int_tether_roll_error,
    double *int_beta_error) {
  Mat2 integrator_gains = {
      {{lateral_gains[kCrosswindLateralInputAileron]
                     [kCrosswindLateralStateIntegratedTetherRoll],
        lateral_gains[kCrosswindLateralInputAileron]
                     [kCrosswindLateralStateIntegratedSideslip]},
       {lateral_gains[kCrosswindLateralInputRudder]
                     [kCrosswindLateralStateIntegratedTetherRoll],
        lateral_gains[kCrosswindLateralInputRudder]
                     [kCrosswindLateralStateIntegratedSideslip]}}};
  Vec2 delta_errors = {delta_aileron_target - delta_aileron_initial,
                       delta_rudder_target - delta_rudder_initial};

  // Compute the change in integrator values required. "Delta" in the
  // name here simply refers to a change, not to a rudder or aileron
  // "delta".
  Vec2 delta_integrators = kVec2Zero;
  if (fabs(Mat2Det(&integrator_gains)) > 1e-6) {
    Mat2Vec2LeftDivide(&integrator_gains, &delta_errors, &delta_integrators);
  }

  *int_tether_roll_error -= delta_integrators.x;
  *int_beta_error -= delta_integrators.y;

  *int_tether_roll_error =
      Saturate(*int_tether_roll_error, params->int_tether_roll_min,
               params->int_tether_roll_max);
  *int_beta_error =
      Saturate(*int_beta_error, params->int_beta_min, params->int_beta_max);
}

// Initializes the tether roll and sideslip integrators to
// approximately match the initial aileron and rudder deflections.
static void InitLateralIntegrators(
    double delta_aileron_0, double delta_rudder_0, double airspeed,
    double tether_roll_cmd, double tether_roll, double beta_cmd, double beta,
    const Vec3 *pqr_cmd, const Vec3 *pqr, const CrosswindInnerParams *params,
    double *int_tether_roll_error, double *int_beta_error) {
  assert(-PI < delta_aileron_0 && delta_aileron_0 < PI);
  assert(-PI < delta_rudder_0 && delta_rudder_0 < PI);
  assert(airspeed >= 0.0);
  assert(-PI / 2.0 <= tether_roll_cmd && tether_roll_cmd <= PI / 2.0);
  assert(-PI / 2.0 <= tether_roll && tether_roll <= PI / 2.0);
  assert(params != NULL && int_beta_error != NULL &&
         int_tether_roll_error != NULL);

  *int_tether_roll_error = 0.0;
  *int_beta_error = 0.0;

  double lateral_gains[kNumCrosswindLateralInputs][kNumCrosswindLateralStates];
  ScheduleLateralGains(airspeed, params, lateral_gains);

  // Calculate the zero-integrator values for the aileron and rudder.
  double unused_moment_z;
  double delta_aileron, delta_rudder;
  CalcLateralFeedback(tether_roll_cmd, tether_roll, beta_cmd, beta, pqr_cmd,
                      pqr, 0.0, 0.0, lateral_gains, &delta_aileron,
                      &delta_rudder, &unused_moment_z);

  // Solve for the values of the integrators that match the initial
  // aileron and rudder deflections.
  BacksolveLateralIntegrators(delta_aileron_0, delta_aileron, delta_rudder_0,
                              delta_rudder, lateral_gains, params,
                              int_tether_roll_error, int_beta_error);
}

void CrosswindInnerInit(const StateEstimate *state_est, const Deltas *deltas_0,
                        double alpha_cmd_0, double loop_angle_0,
                        double previous_accumulated_loop_angle,
                        const CrosswindInnerParams *params,
                        CrosswindInnerState *state) {
  assert(state_est != NULL && deltas_0 != NULL);
  assert(-PI < deltas_0->aileron && deltas_0->aileron < PI);
  assert(-PI < deltas_0->elevator && deltas_0->elevator < PI);
  assert(-PI < deltas_0->rudder && deltas_0->rudder < PI);
  assert(-PI < alpha_cmd_0 && alpha_cmd_0 < PI);
  assert(params != NULL && state != NULL);
  assert(CrosswindInnerValidateParams(params));

  memset(state, 0, sizeof(*state));

  // Attempts to use the angle-of-attack integrator to reduce the
  // transients from the transition-in controller by approximately
  // matching the final elevator position during transition-in.  Note
  // that this does not match perfectly due to differences in commands
  // (e.g. pitch rate command) and because of the saturation of the
  // integrator.
  state->int_alpha_error = InitAlphaErrorIntegrator(
      deltas_0->elevator, state_est->apparent_wind.sph_f.v, alpha_cmd_0,
      state_est->apparent_wind.sph_f.alpha, 0.0, state_est->pqr_f.y, params);

  // Attempts to use the tether roll and sideslip integrators to
  // reduce the transients from the transition-in controller by
  // approximately matching the final aileron and rudder positions
  // during transition-in.  Note that this does not match perfectly
  // because due to differences in commands (e.g. yaw rate command)
  // and because of saturations of the integrators.
  InitLateralIntegrators(
      deltas_0->aileron, deltas_0->rudder, state_est->apparent_wind.sph_f.v,
      0.0, state_est->tether_force_b.sph.roll, 0.0,
      state_est->apparent_wind.sph_f.beta, &kVec3Zero, &state_est->pqr_f,
      params, &state->int_tether_roll_error, &state->int_beta_error);

  state->int_thrust = 0.0;
  state->spoiler_on = false;

  // TODO: Initialize this value to the current thrust
  // command rather than a hard-coded value.
  state->thrust_cmd_z1 = params->initial_thrust;
  state->loop_angle_z1 = loop_angle_0;

  for (int i = 0; i < ARRAYSIZE(state->beta_harmonic_state); ++i) {
    state->beta_harmonic_state[i] = 0.0;
  }

  state->accumulated_loop_angle = previous_accumulated_loop_angle;
}

bool CrosswindInnerValidateParams(const CrosswindInnerParams *params) {
  if (!(params->airspeed_table[kCrosswindInnerMinAirspeed] > 0.0)) {
    assert(!(bool)"The first element of airspeed_table must be positive.");
    return false;
  }

  for (int32_t i = 1; i < ARRAYSIZE(params->airspeed_table); ++i) {
    if (params->airspeed_table[i - 1] >= params->airspeed_table[i]) {
      assert(
          !(bool)"airspeed_table must be strictly monotonically increasing.");
      return false;
    }
  }

  return true;
}

// Controls angle-of-attack.
//
// Args:
//   alpha_cmd: Angle-of-attack command.
//   alpha: Measured angle-of-attack.
//   dCL_cmd: Change in lift coefficient from the flaps command.
//   q_cmd: Pitch rate command.
//   q: Measured pitch rate.
//   airspeed: Measured airspeed, which is used to schedule PID gains
//       and motor commands.
//   flags: The flags are used to reset or hold the integrators based
//       on flight mode changes or fault conditions.
//   params: Inner loop parameters.
//   state: Inner loop state.
//   delta_flap: Output for the flap deflection command.
//
// Returns:
//   Elevator deflection command.
static double ControlAlpha(double alpha_cmd, double alpha, double dCL_cmd,
                           double q_cmd, double q, double airspeed,
                           const CrosswindFlags *flags,
                           const CrosswindInnerParams *params,
                           CrosswindInnerState *state, double *delta_flap,
                           double *moment_y) {
  // If both the pitot and loadcells have failed, then we have no
  // accurate method of estimating angle-of-attack so we hold the
  // integrator.
  IntegratorMode int_mode =
      flags->alpha_beta_fault ? kIntegratorModeHold : kIntegratorModeIntegrate;

  Integrator(alpha_cmd - alpha, params->int_alpha_min, params->int_alpha_max,
             *g_sys.ts, int_mode, &state->int_alpha_error);

  // Schedule matrix gains based on airspeed.
  double longitudinal_gains[kNumCrosswindLongitudinalInputs]
                           [kNumCrosswindLongitudinalStates];
  ScheduleLongitudinalGains(airspeed, params, longitudinal_gains);

  // Apply feedback law to determine the elevator and motor pitch
  // commands.
  double delta_elevator_fb = CalcLongitudinalFeedback(
      alpha_cmd, alpha, q_cmd, q, state->int_alpha_error, longitudinal_gains,
      moment_y);
  double delta_elevator_ff =
      CalcLongitudinalFeedForward(alpha_cmd, dCL_cmd, params, delta_flap);

  // Update telemetry.
  CrosswindTelemetry *cwt = GetCrosswindTelemetry();
  cwt->alpha_cmd = alpha_cmd;
  cwt->int_elevator =
      state->int_alpha_error *
      longitudinal_gains[kCrosswindLongitudinalInputElevator]
                        [kCrosswindLongitudinalStateIntegratedAngleOfAttack];

  return delta_elevator_ff + delta_elevator_fb;
}

// Controls lateral dynamics including the sideslip and tether roll
// angles.  This uses an LQR gain matrix to control the tether roll,
// roll rate, sideslip, and yaw rate with the aileron and rudder.  The
// LQR gain matrix is scheduled based on airspeed.
//
// TODO: The tether roll and beta integrators can result
// in a steady-state rudder offset, which also feeds into a
// steady-state motor differential.  This might not be optimal for
// power generation or motor heating.  A similar comment applies to
// the alpha loop.
//
// Args:
//   tether_roll_cmd: Tether roll angle command.
//   tether_roll: Measured tether roll angle.
//   beta_cmd: Sideslip angle command.
//   beta: Measured sideslip angle.
//   pqr_cmd: Roll, pitch, and yaw rate command (pitch is not used here).
//   pqr: Measured roll, pitch, and yaw rate.
//   airspeed: Measured airspeed, which is used to schedule gains.
//   flags: The flags are used to reset or hold the integrators based
//       on flight mode changes or fault conditions.
//   params: Inner loop parameters.
//   state: Inner loop state.
//   deltas: Returns aileron and rudder commands.
static void ControlLateral(double tether_roll_cmd, double tether_roll,
                           double beta_cmd, double beta, const Vec3 *pqr_cmd,
                           const Vec3 *pqr, double airspeed, double loop_angle,
                           double loop_angle_z1, const CrosswindFlags *flags,
                           const CrosswindInnerParams *params,
                           CrosswindInnerState *state,
                           double lateral_gains[][kNumCrosswindLateralStates],
                           Deltas *deltas, double *moment_z) {
  // If both the pitot and loadcells have failed, then we have no
  // accurate method of estimating sideslip so we hold the
  // integrators.
  IntegratorMode beta_int_mode =
      flags->alpha_beta_fault ? kIntegratorModeHold : kIntegratorModeIntegrate;

  Integrator(tether_roll_cmd - tether_roll, params->int_tether_roll_min,
             params->int_tether_roll_max, *g_sys.ts, kIntegratorModeIntegrate,
             &state->int_tether_roll_error);
  Integrator(beta_cmd - beta, params->int_beta_min, params->int_beta_max,
             *g_sys.ts, beta_int_mode, &state->int_beta_error);

  // Schedule matrix gains based on airspeed.
  ScheduleLateralGains(airspeed, params, lateral_gains);

  // Apply feedback law to determine the aileron, rudder, and motor
  // yaw commands.

  // Note that delta_loop_angle is computed in such a way that the
  // nominal motion results in positive delta_loop_angle.
  double delta_loop_angle = Wrap(loop_angle_z1 - loop_angle, -PI, PI);

  double beta_harmonic_fb = HarmonicIntegrator(
      beta - beta_cmd, loop_angle, delta_loop_angle, params->beta_harmonic_gain,
      params->beta_harmonic_integrator_max, state->beta_harmonic_state);

  CalcLateralFeedback(
      tether_roll_cmd, tether_roll, beta_cmd, beta + beta_harmonic_fb, pqr_cmd,
      pqr, state->int_tether_roll_error, state->int_beta_error, lateral_gains,
      &deltas->aileron, &deltas->rudder, moment_z);

  // Update telemetry.
  CrosswindTelemetry *cwt = GetCrosswindTelemetry();
  cwt->tether_roll_cmd = tether_roll_cmd;
  cwt->beta_cmd = beta_cmd;
  cwt->pqr_cmd = *pqr_cmd;
  cwt->int_aileron = lateral_gains[kCrosswindLateralInputAileron]
                                  [kCrosswindLateralStateIntegratedTetherRoll] *
                         -state->int_tether_roll_error +
                     lateral_gains[kCrosswindLateralInputAileron]
                                  [kCrosswindLateralStateIntegratedSideslip] *
                         -state->int_beta_error;
  cwt->int_rudder = lateral_gains[kCrosswindLateralInputRudder]
                                 [kCrosswindLateralStateIntegratedTetherRoll] *
                        -state->int_tether_roll_error +
                    lateral_gains[kCrosswindLateralInputRudder]
                                 [kCrosswindLateralStateIntegratedSideslip] *
                        -state->int_beta_error;
  cwt->int_tether_roll_error = state->int_tether_roll_error;
  cwt->int_beta_error = state->int_beta_error;

  for (int i = 0; i < ARRAYSIZE(state->beta_harmonic_state); ++i) {
    cwt->beta_harmonic_state[i] = state->beta_harmonic_state[i];
  }
}

// Returns the thrust necessary to balance gravity along the body
// x-axis.
static double CalcFeedForwardThrust(const Mat3 *dcm_g2b) {
  Vec3 g_b;
  Mat3Vec3Mult(dcm_g2b, &g_sys.phys->g_g, &g_b);
  return -g_sys.wing->m * g_b.x;
}

// Controls airspeed with a PI controller that outputs a thrust
// command.  This loop also decides when to use the spoiler.
//
// Args:
//   airspeed_cmd: Airspeed command.
//   airspeed: Measured airspeed.
//   dcm_g2b: Direction cosine matrix between ground and body
//       coordinates.  This is used to calculate a feed-forward term
//       to compensate for the weight of the wing.
//   flags: The flags are used to reset integrators or the state of
//       the spoiler based on flight mode changes.
//   params: Inner loop parameters.
//   state: Inner loop state.
//   delta_spoiler: Spoiler deflection command.
//
// Returns:
//   Thrust command.
static double ControlAirspeed(double airspeed_cmd, double airspeed,
                              const Mat3 *dcm_g2b, double kite_accel_ff,
                              double loop_angle, double flight_mode,
                              const CrosswindFlags *flags,
                              const CrosswindInnerParams *params,
                              const CrosswindExperiments *experiments,
                              const ExperimentState *experiment_state,
                              CrosswindInnerState *state,
                              double *delta_spoiler) {
  assert(params->max_airspeed_control_power_motor > 0.0);
  assert(params->max_airspeed_control_power_gen > 0.0);
  assert(params->max_airspeed_error > 0.0);

  double delta_spoiler_on_rate = params->delta_spoiler_on_rate;
  double delta_spoiler_off_rate = params->delta_spoiler_off_rate;

  if (flight_mode == kFlightModeCrosswindNormal &&
      experiment_state->active_type == kExperimentTypeCrosswindSpoiler) {
    const CrosswindSpoilerExperiment *exp_config =
        &experiments->crosswind_spoiler[experiment_state->case_id];
    if (exp_config->always_on ||
        (BetweenLoopAngles(exp_config->start_loop_angle,
                           exp_config->end_loop_angle, loop_angle))) {
      *delta_spoiler = exp_config->target;
      delta_spoiler_on_rate =
          CrosswindSpoilerExperimentOnRate(exp_config->target);
    } else {
      *delta_spoiler = 0.0;
    }
  } else {
    // Add hysteresis for spoiler turning on and off.
    if (airspeed - airspeed_cmd > params->airspeed_error_spoiler_on) {
      state->spoiler_on = true;
    } else if (airspeed - airspeed_cmd < params->airspeed_error_spoiler_off) {
      state->spoiler_on = false;
    }

    // TODO: The M600 does not pass the IEC cases when the
    // spoiler is used.  This was previously disabled with an #ifdef for
    // the M600, but I re-enabled it so we're forced to deal with the
    // root cause.
    *delta_spoiler = flags->spoiler_enabled && state->spoiler_on
                         ? params->delta_spoiler
                         : 0.0;
  }

  if (flight_mode == kFlightModeCrosswindNormal &&
      experiment_state->staged_type == kExperimentTypeCrosswindSpoiler) {
    // Overwrite the spoiler retraction rate just for the system ID test.
    // Ideally, no control behavior should depend on staged_type. However in
    // this specific case, the retraction rate is only applicable after
    // the test is switched off.
    // The downside is that if we were using spoilers in a nominal flight,
    // the nominal retraction rate will be overwritten as long as a spoiler
    // test is staged. However, this code is intended to be removed before
    // the controller uses spoilers in a nominal flight.
    // TODO: Remove experiment injections after their flight test.
    delta_spoiler_off_rate = CrosswindSpoilerExperimentOffRate();
  }

  // Rate limit the spoiler as suggested by nathantreat@ to minimize the effect
  // of unsteady transients.
  *delta_spoiler =
      RateLimit(*delta_spoiler, delta_spoiler_on_rate, delta_spoiler_off_rate,
                *g_sys.ts, &state->delta_spoiler_z1);

  // Calculate gravity feed-forward.
  double thrust_ff = CalcFeedForwardThrust(dcm_g2b);

  // Apply acceleration feed-forward;
  if (params->enable_acceleration_ff) {
    thrust_ff += g_sys.wing->m * kite_accel_ff;
  }

  double thrust_fb =
      Pid(Saturate(airspeed_cmd - airspeed, -params->max_airspeed_error,
                   params->max_airspeed_error),
          0.0, *g_sys.ts, kIntegratorModeIntegrate, &params->airspeed_pid,
          &state->int_thrust);

  // Update telemetry.
  CrosswindTelemetry *cwt = GetCrosswindTelemetry();
  cwt->airspeed_cmd = airspeed_cmd;
  cwt->thrust_ff = thrust_ff;
  cwt->thrust_fb = thrust_fb;
  cwt->int_thrust = state->int_thrust;

  double thrust_cmd =
      Saturate(thrust_ff + thrust_fb,
               -params->max_airspeed_control_power_gen / fmax(airspeed, 1.0),
               params->max_airspeed_control_power_motor / fmax(airspeed, 1.0));

  return RateLimit(thrust_cmd, -params->max_airspeed_control_thrust_rate,
                   params->max_airspeed_control_thrust_rate, *g_sys.ts,
                   &state->thrust_cmd_z1);
}

void CrosswindInnerStep(
    double tether_roll_cmd, double tether_roll, double alpha_cmd, double alpha,
    double dCL_cmd, double beta_cmd, double beta, double airspeed_cmd,
    double airspeed, const Vec3 *pqr_cmd, const Vec3 *pqr, double loop_angle,
    const Mat3 *dcm_g2b, double kite_accel_ff, double flight_mode,
    const CrosswindFlags *flags, const CrosswindInnerParams *params,
    const CrosswindExperiments *experiments,
    const ExperimentState *experiment_state, CrosswindInnerState *state,
    double lateral_gains[][kNumCrosswindLateralStates], Deltas *deltas,
    ThrustMoment *thrust_moment) {
  deltas->elevator = ControlAlpha(
      alpha_cmd, alpha, dCL_cmd, pqr_cmd->y, pqr->y, airspeed, flags, params,
      state, &deltas->inboard_flap, &thrust_moment->moment.y);

  ControlLateral(tether_roll_cmd, tether_roll, beta_cmd, beta, pqr_cmd, pqr,
                 airspeed, loop_angle, state->loop_angle_z1, flags, params,
                 state, lateral_gains, deltas, &thrust_moment->moment.z);

  double delta_spoiler;
  thrust_moment->thrust = ControlAirspeed(
      airspeed_cmd, airspeed, dcm_g2b, kite_accel_ff, loop_angle, flight_mode,
      flags, params, experiments, experiment_state, state, &delta_spoiler);

  deltas->inboard_flap += delta_spoiler;
  deltas->midboard_flap = 0.0;
  deltas->outboard_flap = 0.0;

  thrust_moment->moment.x = 0.0;

  double delta_loop_angle = Wrap(state->loop_angle_z1 - loop_angle, -PI, PI);
  state->accumulated_loop_angle += delta_loop_angle;

  state->loop_angle_z1 = loop_angle;

  // Update telemetry.
  CrosswindTelemetry *cwt = GetCrosswindTelemetry();
  cwt->loop_count = state->accumulated_loop_angle / 2.0 / PI;
}

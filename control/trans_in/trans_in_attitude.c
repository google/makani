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

#include "control/trans_in/trans_in_attitude.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/actuator_util.h"
#include "control/control_telemetry.h"
#include "control/system_params.h"
#include "control/trans_in/trans_in_types.h"

static bool ValidateLateralGains(const double gains[]
                                                   [kNumTransInLateralStates]) {
  if (gains[kTransInLateralInputAileron][kTransInLateralStateRoll] > 0.0 ||
      gains[kTransInLateralInputAileron][kTransInLateralStateRollRate] > 0.0) {
    assert(!(bool)"Roll control gains have wrong sign.");
    return false;
  }

  if (gains[kTransInLateralInputMotorYaw][kTransInLateralStateYaw] < 0.0 ||
      gains[kTransInLateralInputMotorYaw][kTransInLateralStateYawRate] < 0.0 ||
      gains[kTransInLateralInputMotorYaw][kTransInLateralStateAngleOfSideslip] >
          0.0 ||
      gains[kTransInLateralInputRudder][kTransInLateralStateYaw] > 0.0 ||
      gains[kTransInLateralInputRudder][kTransInLateralStateYawRate] > 0.0 ||
      gains[kTransInLateralInputRudder][kTransInLateralStateAngleOfSideslip] <
          0.0) {
    assert(!(bool)"Yaw control gains have wrong sign.");
    return false;
  }

  return true;
}

static bool ValidateLongitudinalGains(
    const double gains[][kNumTransInLongitudinalStates]) {
  if (gains[kTransInLongitudinalInputMotorPitch]
           [kTransInLongitudinalStatePitch] < 0.0 ||
      gains[kTransInLongitudinalInputMotorPitch]
           [kTransInLongitudinalStatePitchRate] < 0.0 ||
      gains[kTransInLongitudinalInputMotorPitch]
           [kTransInLongitudinalStateIntAngleOfAttack] < 0.0 ||
      gains[kTransInLongitudinalInputElevator][kTransInLongitudinalStatePitch] >
          0.0 ||
      gains[kTransInLongitudinalInputElevator]
           [kTransInLongitudinalStatePitchRate] > 0.0 ||
      gains[kTransInLongitudinalInputElevator]
           [kTransInLongitudinalStateIntAngleOfAttack] > 0.0) {
    assert(!(bool)"Pitch control gains have wrong sign.");
    return false;
  }

  return true;
}

bool TransInAttitudeValidateParams(const TransInAttitudeParams *params) {
  assert(params != NULL);

  if (params->pitch_forward_max_pitch_accel <= 0.1 ||
      params->pitch_forward_max_pitch_accel > 1.5) {
    assert(!(bool)"pitch_forward_max_pitch_accel out of range.");
    return false;
  }

  if (params->pitch_forward_max_pitch_rate < 0.25 ||
      params->pitch_forward_max_pitch_rate > 1.0) {
    assert(!(bool)"pitch_forward_max_pitch_rate out of range.");
    return false;
  }

  if (params->pitch_forward_max_pitch_error > -0.1 ||
      params->pitch_forward_max_pitch_error < -0.35) {
    assert(!(bool)"pitch_forward_max_pitch_error of range.");
    return false;
  }

  if (params->pitch_forward_max_duration > 5.0 ||
      params->pitch_forward_max_duration < 1.5) {
    assert(!(bool)"pitch_forward_max_duration of range.");
    return false;
  }

  if (params->low_tension <
          g_sys.tether->length * g_sys.tether->linear_density ||
      params->low_tension > params->high_tension ||
      params->high_tension > 10 * params->low_tension) {
    assert(!(bool)"Tension thresholds out of range.");
    return false;
  }

  if (params->delta_elevator_alpha_zero > 0.1 ||
      params->delta_elevator_alpha_zero < -0.1) {
    assert(!(bool)"delta_elevator_alpha_zero out of range.");
    return false;
  }

  if (params->ddelta_elevator_dalpha > 0.0 ||
      params->ddelta_elevator_dalpha < -1.5) {
    assert(!(bool)"ddelta_elevator_dalpha out of range.");
    return false;
  }

  if (params->int_release_airspeed_threshold < 10.0 ||
      params->int_release_airspeed_threshold > 20.0) {
    assert(!(bool)"int_release_alpha_threshold out of range.");
    return false;
  }

  if (params->int_release_alpha_threshold < 0.05 ||
      params->int_release_alpha_threshold > 0.2) {
    assert(!(bool)"int_release_alpha_threshold out of range.");
    return false;
  }

  if (params->max_int_angle_of_attack_rate < 0.0 ||
      params->max_int_angle_of_attack_rate > 0.15) {
    assert(!(bool)"max_int_angle_of_attack_rate out of range.");
    return false;
  }

  if (params->max_int_angle_of_attack < 0.0 ||
      params->max_int_angle_of_attack *
              params->long_gains_low_tension
                  [kTransInLongitudinalInputElevator]
                  [kTransInLongitudinalStateIntAngleOfAttack] >
          0.1 ||
      params->max_int_angle_of_attack *
              params->long_gains_high_tension
                  [kTransInLongitudinalInputElevator]
                  [kTransInLongitudinalStateIntAngleOfAttack] >
          0.1) {
    assert(!(bool)"max_int_angle_of_attack out of range.");
    return false;
  }

  if (params->max_int_roll < 0.0 ||
      params->max_int_roll *
              params->lat_gains_low_tension[kTransInLateralInputAileron]
                                           [kTransInLateralStateIntRoll] >
          0.1 ||
      params->max_int_roll *
              params->lat_gains_high_tension[kTransInLateralInputAileron]
                                            [kTransInLateralStateIntRoll] >
          0.1) {
    assert(!(bool)"max_int_roll out of range.");
    return false;
  }

  if (!ValidateLateralGains(params->lat_gains_pitch_forward) ||
      !ValidateLateralGains(params->lat_gains_low_tension) ||
      !ValidateLateralGains(params->lat_gains_high_tension) ||
      !ValidateLongitudinalGains(params->long_gains_pitch_forward) ||
      !ValidateLongitudinalGains(params->long_gains_low_tension) ||
      !ValidateLongitudinalGains(params->long_gains_high_tension)) {
    return false;
  }

  if (params->midboard_flap_ratio < 0.0 || params->midboard_flap_ratio > 1.0) {
    assert(!(bool)"midboard_flap_ratio out of range.");
    return false;
  }

  return true;
}

static void CalcAxisB2Cmd(const Vec3 *euler_ti2cmd, const Mat3 *dcm_ti2b,
                          Vec3 *axis_b2cmd) {
  // Construct the DCM representing the desired attitude.
  Mat3 dcm_ti2cmd;
  AngleToDcm(euler_ti2cmd->z, euler_ti2cmd->y, euler_ti2cmd->x,
             kRotationOrderZyx, &dcm_ti2cmd);

  Mat3 dcm_b2cmd;
  Mat3Mult(&dcm_ti2cmd, kNoTrans, dcm_ti2b, kTrans, &dcm_b2cmd);

  // Produce the tilt error vector.
  Quat q_b2cmd;
  DcmToQuat(&dcm_b2cmd, &q_b2cmd);
  QuatToAxis(&q_b2cmd, axis_b2cmd);
}

static void CalcLateralControl(double angle_of_sideslip_cmd,
                               double angle_of_sideslip, const Vec3 *axis_b2cmd,
                               const Vec3 *pqr_cmd, const Vec3 *pqr,
                               const TetherForceEstimate *tether_force_b,
                               double c_pitched_forward, bool enable_integrator,
                               const TransInAttitudeParams *params,
                               TransInAttitudeState *state, double *motor_roll,
                               double *motor_yaw, double *delta_aileron,
                               double *delta_rudder) {
  double state_error[] = {[kTransInLateralStateRoll] = axis_b2cmd->x,
                          [kTransInLateralStateYaw] = axis_b2cmd->z,
                          [kTransInLateralStateRollRate] = pqr_cmd->x - pqr->x,
                          [kTransInLateralStateYawRate] = pqr_cmd->z - pqr->z,
                          [kTransInLateralStateAngleOfSideslip] =
                              angle_of_sideslip_cmd - angle_of_sideslip,
                          [kTransInLateralStateIntRoll] = state->int_roll};
  assert(ARRAYSIZE(state_error) == kNumTransInLateralStates);

  double gains[kNumTransInLateralInputs][kNumTransInLateralStates];
  CrossfadeArray(&params->lat_gains_low_tension[0][0],
                 &params->lat_gains_high_tension[0][0],
                 kNumTransInLateralInputs * kNumTransInLateralStates,
                 tether_force_b->tension_f, params->low_tension,
                 params->high_tension, &gains[0][0]);

  CrossfadeArray(&params->lat_gains_pitch_forward[0][0], &gains[0][0],
                 kNumTransInLateralInputs * kNumTransInLateralStates,
                 c_pitched_forward, 0.5, 1.0, &gains[0][0]);

  double inputs[kNumTransInLateralInputs];
  MatArrMult(&gains[0][0], kNumTransInLateralInputs, kNumTransInLateralStates,
             state_error, 1, inputs);

  IntegratorMode int_mode =
      enable_integrator ? kIntegratorModeIntegrate : kIntegratorModeHold;
  Integrator(state_error[kTransInLateralStateRoll], -params->max_int_roll,
             params->max_int_roll, *g_sys.ts, int_mode, &state->int_roll);

  *motor_roll = 0.0;
  *motor_yaw = inputs[kTransInLateralInputMotorYaw];
  *delta_aileron = inputs[kTransInLateralInputAileron];
  *delta_rudder = inputs[kTransInLateralInputRudder];
}

static void CalcLongitudinalControl(
    double angle_of_attack_cmd, double angle_of_attack, const Vec3 *axis_b2cmd,
    double pitch_rate_b_cmd, double pitch_rate_b,
    const TetherForceEstimate *tether_force_b, double c_pitched_forward,
    bool enable_integrator, const TransInAttitudeParams *params,
    TransInAttitudeState *state, double *motor_pitch, double *delta_elevator) {
  double state_error[] = {[kTransInLongitudinalStatePitch] = axis_b2cmd->y,
                          [kTransInLongitudinalStatePitchRate] =
                              pitch_rate_b_cmd - pitch_rate_b,
                          [kTransInLongitudinalStateIntAngleOfAttack] =
                              state->int_angle_of_attack};
  assert(ARRAYSIZE(state_error) == kNumTransInLongitudinalStates);

  double gains[kNumTransInLongitudinalInputs][kNumTransInLongitudinalStates];
  CrossfadeArray(&params->long_gains_low_tension[0][0],
                 &params->long_gains_high_tension[0][0],
                 kNumTransInLongitudinalInputs * kNumTransInLongitudinalStates,
                 tether_force_b->tension_f, params->low_tension,
                 params->high_tension, &gains[0][0]);

  CrossfadeArray(&params->long_gains_pitch_forward[0][0], &gains[0][0],
                 kNumTransInLongitudinalInputs * kNumTransInLongitudinalStates,
                 c_pitched_forward, 0.5, 1.0, &gains[0][0]);

  double inputs[kNumTransInLongitudinalInputs];
  MatArrMult(&gains[0][0], kNumTransInLongitudinalInputs,
             kNumTransInLongitudinalStates, state_error, 1, inputs);

  IntegratorMode int_mode =
      enable_integrator ? kIntegratorModeIntegrate : kIntegratorModeHold;
  Integrator(Saturate(angle_of_attack_cmd - angle_of_attack,
                      -params->max_int_angle_of_attack_rate,
                      params->max_int_angle_of_attack_rate),
             -params->max_int_angle_of_attack, params->max_int_angle_of_attack,
             *g_sys.ts, int_mode, &state->int_angle_of_attack);

  *motor_pitch = inputs[kTransInLongitudinalInputMotorPitch];
  *delta_elevator = inputs[kTransInLongitudinalInputElevator] +
                    params->delta_elevator_alpha_zero +
                    params->ddelta_elevator_dalpha * angle_of_attack_cmd;
}

void TransInAttitudeInit(double pitch_ti_cmd, const Mat3 *dcm_ti2b,
                         const Vec3 *motor_moment_z1,
                         const TransInAttitudeParams *params,
                         TransInAttitudeState *state) {
  assert(dcm_ti2b != NULL && params != NULL && state != NULL);
  memset(state, 0, sizeof(*state));

  state->initial_pitch_moment =
      Saturate(motor_moment_z1->y, params->min_initial_pitch_moment, 0.0);
  state->initial_yaw_moment =
      Saturate(motor_moment_z1->z, -params->max_initial_yaw_moment,
               params->max_initial_yaw_moment);

  // Calculate the pitch error.
  double yaw_not_used, roll_not_used;
  DcmToAngle(dcm_ti2b, kRotationOrderZyx, &yaw_not_used,
             &state->initial_pitch_ti, &roll_not_used);
  double pitch_error = pitch_ti_cmd - state->initial_pitch_ti;

  // A negative pitch error indicates the kite is too far pitched up.
  // If this error is greater than a threshold we are already pitched
  // forward, and we simply turn on the forward flight controller.
  if (pitch_error > params->pitch_forward_max_pitch_error) {
    state->pitch_forward_duration = 0.0;
    state->pitch_forward_pitch_rate = 0.0;
  } else {
    // We limit the duration either by the maximum desired pitch rate,
    // or the maximum desired pitch acceleration.
    state->pitch_forward_duration = fmax(
        -2.0 * pitch_error / params->pitch_forward_max_pitch_rate,
        Sqrt(-pitch_error * 2.0 * PI / params->pitch_forward_max_pitch_accel));
    state->pitch_forward_duration =
        fmin(state->pitch_forward_duration, params->pitch_forward_max_duration);
    state->pitch_forward_pitch_rate =
        -2.0 * pitch_error / state->pitch_forward_duration;
  }

  state->release_integrator = false;
  state->int_angle_of_attack = 0.0;
  state->int_roll = 0.0;
}

// Modify the pitch angle and pitch rate commands to follow a desired
// initial pitch-forward trajectory.  We calculate a smooth trajectory
// between the initial pitch angle and the desired pitch angle:
//
// theta_cmd =
//     theta_0 + 0.5 * q_max * ((T / 2.0 / PI) * sin(2.0 * PI * t / T) - t)
// q_cmd = 0.5 * q_max * (cos(t * 2.0 * PI / T) - 1.0)
//
// for t in [0, T] where T = state->pitch_forward_duration.  This
// trajectory is blended into the externally commanded trajectory over
// the interval [0.5 * T, T].
//
// The return value fades from 0 to 1 as the end of the maneuver approaches.
//
// TODO: Add a unit test for this function and the
// initialization function to check the two limiting cases (rate
// limiting and acceleration limiting) and the crossfade.
static double CalcPitchForwardCommand(double flight_mode_time,
                                      TransInAttitudeState *state,
                                      Vec3 *eulers_ti_cmd,
                                      double *pitch_rate_b_cmd,
                                      Vec3 *motor_moment_ff) {
  double c =
      Crossfade(0.0, 1.0, flight_mode_time, 0.0, state->pitch_forward_duration);

  double pitch_forward_pitch_ti_cmd = 0.5 * state->pitch_forward_pitch_rate *
                                          state->pitch_forward_duration *
                                          (sin(2.0 * PI * c) / 2.0 / PI - c) +
                                      state->initial_pitch_ti;

  double pitch_forward_pitch_rate_b_cmd =
      0.5 * state->pitch_forward_pitch_rate * (cos(c * 2.0 * PI) - 1.0);

  eulers_ti_cmd->x = Crossfade(0.0, eulers_ti_cmd->x, c, 0.5, 1.0);
  eulers_ti_cmd->y =
      Crossfade(pitch_forward_pitch_ti_cmd, eulers_ti_cmd->y, c, 0.5, 1.0);
  eulers_ti_cmd->z = Crossfade(0.0, eulers_ti_cmd->z, c, 0.5, 1.0);

  *pitch_rate_b_cmd =
      Crossfade(pitch_forward_pitch_rate_b_cmd, *pitch_rate_b_cmd, c, 0.5, 1.0);

  motor_moment_ff->x = 0.0;
  motor_moment_ff->y = Crossfade(state->initial_pitch_moment, 0.0, c, 0.0, 0.5);
  motor_moment_ff->z = Crossfade(state->initial_yaw_moment, 0.0, c, 0.0, 0.5);

  return c;
}

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
                         ThrustMoment *thrust_moment, Deltas *deltas) {
  assert(dcm_ti2b != NULL && pqr_cmd != NULL && pqr != NULL &&
         tether_force_b != NULL && params != NULL && thrust_moment != NULL &&
         deltas != NULL);

  Vec3 eulers_ti2cmd = {roll_ti_cmd, pitch_trim_ti + angle_of_attack_cmd,
                        yaw_ti_cmd};
  double pitch_rate_b_cmd = pqr_cmd->y;

  Vec3 pitch_forward_feed_forward_moment;
  double c_pitched_forward = CalcPitchForwardCommand(
      flight_mode_time, state, &eulers_ti2cmd, &pitch_rate_b_cmd,
      &pitch_forward_feed_forward_moment);

  Vec3 axis_b2cmd;
  CalcAxisB2Cmd(&eulers_ti2cmd, dcm_ti2b, &axis_b2cmd);

  state->release_integrator =
      state->release_integrator ||
      (c_pitched_forward >= 0.95 &&
       airspeed > params->int_release_airspeed_threshold &&
       angle_of_attack < params->int_release_alpha_threshold);

  CalcLateralControl(angle_of_sideslip_cmd, angle_of_sideslip, &axis_b2cmd,
                     pqr_cmd, pqr, tether_force_b, c_pitched_forward,
                     state->release_integrator, params, state,
                     &thrust_moment->moment.x, &thrust_moment->moment.z,
                     &deltas->aileron, &deltas->rudder);

  CalcLongitudinalControl(angle_of_attack_cmd, angle_of_attack, &axis_b2cmd,
                          pitch_rate_b_cmd, pqr->y, tether_force_b,
                          c_pitched_forward, state->release_integrator, params,
                          state, &thrust_moment->moment.y, &deltas->elevator);

  thrust_moment->thrust = thrust_cmd;
  thrust_moment->moment.x += pitch_forward_feed_forward_moment.x;
  thrust_moment->moment.y += pitch_forward_feed_forward_moment.y;
  thrust_moment->moment.z += pitch_forward_feed_forward_moment.z;

  deltas->inboard_flap = delta_flap_cmd;
  deltas->midboard_flap = params->midboard_flap_ratio * delta_flap_cmd;
  deltas->outboard_flap = 0.0;

  // Update telemetry.
  TransInTelemetry *tt = GetTransInTelemetry();
  tt->axis_b2cmd = axis_b2cmd;
  tt->eulers_ti2cmd = eulers_ti2cmd;
  tt->pitch_rate_b_cmd = pitch_rate_b_cmd;
  tt->int_angle_of_attack = state->int_angle_of_attack;
  tt->int_roll = state->int_roll;
}

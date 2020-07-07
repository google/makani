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

#include "control/hover/hover_angles.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/hover/hover_experiments.h"
#include "control/hover/hover_frame.h"
#include "control/system_params.h"

// Returns true if the state of the angles loop is within a normal
// range.
static bool ValidateState(const HoverAnglesParams *params,
                          const HoverAnglesState *state) {
  if (!(params->int_pitch_pid.int_output_min <= state->int_elevator_moment &&
        state->int_elevator_moment <= params->int_pitch_pid.int_output_max)) {
    assert(!(bool)"int_elevator_moment is outside the integrator saturations.");
    return false;
  }

  if (!(params->int_yaw_pid.int_output_min <= state->int_rudder_moment &&
        state->int_rudder_moment <= params->int_yaw_pid.int_output_max)) {
    assert(!(bool)"int_rudder_moment is outside the integrator saturations.");
    return false;
  }

  if (!(params->pitch_pid.int_output_min <= state->int_moment.y &&
        state->int_moment.y <= params->pitch_pid.int_output_max)) {
    assert(!(bool)"int_moment.y is outside the integrator saturations.");
    return false;
  }

  if (!(params->yaw_pid.int_output_min <= state->int_moment.z &&
        state->int_moment.z <= params->yaw_pid.int_output_max)) {
    assert(!(bool)"int_moment.z is outside the integrator saturations.");
    return false;
  }

  if (!(Vec3Norm(&state->angles_error) <= PI)) {
    assert(!(bool)"angles_error magnitude must be less than or equal to pi.");
    return false;
  }

  return true;
}

void HoverAnglesInit(const HoverAnglesParams *params, HoverAnglesState *state) {
  assert(params != NULL && state != NULL);
  assert(HoverAnglesValidateParams(params));

  memset(state, 0, sizeof(*state));
  state->int_elevator_moment = 0.0;
  state->int_rudder_moment = 0.0;
  state->int_moment = kVec3Zero;
  state->angles_error = kVec3Zero;
  state->extra_pitch_moment_scale_z1 = 1.0;

  if (!ValidateState(params, state)) assert(false);
}

bool HoverAnglesValidateParams(const HoverAnglesParams *params) {
  if (!(params->perch_contact_extra_pitch_moment_max > 0.0)) {
    assert(!(bool)"perch_contact_extra_pitch_moment_max must be positive.");
    return false;
  }

  if (!(params->perch_contact_extra_pitch_moment_min <=
        params->perch_contact_extra_pitch_moment_max)) {
    assert(!(
        bool)"perch_contact_extra_pitch_moment_min must be less than"
             " or equal to perch_contact_extra_pitch_moment_max");
    return false;
  }

  if (!(params->perch_contact_total_pitch_moment_min <= 0.0)) {
    assert(!(bool)"perch_contact_total_pitch_moment_min must be non-positive.");
    return false;
  }

  if (!(params->perch_contact_total_pitch_moment_max > 0.0)) {
    assert(!(bool)"perch_contact_total_pitch_moment_max must be positive.");
    return false;
  }

  if (!(params->perch_contact_total_pitch_moment_min <=
        params->perch_contact_total_pitch_moment_max)) {
    assert(!(
        bool)"perch_contact_total_pitch_moment_min must be less than"
             " or equal to perch_contact_total_pitch_moment_max");
    return false;
  }

  if (!(params->perch_contact_extra_pitch_moment_fade_angle_min <=
        params->perch_contact_extra_pitch_moment_fade_angle_max)) {
    assert(!(
        bool)"perch_contact_extra_pitch_moment_fade_angle_min must be less than"
             " or equal to perch_contact_extra_pitch_moment_fade_angle_max.");
  }

  if (!(params->bridle_roll_damping_factor <= 0.0)) {
    assert(!(bool)"bridle_roll_damping_factor must be non-positive.");
    return false;
  }

  if (!(params->blown_flaps_roll_rate_gain >= 0.0)) {
    assert(!(bool)"blown_flaps_roll_rate_gain must be non-negative.");
    return false;
  }

  if (!(params->roll_pid.kp == 0.0 && params->roll_pid.ki == 0.0 &&
        params->roll_pid.kd == 0.0)) {
    assert(!(bool)"roll_pid gains must be zero.");
    return false;
  }

  if (!(params->low_thrust_pitch_pid.kp >= 0.0 &&
        params->low_thrust_pitch_pid.ki >= 0.0 &&
        params->low_thrust_pitch_pid.kd >= 0.0)) {
    assert(!(bool)"low_thrust_pitch_pid gains must be non-negative.");
    return false;
  }

  if (!(params->low_thrust_pitch_pid.int_output_min <=
        params->low_thrust_pitch_pid.int_output_max)) {
    assert(!(
        bool)"low_thrust_pitch_pid.int_output_max must be greater than or equal"
             " to low_thrust_pitch_pid.int_output_min.");
    return false;
  }

  if (!(params->pitch_pid.kp >= 0.0 && params->pitch_pid.ki >= 0.0 &&
        params->pitch_pid.kd >= 0.0)) {
    assert(!(bool)"pitch_pid gains must be non-negative.");
    return false;
  }

  if (!(params->pitch_pid.int_output_min <= params->pitch_pid.int_output_max)) {
    assert(!(
        bool)"pitch_pid.int_output_max must be greater than or equal"
             " to pitch_pid.int_output_min.");
    return false;
  }

  if (!(params->yaw_pid.kp >= 0.0 && params->yaw_pid.ki >= 0.0 &&
        params->yaw_pid.kd >= 0.0)) {
    assert(!(bool)"yaw_pid gains must be non-negative.");
    return false;
  }

  if (!(params->yaw_pid.int_output_min <= params->yaw_pid.int_output_max)) {
    assert(!(
        bool)"yaw_pid.int_output_max must be greater than or equal"
             " to yaw_pid.int_output_min.");
    return false;
  }

  if (!(params->int_pitch_pid.kp >= 0.0 && params->int_pitch_pid.ki >= 0.0 &&
        params->int_pitch_pid.kd >= 0.0)) {
    assert(!(bool)"int_pitch_pid gains must be non-negative.");
    return false;
  }

  if (!(params->int_pitch_pid.int_output_min <=
        params->int_pitch_pid.int_output_max)) {
    assert(!(
        bool)"int_pitch_pid.int_output_max must be greater than or equal"
             " to int_pitch_pid.int_output_min.");
    return false;
  }

  if (!(params->int_yaw_pid.kp >= 0.0 && params->int_yaw_pid.ki >= 0.0 &&
        params->int_yaw_pid.kd >= 0.0)) {
    assert(!(bool)"int_yaw_pid gains must be non-negative.");
    return false;
  }

  if (!(params->int_yaw_pid.int_output_min <=
        params->int_yaw_pid.int_output_max)) {
    assert(!(
        bool)"int_yaw_pid.int_output_max must be greater than or equal"
             " to int_yaw_pid.int_output_min.");
    return false;
  }

  return true;
}

void HoverAnglesGetAnglesError(const HoverAnglesState *state,
                               Vec3 *angles_error) {
  assert(state != NULL && angles_error != NULL);

  *angles_error = state->angles_error;
}

void HoverAnglesGetAngles(const Vec3 *wing_pos_g, const Vec3 *hover_origin_g,
                          const Mat3 *dcm_g2b, Vec3 *angles) {
  assert(wing_pos_g != NULL && hover_origin_g != NULL && dcm_g2b != NULL &&
         angles != NULL);
  assert(Mat3IsSpecialOrthogonal(dcm_g2b, 1e-9));

  Mat3 dcm_g2h;
  CalcDcmGToH(wing_pos_g, hover_origin_g, &dcm_g2h);

  Mat3 dcm_h2g;
  Mat3Trans(&dcm_g2h, &dcm_h2g);

  Mat3 dcm_h2b;
  Mat3Mat3Mult(dcm_g2b, &dcm_h2g, &dcm_h2b);

  Quat q_h2b;
  DcmToQuat(&dcm_h2b, &q_h2b);
  QuatToAxis(&q_h2b, angles);
}

// Calculates the axis-angle error.  Although the axis-angle command
// and measurement are stored as vectors, they are not true vectors
// and can not be subtracted from each other to find the axis-angle
// error.  Instead, the axis-angle command and measurement are
// converted to a quaternion representation, where a quaternion error
// is calculated, and then the quaternion error is converted back to
// an axis-angle representation.
static void CalcAnglesError(const Vec3 *angles_cmd, const Vec3 *angles,
                            Vec3 *angles_error) {
  Quat q_h2b_cmd;
  AxisToQuat(angles_cmd, &q_h2b_cmd);

  Quat q_b2h;
  AxisToQuat(angles, &q_b2h);
  QuatInv(&q_b2h, &q_b2h);

  Quat q_b2b_cmd;
  QuatMultiply(&q_h2b_cmd, &q_b2h, &q_b2b_cmd);
  QuatToAxis(&q_b2b_cmd, angles_error);
}

static bool InNormalPerchContact(FlightMode flight_mode) {
  return (flight_mode == kFlightModePerched ||
          flight_mode == kFlightModeHoverAscend ||
          flight_mode == kFlightModeHoverDescend);
}

// Selects roll, pitch, and yaw integrator modes based on flight mode,
// proximity to perch, etc.
static void SelectIntegratorModes(FlightMode flight_mode, double thrust_z1,
                                  const FlightStatus *flight_status,
                                  const ExperimentState *experiment_state,
                                  IntegratorMode *roll_int_mode,
                                  IntegratorMode *pitch_int_mode,
                                  IntegratorMode *yaw_int_mode) {
  if (flight_mode == kFlightModePerched) {
    // Reset integrators in any non-hover flight mode.
    *roll_int_mode = kIntegratorModeReset;
    *pitch_int_mode = kIntegratorModeReset;
    *yaw_int_mode = kIntegratorModeReset;
  } else if (thrust_z1 < g_sys.wing->m * g_sys.phys->g / 2.0 &&
             flight_mode != kFlightModeHoverTransOut) {
    // Hold integrators if we do not have significant thrust.  This
    // should only occur if we are perched or hanging from the
    // constraint system, and thus are constrained.
    *roll_int_mode = kIntegratorModeHold;
    *pitch_int_mode = kIntegratorModeHold;
    *yaw_int_mode = kIntegratorModeHold;
  } else if (InNormalPerchContact(flight_mode)) {
    // Hold the pitch integrator near the perch, as the vehicle
    // transitions from constrained to unconstrained pitch.
    *roll_int_mode = kIntegratorModeIntegrate;
    *pitch_int_mode = kIntegratorModeHold;
    *yaw_int_mode = kIntegratorModeIntegrate;
  } else {
    *roll_int_mode = kIntegratorModeIntegrate;
    *yaw_int_mode = kIntegratorModeIntegrate;
    if (HoverElevatorExperimentIsEnabled(flight_status, experiment_state)) {
      *pitch_int_mode = kIntegratorModeHold;
    } else {
      *pitch_int_mode = kIntegratorModeIntegrate;
    }
  }

  // Prevent the yaw integrator from fighting the yaw moment due to
  // the asymmetric heights of the perch panels.  This was preventing
  // the wing from landing in simulation.
  if (flight_mode == kFlightModeHoverDescend) {
    *yaw_int_mode = kIntegratorModeHold;
  }
}

// Schedules pitch gains based on flight mode and thrust.  This is
// here only to remove the symmetric torsional pitch oscillations that
// appear as the wing descends onto constraints.
//
// NOTE: Based on the 2016-04-26 flight tests, the lower
// pitch gains do make these oscillations go away; however they also
// lead to larger excursions in pitch angle during hover in high wind.
static void SchedulePitchGains(FlightMode flight_mode, double thrust_z1,
                               const HoverAnglesParams *params,
                               PidParams *pitch_pid) {
  if (flight_mode == kFlightModeHoverDescend) {
    assert(thrust_z1 >= 0.0);
    CrossfadePidParams(&params->low_thrust_pitch_pid, &params->pitch_pid,
                       thrust_z1, 0.0, 0.75 * g_sys.wing->m * g_sys.phys->g,
                       pitch_pid);
  } else {
    *pitch_pid = params->pitch_pid;
  }
}

// Modifies pitch moment command during contact with the perch.  When
// the wing contacts the perch, we no longer attempt to control pitch,
// rather we simply apply a constant pitch back moment.  However, if
// we do temporarily lose contact with the perch, we should continue
// to control pitch normally.  We achieve this by using the normal
// controller, but applying a set of crossfades and saturations to the
// pitch moment.
static double ModifyPitchMomentDuringPerching(
    bool perching, double pitch_moment, double pitch_error,
    const HoverAnglesParams *params, double *extra_pitch_moment_scale_z1) {
  assert(fabs(pitch_error) <= PI);
  assert(params != NULL);
  assert(extra_pitch_moment_scale_z1 != NULL);

  // Force controllers with small proportional gains to apply a
  // pitch back moment during perching.

  // Do not apply the pitch back moment if we are meeting our angle
  // command.  This is a safety precaution in case the wing is above
  // the perch panels.
  double extra_pitch_moment =
      Crossfade(params->perch_contact_extra_pitch_moment_min,
                params->perch_contact_extra_pitch_moment_max, pitch_error,
                params->perch_contact_extra_pitch_moment_fade_angle_min,
                params->perch_contact_extra_pitch_moment_fade_angle_max);

  double perching_pitch_moment =
      Saturate(pitch_moment + extra_pitch_moment,
               params->perch_contact_total_pitch_moment_min,
               params->perch_contact_total_pitch_moment_max);

  double scale =
      RateLimit(perching, -0.5, 0.5, *g_sys.ts, extra_pitch_moment_scale_z1);
  return Mix(pitch_moment, perching_pitch_moment, scale);
}

// Uses the elevator to offset the integrated pitch moment from the
// pitch loop.  Returns a pitch moment command to be met by the
// elevator in the output stage.
static double ControlIntegratedPitchError(double int_pitch_error,
                                          IntegratorMode pitch_int_mode,
                                          const HoverAnglesParams *params,
                                          HoverAnglesState *state) {
  return params->nominal_elevator_pitch_moment +
         Pid(int_pitch_error, 0.0, *g_sys.ts, pitch_int_mode,
             &params->int_pitch_pid, &state->int_elevator_moment);
}

// Uses the rudder to offset the integrated yaw moment from the yaw
// loop.  Returns a yaw moment command to be met by the rudder in the
// output stage.
static double ControlIntegratedYawError(double int_yaw_error,
                                        IntegratorMode yaw_int_mode,
                                        const HoverAnglesParams *params,
                                        HoverAnglesState *state) {
  return Pid(int_yaw_error, 0.0, *g_sys.ts, yaw_int_mode, &params->int_yaw_pid,
             &state->int_rudder_moment);
}

void HoverAnglesStep(const Vec3 *angles_cmd, const Vec3 *angles,
                     const Vec3 *pqr_cmd, const Vec3 *pqr,
                     const WinchEstimate *winch, const VesselEstimate *vessel,
                     double thrust_z1, const FlightStatus *flight_status,
                     const HoverAnglesParams *params,
                     const ExperimentState *experiment_state,
                     HoverAnglesState *state, Vec3 *moment_cmd,
                     double *blown_flaps_roll_moment_cmd,
                     double *elevator_pitch_moment_cmd,
                     double *rudder_yaw_moment_cmd) {
  assert(angles_cmd != NULL && Vec3Norm(angles_cmd) <= PI);
  assert(angles != NULL && Vec3Norm(angles) <= PI);
  assert(pqr_cmd != NULL && pqr != NULL);
  assert(winch != NULL);
  assert(0 <= flight_status->flight_mode &&
         flight_status->flight_mode < kNumFlightModes);
  assert(params != NULL && state != NULL && ValidateState(params, state));
  assert(moment_cmd != NULL && elevator_pitch_moment_cmd != NULL);

  // Calculate angle error.
  CalcAnglesError(angles_cmd, angles, &state->angles_error);

  // Calculate angular rate error.
  Vec3 pqr_error;
  Vec3Sub(pqr_cmd, pqr, &pqr_error);

  // Select integrator modes.
  IntegratorMode roll_int_mode, pitch_int_mode, yaw_int_mode;
  SelectIntegratorModes(flight_status->flight_mode, thrust_z1, flight_status,
                        experiment_state, &roll_int_mode, &pitch_int_mode,
                        &yaw_int_mode);

  PidParams pitch_pid;
  SchedulePitchGains(flight_status->flight_mode, thrust_z1, params, &pitch_pid);

  // Apply feed-forward and feedback laws.

  // TODO: Account for the moment due to tether tension
  // acting on the bridle point.
  Vec3 moment_ff = {0.0, -params->nominal_elevator_pitch_moment, 0.0};

  Vec3 moment_fb;
  moment_fb.x = Pid(state->angles_error.x, pqr_error.x, *g_sys.ts,
                    roll_int_mode, &params->roll_pid, &state->int_moment.x);
  moment_fb.y = Pid(state->angles_error.y, pqr_error.y, *g_sys.ts,
                    pitch_int_mode, &pitch_pid, &state->int_moment.y);
  moment_fb.z =
      Pid(state->angles_error.z,
          pqr_error.z + params->bridle_roll_damping_factor * pqr_error.x,
          *g_sys.ts, yaw_int_mode, &params->yaw_pid, &state->int_moment.z);

  Vec3Add(&moment_ff, &moment_fb, moment_cmd);

  // Blown flaps roll damping.
  *blown_flaps_roll_moment_cmd =
      params->blown_flaps_roll_rate_gain * pqr_error.x;

  // The pitch angle error given to ModifyPitchMomentDuringPerching must
  // be compensated for rotational motion of the buoy offshore. Specifically,
  // we want the "pitch" angle of the platform frame in the direction of
  // the kite. Because the platform frame's y axis is normally pointed
  // at the kite, we want the x axis Euler angle of dcm_g2p. Nominally,
  // the platform's x axis and the kite's y axis are parallel during
  // launch/land (ignoring the yaw attitude of the kite).
  // A larger x axis Euler angle means the perch pitches forward towards
  // the kite, hence the pitch error (command - measured) needs to be increased
  // to pitch back further in order to press the perch peg onto the panel.
  double angles_error_y = state->angles_error.y;
  double g2p_eulers_x, unused;
  DcmToAngle(&vessel->dcm_g2p, kRotationOrderZyx, &unused, &unused,
             &g2p_eulers_x);
  angles_error_y += g2p_eulers_x;

  // Modify pitch moment command during contact with the perch.
  moment_cmd->y = ModifyPitchMomentDuringPerching(
      InNormalPerchContact(flight_status->flight_mode), moment_cmd->y,
      angles_error_y, params, &state->extra_pitch_moment_scale_z1);

  // Saturate the moments the attitude loop will request.  In normal
  // flight, this is important to avoid a limit cycle caused by the
  // power system's rate limit on motor speed.
  if (flight_status->flight_mode == kFlightModeHoverAccel) {
    SaturateVec3(moment_cmd, &params->min_accel_moment,
                 &params->max_accel_moment, moment_cmd);
  } else {
    SaturateVec3(moment_cmd, &params->min_moment, &params->max_moment,
                 moment_cmd);
  }

  // Use the elevator and rudder to reduce the integrated pitch and
  // yaw moments, respectively.
  *elevator_pitch_moment_cmd = ControlIntegratedPitchError(
      state->int_moment.y / pitch_pid.ki, pitch_int_mode, params, state);
  *rudder_yaw_moment_cmd = ControlIntegratedYawError(
      state->int_moment.z / params->yaw_pid.ki, yaw_int_mode, params, state);

  // Update telemetry.
  HoverTelemetry *ht = GetHoverTelemetry();
  ht->angles_cmd = *angles_cmd;
  ht->angles = *angles;
  ht->moment_ff = moment_ff;
  ht->moment_fb = moment_fb;
  ht->moment_cmd = *moment_cmd;
  ht->int_moment = state->int_moment;
  ht->elevator_pitch_moment = *elevator_pitch_moment_cmd;
  ht->rudder_yaw_moment = *rudder_yaw_moment_cmd;
}

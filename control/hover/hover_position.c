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

#include "control/hover/hover_position.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/common.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/hover/hover_angles.h"
#include "control/hover/hover_frame.h"
#include "control/system_params.h"
#include "control/system_types.h"

static bool ValidateState(const HoverPositionParams *params,
                          const HoverPositionState *state) {
  // Find absolute minimum and maximum integrated values across all
  // tangential controllers.  These values must be expanded by the
  // machine precision to fully capture the range of expected values.
  double min_int_angles_z = fmin(
      fmin(params->low_altitude_long_tether_tangential_pid.int_output_min,
           params->high_altitude_long_tether_tangential_pid.int_output_min),
      params->short_tether_tangential_pid.int_output_min);
  min_int_angles_z -= fabs(min_int_angles_z) * DBL_EPSILON;
  double max_int_angles_z = fmax(
      fmax(params->low_altitude_long_tether_tangential_pid.int_output_max,
           params->high_altitude_long_tether_tangential_pid.int_output_max),
      params->short_tether_tangential_pid.int_output_max);
  max_int_angles_z += fabs(max_int_angles_z) * DBL_EPSILON;

  if (!(state->int_angles.x == 0.0 && state->int_angles.y == 0.0 &&
        min_int_angles_z <= state->int_angles.z &&
        state->int_angles.z <= max_int_angles_z)) {
    assert(!(bool)"int_angles is outside the integrator saturations.");
    return false;
  }

  return true;
}

void HoverPositionInit(double previous_int_yaw_angle,
                       const HoverPositionParams *params,
                       HoverPositionState *state) {
  assert(params != NULL && state != NULL);
  assert(HoverPositionValidateParams(params));

  memset(state, 0, sizeof(*state));
  state->int_angles = kVec3Zero;

  // The previous integrated yaw angle is passed in during
  // initialization because this parameter can take a long time to
  // settle especially at the lower gains at high altitudes.
  state->int_angles.z = previous_int_yaw_angle;

  if (!ValidateState(params, state)) assert(false);
}

bool HoverPositionValidateParams(const HoverPositionParams *params) {
  if (!(-PI < params->eulers_ff.x && params->eulers_ff.x < PI &&
        -PI / 2.0 < params->eulers_ff.y && params->eulers_ff.y < PI / 2.0 &&
        -PI < params->eulers_ff.z && params->eulers_ff.z < PI)) {
    assert(
        !(bool)"euler_ff must be in the range: |euler_ff| <= [pi, pi/2, pi].");
    return false;
  }

  if (!(0.0 <= params->short_tether &&
        params->short_tether <= params->long_tether)) {
    assert(!(bool)"long_tether must be longer than short_tether.");
    return false;
  }

  if (!(params->low_altitude <= params->high_altitude)) {
    assert(!(bool)"low_altitude must be less than high_altitude.");
    return false;
  }

  // TODO: Validate short and long tether radial gains
  // once that control strategy has been finalized.

  if (!(params->short_tether_tangential_pid.kp > 0.0 &&
        params->short_tether_tangential_pid.ki >= 0.0 &&
        params->short_tether_tangential_pid.kd > 0.0)) {
    assert(!(bool)"short_tether_tangential_pid gains must be non-negative.");
    return false;
  }

  if (!(params->short_tether_tangential_pid.int_output_min <=
        params->short_tether_tangential_pid.int_output_max)) {
    assert(!(
        bool)"short_tether_tangential_pid.int_output_max must be greater than "
             "or"
             " equal to short_tether_tangential_pid.int_output_min.");
    return false;
  }

  if (!(params->low_altitude_long_tether_tangential_pid.kp > 0.0 &&
        params->low_altitude_long_tether_tangential_pid.ki >= 0.0 &&
        params->low_altitude_long_tether_tangential_pid.kd > 0.0)) {
    assert(!(
        bool)"low_altitude_long_tether_tangential_pid gains must be"
             " non-negative.");
    return false;
  }

  if (!(params->low_altitude_long_tether_tangential_pid.int_output_min <=
        params->low_altitude_long_tether_tangential_pid.int_output_max)) {
    assert(!(
        bool)"low_altitude_long_tether_tangential_pid.int_output_max must be"
             " greater than or equal to"
             " long_tether_tangential_pid.int_output_min.");
    return false;
  }

  if (!(params->high_altitude_long_tether_tangential_pid.kp > 0.0 &&
        params->high_altitude_long_tether_tangential_pid.ki >= 0.0 &&
        params->high_altitude_long_tether_tangential_pid.kd > 0.0)) {
    assert(!(
        bool)"high_altitude_long_tether_tangential_pid gains must be"
             " non-negative.");
    return false;
  }

  if (!(params->high_altitude_long_tether_tangential_pid.int_output_min <=
        params->high_altitude_long_tether_tangential_pid.int_output_max)) {
    assert(!(
        bool)"high_altitude_long_tether_tangential_pid.int_output_max must be"
             " greater than or equal to"
             " long_tether_tangential_pid.int_output_min.");
    return false;
  }

  if (!(params->max_pos_angle_fb > 0.0)) {
    assert(!(bool)"max_pos_angle_fb must be positive.");
    return false;
  }

  if (!(params->max_vel_angle_fb > 0.0)) {
    assert(!(bool)"max_vel_angle_fb must be positive.");
    return false;
  }

  if (!(0.0 <= params->k_pilot.x && params->k_pilot.x < 1.0 &&
        0.0 <= params->k_pilot.y && params->k_pilot.y < 1.0 &&
        0.0 <= params->k_pilot.z && params->k_pilot.z < 1.0)) {
    assert(!(bool)"k_pilot gains must be positive and reasonably small.");
    return false;
  }

  if (!(params->transout_angles_cmd_crossfade_start_times.x <=
            params->transout_angles_cmd_crossfade_end_times.x &&
        params->transout_angles_cmd_crossfade_start_times.y <=
            params->transout_angles_cmd_crossfade_end_times.y &&
        params->transout_angles_cmd_crossfade_start_times.z <=
            params->transout_angles_cmd_crossfade_end_times.z)) {
    assert(!(
        bool)"transout_angles_cmd_crossfade_start_times must be less than "
             " transout_angles_cmd_crossfade_end_times.");
    return false;
  }

  if (!(params->transout_pqr_cmd_crossfade_duration.x >= 0.0 &&
        params->transout_pqr_cmd_crossfade_duration.y >= 0.0 &&
        params->transout_pqr_cmd_crossfade_duration.z >= 0.0)) {
    assert(!(bool)"transout_pqr_cmd_crossfade_length must be non-negative.");
  }

  if (!(params->min_angles.x <= params->max_angles.x &&
        params->min_angles.y <= params->max_angles.y &&
        params->min_angles.z <= params->max_angles.z)) {
    assert(!(bool)"min_angles must be less than max_angles.");
    return false;
  }

  return true;
}

// Calculates the velocity error in body coordinates.  The velocity
// error is modified by removing the z-component in the ground frame,
// and is then transformed to body coordinates and saturated.
static void CalcVelocityError(const Vec3 *wing_vel_g_cmd,
                              const Vec3 *wing_vel_g, const Mat3 *dcm_g2b,
                              double tangential_kd,
                              const FlightStatus *flight_status,
                              const HoverPositionParams *params,
                              Vec3 *wing_vel_b_error) {
  assert(tangential_kd > 0.0);

  // Calculate velocity error in ground frame.
  Vec3 wing_vel_g_error;
  Vec3Sub(wing_vel_g_cmd, wing_vel_g, &wing_vel_g_error);

  // Remove error in vertical direction because altitude is handled by
  // a separate controller.
  wing_vel_g_error.z = 0.0;

  // Transform to body coordinates.
  Mat3Vec3Mult(dcm_g2b, &wing_vel_g_error, wing_vel_b_error);

  // Saturate velocity error so that the derivative term of the
  // controller can never command a larger angle than
  // max_vel_angle_fb.  Note that there is a separate maximum error
  // for the radial component following transition-out, which helps
  // the kite gracefully return to the tether sphere.
  double max_error = params->max_vel_angle_fb / tangential_kd;
  double radial_max_error = max_error;
  if (flight_status->flight_mode == kFlightModeHoverTransOut &&
      flight_status->last_flight_mode == kFlightModeCrosswindPrepTransOut) {
    radial_max_error = Crossfade(2.0 * max_error, max_error,
                                 flight_status->flight_mode_time, 0.0, 20.0);
  }
  wing_vel_b_error->x = Saturate(wing_vel_b_error->x, -max_error, max_error);
  wing_vel_b_error->y = Saturate(wing_vel_b_error->y, -max_error, max_error);
  wing_vel_b_error->z =
      Saturate(wing_vel_b_error->z, -radial_max_error, radial_max_error);
}

// Calculates the tangential position error in body coordinates.  Note
// that we do not control radial position, other than with winch
// payout and tension, and altitude is controlled in a separate loop.
// The position error in the hover frame is saturated along the
// tangential direction before being transformed to the body frame.
//
// TODO: This function defines the hover frame in terms
// of the measured rather than commanded position.  This seems
// appropriate but we should add a detailed comment describing this
// choice.
static void CalcPositionError(const Vec3 *wing_pos_g_cmd,
                              const Vec3 *wing_pos_g,
                              const Vec3 *hover_origin_g, const Mat3 *dcm_g2b,
                              double tangential_kp,
                              const HoverPositionParams *params,
                              Vec3 *wing_pos_b_error) {
  assert(tangential_kp > 0.0);

  // Calculate position error in ground frame.
  Vec3 wing_pos_g_error;
  Vec3Sub(wing_pos_g_cmd, wing_pos_g, &wing_pos_g_error);

  // Rotate position error to hover frame.
  Vec3 wing_pos_h_error;
  RotateGToH(&wing_pos_g_error, wing_pos_g, hover_origin_g, &wing_pos_h_error);

  // Project position error onto the tangent vector and saturate the
  // projected error.
  double max_tangential_pos_error = params->max_pos_angle_fb / tangential_kp;
  wing_pos_h_error.x = 0.0;
  wing_pos_h_error.y = Saturate(wing_pos_h_error.y, -max_tangential_pos_error,
                                max_tangential_pos_error);
  wing_pos_h_error.z = 0.0;

  // Transform to body frame for conversion to hover angle commands.
  RotateHToG(&wing_pos_h_error, wing_pos_g, hover_origin_g, &wing_pos_g_error);
  Mat3Vec3Mult(dcm_g2b, &wing_pos_g_error, wing_pos_b_error);
}

// Calculates the angle command, in an Euler vector representation,
// that results in no tangential position movement. The pitch command,
// which is used to govern tension (see hover_tension.c) is added as
// an additional rotation.
//
// TODO: This doesn't account for the dependence of the Z
// rotation on the length of the tether when the propellers are all
// rotating the same direction.
static void CalcFeedForwardAngles(double pitch_cmd,
                                  const HoverPositionParams *params,
                                  Vec3 *angles_ff) {
  assert(-PI / 2.0 <= pitch_cmd && pitch_cmd <= PI / 2.0);

  // Introduce a temporary 'nominal' frame, which is defined to be a
  // trim orientation where there is no roll moment from bridling,
  // there is no tension due to the thrust vector (ignoring the
  // vectoring of thrust by the airframe), and there is no side force
  // from pylon lift or a tilted thrust vector.  Because this trim
  // vector is found empirically and because the angles are small, the
  // rotation order is not critical.  However, we choose the order XYZ
  // because X and Y have straightforward geometric interpretations:
  // rotate about the body x-axis to align the tether with the wing's
  // center-of-mass, i.e. remove the angle from the bridle offset, and
  // then pitch back to orient the propellers straight up as they are
  // tilted down 3 degrees in body coordinates.  The final rotation
  // about the body z-axis is used to align the combined rotor thrust
  // and pylon lift vector opposite the tether.
  Mat3 dcm_nominal2b;
  AngleToDcm(params->eulers_ff.x, params->eulers_ff.y, params->eulers_ff.z,
             kRotationOrderXyz, &dcm_nominal2b);

  // Rotate by pitch_cmd to go from the hover frame to the nominal
  // frame.  Because the nominal frame is defined to have no y-axis
  // force, this should be a pure rotation of the force vector about
  // the hover y-axis.  Note that this is separated from the Euler
  // angles above because their rotation order does not begin with Y.
  Mat3 dcm_h2nominal;
  AngleToDcm(0.0, pitch_cmd, 0.0, kRotationOrderZyx, &dcm_h2nominal);

  // Combine the rotations to find the rotation from the hover to body
  // frame.
  Mat3 dcm_h2b;
  Mat3Mat3Mult(&dcm_nominal2b, &dcm_h2nominal, &dcm_h2b);

  // Convert to an Euler vector representation by passing through a
  // quaternion representation.
  Quat q_h2b;
  DcmToQuat(&dcm_h2b, &q_h2b);
  QuatToAxis(&q_h2b, angles_ff);
}

// Schedules PID gains as a function of payout and altitude.  The
// radial and tangential dynamics of the system change considerably as
// a function of payout because of the change in the catenary spring
// constant and the introduction of tether transverse swinging modes.
static void SchedulePid(double payout, double wing_pos_z_g,
                        const HoverPositionParams *params,
                        PidParams *radial_pid, PidParams *tangential_pid) {
  assert(-1.0 < payout && payout < g_sys.tether->length + 10.0);
  assert(params != NULL && radial_pid != NULL && tangential_pid != NULL);
  assert(radial_pid != tangential_pid);

  CrossfadePidParams(&params->short_tether_radial_pid,
                     &params->long_tether_radial_pid, payout,
                     params->short_tether, params->long_tether, radial_pid);

  PidParams long_tether_tangential_pid;
  CrossfadePidParams(&params->low_altitude_long_tether_tangential_pid,
                     &params->high_altitude_long_tether_tangential_pid,
                     -wing_pos_z_g, params->low_altitude, params->high_altitude,
                     &long_tether_tangential_pid);
  CrossfadePidParams(&params->short_tether_tangential_pid,
                     &long_tether_tangential_pid, payout, params->short_tether,
                     params->long_tether, tangential_pid);
}

// Converts the joystick roll, pitch, yaw sticks to roll, pitch, yaw
// feedback angles.
static void ConvertJoystickToAngles(const JoystickData *joystick,
                                    const HoverPositionParams *params,
                                    Vec3 *angles_fb) {
  assert(-1.0 <= joystick->roll && joystick->roll <= 1.0);
  assert(-1.0 <= joystick->pitch && joystick->pitch <= 1.0);
  assert(-1.0 <= joystick->yaw && joystick->yaw <= 1.0);

  angles_fb->x = params->k_pilot.x * Saturate(joystick->roll, -1.0, 1.0);
  angles_fb->y = params->k_pilot.y * Saturate(joystick->pitch, -1.0, 1.0);
  angles_fb->z = params->k_pilot.z * Saturate(joystick->yaw, -1.0, 1.0);
}

// Fades the position feedback to zero once the wing has landed on
// the perch.
static void FadeFeedbackAfterDescend(const Vec3 *angles_fb,
                                     const Vec3 *angles_ff, const Mat3 *dcm_g2b,
                                     const Vec3 *wing_pos_g_cmd,
                                     const Vec3 *wing_pos_g,
                                     const Vec3 *hover_origin_g,
                                     double wing_offset_from_perched_g_z,
                                     Vec3 *faded_angles_fb) {
  // Calculate the current Euler vector angles of the wing.
  Vec3 angles;
  HoverAnglesGetAngles(wing_pos_g, hover_origin_g, dcm_g2b, &angles);

  // Given the current attitude and feed-forward term, calculate the
  // feedback term that is required to produce zero total command (the
  // feed-forward term will be added back later).
  Vec3 zero_fb;
  Vec3Sub(&angles, angles_ff, &zero_fb);

  // Fade the current feedback term with the term that produces zero
  // command as a function of both the difference between commanded
  // and measured wing altitude and the direct measurement of the wing
  // height above the perch.  Note that the difference between
  // commanded and measured altitude is an indicator that the wing has
  // landed on the perch because the altitude is commanded far below
  // the perched altitude (see descend_offset_g_z) to ensure that the
  // wing will land even if GPS has drifted.
  Vec3 mixed_angles_fb;
  CrossfadeVec3(angles_fb, &zero_fb, wing_pos_g_cmd->z - wing_pos_g->z, 0.5,
                2.0, &mixed_angles_fb);
  CrossfadeVec3(angles_fb, &mixed_angles_fb, wing_offset_from_perched_g_z, -1.0,
                0.0, faded_angles_fb);
}

void HoverPositionStep(const Vec3 *wing_pos_g_cmd, const Vec3 *wing_pos_g,
                       const Vec3 *wing_vel_g_cmd, const Vec3 *wing_vel_g,
                       const Vec3 *hover_origin_g, const Vec3 *pqr,
                       const Vec3 *angles, double pitch_cmd, double wind_speed,
                       const JoystickData *joystick, const Mat3 *dcm_g2b,
                       double payout, const FlightStatus *flight_status,
                       double wing_offset_from_perched_g_z,
                       const HoverPositionParams *params,
                       HoverPositionState *state, Vec3 *pqr_cmd,
                       Vec3 *angles_cmd) {
  assert(wing_pos_g_cmd != NULL && wing_pos_g != NULL);
  assert(wing_vel_g_cmd != NULL && Vec3Norm(wing_vel_g_cmd) < 340.0);
  assert(wing_vel_g != NULL && Vec3Norm(wing_vel_g) < 340.0);
  assert(-PI / 2.0 <= pitch_cmd && pitch_cmd <= PI / 2.0);
  assert(joystick != NULL);
  assert(dcm_g2b != NULL && Mat3IsSpecialOrthogonal(dcm_g2b, 1e-9));
  assert(-1.0 < payout && payout < g_sys.tether->length + 10.0);
  assert(flight_status != NULL &&
         IsValidFlightMode(flight_status->flight_mode));
  assert(params != NULL && state != NULL && ValidateState(params, state));
  assert(angles_cmd != NULL);

  // Schedule PID gains as a function of payout.
  PidParams radial_pid, tangential_pid;
  SchedulePid(payout, wing_pos_g->z, params, &radial_pid, &tangential_pid);

  // Calculate saturated position and velocity errors.  The maximum
  // errors are scaled by the tangential gains, so the angle feedback
  // to the inner loop is limited.
  Vec3 wing_pos_b_error;
  CalcPositionError(wing_pos_g_cmd, wing_pos_g, hover_origin_g, dcm_g2b,
                    tangential_pid.kp, params, &wing_pos_b_error);

  Vec3 wing_vel_b_error;
  CalcVelocityError(wing_vel_g_cmd, wing_vel_g, dcm_g2b, tangential_pid.kd,
                    flight_status, params, &wing_vel_b_error);

  Vec3 angles_ff;
  CalcFeedForwardAngles(pitch_cmd, params, &angles_ff);

  // Reset integrator when autonomous hover controller does not have
  // control.
  IntegratorMode int_mode;
  if (!AnyAutoHoverFlightMode(flight_status->flight_mode)) {
    int_mode = kIntegratorModeReset;
  } else {
    int_mode = kIntegratorModeIntegrate;
  }

  // Convert ground frame x and y errors into desired body angles.  An
  // error in wing_pos_b.z becomes a rotation about the body y-axis, and an
  // error in wing_pos_b.y becomes a rotation about the body z-axis.
  //
  // TODO: There was some effort in the past to make the
  // radial and tangential directions correspond to the actual radial
  // and tangential directions rather than the body directions, but I
  // removed it for simplicity.  We should revisit that method.
  Vec3 angles_fb;
  angles_fb.x = 0.0;
  angles_fb.y = Pid(wing_pos_b_error.z, wing_vel_b_error.z, *g_sys.ts, int_mode,
                    &radial_pid, &state->int_angles.y);
  angles_fb.z = Pid(wing_pos_b_error.y, wing_vel_b_error.y, *g_sys.ts, int_mode,
                    &tangential_pid, &state->int_angles.z);

  // Modify feedback term based on flight mode.
  if (flight_status->flight_mode == kFlightModePilotHover) {
    ConvertJoystickToAngles(joystick, params, &angles_fb);
    // Populate the yaw integrator to ensure a smooth transition from
    // PilotHover to autonomous.
    state->int_angles.z = angles_fb.z;
  } else if (flight_status->flight_mode == kFlightModeHoverDescend) {
    FadeFeedbackAfterDescend(&angles_fb, &angles_ff, dcm_g2b, wing_pos_g_cmd,
                             wing_pos_g, hover_origin_g,
                             wing_offset_from_perched_g_z, &angles_fb);
  }

  // Sum feedback and feed-forward commands.
  //
  // TODO: Here, and elsewhere, we are relying on these
  // angles being small to be able to add them like vectors.  We
  // should properly add these as rotations.
  Vec3Add(&angles_ff, &angles_fb, angles_cmd);
  *pqr_cmd = kVec3Zero;

  double transout_pitch_cmd = Crossfade(
      params->transout_low_wind_pitch_cmd, params->transout_high_wind_pitch_cmd,
      wind_speed, params->transout_pitch_low_wind_speed,
      params->transout_pitch_high_wind_speed);

  // Limit the output attitude command based on flight mode.
  if (flight_status->flight_mode == kFlightModeHoverTransOut &&
      flight_status->last_flight_mode == kFlightModeCrosswindPrepTransOut) {
    // Only saturate the pitch command on the high end because the wing can
    // start out at a large negative pitch in trans-out.
    angles_cmd->x =
        Saturate(angles_cmd->x, params->min_angles.x, params->max_angles.x);
    angles_cmd->y = fmin(transout_pitch_cmd, params->max_angles.y);
    angles_cmd->z =
        Saturate(angles_cmd->z, params->min_angles.z, params->max_angles.z);

    // Crossfade to the desired attitude and body rotation rate over several
    // seconds to prevent large transients in motor torque.
    double t = flight_status->flight_mode_time;
    const Vec3 *start = &params->transout_angles_cmd_crossfade_start_times;
    const Vec3 *end = &params->transout_angles_cmd_crossfade_end_times;
    angles_cmd->x = Crossfade(angles->x, angles_cmd->x, t, start->x, end->x);
    angles_cmd->y = Crossfade(angles->y, angles_cmd->y, t, start->y, end->y);
    angles_cmd->z = Crossfade(angles->z, angles_cmd->z, t, start->z, end->z);

    const Vec3 *duration = &params->transout_pqr_cmd_crossfade_duration;
    pqr_cmd->x = Crossfade(pqr->x, pqr_cmd->x, t, 0.0, duration->x);
    pqr_cmd->y = Crossfade(pqr->y, pqr_cmd->y, t, 0.0, duration->y);
    pqr_cmd->z = Crossfade(pqr->z, pqr_cmd->z, t, 0.0, duration->z);
  } else {
    if (flight_status->flight_mode == kFlightModeHoverPrepTransformGsDown &&
        flight_status->last_flight_mode == kFlightModeHoverTransOut) {
      angles_cmd->y = Crossfade(transout_pitch_cmd, angles_cmd->y,
                                flight_status->flight_mode_time, 0.0, 5.0);
    }
    // Saturate the attitude command in all flight modes except
    // transition-out where the wing may start from an extreme angle.
    SaturateVec3(angles_cmd, &params->min_angles, &params->max_angles,
                 angles_cmd);
  }

  // Update telemetry.
  HoverTelemetry *ht = GetHoverTelemetry();
  ht->wing_pos_g_cmd = *wing_pos_g_cmd;
  ht->wing_vel_g_cmd = *wing_vel_g_cmd;
  ht->wing_pos_b_error = wing_pos_b_error;
  ht->wing_vel_b_error = wing_vel_b_error;
  ht->pqr_cmd = *pqr_cmd;
  ht->angles_ff = angles_ff;
  ht->angles_fb = angles_fb;
  ht->int_angles = state->int_angles;
}

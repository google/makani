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

#include "control/trans_in/trans_in.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/plc_messages.h"
#include "common/c_math/geometry.h"
#include "common/c_math/linalg.h"
#include "common/c_math/linalg_common.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/actuator_util.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/ground_frame.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "control/trans_in/trans_in_attitude.h"
#include "control/trans_in/trans_in_lateral.h"
#include "control/trans_in/trans_in_longitudinal.h"
#include "control/trans_in/trans_in_mode.h"
#include "control/trans_in/trans_in_types.h"

// The longitudinal control strategy for trans-in distinguishes
// between the aerodynamic climb angle and the geometric climb angle
// of the kite.  The geometric climb angle is the angle made by the
// kite's inertial velocity to x-y plane of the g coordinate frame,
// whereas the aerodynamic climb angle is the angle made by the
// relative velocity (i.e. including the wind):
//
// wing_vel_g     wind_g
//       ^<---------.
//      /        _.'.
//     /.      _.'    `
//    /  ` _.'        `
//   / _.'\            '  aero_climb_angle
//  o.'   |            |
//   climb_angle
//
// To relate these quantities without relying on the wind sensor or
// angle-of-attack measurements, we rely on the assumption that the
// kite is flying directly into the wind, and that updrafts are not
// significant.  This implies:
//
// airspeed sin(aero_climb_angle) = kite_speed sin(climb_angle)
//                                = -wing_vel_g_z.
//
static double CalcAeroClimbAngle(double wing_vel_g_z, double airspeed) {
  // TODO: The longitudinal controller needs more explicit
  // error handling logic for the case of a failed airspeed
  // measurement.
  if (airspeed < -wing_vel_g_z) {
    // If the airspeed is smaller than our upward velocity, then we
    // likely have no forward velocity and are in a major updraft, or no
    // velocity and we are in still air.  We rely on the minimum airspeed
    // loop to pitch us forward in this case.
    return PI / 2.0;
  } else if (airspeed <= 1.0 || airspeed < wing_vel_g_z) {
    // This condition requires a positive wing_vel_g_z, so the kite is
    // sinking with low airspeed.
    return 0.0;
  } else {
    return Asin(-wing_vel_g_z / airspeed);
  }
}

static void CalcDcmG2Ti(double ti_origin_azimuth, Mat3 *dcm_g2ti) {
  AngleToDcm(ti_origin_azimuth - VecGToAzimuth(&kVec3X) + PI, 0.0, 0.0,
             kRotationOrderZyx, dcm_g2ti);
}

static void CalcDcmTi2B(double ti_origin_azimuth, const Mat3 *dcm_g2b,
                        Mat3 *dcm_ti2b) {
  Mat3 dcm_g2ti;
  CalcDcmG2Ti(ti_origin_azimuth, &dcm_g2ti);
  Mat3Mult(dcm_g2b, kNoTrans, &dcm_g2ti, kTrans, dcm_ti2b);
}

bool TransInValidateParams(const TransInParams *params) {
  if (0.0 > params->prop_inflow_airspeed_bias) {
    assert(!(bool)"prop_inflow_airspeed_bias must be non-negative.");
    return false;
  }

  if (0.0 >= params->prop_inflow_low_airspeed) {
    assert(!(bool)"prop_inflow_low_airspeed must be positive.");
    return false;
  }

  if (params->prop_inflow_low_airspeed >= params->prop_inflow_high_airspeed) {
    assert(!(
        bool)"prop_inflow_high_airspeed must be greater than "
             "prop_inflow_low_airspeed.");
    return false;
  }

  if (params->turn_start_pos_ti_x > 0.0 ||
      params->turn_start_pos_ti_x < -g_sys.tether->length) {
    assert(!(bool)"turn_start_pos_ti_x out of range.");
    return false;
  }

  if (params->turn_radius > 10.0 * g_sys.wing->b ||
      params->turn_radius < 3.0 * g_sys.wing->b) {
    assert(!(bool)"turn_radius out of range.");
    return false;
  }

  if (params->turn_course_angle < -PI / 4.0 ||
      params->turn_course_angle > 0.0) {
    assert(!(bool)"turn_course_angle out of range.");
    return false;
  }

  if (fabs(params->lateral.CL_max - params->longitudinal.CL_0 -
           params->longitudinal.dCL_dalpha *
               params->longitudinal.max_angle_of_attack_cmd -
           params->longitudinal.dCL_dflap *
               params->longitudinal.max_delta_flap_cmd) > 1e-3) {
    assert(!(
        bool)"CL_max does not match between lateral and longitudinal control.");
    return false;
  }

  if (fabs(params->lateral.max_aero_climb_angle -
           params->longitudinal.max_aero_climb_angle_cmd) > 0.0) {
    assert(!(
        bool)"max_aero_climb_angle does not match between lateral and "
             "longitudinal control.");
    return false;
  }

  return TransInAttitudeValidateParams(&params->attitude) &&
         TransInLateralValidateParams(&params->lateral) &&
         TransInLongitudinalValidateParams(&params->longitudinal);
}

void TransInInit(const StateEstimate *state_est, const Vec3 *motor_moment_z1,
                 const TransInParams *params, TransInState *state) {
  assert(state_est != NULL && params != NULL && state != NULL);
  assert(TransInValidateParams(params));

  memset(state, 0, sizeof(*state));
  state->ti_origin_azimuth = VecGToAzimuth(&state_est->Xg);

  Mat3 dcm_ti2b;
  CalcDcmTi2B(state->ti_origin_azimuth, &state_est->dcm_g2b, &dcm_ti2b);

  double aero_climb_angle =
      CalcAeroClimbAngle(state_est->Vg_f.z, state_est->apparent_wind.sph_f.v);
  TransInAttitudeInit(aero_climb_angle, &dcm_ti2b, motor_moment_z1,
                      &params->attitude, &state->attitude);
}

bool TransInIsReadyForMode(FlightMode proposed_flight_mode,
                           const FlightStatus *flight_status,
                           const StateEstimate *state_est,
                           const TransInParams *params,
                           const TransInState *state) {
  (void)state;
  return TransInModeIsReadyFor(proposed_flight_mode, flight_status, state_est,
                               &params->mode);
}

// Transform position, velocity, and attitude into the trans-in frame.
//
// The trans-in frame is rotated about the ground frame z-axis
// based on the kite's initial position on the first entry to
// trans-in:
//
// Kite's Initial Position
//    |
//   o|o            g-frame origin = ti-frame-origin
//   o|o
//    |             x------> x_ti
//   o|o            |
//   o|o            |
//    |             v y_ti
//
//
//  Kite's Initial Position
//  _    _
//  |  . |
//    ||
//     \            g-frame origin = ti-frame origin
//
//   ---|           o-------> x_ti
//                  |
//                  |
//                  v z_ti
//
// The kite's position and velocity and represented in cylindrical coordinates
// as well as Cartesian coordinates.
static void CalcTransInCoordinates(
    const Mat3 *dcm_g2b, const Vec3 *wing_pos_g, const Vec3 *wing_vel_g,
    double ti_origin_azimuth, Vec3 *wing_pos_ti, Vec3 *wing_vel_ti,
    Mat3 *dcm_ti2b, double *radial_pos_ti, double *elevation_angle_ti,
    double *radial_vel_ti, double *tangential_vel_ti) {
  Mat3 dcm_g2ti;
  CalcDcmG2Ti(ti_origin_azimuth, &dcm_g2ti);

  // Find the typical Euler angles but referenced to the ti frame.
  Vec3 eulers_ti2b;
  Mat3Mult(dcm_g2b, kNoTrans, &dcm_g2ti, kTrans, dcm_ti2b);
  DcmToAngle(dcm_ti2b, kRotationOrderZyx, &eulers_ti2b.z, &eulers_ti2b.y,
             &eulers_ti2b.x);

  // Find inertial velocities in ti frame.
  Mat3Vec3Mult(&dcm_g2ti, wing_vel_g, wing_vel_ti);

  // Find position in the ti frame.
  Mat3Vec3Mult(&dcm_g2ti, wing_pos_g, wing_pos_ti);

  // Calculate the elevation angle.
  *radial_pos_ti = Vec3XzNorm(wing_pos_ti);
  *elevation_angle_ti = atan2(-wing_pos_ti->z, -wing_pos_ti->x);

  *radial_vel_ti =
      (wing_pos_ti->x * wing_vel_ti->x + wing_pos_ti->z * wing_vel_ti->z) /
      fmax(10.0, *radial_pos_ti);
  *tangential_vel_ti =
      (wing_pos_ti->x * wing_vel_ti->z - wing_pos_ti->z * wing_vel_ti->x) /
      fmax(10.0, *radial_pos_ti);

  // Update telemetry.
  TransInTelemetry *tt = GetTransInTelemetry();
  tt->ti_origin_azimuth = ti_origin_azimuth;
  tt->eulers_ti2b = eulers_ti2b;
}

static void CalcTransInOutput(const ThrustMoment *thrust_moment,
                              const Deltas *deltas, double airspeed,
                              const StateEstimate *state_est,
                              const TransInOutputParams *params,
                              ControlOutput *control_output) {
  memset(control_output, 0, sizeof(*control_output));

  control_output->gs_mode_request = kGroundStationModeHighTension;

  // Hold the GS targeting data to keep the azimuth brake engaged until we
  // reach crosswind.
  //
  // TODO(b/70903014): Review this position-holding strategy.
  control_output->gs_azi_cmd.target = 0.0;
  control_output->gs_azi_cmd.dead_zone = 0.0;
  control_output->hold_gs_azi_cmd = true;

  Deltas deltas_available;
  MixFlaps(deltas, params->flap_offsets, params->lower_flap_limits,
           params->upper_flap_limits, control_output->flaps, &deltas_available);

  ThrustMoment thrust_moment_avail;
  double v_app_locals[kNumMotors];
  double rotors[kNumMotors];
  MixRotors(thrust_moment, &params->thrust_moment_weights, airspeed,
            &state_est->pqr_f, (StackingState)state_est->stacking_state, false,
            g_sys.phys->rho, g_sys.rotors, g_cont.rotor_control, rotors,
            &thrust_moment_avail, v_app_locals);

  // Deal speed command into motor representation.
  // TODO: Make common function for generating these across modes.
  for (int i = 0; i < kNumMotors; ++i) {
    control_output->motor_speed_upper_limit[i] = rotors[i];
    control_output->motor_speed_lower_limit[i] = rotors[i];
    control_output->motor_torque[i] = 0.0;
  }

  // Set the winch velocity command.
  control_output->winch_vel_cmd = 0.0;

  // Set the detwist servo command.
  control_output->detwist_cmd =
      state_est->tether_ground_angles.departure_detwist_angle;

  // Determine tether release from the joystick.
  control_output->tether_release =
      state_est->joystick.valid && state_est->joystick.data.release;

  // Don't stop the motors.
  control_output->stop_motors = false;

  // These values are set by the outer control system.
  control_output->run_motors = false;
  control_output->sync.sequence = 0U;
  control_output->sync.flight_mode = -1;

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->deltas = *deltas;
  ct->deltas_avail = deltas_available;
  ct->thrust_moment = *thrust_moment;
  ct->thrust_moment_avail = thrust_moment_avail;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    ct->v_app_locals[i] = v_app_locals[i];
  }
}

// Calculate lateral position and velocity commands based on forward progress.
//
//     '
//      '  <--- straight line path with heading params->turn_course_angle.
//       '
//         .
//          . <---  turn with radius params->turn_radius.
//          .
//          | <---  params->turn_start_pos_ti_x.
//          |
//          |
//
static void CalcLateralPositionCommand(double wing_pos_ti_x,
                                       double wing_vel_ti_x,
                                       const TransInParams *params,
                                       double *wing_pos_ti_y_cmd,
                                       double *wing_vel_ti_y_cmd) {
  double turn_direction = Sign(params->turn_course_angle);

  double turn_end_pos_ti_x =
      params->turn_start_pos_ti_x +
      turn_direction * params->turn_radius * sin(params->turn_course_angle);

  double turn_end_pos_ti_y = turn_direction * params->turn_radius *
                             (1.0 - cos(params->turn_course_angle));

  if (wing_pos_ti_x > turn_end_pos_ti_x) {
    *wing_pos_ti_y_cmd =
        turn_end_pos_ti_y +
        tan(params->turn_course_angle) * (wing_pos_ti_x - turn_end_pos_ti_x);
    *wing_vel_ti_y_cmd = tan(params->turn_course_angle) * wing_vel_ti_x;

  } else {
    double angle = Asin(fmax(0.0, wing_pos_ti_x - params->turn_start_pos_ti_x) /
                        params->turn_radius);
    *wing_pos_ti_y_cmd =
        turn_direction * params->turn_radius * (1.0 - cos(angle));
    *wing_vel_ti_y_cmd = turn_direction * tan(angle) * wing_vel_ti_x;
  }
}

// During "transition-in" the tether is slack and the vehicle is in
// airplane-like flight.  MIMO state-feedback is used to control the
// vehicle to a straight-line trajectory along a chord in the tether
// sphere until tension returns and the crosswind controller takes
// over.
void TransInStep(const FlightStatus *flight_status,
                 const StateEstimate *state_est, const TransInParams *params,
                 TransInState *state, ControlOutput *control_output) {
  double radial_pos_ti, elevation_angle_ti;
  double radial_vel_ti, tangential_vel_ti;
  Vec3 wing_pos_ti, wing_vel_ti;
  Mat3 dcm_ti2b;
  CalcTransInCoordinates(&state_est->dcm_g2b, &state_est->Xg, &state_est->Vg,
                         state->ti_origin_azimuth, &wing_pos_ti, &wing_vel_ti,
                         &dcm_ti2b, &radial_pos_ti, &elevation_angle_ti,
                         &radial_vel_ti, &tangential_vel_ti);

  // Alter the airspeed measurement to account for propeller inflow.
  //
  // Due to the Pitot tube location and the combination of low
  // airspeed and high thrust during trans-in, there is a significant
  // bias on the airspeed measured by the Pitot.  Here we partially
  // subtract this bias to reduce its effect on:
  //   - The rotor speed commands from the controller,
  //   - The pitch trim command sent to the attitude loop,
  //   - The dynamic pressure measurement used in the longitudinal and lateral
  //     loop.
  double airspeed = state_est->apparent_wind.sph_f.v;
  airspeed -= Crossfade(params->prop_inflow_airspeed_bias, 0.0, airspeed,
                        params->prop_inflow_low_airspeed,
                        params->prop_inflow_high_airspeed);
  double aero_climb_angle = CalcAeroClimbAngle(state_est->Vg_f.z, airspeed);

  double angle_of_attack_cmd;
  Vec3 pqr_cmd;
  double thrust_cmd, delta_flap_cmd;
  TransInLongitudinalStep(radial_pos_ti, elevation_angle_ti, radial_vel_ti,
                          tangential_vel_ti, airspeed, aero_climb_angle,
                          &state_est->tether_force_b, &params->longitudinal,
                          &angle_of_attack_cmd, &pqr_cmd.y, &thrust_cmd,
                          &delta_flap_cmd);

  double wing_pos_ti_y_cmd, wing_vel_ti_y_cmd;
  CalcLateralPositionCommand(wing_pos_ti.x, wing_vel_ti.x, params,
                             &wing_pos_ti_y_cmd, &wing_vel_ti_y_cmd);

  double angle_of_sideslip_cmd;
  double roll_ti_cmd, yaw_ti_cmd;
  TransInLateralStep(wing_pos_ti_y_cmd, wing_pos_ti.y, wing_vel_ti_y_cmd,
                     wing_vel_ti.y, airspeed, aero_climb_angle,
                     &params->lateral, &angle_of_sideslip_cmd, &roll_ti_cmd,
                     &yaw_ti_cmd, &pqr_cmd.x, &pqr_cmd.z);

  ThrustMoment thrust_moment = {0.0, kVec3Zero};
  Deltas deltas = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // The aero_climb_angle is computed with respect to g-coordinate frame
  // x-y plane, whereas the pitch loop controls pitch in the trans-in
  // frame, so we subtract the angle of ascent here.
  TransInAttitudeStep(
      flight_status->flight_mode_time, aero_climb_angle, angle_of_attack_cmd,
      state_est->apparent_wind.sph_f.alpha, angle_of_sideslip_cmd,
      state_est->apparent_wind.sph_f.beta, roll_ti_cmd, yaw_ti_cmd, &dcm_ti2b,
      &pqr_cmd, &state_est->pqr_f, thrust_cmd, delta_flap_cmd, airspeed,
      &state_est->tether_force_b, &params->attitude, &state->attitude,
      &thrust_moment, &deltas);

  CalcTransInOutput(&thrust_moment, &deltas, airspeed, state_est,
                    &params->output, control_output);

  // Update telemetry.
  TransInTelemetry *tt = GetTransInTelemetry();
  tt->wing_pos_ti = wing_pos_ti;
  tt->wing_vel_ti = wing_vel_ti;
  tt->wing_pos_ti_y_cmd = wing_pos_ti_y_cmd;
  tt->wing_vel_ti_y_cmd = wing_vel_ti_y_cmd;
  tt->aero_climb_angle = aero_climb_angle;
  tt->pqr_cmd = pqr_cmd;
}

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

#include "control/trans_in/trans_in_longitudinal.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/geometry.h"
#include "common/c_math/linalg.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/actuator_util.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/estimator/estimator_types.h"
#include "control/system_params.h"
#include "control/trans_in/trans_in_types.h"

bool TransInLongitudinalValidateParams(
    const TransInLongitudinalParams *params) {
  assert(params != NULL);

  if (params->min_aero_climb_angle_cmd < PI * -15.0 / 180.0 ||
      params->min_aero_climb_angle_cmd > PI * 0.0 / 180.0) {
    assert(!(bool)"min_aero_climb_angle_cmd out of range.");
    return false;
  }

  if (params->max_aero_climb_angle_cmd < PI * 50.0 / 180.0 ||
      params->max_aero_climb_angle_cmd > PI * 65.0 / 180.0) {
    assert(!(bool)"max_aero_climb_angle_cmd out of range.");
    return false;
  }

  if (params->min_airspeed < 0.0 ||
      g_sys.phys->g * g_sys.wing->m > params->CL_0 * g_sys.wing->A *
                                          params->min_airspeed *
                                          params->min_airspeed) {
    assert(!(bool)"min_airspeed is too low.");
    return false;
  }

  if (fabs(params->thrust_pitch) > 0.1) {
    assert(!(bool)"thrust_pitch is out of range.");
    return false;
  }

  if (params->radial_tracking_freq_hz < 0.05 ||
      params->radial_tracking_freq_hz > 0.15) {
    assert(!(bool)"radial_tracking_freq_hz out of range.");
    return false;
  }

  if (params->radial_tracking_damping_ratio < 0.5 ||
      params->radial_tracking_damping_ratio > 2.0) {
    assert(!(bool)"radial_tracking_damping_ratio out of range.");
    return false;
  }

  if (params->tension_control_radial_error_threshold < g_sys.wing->bridle_rad ||
      params->tension_control_radial_error_threshold > 30.0) {
    assert(!(bool)"tension_control_radial_error_threshold out of range");
    return false;
  }

  if (params->tension_control_elevation_angle_threshold < 0.0 ||
      params->tension_control_elevation_angle_threshold > 0.7) {
    assert(!(bool)"tension_control_elevation_angle_threshold out of range");
    return false;
  }

  if (params->min_tension_cmd < 0.0) {
    assert(!(bool)"min_tension_cmd must be non-negative.");
    return false;
  }

  if (params->CL_0 > 2.3 || params->CL_0 < 1.4) {
    assert(!(bool)"CL_0 is out of range.");
    return false;
  }

  if (params->dCL_dalpha > 8.0 || params->dCL_dalpha < 5.0) {
    assert(!(bool)"dCL_dalpha is out of range.");
    return false;
  }

  if (params->dCL_dflap > 2.0 || params->dCL_dflap < 0.0) {
    assert(!(bool)"dCL_dflap is out of range.");
    return false;
  }

  if (params->min_angle_of_attack_cmd < -0.15 ||
      params->min_angle_of_attack_cmd > -0.05) {
    assert(!(bool)"min_angle_of_attack_cmd out of range.");
    return false;
  }

  if (params->max_angle_of_attack_cmd < 0.0 ||
      params->max_angle_of_attack_cmd > 0.1) {
    assert(!(bool)"max_angle_of_attack_cmd out of range.");
    return false;
  }

  if (params->min_delta_flap_cmd < -0.3 || params->min_delta_flap_cmd > 0.0) {
    assert(!(bool)"min_delta_flap_cmd out of range.");
    return false;
  }

  if (params->max_delta_flap_cmd < 0.0 || params->max_delta_flap_cmd > 0.3) {
    assert(!(bool)"max_delta_flap_cmd out of range.");
    return false;
  }

  if (params->max_pitch_rate_b_cmd < 0.0) {
    assert(!(bool)"max_pitch_rate_b_cmd must be non-negative.");
    return false;
  }

  if (params->thrust_cmd < g_sys.wing->m * g_sys.phys->g) {
    assert(!(bool)"thrust_cmd too small.");
    return false;
  }

  return true;
}

// Compute the aero_climb_angle command for a given radial velocity error.
//
// The radial velocity obeys:
//
//   radial_vel_ti =
//       airspeed * sin(aero_climb_angle - tangent_climb_angle) + wind terms.
//
// We want to select an aero_climb_angle_cmd corresponding to a given
// radial_vel_ti_cmd, i.e.:
//
//   radial_vel_ti_cmd =
//       airspeed * sin(aero_climb_angle_cmd - tangent_climb_angle)
//       + wind_terms.
//
// This yields:
//   aero_climb_angle_cmd =
//       tangent_climb_angle +
//       Asin((radial_vel_ti_cmd - radial_vel_ti) / airspeed
//            + sin(aero_climb_angle - tangent_climb_angle))
//
// This function computes this command with saturations.
static double CalcAeroClimbAngleCmd(double tangent_climb_angle,
                                    double radial_vel_ti_err, double airspeed,
                                    double aero_climb_angle,
                                    const TransInLongitudinalParams *params) {
  double asin_argument =
      radial_vel_ti_err / fmax(airspeed, params->min_airspeed) +
      sin(aero_climb_angle - tangent_climb_angle);
  if (asin_argument >
      sin(params->max_aero_climb_angle_cmd - tangent_climb_angle)) {
    return params->max_aero_climb_angle_cmd;
  } else if (asin_argument <
             sin(params->min_aero_climb_angle_cmd - tangent_climb_angle)) {
    return params->min_aero_climb_angle_cmd;
  } else {
    return tangent_climb_angle + Asin(asin_argument);
  }
}

// Control position and tension.
//
// Command a change in aerodynamic climb angle to achieve flight
// tangent to the tether sphere.
static void ControlPosition(double radial_pos_ti, double elevation_angle_ti,
                            double radial_vel_ti, double tangential_vel_ti,
                            double airspeed, double aero_climb_angle,
                            const TetherForceEstimate *tether_force_b,
                            const TransInLongitudinalParams *params,
                            double *normal_accel_cmd, double *tension_cmd) {
  // Our goal is to regulate radial_pos_ti to tether_length and
  // climb_angle to the tangent climb angle of PI/2 -
  // elevation_angle_ti.  We define:
  //
  //   delta     = tether_length - radial_pos_ti,
  //   epsilon   = PI/2 - elevation_angle_ti - climb_angle,
  //   epsilon_a = PI/2 - elevation_angle_ti - aero_climb_angle.
  //
  // The radial position and elevation of the kite satisfy the kinematic
  // equations:
  //
  //   d/dt radial_pos_ti = -V * sin(epsilon),
  //   d/dt elevation_angle_ti = V / radial_pos_ti * cos(epsilon),
  //
  // where V is the kite speed.  The second derivative of radial
  // position is:
  //
  //   (d/dt)^2 radial_pos_ti =
  //       -d/dt V sin(epsilon)
  //       + V cos(epsilon) ((V / radial_pos_ti) * cos(epsilon)
  //                         + d/dt climb_angle).
  //
  // The velocity loop controls the normal acceleration (i.e.
  // airspeed * d/dt aero_climb_angle).  We define:
  //
  //   eta = climb_angle - aero_climb_angle,
  //
  // If we assume that airspeed is constant, then:
  //
  //   d/dt V ~ sin(eta) * airspeed * d/dt aero_climb_angle,
  //   V d/dt climb_angle ~ cos(eta) * airspeed * d/dt aero_climb_angle,
  //
  // and:
  //
  //   (d/dt)^2 radial_pos_ti ~
  //       cos(epsilon_a) * airspeed * d/dt aero_climb_angle +
  //       V^2 / r * cos(epsilon)^2.
  //
  // Linearizing about delta being small we get:
  //
  //   (d/dt)^2 delta ~
  //       -cos(epsilon_a) * airspeed * d/dt aero_climb_angle +
  //       -V^2 / R (1 - delta / R) * cos(epsilon)^2.
  //
  // We neglect the delta / R term and attempt pole placement with a prescribed
  // natural frequency and damping ratio.

  // TODO: This does not account for the GSG location.
  double radial_pos_ti_err =
      g_sys.tether->length + g_sys.wing->bridle_rad - radial_pos_ti;
  double radial_vel_ti_err = (PI * params->radial_tracking_freq_hz /
                              params->radial_tracking_damping_ratio) *
                                 radial_pos_ti_err -
                             radial_vel_ti;

  double tangent_climb_angle = PI / 2.0 - elevation_angle_ti;
  double aero_climb_angle_cmd =
      CalcAeroClimbAngleCmd(tangent_climb_angle, radial_vel_ti_err, airspeed,
                            aero_climb_angle, params);
  double aero_climb_angle_err = aero_climb_angle_cmd - aero_climb_angle;
  double normal_accel_fb = 4.0 * PI * params->radial_tracking_damping_ratio *
                           params->radial_tracking_freq_hz * airspeed *
                           aero_climb_angle_err;

  double normal_accel_ff =
      -tangential_vel_ti * tangential_vel_ti / g_sys.tether->length /
      fmax(0.3, cos(tangent_climb_angle - aero_climb_angle));
  *normal_accel_cmd = normal_accel_ff + normal_accel_fb;

  // The tension command is only applied when the radial error is sufficiently
  // small.
  double CL_max = params->CL_0 +
                  params->max_angle_of_attack_cmd * params->dCL_dalpha +
                  params->max_delta_flap_cmd * params->dCL_dflap;
  *tension_cmd =
      params->min_tension_cmd +
      fmax(0.0, 0.5 * g_sys.phys->rho * (airspeed + params->min_airspeed) *
                    (airspeed - params->min_airspeed) * g_sys.wing->A * CL_max);
  *tension_cmd =
      Crossfade(*tension_cmd, tether_force_b->vector_f.z, radial_pos_ti_err,
                0.5 * params->tension_control_radial_error_threshold,
                params->tension_control_radial_error_threshold);

  // This second cross-fade was added to avoid large tension being commanded
  // near the beginning of trans-in during high wind speeds.
  *tension_cmd =
      Crossfade(tether_force_b->vector_f.z, *tension_cmd, elevation_angle_ti,
                0.7 * params->tension_control_elevation_angle_threshold,
                params->tension_control_elevation_angle_threshold);

  // Update telemetry.
  TransInTelemetry *tt = GetTransInTelemetry();
  tt->tension_cmd = *tension_cmd;
  tt->aero_climb_angle_cmd = aero_climb_angle_cmd;
  tt->radial_vel_ti_cmd = radial_vel_ti_err + radial_vel_ti;
  tt->radial_vel_ti = radial_vel_ti;
}

static void MixNormalForceCommand(double dnormal_force,
                                  double dnormal_force_dflap,
                                  double dnormal_force_dalpha,
                                  const TransInLongitudinalParams *params,
                                  double *delta_flap_cmd,
                                  double *angle_of_attack_cmd) {
  double dnormal_force_max =
      dnormal_force_dalpha * params->max_angle_of_attack_cmd +
      dnormal_force_dflap * params->max_delta_flap_cmd;
  double dnormal_force_min =
      dnormal_force_dalpha * params->min_angle_of_attack_cmd +
      dnormal_force_dflap * params->min_delta_flap_cmd;

  *delta_flap_cmd =
      Crossfade(params->min_delta_flap_cmd, params->max_delta_flap_cmd,
                dnormal_force, dnormal_force_min, dnormal_force_max);

  *angle_of_attack_cmd = Crossfade(
      params->min_angle_of_attack_cmd, params->max_angle_of_attack_cmd,
      dnormal_force, dnormal_force_min, dnormal_force_max);
}

static double CalcPitchRateBCmd(double airspeed, double normal_force_0,
                                double dnormal_force_dflap,
                                double dnormal_force_dalpha,
                                double delta_flap_cmd,
                                double angle_of_attack_cmd,
                                const TransInLongitudinalParams *params) {
  // The feed-forward pitch rate should be equal to the change in climb rate:
  //
  //   m airspeed d/dt climb_angle = normal_force.
  //
  double expected_normal_force = normal_force_0 +
                                 dnormal_force_dalpha * angle_of_attack_cmd +
                                 dnormal_force_dflap * delta_flap_cmd;
  return Saturate(expected_normal_force / (g_sys.wing->m * airspeed),
                  -params->max_pitch_rate_b_cmd, params->max_pitch_rate_b_cmd);
}

// TODO: Several places in this function could benefit
// from including the effective mass resulting from accelerating
// the tether.
static void ControlVelocity(double airspeed, double normal_accel_cmd,
                            double aero_climb_angle, double tension_cmd,
                            const TetherForceEstimate *tether_force_b,
                            const TransInLongitudinalParams *params,
                            double *delta_flap_cmd, double *angle_of_attack_cmd,
                            double *pitch_rate_b_cmd) {
  *delta_flap_cmd = params->min_delta_flap_cmd;
  *angle_of_attack_cmd = params->min_angle_of_attack_cmd;
  *pitch_rate_b_cmd = 0.0;

  // Below this threshold airspeed no angle-of-attack or feed-forward
  // pitch rate command are applied.
  const double threshold_airspeed = 0.707 * params->min_airspeed;
  if (airspeed > threshold_airspeed) {
    // We assume the following simplified model for force balance:
    //
    //   m airspeed d/dt aero_climb_angle
    //     = L - Fz_tether - m g cos(aero_climb_angle)
    //       + thrust * sin(thrust_pitch + angle_of_attack).
    double dynamic_pressure = 0.5 * g_sys.phys->rho * airspeed * airspeed;
    double thrust =
        fmax(0.0, CalcMaxTotalThrust(airspeed, g_cont.rotor_control));
    double normal_force_cmd = g_sys.wing->m * normal_accel_cmd + tension_cmd -
                              tether_force_b->vector_f.z;
    double normal_force_0 =
        dynamic_pressure * g_sys.wing->A * params->CL_0 +
        -tether_force_b->vector_f.z +
        -g_sys.wing->m * g_sys.phys->g * cos(aero_climb_angle) +
        thrust * sin(params->thrust_pitch);
    double dnormal_force_dalpha =
        dynamic_pressure * g_sys.wing->A * params->dCL_dalpha +
        thrust * cos(params->thrust_pitch);
    double dnormal_force_dflap =
        dynamic_pressure * g_sys.wing->A * params->dCL_dflap;

    MixNormalForceCommand(normal_force_cmd - normal_force_0,
                          dnormal_force_dflap, dnormal_force_dalpha, params,
                          delta_flap_cmd, angle_of_attack_cmd);

    *angle_of_attack_cmd =
        Crossfade(params->min_angle_of_attack_cmd, *angle_of_attack_cmd,
                  airspeed, threshold_airspeed, 0.86 * params->min_airspeed);
    *delta_flap_cmd =
        Crossfade(params->min_delta_flap_cmd, *delta_flap_cmd, airspeed,
                  threshold_airspeed, 0.86 * params->min_airspeed);

    *pitch_rate_b_cmd = CalcPitchRateBCmd(
        airspeed, normal_force_0, dnormal_force_dflap, dnormal_force_dalpha,
        *delta_flap_cmd, *angle_of_attack_cmd, params);
    *pitch_rate_b_cmd =
        Crossfade(0.0, *pitch_rate_b_cmd, airspeed, threshold_airspeed,
                  0.86 * params->min_airspeed);
  }

  // Update telemetry.
  TransInTelemetry *tt = GetTransInTelemetry();
  tt->angle_of_attack_cmd = *angle_of_attack_cmd;
}

void TransInLongitudinalStep(double radial_pos_ti, double elevation_angle_ti,
                             double radial_vel_ti, double tangential_vel_ti,
                             double airspeed, double aero_climb_angle,
                             const TetherForceEstimate *tether_force_b,
                             const TransInLongitudinalParams *params,
                             double *angle_of_attack_cmd,
                             double *pitch_rate_b_cmd, double *thrust_cmd,
                             double *delta_flap_cmd) {
  assert(radial_pos_ti >= 0.0 && elevation_angle_ti >= 0.0 && airspeed >= 0.0);
  assert(tether_force_b != NULL && params != NULL);
  assert(angle_of_attack_cmd != NULL && pitch_rate_b_cmd != NULL &&
         thrust_cmd != NULL && delta_flap_cmd != NULL);

  double normal_accel_cmd, tension_cmd;
  ControlPosition(radial_pos_ti, elevation_angle_ti, radial_vel_ti,
                  tangential_vel_ti, airspeed, aero_climb_angle, tether_force_b,
                  params, &normal_accel_cmd, &tension_cmd);

  ControlVelocity(airspeed, normal_accel_cmd, aero_climb_angle, tension_cmd,
                  tether_force_b, params, delta_flap_cmd, angle_of_attack_cmd,
                  pitch_rate_b_cmd);

  *thrust_cmd = params->thrust_cmd;
}

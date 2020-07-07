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

#include "control/trans_in/trans_in_lateral.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>

#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/simple_aero.h"
#include "control/system_params.h"
#include "control/trans_in/trans_in_types.h"

bool TransInLateralValidateParams(const TransInLateralParams *params) {
  assert(params != NULL);

  if (params->CL_max > 2.3 || params->CL_max < 1.4) {
    assert(!(bool)"CL_max is out of range.");
    return false;
  }

  if (params->max_aero_climb_angle < PI / 4.0 ||
      params->max_aero_climb_angle > 70.0 * PI / 180.0) {
    assert(!(bool)"max_aero_climb_angle is out of range.");
    return false;
  }

  if (params->lateral_tracking_ref_length > 400.0 ||
      params->lateral_tracking_ref_length < 50.0) {
    assert(!(bool)"lateral_tracking_ref_length is out of range.");
    return false;
  }

  if (params->max_lateral_tracking_freq_hz > 0.1 ||
      params->max_lateral_tracking_freq_hz < 0.01) {
    assert(!(bool)"max_lateral_tracking_freq_hz is out of range.");
    return false;
  }

  if (params->lateral_tracking_damping_ratio < 0.5 ||
      params->lateral_tracking_damping_ratio > 1.0) {
    assert(!(bool)"lateral_tracking_damping_ratio is out of range.");
    return false;
  }

  if (params->max_pos_ti_y_err < 0.0 || params->max_pos_ti_y_err > 200.0) {
    assert(!(bool)"max_pos_ti_y_err is out of range.");
    return false;
  }

  if (fabs(params->angle_of_sideslip_cmd) > 0.1) {
    assert(!(bool)"angle_of_sideslip_cmd is out of range.");
    return false;
  }

  if (fabs(params->roll_ti_cmd) > 0.15) {
    assert(!(bool)"roll_ti_cmd is out of range.");
    return false;
  }

  if (fabs(params->yaw_ti_cmd) > 0.15) {
    assert(!(bool)"yaw_ti_cmd is out of range.");
    return false;
  }

  if (params->max_delta_roll_ti_cmd < 0.1 ||
      params->max_delta_roll_ti_cmd > 0.35) {
    assert(!(bool)"max_delta_roll_ti_cmd is out of range.");
    return false;
  }

  if (params->max_yaw_rate_ti_cmd < 0.0 || params->max_yaw_rate_ti_cmd > 0.3) {
    assert(!(bool)"max_yaw_rate_ti_cmd out of range.");
    return false;
  }

  return true;
}

// The objective of the lateral control is to regulate the kite to fly
// in the plane defined by the wing_pos_ti_y = 0.
//
// The attitude of the kite is given by its roll (phi), pitch (theta),
// and yaw (psi) angles:
//
//   R_ti^b = R_x(phi) R_y(theta) R_z(psi).
//
// The wind coordinate system is defined relative to the body using
// the angle-of-attack (alpha) and angle-of-sideslip (beta):
//
//   R_w^b = R_y(alpha) R_z(beta).
//
// Finally, the rotation from wind to body coordinates can be written
// in terms of its aerodynamic roll (mu), climb angle (gamma), and
// course angle (chi):
//
//   R_ti^w = R_x(mu) R_y(gamma) R_z(chi).
//
// The kinematics of the kite can be written in terms of the airspeed
// (V) and the lateral wind speed (v_wind):
//
//   d/dt y = V cos(gamma) sin(chi) + v_wind,
//
//   [d/dt V ; V cos(gamma) d/dt chi; -V d/dt gamma] =
//       R_x(-mu) wing_accel_w
//
// where:
//
//   m wing_accel^w = [-D; Y; -L]
//                    + R_z(-beta) R_y(-alpha-theta_motor) * [T;0;0]
//                    + R_ti^w ([0; 0; mg] + F_tether^ti),
//
// We approximate the tether as additional weight:
//
//   [0; 0; mg] + F_tether^ti ~ [0; 0; W].
//
// We assume that d/dt V ~ 0 and d/dt gamma ~ 0:
//
//   0 = (-L - T sin(alpha + theta_motor)) * cos(mu)
//       + (Y - T cos(alpha + theta_motor) * sin(beta)) * sin(mu)
//       - W cos(gamma),
//
//   (d/dt)^2 y ~ V cos(gamma) cos(chi) d/dt chi.
//
// The kinematics give:
//
//   m V cos(gamma) d/dt chi
//       = (L + T sin(alpha + theta_motor)) * sin(mu)
//         + (Y - T cos(alpha + theta_motor) * sin(beta)) * cos(mu).
//
// We approximate Y ~ 0, and alpha, theta_motor, mu, beta, and chi
// being small:
//
//   m (d/dt)^2 y ~ W cos(gamma) * tan(mu) - T * sin(beta).
//
// Neglecting beta all together, we find that phi ~ mu, so that:
//
//   m (d/dt)^2 y ~ W cos(gamma) * tan(phi).
//
void TransInLateralStep(double wing_pos_ti_y_cmd, double wing_pos_ti_y,
                        double wing_vel_ti_y_cmd, double wing_vel_ti_y,
                        double airspeed, double aero_climb_angle,
                        const TransInLateralParams *params,
                        double *angle_of_sideslip_cmd, double *roll_ti_cmd,
                        double *yaw_ti_cmd, double *roll_rate_b_cmd,
                        double *yaw_rate_b_cmd) {
  assert(params != NULL && roll_ti_cmd != NULL && yaw_ti_cmd != NULL &&
         roll_rate_b_cmd != NULL && yaw_rate_b_cmd != NULL);

  double dynamic_pressure = 0.5 * g_sys.phys->rho * airspeed * airspeed;

  // Compute expected lift:
  //
  //   L ~ W cos(aero_climb_angle),
  //
  // where W is the combined weight of the kite + tether.
  double weight =
      (g_sys.wing->m + g_sys.tether->length * g_sys.tether->linear_density) *
      g_sys.phys->g;
  double expected_lift =
      weight * cos(fmin(params->max_aero_climb_angle, aero_climb_angle));
  double maximum_lift = dynamic_pressure * g_sys.wing->A * params->CL_max;

  // If the dynamic pressure is too low, the commanded roll angle is
  // held at zero to avoid large bank commands before there is
  // sufficient airspeed.
  double delta_roll_ti_cmd = 0.0;
  double delta_yaw_ti_cmd = 0.0;
  double yaw_rate_ti_cmd = 0.0;
  if (maximum_lift > 0.9 * expected_lift) {
    // Compute an acceleration command to achieve a desired second
    // order response.
    double omega = fmin(airspeed / params->lateral_tracking_ref_length,
                        2.0 * PI * params->max_lateral_tracking_freq_hz);
    double accel_ti_y_per_pos_ti_y = omega * omega;
    double accel_ti_y_per_vel_ti_y =
        2.0 * omega * params->lateral_tracking_damping_ratio;
    double wing_accel_ti_y_cmd =
        accel_ti_y_per_pos_ti_y * Saturate(wing_pos_ti_y_cmd - wing_pos_ti_y,
                                           -params->max_pos_ti_y_err,
                                           params->max_pos_ti_y_err) +
        accel_ti_y_per_vel_ti_y * (wing_vel_ti_y_cmd - wing_vel_ti_y);

    // This cross-fade avoids a discontinuous command.
    //
    // TODO: The effective mass due to accelerating the
    // tether should be included here.
    double force_cmd =
        Crossfade(0.0, g_sys.wing->m * wing_accel_ti_y_cmd, maximum_lift,
                  0.9 * expected_lift, expected_lift);
    double max_force = params->max_delta_roll_ti_cmd * expected_lift;
    delta_roll_ti_cmd =
        Crossfade(-params->max_delta_roll_ti_cmd, params->max_delta_roll_ti_cmd,
                  force_cmd, -max_force, max_force);

    delta_yaw_ti_cmd =
        Asin(wing_vel_ti_y_cmd /
             (airspeed *
              cos(Saturate(aero_climb_angle, -params->max_aero_climb_angle,
                           params->max_aero_climb_angle))));

    delta_yaw_ti_cmd = Crossfade(0.0, delta_yaw_ti_cmd, maximum_lift,
                                 0.9 * expected_lift, expected_lift);

    // Compute a feed-forward yaw rate for turn coordination,
    // approximating aerodynamic climb angle as pitch angle.  See:
    //
    //   Stevens and Lewis, Aircraft Control and Simulation. 2nd Ed.
    //   Equation 3.6-7, pg. 190.
    yaw_rate_ti_cmd = (g_sys.phys->g * cos(aero_climb_angle) / airspeed) *
                      tan(delta_roll_ti_cmd);
  }

  *angle_of_sideslip_cmd = params->angle_of_sideslip_cmd;
  *roll_ti_cmd = params->roll_ti_cmd + delta_roll_ti_cmd;
  *yaw_ti_cmd = params->yaw_ti_cmd + delta_yaw_ti_cmd;

  yaw_rate_ti_cmd = Saturate(yaw_rate_ti_cmd, -params->max_yaw_rate_ti_cmd,
                             params->max_yaw_rate_ti_cmd);

  // We assume zero roll and pitch rates to compute the body rates
  // corresponding to the feed-forward yaw rate.  We also neglect the
  // effect of our roll angle. See:
  //
  //   Stevens and Lewis, Aircraft Control and Simulation. 2nd Ed.
  //   Equation 1.3-21, pg. 26.
  *roll_rate_b_cmd = -sin(aero_climb_angle) * yaw_rate_ti_cmd;
  *yaw_rate_b_cmd = cos(aero_climb_angle) * yaw_rate_ti_cmd;
}

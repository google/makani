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

#include "control/crosswind/crosswind_util.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>

#include "common/c_math/geometry.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_frame.h"
#include "control/ground_frame.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/system_types.h"

double CalcCurvature(const Vec3 *acc_b, const Vec3 *vel_b) {
  assert(acc_b != NULL && vel_b != NULL);

  Vec3 acc_cross_vel_b;
  Vec3Cross(acc_b, vel_b, &acc_cross_vel_b);
  // TODO: Remove arbitrary 1e-5.
  double norm_vel_b = Vec3NormBound(vel_b, 1e-5);
  return Sign(acc_b->y) * Vec3Norm(&acc_cross_vel_b) /
         (norm_vel_b * norm_vel_b * norm_vel_b);
}

double CurvatureToTetherRoll(double aero_curvature, double lift_coeff,
                             double side_force_coeff) {
  assert(lift_coeff > 0.0);

  // Forces on the wing must accelerate both the wing and the tether.
  // The effective mass of the tether as if it were concentrated at
  // the wing is derived by modeling the tether as a rigid rod of
  // uniform density.
  const double effective_mass =
      g_sys.wing->m + g_sys.tether->linear_density * g_sys.tether->length / 3.0;

  double acc_coeff =
      aero_curvature * effective_mass / (0.5 * g_sys.phys->rho * g_sys.wing->A);
  return Saturate(
      (side_force_coeff - acc_coeff) / fmax(lift_coeff, DBL_EPSILON), -PI / 2.0,
      PI / 2.0);
}

double CalcCircularLoopKinematics(const Vec3 *wind_g, const Vec3 *path_center_g,
                                  double loop_angle, double loop_radius,
                                  double airspeed,
                                  double d_airspeed_d_loopangle, Vec3 *vel_g,
                                  Vec3 *acc_g) {
  // Get the loop geometry.
  Vec3 pos_g, vel_g_hat, acc_g_perp_hat;
  CalcCircularLoopGeometry(path_center_g, loop_angle, loop_radius, &pos_g,
                           &vel_g_hat, &acc_g_perp_hat);

  // Call CalcCrosswindKinematics once with airspeed_dot = 0 to get the required
  // kite speed, which is in turn needed to calculate airspeed_dot. We then call
  // CalcCrosswindKinematics again with the true value of airspeed_dot in order
  // to calculate the required acceleration.

  CalcCrosswindKinematics(wind_g, &vel_g_hat, &acc_g_perp_hat, loop_radius,
                          airspeed, 0.0, vel_g, NULL);

  // Calculate the rate of change of airspeed.
  //
  // This is simply the chain rule:
  //
  //     airspeed_dot = (d airspeed / d loop angle) * (d loop angle / d time)
  //
  // where (d loop angle / d time) is -kitespeed / loop_radius, with
  // the minus sign appearing because we fly around the loop in the
  // direction of decreasing loop angle.

  const double kitespeed = Vec3Norm(vel_g);

  const double airspeed_dot =
      d_airspeed_d_loopangle * (-kitespeed / loop_radius);

  double kiteaccel =
      CalcCrosswindKinematics(wind_g, &vel_g_hat, &acc_g_perp_hat, loop_radius,
                              airspeed, airspeed_dot, vel_g, acc_g);

  return kiteaccel;
}

double CalcCrosswindKinematics(const Vec3 *wind_g, const Vec3 *vel_g_hat,
                               const Vec3 *acc_g_perp_hat, double local_radius,
                               double airspeed, double airspeed_dot,
                               Vec3 *vel_g, Vec3 *acc_g) {
  // Verify that these are indeed unit vectors.
  assert(fabs(Vec3Norm(vel_g_hat) - 1.0) < 1e-6);
  assert(fabs(Vec3Norm(acc_g_perp_hat) - 1.0) < 1e-6);

  // Verify that the given acceleration is indeed centripetal.
  assert(fabs(Vec3Dot(vel_g_hat, acc_g_perp_hat)) < 1e-6);

  // Compute parallel and perpendicular components of the wind.
  //
  // It's useful to split up the wind vector into components that are
  // parallel and perpendicular to the kite velocity.  These obey the
  // following scalar equation:
  //
  //     wind^2 = wind_par^2 + wind_perp^2

  const double wind = Vec3Norm(wind_g);
  const double wind_par = Vec3Dot(vel_g_hat, wind_g);
  const double wind_perp = Sqrt(Square(wind) - Square(wind_par));

  // Calculate the kite speed we need to achieve a given airspeed.
  //
  // Given wind_par and wind_perp, this becomes a purely scalar
  // equation:
  //
  //     airspeed^2 = (kitespeed - wind_par)^2 + wind_perp^2

  const double kitespeed =
      wind_par + Sqrt(Square(airspeed) - Square(wind_perp));

  // Calculate the rate of change of wind_par.
  //
  // We assume that the wind vector is constant.

  const double wind_par_dot =
      Vec3Dot(acc_g_perp_hat, wind_g) * kitespeed / local_radius;

  // Finally we can compute the required acceleration to track the
  // changing airspeed command.
  //
  //     airspeed^2 = (kitespeed - wind_par)^2 + wind_perp^2
  //     airspeed^2 = (kitespeed - wind_par)^2 + wind^2 - wind_par^2
  //
  // Now take the derivative
  //
  //    2 airspeed airspeed_dot =
  //                 2 (kitespeed - wind_par) * (kitespeed_dot - wind_par_dot)
  //               + 2 wind * wind_dot - 2 wind_par * wind_par_dot
  //
  // which can be solved for kitespeed_dot = kiteaccel.

  const double kiteaccel =
      (airspeed * airspeed_dot + kitespeed * wind_par_dot) /
      (kitespeed - wind_par);

  if (vel_g != NULL) {
    Vec3Scale(vel_g_hat, kitespeed, vel_g);
  }

  if (acc_g != NULL) {
    Vec3LinComb(kiteaccel, vel_g_hat, Square(kitespeed) / local_radius,
                acc_g_perp_hat, acc_g);
  }

  return kiteaccel;
}

void CalcCircularLoopGeometry(const Vec3 *path_center_g, double loop_angle,
                              double loop_radius, Vec3 *pos_g, Vec3 *vel_hat_g,
                              Vec3 *acc_perp_hat_g) {
  Vec3 wing_pos_cw = {.x = 0.0, .y = cos(loop_angle), .z = sin(loop_angle)};
  Vec3 wing_vel_cw = {.x = 0.0, .y = sin(loop_angle), .z = -cos(loop_angle)};
  Vec3 wing_acc_cw = {.x = 0.0, .y = -cos(loop_angle), .z = -sin(loop_angle)};

  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  Vec3Scale(&wing_pos_cw, loop_radius, &wing_pos_cw);
  Mat3TransVec3Mult(&dcm_g2cw, &wing_pos_cw, pos_g);

  if (pos_g != NULL) {
    Vec3Add(pos_g, path_center_g, pos_g);
  }
  if (vel_hat_g != NULL) {
    Mat3TransVec3Mult(&dcm_g2cw, &wing_vel_cw, vel_hat_g);
  }
  if (acc_perp_hat_g != NULL) {
    Mat3TransVec3Mult(&dcm_g2cw, &wing_acc_cw, acc_perp_hat_g);
  }
}

double CalcLoopAngle(const Vec3 *path_center_g, const Vec3 *wing_pos_g) {
  assert(path_center_g != NULL && wing_pos_g != NULL);

  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  Vec3 wing_pos_cw;
  Vec3 wing_point_vec;
  Vec3Sub(wing_pos_g, path_center_g, &wing_point_vec);
  Mat3Vec3Mult(&dcm_g2cw, &wing_point_vec, &wing_pos_cw);
  return PI - atan2(wing_pos_cw.z, -wing_pos_cw.y);
}

bool BetweenLoopAngles(double start_angle, double end_angle,
                       double current_angle) {
  // The loop angle starts at 9 o'clock and moving counter clock-wise, to
  // the opposite direction of the flight path.
  // Therefore, if the kite does not cross the wraparound point (9 o'clock),
  // it travels from a larger angle (the start angle) to a smaller angle (the
  // end angle).
  if (start_angle >= end_angle) {
    return current_angle >= end_angle && current_angle <= start_angle;
  } else {
    // Handle the wraparound the case where the kite crosses 9 o'clock in the
    // crosswind loop.
    return current_angle >= end_angle || current_angle <= start_angle;
  }
}

void CalcCrosswindAttitude(const Vec3 *wind_g, const Vec3 *path_center_g,
                           double loop_angle, double path_radius,
                           double tether_roll, double alpha_cmd,
                           double beta_cmd, double airspeed_cmd,
                           double d_airspeed_d_loopangle, Mat3 *dcm_g2b) {
  // The "airspeed roll angle" should actually be used here.
  // TODO: Use the airspeed roll angle here.
  Mat3 dcm_f2b;
  CalcDcmFToB(-tether_roll, beta_cmd, alpha_cmd, &dcm_f2b);

  // Get the commanded kite velocity with respect to ground.
  Vec3 wing_vel_g;
  CalcCircularLoopKinematics(wind_g, path_center_g, loop_angle, path_radius,
                             airspeed_cmd, d_airspeed_d_loopangle, &wing_vel_g,
                             NULL);

  // Construct the commanded kite position in g based upon loop_angle.
  // TODO: Construct actual anchor2kite_g command based on
  // some estimate of the anchor position in g instead of based upon
  // the origin of g.
  Vec3 anchor2kite_g;
  CalcCircularLoopGeometry(path_center_g, loop_angle, path_radius,
                           &anchor2kite_g, NULL, NULL);

  // Construct dcm from f to g
  Vec3 apparent_wind_g;
  Vec3Sub(wind_g, &wing_vel_g, &apparent_wind_g);
  Mat3 dcm_f2g;
  CalcDcmFToG(&apparent_wind_g, &anchor2kite_g, &dcm_f2g);

  // Combine g to f and f to b
  Mat3Mult(&dcm_f2b, kNoTrans, &dcm_f2g, kTrans, dcm_g2b);
}

void CalcCrosswindPqr(const Vec3 *wind_g, const Vec3 *path_center_g,
                      double loop_angle, double path_radius, double tether_roll,
                      double tether_roll_dot, double alpha_cmd, double beta_cmd,
                      double airspeed_cmd, double d_airspeed_d_loopangle,
                      double kitespeed, Vec3 *pqr) {
  // Use central difference to compute the differential rotation
  // between (loop_angle-h) and (loop_angle+h).
  const double d_loopangle = 1e-3;  // Radians.
  const double d_loopangle_d_time = -kitespeed / path_radius;
  const double d_roll_d_loopangle = tether_roll_dot / d_loopangle_d_time;

  Mat3 dcm_g2b_m, dcm_g2b_p;
  CalcCrosswindAttitude(
      wind_g, path_center_g, loop_angle - d_loopangle, path_radius,
      tether_roll - d_roll_d_loopangle * d_loopangle, alpha_cmd, beta_cmd,
      airspeed_cmd - d_airspeed_d_loopangle * d_loopangle,
      d_airspeed_d_loopangle, &dcm_g2b_m);

  CalcCrosswindAttitude(
      wind_g, path_center_g, loop_angle + d_loopangle, path_radius,
      tether_roll + d_roll_d_loopangle * d_loopangle, alpha_cmd, beta_cmd,
      airspeed_cmd + d_airspeed_d_loopangle * d_loopangle,
      d_airspeed_d_loopangle, &dcm_g2b_p);

  // Now we have a DCM corresponding to a differential rotation; here
  // we turn it into a pqr vector.  With a differential rotation, the
  // Euler angles act like a vector.
  Mat3 dcm_m2p;
  Mat3Mult(&dcm_g2b_p, kNoTrans, &dcm_g2b_m, kTrans, &dcm_m2p);
  DcmToAngle(&dcm_m2p, kRotationOrderZyx, &pqr->z, &pqr->y, &pqr->x);

  Vec3Scale(pqr, d_loopangle_d_time / (2 * d_loopangle), pqr);
}

// TODO: Combine this with CalcCrosswindPqr to reduce the
// number of evaluations of CalcCrosswindAttitude from four to three.
void CalcCrosswindPqrDot(const Vec3 *wind_g, const Vec3 *path_center_g,
                         double loop_angle, double path_radius,
                         double tether_roll, double tether_roll_dot,
                         double alpha_cmd, double beta_cmd, double airspeed_cmd,
                         double d_airspeed_d_loopangle, double kitespeed,
                         Vec3 *pqr_cmd_dot) {
  // Use central difference to compute the differential rotation
  // between (loop_angle-h) and (loop_angle+h).
  const double d_loopangle = 1e-2;  // Radians.
  const double d_loopangle_d_time = -kitespeed / path_radius;
  const double d_roll_d_loopangle = tether_roll_dot / d_loopangle_d_time;

  Vec3 pqr_m, pqr_p;
  CalcCrosswindPqr(wind_g, path_center_g, loop_angle - d_loopangle, path_radius,
                   tether_roll - d_roll_d_loopangle * d_loopangle,
                   tether_roll_dot, alpha_cmd, beta_cmd,
                   airspeed_cmd - d_airspeed_d_loopangle * d_loopangle,
                   d_airspeed_d_loopangle, kitespeed, &pqr_m);

  CalcCrosswindPqr(wind_g, path_center_g, loop_angle + d_loopangle, path_radius,
                   tether_roll + d_roll_d_loopangle * d_loopangle,
                   tether_roll_dot, alpha_cmd, beta_cmd,
                   airspeed_cmd + d_airspeed_d_loopangle * d_loopangle,
                   d_airspeed_d_loopangle, kitespeed, &pqr_p);

  const double dt = d_loopangle / d_loopangle_d_time;
  Vec3Sub(&pqr_p, &pqr_m, pqr_cmd_dot);
  Vec3Scale(pqr_cmd_dot, 1.0 / (2.0 * dt), pqr_cmd_dot);
}

double HarmonicIntegrator(double u, double angle, double d_angle, double ki,
                          double int_max, double state[]) {
  state[0] += ki * sin(angle) * u * d_angle;
  state[1] += ki * cos(angle) * u * d_angle;

  state[0] = Saturate(state[0], -int_max, int_max);
  state[1] = Saturate(state[1], -int_max, int_max);

  return state[0] * sin(angle) + state[1] * cos(angle);
}

void SlewAzimuthWhenNecessary(const Vec3 *wing_pos_g,
                              const Vec3 *raw_path_center_g,
                              double max_crosswind_y_position_for_slew,
                              Vec3 *path_center_g) {
  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  Vec3 wing_pos_cw, raw_path_center_cw;
  Mat3Vec3Mult(&dcm_g2cw, wing_pos_g, &wing_pos_cw);
  wing_pos_cw.x = 0.0;
  Mat3Vec3Mult(&dcm_g2cw, raw_path_center_g, &raw_path_center_cw);
  raw_path_center_cw.x = 0.0;

  double path_center_azimuth, unused_elevation, unused_radius;
  double unused_azimuth, path_center_elevation, path_center_radius;

  // Get the existing azimuth and radius from the existing path center
  VecGToSph(path_center_g, &path_center_azimuth, &unused_elevation,
            &path_center_radius);
  // Grab the desired elevation from the desired path center location
  VecGToSph(raw_path_center_g, &unused_azimuth, &path_center_elevation,
            &unused_radius);

  // We should slew the crosswind path during the straight line part of the
  // racetrack. Otherwise, leave the azimuth alone.
  if (IsOnTheRacetrackHorizontalPath(&wing_pos_cw, &raw_path_center_cw)) {
    // The slew is only initialized when the kite is near the top or bottom
    // of the loop.
    if (fabs(wing_pos_cw.y) < max_crosswind_y_position_for_slew) {
      // Grab the azimuth from the current position when slewing
      VecGToSph(wing_pos_g, &path_center_azimuth, &unused_elevation,
                &unused_radius);
    }
  }

  // Assign the path center location
  SphToVecG(path_center_azimuth, path_center_elevation, path_center_radius,
            path_center_g);
}

bool IsOnTheRacetrackHorizontalPath(const Vec3 *target_pos_cw,
                                    const Vec3 *raw_path_center_cw) {
  // The straight line part of the racetrack path occurs when the desired
  // path center's cw.y position is "forward of" the target's cw.y position.
  // We assume the kite flies clockwise loops so that its "forward" cw.y
  // direction can be deduced from its cw.y and cw.z position. The racetrack
  // pattern begins just after the kite crosses the 12 or 6 o'clock positions.
  //
  // Check if the kite and the raw path center are on the same side of the
  // crosswind y axis.
  if (raw_path_center_cw->y * target_pos_cw->y > 0.0) {
    // Check if the kite's y position is closer to the origin.
    if (fabs(target_pos_cw->y) < fabs(raw_path_center_cw->y)) {
      // Check if the kite is moving toward the raw path center according
      // to a clockwise circle direction viewed from the groundstation.
      if (raw_path_center_cw->y * target_pos_cw->z > 0.0) {
        return true;
      }
    }
  }
  return false;
}

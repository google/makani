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

#ifndef CONTROL_CROSSWIND_CROSSWIND_UTIL_H_
#define CONTROL_CROSSWIND_CROSSWIND_UTIL_H_

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Returns the curvature of an object with a specific acceleration and
// velocity, defined in the body frame.
double CalcCurvature(const Vec3 *acc_b, const Vec3 *vel_b);

// Converts an aerodynamic curvature, k_aero, lift coefficient, C_L,
// and side force coefficient, C_Y, to an equivalent tether roll based
// on the force balance equation:
//
//             C_Y - C_L * tether_roll
//   k_aero = -------------------------
//             m_eff / (0.5 * rho * A)
//
// This equation assumes a small body roll angle from the crosswind
// flight plane and a small angle approximation on the tether roll
// angle (even though it can be fairly significant).
double CurvatureToTetherRoll(double aero_curvature, double lift_coeff,
                             double side_force_coeff);

// Calculates the angle [rad] of the wing about the loop based on the
// position of the wing.  The loop angle is defined to be 0 along the
// +y crosswind axis (left side of loop) during a normal clockwise
// loop.  It increases counter-clockwise when viewed from the ground
// station.  The loop angle is always in the range [0, 2*pi].
double CalcLoopAngle(const Vec3 *path_center_g, const Vec3 *wing_pos_g);

// Check if the loop angle falls between the start and end angles.
bool BetweenLoopAngles(double start_angle, double end_angle, double loop_angle);

// Calculate the position vector, unit vector in the direction of
// velocity, and unit vector in the direction of centripetal
// acceleration, of the crosswind circle, given a path center and loop
// angle.
void CalcCircularLoopGeometry(const Vec3 *path_center_g, double loop_angle,
                              double loop_radius, Vec3 *pos_g, Vec3 *vel_hat_g,
                              Vec3 *acc_perp_hat_g);

// Calculate the required velocity and acceleration to track a
// changing airspeed command as the kite goes around the loop, taking
// into account the path geometry and the wind.  This wrapper function
// calls the more general CalcCrosswindKinematics().
//
// Input arguments:
//
// wind_g:        Vector of the direction the wind is blowing towards [m/s]
// path_center_g: Center position of the crosswind loop in ground frame [m]
// loop_angle:    Position on the crosswind loop by the usual definition [rad]
// loop_radius:   Radius of the crosswind circle [m]
// airspeed:      Commanded airspeed [m/s]
// d_airspeed_d_loopangle: Derivative of airspeed command with respect to
//                loop angle [m/s/rad]
//
// Output arguments:
//
// vel_g:         Kite (inertial) velocity vector [m/s].
// acc_g:         Kite acceleration vector [m/s^2].
//
// Return value:
//
// kiteaccel:     Required acceleration [m/s/s] to track airspeed command.
double CalcCircularLoopKinematics(const Vec3 *wind_g, const Vec3 *path_center_g,
                                  double loop_angle, double loop_radius,
                                  double airspeed,
                                  double d_airspeed_d_loopangle, Vec3 *vel_g,
                                  Vec3 *acc_g);

// Calculate the required velocity and acceleration to track a
// changing airspeed command as the kite goes around the loop, taking
// into account the path geometry and the wind.
//
// Input arguments:
//
// wind_g:          Vector of the direction the wind is blowing towards [m/s]
// vel_g_hat:       Unit vector tangent to the commanded path
// acc_g_perp_hat:  Unit vector in the direction of centripetal acceleration
// local_radius:    Local radius of curvature for the path [m]
// airspeed:        Commanded airspeed [m/s]
// airspeed_dot:    Rate of commanded airspeed [m/s^2]
//
// Output arguments:
//
// vel_g:           Kite (inertial) velocity vector [m/s].
// acc_g:           Kite acceleration vector [m/s^2].
//
// Return value:
//
// kiteaccel:     Required acceleration [m/s/s] to track airspeed command.
double CalcCrosswindKinematics(const Vec3 *wind_g, const Vec3 *vel_g_hat,
                               const Vec3 *acc_g_perp_hat, double local_radius,
                               double airspeed, double airspeed_dot,
                               Vec3 *vel_g, Vec3 *acc_g);

// Calculate the kite attitude (as expressed via dcm_g2b) required to attain
// the desired aero angles and tether roll command for the given loop geometry.
//
// Input arguments:
//
// wind_g:        Vector of the direction the wind is blowing towards [m/s]
// path_center_g: Center position of the crosswind loop in ground frame [m]
// loop_angle:    Position on the crosswind loop by the usual definition [rad]
// loop_radius:   Radius of the crosswind circle [m]
// tether_roll:   Tether roll angle [rad].
// alpha_cmd:     Angle-of-attack [rad].
// beta_cmd:      Sideslip [rad].
// airspeed_cmd:  Commanded airspeed [m/s]
// d_airspeed_d_loopangle: Derivative of airspeed command with respect to
//                loop angle [m/s/rad]
//
// Output arguments:
//
// dcm_g2b:       Direction cosine matrix expressing the ideal kite attitude.
void CalcCrosswindAttitude(const Vec3 *wind_g, const Vec3 *path_center_g,
                           double loop_angle, double path_radius,
                           double tether_roll, double alpha_cmd,
                           double beta_cmd, double airspeed_cmd,
                           double d_airspeed_d_loopangle, Mat3 *dcm_g2b);

// Calculate the body rates (pqr) required to follow the nominal
// crosswind loop trajectory while following the alpha, beta, and
// tether roll commands.
//
// Input arguments:
//
// wind_g:        Vector of the direction the wind is blowing towards [m/s]
// path_center_g: Center position of the crosswind loop in ground frame [m]
// loop_angle:    Position on the crosswind loop by the usual definition [rad]
// loop_radius:   Radius of the crosswind circle [m]
// tether_roll:   Tether roll angle [rad].
// alpha_cmd:     Angle-of-attack [rad].
// beta_cmd:      Sideslip [rad].
// airspeed_cmd:  Commanded airspeed [m/s]
// d_airspeed_d_loopangle: Derivative of airspeed command with respect to
//                loop angle [m/s/rad]
//
// Output arguments:
//
// pqr:           Body rates vector [rad/s].
void CalcCrosswindPqr(const Vec3 *wind_g, const Vec3 *path_center_g,
                      double loop_angle, double path_radius, double tether_roll,
                      double tether_roll_dot, double alpha_cmd, double beta_cmd,
                      double airspeed_cmd, double d_airspeed_d_loopangle,
                      double kitespeed, Vec3 *pqr);

// Calculate the body acceleration (pqr_cmd_dot) required to follow the nominal
// crosswind loop trajectory while following the alpha, beta, and
// tether roll commands.
//
// Input arguments:
//
// wind_g:        Vector of the direction the wind is blowing towards [m/s]
// path_center_g: Center position of the crosswind loop in ground frame [m]
// loop_angle:    Position on the crosswind loop by the usual definition [rad]
// loop_radius:   Radius of the crosswind circle [m]
// tether_roll:   Tether roll angle [rad].
// alpha_cmd:     Angle-of-attack [rad].
// beta_cmd:      Sideslip [rad].
// airspeed_cmd:  Commanded airspeed [m/s]
// d_airspeed_d_loopangle: Derivative of airspeed command with respect to
//                loop angle [m/s/rad]
//
// Output arguments:
//
// pqr_cmd_dot:   Angular acceleration vector in body frame.
void CalcCrosswindPqrDot(const Vec3 *wind_g, const Vec3 *path_center_g,
                         double loop_angle, double path_radius,
                         double tether_roll, double tether_roll_dot,
                         double alpha_cmd, double beta_cmd, double airspeed_cmd,
                         double d_airspeed_d_loopangle, double kitespeed,
                         Vec3 *pqr_cmd_dot);

// Harmonic integrator.
double HarmonicIntegrator(double u, double angle, double d_angle, double ki,
                          double int_max, double state[]);

// Move the path center azimuth when the kite is near the 12 o'clock or
// 6 o'clock loop positions.
//
// Input arguments:
//
// wing_pos_g:              Kite position in frame g [m]
// raw_path_center_g:       Desired loop center in frame g [m]
// max_crosswind_y_position_for_slew': Max distance [m] from 12 o'clock or 6
//                          o'clock where an azimuth slew is allowed to begin.
// flight_mode:             Flight mode
//
// Output arguments:
//
// path_center_g:           Path center location in frame g [m]
void SlewAzimuthWhenNecessary(const Vec3 *wing_pos_g,
                              const Vec3 *raw_path_center_g,
                              double max_crosswind_y_position_for_slew,
                              Vec3 *path_center_g);

// Return true if the target crosswind position should be commanded
// toward a straight line path in order to execute a racetrack style
// azimuth slew.
//
// Input arguments:
//
// target_pos_cw:          Target position under consideration in cw frame [m]
// raw_path_center_cw:     Desired path center in cw frame [m]
bool IsOnTheRacetrackHorizontalPath(const Vec3 *target_pos_cw,
                                    const Vec3 *raw_path_center_cw);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_UTIL_H_

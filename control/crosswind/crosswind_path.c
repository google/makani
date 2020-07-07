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

#include "control/crosswind/crosswind_path.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/geometry.h"
#include "common/c_math/linalg_common.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "control/control_telemetry.h"
#include "control/crosswind/crosswind_frame.h"
#include "control/crosswind/crosswind_util.h"
#include "control/estimator/estimator_types.h"
#include "control/system_params.h"
#include "control/system_types.h"

void CrosswindPathInit(const StateEstimate *state_est,
                       const CrosswindPathParams *params,
                       double initial_path_radius, CrosswindPathState *state) {
  memset(state, 0, sizeof(*state));
  state->path_radius_target = initial_path_radius;
  state->k_geom_curr_f_z1 = CalcCurvature(&state_est->Ab_f, &state_est->Vb);
  state->speed_f_z1 = Vec3Norm(&state_est->Vg);
  state->k_geom_cmd_f_z1 = -1.0 / initial_path_radius;
  state->k_aero_cmd_f_z1 = -1.0 / initial_path_radius;
  state->current_curvature_time = params->current_curvature_time;
  state->int_crosstrack = 0.0;
}

// For any point in the crosswind flight plane, calculates the ideal
// heading to follow a circular path.  This is essentially calculating
// the vector field that describes the flight path.
void CrosswindPathCalcBestHeading(const Vec3 *target_pos_cw,
                                  double path_radius_cmd,
                                  CrosswindPathType path_type,
                                  const Vec3 *raw_path_center_cw,
                                  const CrosswindPathParams *params,
                                  Vec3 *best_vel_cw) {
  assert(target_pos_cw->x < DBL_EPSILON);
  assert(params->min_turning_radius > 0.0);
  assert(abs(params->loop_dir) == 1);

  double radius = Vec3Norm(target_pos_cw);

  // If near the singularity, assume best heading is up.
  if (radius < 1e-5) {
    best_vel_cw->x = 0.0;
    best_vel_cw->y = 0.0;
    best_vel_cw->z = -1.0;
    return;
  }

  // If attempting to transition out, command vertical trajectories
  // when the kite approaches the 9 o'clock loop position. This
  // prevents the kite from starting to fly a new loop if it is
  // struggling to slow down.
  if (path_type == kCrosswindPathPrepareTransitionOut &&
      target_pos_cw->z <= params->cw_z_for_vertical_paths) {
    best_vel_cw->x = 0.0;
    best_vel_cw->y = 0.0;
    best_vel_cw->z = -1.0;
    return;
  }

  Vec3 new_target_pos_cw = *target_pos_cw;
  // Modify the target position to create path slews.
  if (IsOnTheRacetrackHorizontalPath(target_pos_cw, raw_path_center_cw)) {
    // Here we slew the path center horizontally with the kite's motion
    // and so the kite's y position in the crosswind plan remains zero by
    // definition. During this time the kite uses only the y=0 slice of the
    // vector field. This pushes the kite toward the top or bottom of
    // the loop as it normally would at the 12 or 6 o'clock positions,
    // creating a stable horizontal path command during the slew.
    new_target_pos_cw.y = 0.0;
  }

  // Calculate radial, normal, and tangent unit vectors.
  Vec3 e_rad, e_tang, e_norm = kVec3X;
  Vec3Scale(&new_target_pos_cw, 1.0 / radius, &e_rad);
  Vec3Cross(&e_norm, &e_rad, &e_tang);
  Vec3Scale(&e_tang, params->loop_dir, &e_tang);

  // Mix radial and tangent vectors.  Use just radial if you are more
  // than a turning radius away.
  double c_rad, c_tang;
  double radius_error = radius - path_radius_cmd;
  if (fabs(radius_error) < params->min_turning_radius) {
    c_tang = cos(0.5 * PI * radius_error / params->min_turning_radius);
    c_rad = -sin(0.5 * PI * radius_error / params->min_turning_radius);
  } else {
    c_tang = 0.0;
    c_rad = -Sign(radius_error);
  }
  Vec3LinComb(c_rad, &e_rad, c_tang, &e_tang, best_vel_cw);
}

// Returns the position and velocity of the wing after following the
// specified curvature for the specified time.
static void ApplyCurvature(const Vec3 *pos_cw, const Vec3 *vel_cw,
                           const double k_tab[], int32_t num_k,
                           double time_horizon_in, double speed,
                           double time_horizon_speed_limit, Vec3 pos_cw_out[],
                           Vec3 vel_cw_out[]) {
  double time_horizon =
      fmin(time_horizon_in,
           time_horizon_in / fmax(speed / time_horizon_speed_limit, 1e-5));

  Vec2 vel_cw_yz = {vel_cw->y, vel_cw->z};
  Mat3 dcm_b2cw = {{{0.0, 0.0, 0.0},
                    {0.0, vel_cw->y, -vel_cw->z},
                    {0.0, vel_cw->z, vel_cw->y}}};
  Mat3Scale(&dcm_b2cw, 1.0 / Vec2NormBound(&vel_cw_yz, 1e-5), &dcm_b2cw);

  for (int32_t i = 0; i < num_k; ++i) {
    double fudge = (fabs(k_tab[i]) < DBL_EPSILON) ? 1e-6 : 0.0;
    double radius = Saturate(1.0 / (k_tab[i] + fudge), -1e5, 1e5);
    double theta = speed * time_horizon / radius;
    Vec3 temp_pos_b = {0.0, radius * sin(theta), radius * (1.0 - cos(theta))};
    Vec3 temp_vel_b = {0.0, speed * cos(theta), speed * sin(theta)};

    Mat3Vec3Axpby(&dcm_b2cw, kNoTrans, &temp_pos_b, 1, pos_cw, &pos_cw_out[i]);
    Mat3Vec3Mult(&dcm_b2cw, &temp_vel_b, &vel_cw_out[i]);
  }
}

// Calculates the additional curvature due to the difference between
// airspeed and inertial velocity, when compared to the simple
// aerodynamic curvature relation (see crosswind_util.h).  The
// additional curvature may be expressed as a constant times the
// aerodynamic curvature:
//
//   apparent_wind_curvature = (c - 1) * aero_curvature
//
// where c is defined by:
//
//       v_app^2   |v_w - v_i|^2         v_w                   v_w^2
//   c = ------- = ------------- = 1 - 2 --- cos(wind_angle) + -----
//        v_i^2        v_i^2             v_i                   v_i^2
//
// TODO: Revisit whether to account for this curvature
// correction in an additive or multiplicative manner.  Also, revisit
// whether we should simply use airspeed rather than wind in the
// correction as we have a direct measurement of airspeed.
static double CalcApparentWindCurvature(double aero_curvature,
                                        const Mat3 *dcm_g2cw,
                                        const Vec3 *wing_vel_cw, double speed,
                                        const Vec3 *wind_g) {
  Vec3 wing_vel_g;
  Mat3TransVec3Mult(dcm_g2cw, wing_vel_cw, &wing_vel_g);

  Vec3 unit_wing_vel_g;
  Vec3Scale(&wing_vel_g, 1.0 / Vec3NormBound(&wing_vel_g, 1e-5),
            &unit_wing_vel_g);

  Vec3 unit_wind_g;
  Vec3Scale(wind_g, 1.0 / Vec3NormBound(wind_g, 1e-5), &unit_wind_g);

  // Calculate the angle between the wind vector and the inertial
  // velocity vector.
  double wind_angle = Acos(Vec3Dot(&unit_wing_vel_g, &unit_wind_g));

  // Calculate the ratio between the wind speed and the inertial speed.
  double speed_ratio = Vec3Norm(wind_g) / fmax(speed, 1e-5);

  return (-2.0 * speed_ratio * cos(wind_angle) + speed_ratio * speed_ratio) *
         aero_curvature;
}

// Calculates the additional curvature due to gravity, when compared
// to the simple aerodynamic curvature relation (see
// crosswind_util.h).  Specifically, the curvature due to the
// gravitational force is:
//
//                       m_wing   g_parallel
//   gravity_curvature = ------ * ---------- * sin(gravity_angle)
//                       m_eff      v_i^2
//
// where the gravity_angle is the angle between gravity vector and the
// current heading.
static double CalcGravityCurvature(const Mat3 *dcm_g2cw,
                                   const Vec3 *wing_vel_cw, double speed,
                                   LoopDirection loop_dir, const Vec3 *g_g) {
  // Calculate the magnitude of the gravity vector in the crosswind
  // flight plane.
  Vec3 g_parallel_cw;
  Mat3Vec3Mult(dcm_g2cw, g_g, &g_parallel_cw);
  g_parallel_cw.x = 0.0;
  double g_parallel = Vec3Norm(&g_parallel_cw);

  // Calculate the angle between the gravity vector projected onto the
  // crosswind flight plane and the current heading.  For a clockwise
  // loop as viewed from the ground-station, this is defined to be
  // 0 along the right side of the loop, +pi/2 along the top, etc.
  double gravity_angle = -atan2(wing_vel_cw->y, wing_vel_cw->z);

  // Forces on the wing must accelerate both the wing and the tether.
  // The effective mass of the tether as if it were concentrated at
  // the wing is derived by modeling the tether as a rigid rod of
  // uniform density.
  const double effective_mass =
      g_sys.wing->m + g_sys.tether->linear_density * g_sys.tether->length / 3.0;

  double loop_dir_sign = (loop_dir == kLoopDirectionCcw) ? 1.0 : -1.0;
  return loop_dir_sign * g_sys.wing->m / effective_mass * g_parallel *
         sin(gravity_angle) / fmax(speed * speed, 1e-5);
}

// Adds the gravity and apparent wind curvature corrections to an
// aerodynamic curvature.
static void AddCurvatureCorrections(const double aero_curvatures[],
                                    int32_t num_aero_curvatures,
                                    const Mat3 *dcm_g2cw,
                                    const Vec3 *wing_vel_cw, double speed,
                                    const Vec3 *wind_g, LoopDirection loop_dir,
                                    const Vec3 *g_g, double curvatures[]) {
  double gravity_curvature =
      CalcGravityCurvature(dcm_g2cw, wing_vel_cw, speed, loop_dir, g_g);
  for (int32_t i = 0; i < num_aero_curvatures; ++i) {
    double wind_curvature = CalcApparentWindCurvature(
        aero_curvatures[i], dcm_g2cw, wing_vel_cw, speed, wind_g);
    curvatures[i] = aero_curvatures[i] + gravity_curvature + wind_curvature;
  }
}

// Removes the gravity and apparent wind curvature corrections to give
// the equivalent aerodynamic curvature.
static double RemoveCurvatureCorrections(double curvature, const Mat3 *dcm_g2cw,
                                         const Vec3 *wing_vel_cw, double speed,
                                         const Vec3 *wind_g,
                                         LoopDirection loop_dir,
                                         const Vec3 *g_g) {
  double gravity_curvature =
      CalcGravityCurvature(dcm_g2cw, wing_vel_cw, speed, loop_dir, g_g);

  // The additional curvature due to the apparent wind is typically
  // represented as a constant times the aerodynamic curvature.
  // Passing 1.0 in as the aerodynamic curvature extracts this
  // constant.
  double wind_curvature_factor =
      CalcApparentWindCurvature(1.0, dcm_g2cw, wing_vel_cw, speed, wind_g);

  // Protect from division by zero.
  double denom = 1.0 + wind_curvature_factor;
  const double min_denom = 1e-5;
  if (fabs(denom) < min_denom) {
    denom = (denom > 0.0) ? min_denom : -min_denom;
  }

  return (curvature - gravity_curvature) / denom;
}

// Looks ahead some distance, assuming the wing is commanded to a
// range of curvatures. The curvature which best flies the wing in the
// desired direction is then selected for the curvature command.
//
// Args:
//   dcm_g2cw: DCM to rotate from ground to crosswind coordinates.
//   wing_pos_g: Position of wing in ground coordinates.
//   wing_vel_g: Velocity of wing in ground coordinates.
//   k_tab: Table of curvatures to test.
//   k_geom_curr: Current geometric curvature.
//   speed_f: Filtered inertial speed.
//   wind_g: Wind speed in ground coordinates.
//   current_curvature_time: Time [s] to project the current curvature
//       into the future.
//   params: Path loop parameters.
//   target_pos_cw[]: Outputs list of future positions given a specified
//       curvature.
//   target_vel_cw[]: Outputs list of future velocities given a specified
//       curvature.
//   current_pos_cw: Outputs current position in crosswind flight plane.
//   current_vel_cw: Outputs current velocity in crosswind flight plane.
//   final_vel_cw: Outputs predicted final velocity vector in crosswind
//       flight plane.
//   current_speed: Outputs current speed.
//   final_speed: Outputs predicted final speed.
static void PredictTargets(
    const Mat3 *dcm_g2cw, const Vec3 *wing_pos_g, const Vec3 *wing_vel_g,
    const double *k_tab, double k_geom_curr, double speed_f, const Vec3 *wind_g,
    double current_curvature_time, double path_radius_cmd,
    const CrosswindPathParams *params, Vec3 target_pos_cw[],
    Vec3 target_vel_cw[], Vec3 *current_pos_cw, Vec3 *current_vel_cw,
    Vec3 *final_vel_cw, double *current_speed, double *final_speed) {
  // Project position and velocity into crosswind coordinates.
  Mat3Vec3Mult(dcm_g2cw, wing_pos_g, current_pos_cw);
  current_pos_cw->x = 0.0;
  Mat3Vec3Mult(dcm_g2cw, wing_vel_g, current_vel_cw);
  current_vel_cw->x = 0.0;

  // Modify the commanded_curvature_time such that the total
  // look-ahead time remains constant.
  double commanded_curvature_time = params->commanded_curvature_time +
                                    params->current_curvature_time -
                                    current_curvature_time;

  // Apply current measured curvature for time: current_curvature_time.
  *current_speed = Vec3NormBound(current_vel_cw, 1.0);
  Vec3 middle_pos_cw, middle_vel_cw;
  ApplyCurvature(current_pos_cw, current_vel_cw, &k_geom_curr, 1,
                 current_curvature_time, *current_speed,
                 params->time_horizon_speed_limit, &middle_pos_cw,
                 &middle_vel_cw);

  // Calculate approximate heading (final_vel_cw) and speed (final_speed) at
  // time: current_curvature_time + commanded_curvature_time / 2.
  Vec3 final_pos_cw;
  ApplyCurvature(current_pos_cw, current_vel_cw, &k_geom_curr, 1,
                 current_curvature_time + commanded_curvature_time / 2.0,
                 *current_speed, params->time_horizon_speed_limit,
                 &final_pos_cw, final_vel_cw);

  // The final speed is the estimated speed half way through the
  // commanded_curvature_time interval.  This is the speed used to
  // determine the distance to project the current curvature forward.
  *final_speed =
      fmax(10.0, *current_speed +
                     0.2 * speed_f * (final_pos_cw.z - current_pos_cw->z) /
                         (2.0 * path_radius_cmd));

  // Calculate a gravity and wind asymmetry corrected curvature.
  double k_tab_corr[CROSSWIND_PATH_CURVATURE_TABLE_LENGTH];
  AddCurvatureCorrections(k_tab, CROSSWIND_PATH_CURVATURE_TABLE_LENGTH,
                          dcm_g2cw, final_vel_cw, *final_speed, wind_g,
                          params->loop_dir, &g_sys.phys->g_g, k_tab_corr);

  // Apply new curvature for time: commanded_curvature_time.
  ApplyCurvature(&middle_pos_cw, &middle_vel_cw, k_tab_corr,
                 CROSSWIND_PATH_CURVATURE_TABLE_LENGTH,
                 commanded_curvature_time, *final_speed,
                 params->time_horizon_speed_limit, target_pos_cw,
                 target_vel_cw);
}

// Standard cost function for the heading is simply the normalized dot
// product of the predicted and "ideal" velocities.
static double CalcHeadingCost(const Vec3 *target_vel_cw,
                              const Vec3 *best_vel_cw) {
  return Vec3Dot(best_vel_cw, target_vel_cw) /
         Vec3NormBound(target_vel_cw, 1e-5);
}

// Standard cost function for the predictive crosswind controller.
//
//   cost = cost_heading
//
// where cost_heading is the cost function for direction (dot preferred
// with predicted directions).
static void CalcCosts(const Vec3 target_pos_cw[], const Vec3 target_vel_cw[],
                      double path_radius_cmd, CrosswindPathType path_type,
                      const Vec3 *raw_path_center_cw,
                      const CrosswindPathParams *params, double costs[]) {
  Vec3 best_vel_cw;
  for (int32_t i = 0; i < CROSSWIND_PATH_CURVATURE_TABLE_LENGTH; ++i) {
    CrosswindPathCalcBestHeading(&target_pos_cw[i], path_radius_cmd, path_type,
                                 raw_path_center_cw, params, &best_vel_cw);
    costs[i] = CalcHeadingCost(&target_vel_cw[i], &best_vel_cw);
  }
}

// Determines which curvature maximizes the cost function.
static double CalcBestCurvature(const double k_tab[], const double costs[]) {
  // Find k of maximum cost.
  int32_t max_ind;
  MaxArray(costs, CROSSWIND_PATH_CURVATURE_TABLE_LENGTH, &max_ind);
  double k_best = k_tab[max_ind];

  // Quadratic approximation to find better k.
  double coeff[3];
  max_ind =
      SaturateInt32(max_ind, 1, CROSSWIND_PATH_CURVATURE_TABLE_LENGTH - 2);
  PolyFit2(&k_tab[max_ind - 1], &costs[max_ind - 1], coeff);
  if (coeff[0] < 0.0) {
    k_best = -coeff[1] / (2.0 * coeff[0]);
  }

  return k_best;
}

// Updates the current curvature look-ahead time based on whether the
// position and velocity solutions use GPS or GLAS.  The GLAS
// solutions are very sensitive to excitations in the tether, which
// can be induced by the current curvature section of the look-ahead
// because it is effectively increasing the proportional and
// derivative gains of the linearized path controller.  This slowly
// reduces the current curvature time to zero when the GPS is
// inactive.
static void UpdateCurrentCurvatureTime(bool gps_active,
                                       const CrosswindPathParams *params,
                                       CrosswindPathState *state) {
  double current_curvature_time_setpoint =
      gps_active ? params->current_curvature_time : 0.0;
  RateLimit(current_curvature_time_setpoint,
            -params->current_curvature_time / 10.0,
            params->current_curvature_time / 10.0, *g_sys.ts,
            &state->current_curvature_time);
}

// Calculates the Euler angles from (a permutation of) the crosswind
// frame to the body frame.  The axes of the crosswind frame are
// permuted so that the permuted frame's z is normal to the circle
// pointing towards the ground-station, which is useful for doing the
// typical transformation of a DCM to Euler angles in the ZYX rotation
// order.
static void CalcCrosswindEulers(const Mat3 *dcm_g2b, const Mat3 *dcm_g2cw,
                                Vec3 *eulers_cw) {
  Mat3 dcm_cw2b;
  Mat3Mult(dcm_g2b, kNoTrans, dcm_g2cw, kTrans, &dcm_cw2b);

  // Create a temporary frame that is a permutation of the crosswind
  // frame's axes such that z points towards the ground station and x
  // points along the crosswind frame's y-axis.
  Mat3 dcm_tmp2cw = {{{0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}}};
  Mat3 dcm_tmp2b;
  Mat3Mat3Mult(&dcm_cw2b, &dcm_tmp2cw, &dcm_tmp2b);

  DcmToAngle(&dcm_tmp2b, kRotationOrderZyx, &eulers_cw->z, &eulers_cw->y,
             &eulers_cw->x);
}

void CrosswindPathStep(FlightMode flight_mode, const Vec3 *path_center_g,
                       const Vec3 *raw_path_center_g,
                       CrosswindPathType path_type,
                       const StateEstimate *state_est,
                       const PlaybookEntry *playbook_entry,
                       const CrosswindPathParams *params,
                       CrosswindPathState *state, double *k_aero_cmd,
                       double *k_geom_cmd, double *k_aero_curr_f,
                       double *k_geom_curr_f) {
  assert(path_center_g != NULL && state_est != NULL && params != NULL &&
         state != NULL && k_aero_cmd != NULL && k_geom_cmd != NULL &&
         k_aero_curr_f != NULL && k_geom_curr_f != NULL);

  if (flight_mode == kFlightModeCrosswindPrepTransOut) {
    RateLimit(params->preptransout_radius_cmd, -params->path_radius_rate,
              params->path_radius_rate, *g_sys.ts, &state->path_radius_target);
  } else {
    RateLimit(playbook_entry->path_radius_target, -params->path_radius_rate,
              params->path_radius_rate, *g_sys.ts, &state->path_radius_target);
  }
  double path_radius_cmd = state->path_radius_target;

  // Adjusts the current curvature time based on whether GPS is being
  // used.
  UpdateCurrentCurvatureTime(state_est->gps_active, params, state);

  // Calculate current geometric curvature.
  double k_geom_curr = CalcCurvature(&state_est->Ab_f, &state_est->Vb);

  *k_geom_curr_f = Lpf(Saturate(k_geom_curr, -0.1, 0.1), params->fc_k_geom_curr,
                       *g_sys.ts, &state->k_geom_curr_f_z1);
  double speed_f = Lpf(Vec3Norm(&state_est->Vg), params->fc_speed, *g_sys.ts,
                       &state->speed_f_z1);

  // Calculate the DCM to the crosswind coordinate system.
  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);

  // Use predictor to determine list of target positions and headings.
  double current_speed, final_speed;
  Vec3 current_pos_cw, current_vel_cw, final_vel_cw;
  Vec3 target_pos_cw[CROSSWIND_PATH_CURVATURE_TABLE_LENGTH];
  Vec3 target_vel_cw[CROSSWIND_PATH_CURVATURE_TABLE_LENGTH];
  // TODO: This code does not check state_est->wind_g.valid.
  PredictTargets(&dcm_g2cw, &state_est->Xg, &state_est->Vg, params->k_tab,
                 *k_geom_curr_f, speed_f, &state_est->wind_g.vector_f,
                 state->current_curvature_time, path_radius_cmd, params,
                 target_pos_cw, target_vel_cw, &current_pos_cw, &current_vel_cw,
                 &final_vel_cw, &current_speed, &final_speed);

  Vec3 raw_path_center_cw;
  Mat3Vec3Mult(&dcm_g2cw, raw_path_center_g, &raw_path_center_cw);
  raw_path_center_cw.x = 0.0;

  // Evaluate the cost function at each of these target positions.
  double costs[CROSSWIND_PATH_CURVATURE_TABLE_LENGTH];
  CalcCosts(target_pos_cw, target_vel_cw, path_radius_cmd, path_type,
            &raw_path_center_cw, params, costs);

  // Choose the best curvature.
  double k_aero_best = CalcBestCurvature(params->k_tab, costs);

  // Calcuate integral feedback.
  //
  // TODO: Consider replacing the crosstrack controller.
  assert(params->crosstrack_pid.kp == 0.0);
  assert(params->crosstrack_pid.ki > 0.0);
  assert(params->crosstrack_pid.kd == 0.0);

  const double current_pos_radius = Vec3YzNorm(&current_pos_cw);

  // Saturate the crosstrack position error.
  double crosstrack_radius_error_limited = Saturate(
      path_radius_cmd - current_pos_radius, -params->crosswind_max_radius_error,
      params->crosswind_max_radius_error);
  // Compute the crosstrack error integral.
  double k_aero_intfb = Pid(crosstrack_radius_error_limited, 0.0, *g_sys.ts,
                            kIntegratorModeIntegrate, &params->crosstrack_pid,
                            &state->int_crosstrack);

  // Total aero curvature command (nonlinear PDA + linear I)
  k_aero_best = k_aero_best + k_aero_intfb;

  // Prepare outputs.
  *k_aero_cmd =
      Saturate(k_aero_best, params->k_tab[0],
               params->k_tab[CROSSWIND_PATH_CURVATURE_TABLE_LENGTH - 1]);
  // TODO: This code does not check state_est->wind_g.valid.
  AddCurvatureCorrections(k_aero_cmd, 1, &dcm_g2cw, &final_vel_cw, final_speed,
                          &state_est->wind_g.vector_f, params->loop_dir,
                          &g_sys.phys->g_g, k_geom_cmd);
  *k_aero_cmd = Lpf(*k_aero_cmd, params->fc_k_aero_cmd, *g_sys.ts,
                    &state->k_aero_cmd_f_z1);
  *k_geom_cmd = Lpf(*k_geom_cmd, params->fc_k_geom_cmd, *g_sys.ts,
                    &state->k_geom_cmd_f_z1);
  *k_aero_curr_f = RemoveCurvatureCorrections(
      *k_geom_curr_f, &dcm_g2cw, &current_vel_cw, current_speed,
      &state_est->wind_g.vector_f, params->loop_dir, &g_sys.phys->g_g);

  assert(params->max_k_aero_error > 0);
  *k_aero_cmd = Saturate(*k_aero_cmd, *k_aero_curr_f - params->max_k_aero_error,
                         *k_aero_curr_f + params->max_k_aero_error);

  // Update telemetry.
  Vec3 eulers_cw;
  CalcCrosswindEulers(&state_est->dcm_g2b, &dcm_g2cw, &eulers_cw);

  Vec3 best_target_pos_cw;
  Interp1Vec3(params->k_tab, target_pos_cw,
              CROSSWIND_PATH_CURVATURE_TABLE_LENGTH, k_aero_best,
              kInterpOptionSaturate, &best_target_pos_cw);
  CrosswindTelemetry *cwt = GetCrosswindTelemetry();
  cwt->path_radius_target = state->path_radius_target;
  cwt->eulers_cw = eulers_cw;
  cwt->target_pos_cw.x = best_target_pos_cw.y;
  cwt->target_pos_cw.y = best_target_pos_cw.z;
  cwt->current_pos_cw.x = current_pos_cw.y;
  cwt->current_pos_cw.y = current_pos_cw.z;
  cwt->k_geom_cmd = *k_geom_cmd;
  cwt->k_aero_cmd = *k_aero_cmd;
  cwt->k_geom_curr = *k_geom_curr_f;
  cwt->k_aero_curr = *k_aero_curr_f;
  cwt->int_crosstrack = state->int_crosstrack;
}

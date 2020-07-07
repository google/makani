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

// This file is for stateless, math-like transformations to sensor
// data.

#include "control/sensor_util.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/coord_trans.h"
#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/ground_frame.h"
#include "control/perch_frame.h"
#include "control/sensor_types.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "control/tether_util.h"
#include "control/vessel_frame.h"
#include "system/labels.h"

void GsGpsPosEcefToGsPosEcef(const Vec3 *gs_gps_pos_ecef,
                             const Vec3 *gs_gps_pos_v, double g_heading,
                             Vec3 *gs_pos_ecef, Mat3 *dcm_ecef2g) {
  // TODO: Calculate dcm_g2v in the controller and use it here as
  // an input parameter.
  Vec3 gs_gps_pos_g;
  Mat3 dcm_g2v = kMat3Identity;
  Mat3TransVec3Mult(&dcm_g2v, gs_gps_pos_v, &gs_gps_pos_g);

  // We assume that the GS GPS antenna is sufficiently close to the vessel
  // frame origin that the NED-to-ECEF rotation obtained using gs_gps_pos_ecef
  // as the reference point is negligibly different from the rotation with
  // gs_pos_ecef as its reference point.
  // TODO: Replace gs_gps_pos_ecef by g_frame_ecef_pos to account
  // for the motion of the buoy relative to the ground frame.
  CalcDcmEcefToG(gs_gps_pos_ecef, g_heading, dcm_ecef2g);

  Vec3 r_ecef_gps_antenna;
  Mat3TransVec3Mult(dcm_ecef2g, &gs_gps_pos_g, &r_ecef_gps_antenna);
  Vec3Sub(gs_gps_pos_ecef, &r_ecef_gps_antenna, gs_pos_ecef);
}

void ConvSigmaEcefToLocal(const Vec3 *sigma_X_ecef, const Vec3 *sigma_V_ecef,
                          const Mat3 *dcm_ecef2g, Vec3 *sigma_Xg,
                          Vec3 *sigma_Vg) {
  Mat3 cov_X_ecef = {{{sigma_X_ecef->x * sigma_X_ecef->x, 0.0, 0.0},
                      {0.0, sigma_X_ecef->y * sigma_X_ecef->y, 0.0},
                      {0.0, 0.0, sigma_X_ecef->z * sigma_X_ecef->z}}};
  Mat3 cov_V_ecef = {{{sigma_V_ecef->x * sigma_V_ecef->x, 0.0, 0.0},
                      {0.0, sigma_V_ecef->y * sigma_V_ecef->y, 0.0},
                      {0.0, 0.0, sigma_V_ecef->z * sigma_V_ecef->z}}};
  Mat3 cov_Xg, cov_Vg;

  RotateCov(&cov_X_ecef, dcm_ecef2g, &cov_Xg);
  RotateCov(&cov_V_ecef, dcm_ecef2g, &cov_Vg);

  // TODO: This is an approximation until we figure out
  // the appropriate thing to do with the covariance matrix in the
  // state estimator.
  sigma_Xg->x = Sqrt(cov_Xg.d[0][0]);
  sigma_Xg->y = Sqrt(cov_Xg.d[1][1]);
  sigma_Xg->z = Sqrt(cov_Xg.d[2][2]);
  sigma_Vg->x = Sqrt(cov_Vg.d[0][0]);
  sigma_Vg->y = Sqrt(cov_Vg.d[1][1]);
  sigma_Vg->z = Sqrt(cov_Vg.d[2][2]);
}

void TetherForceSphToCart(const TetherForceSph *sph, Vec3 *cart) {
  cart->x = sph->tension * cos(sph->roll) * sin(sph->pitch);
  cart->y = sph->tension * -sin(sph->roll);
  cart->z = sph->tension * cos(sph->roll) * cos(sph->pitch);
}

void TetherForceCartToSph(const Vec3 *cart, TetherForceSph *sph) {
  sph->tension = Vec3Norm(cart);
  sph->roll = atan2(-cart->y, Vec3XzNorm(cart));
  sph->pitch = atan2(cart->x, cart->z);
}

// Determines the z-component of force measured by a loadcell in the loadcell's
// frame.
//
// If we assume that the bridles are rigid and under tension, then the force
// vector on a given loadcell must:
//   (1) Be in the same direction as the loadcell-to-knot vector.
//   (2) Form a fixed angle phi (the "bridle angle") with the
//       loadcell-to-other-loadcell vector.
//
// In local loadcell coordinates, let F = [f1, f2, f3]^T be the force vector,
// and x = [x1, x2, x3]^T be the loadcell-to-other-loadcell vector. Note that f1
// and f2 are measured directly by the loadcell. Then
//     F * x = |F| |x| cos(phi).
// Squaring both sides yields a quadratic equation in f3.
//
// For a full description of the method, see
//     go/makani-doc/controls/tension_estimation.md.
//
// Args:
//   force_x: The x-component of the force in the loadcell's frame.
//   force_y: The y-component of the force in the loadcell's frame.
//   cos_bridle_angle: Cosine of the bridle angle, the angle between the
//       loadcell-to-knot and loadcell-to-other-loadcell vectors.
//   this_to_other: Vector from this loadcell to the other loadcell,
//       in this loadcell's frame.
static double LoadcellForceBZ(double force_x, double force_y,
                              double cos_bridle_angle,
                              const Vec3 *this_to_other) {
  double s1 = force_x * this_to_other->x + force_y * this_to_other->y;
  double s2 = Square(force_x) + Square(force_y);
  double s3_squared = Square(Vec3Norm(this_to_other) * cos_bridle_angle);

  double a = Square(this_to_other->z) - s3_squared;
  double b = 2.0 * s1 * this_to_other->z;
  double c = Square(s1) - s2 * s3_squared;

  // This assertion is stronger than needed, but if a starts getting small,
  // someone should give the algebra some more careful thought. It's value is
  // determined entirely by configuration parameters.
  assert(fabs(a) >= 1.0);

  if (fabs(a) < DBL_TOL) {
    return 0.0;
  } else {
    return (-b - Sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
  }
}

static double CalcCosAngle(const Vec3 *u, const Vec3 *v) {
  double u_norm = Vec3Norm(u);
  double v_norm = Vec3Norm(v);
  if (u_norm < DBL_TOL || v_norm < DBL_TOL) {
    assert(false);
    return 0;
  }
  return Vec3Dot(u, v) / (u_norm * v_norm);
}

// Computes the force exerted on a loadcell by its bridle in the body frame.
static void CalcLoadcellForceB(const Vec2 *channel_measurements,
                               const Vec3 *this_bridle_pos_b,
                               const Vec3 *other_bridle_pos_b,
                               const Vec3 *knot0_pos_b,
                               const LoadcellParams *loadcell_params,
                               Vec3 *force_b) {
  // Incorporate the force components measured by the loadcell.
  Vec2 force_local_xy;
  Mat2Vec2Mult(&loadcell_params->channels_to_force_local_xy,
               channel_measurements, &force_local_xy);

  Vec3 this_to_other_b, this_to_other_local;
  Vec3Sub(other_bridle_pos_b, this_bridle_pos_b, &this_to_other_b);
  Mat3TransVec3Mult(&loadcell_params->dcm_loadcell2b, &this_to_other_b,
                    &this_to_other_local);
  Vec3 this_to_knot0_b;
  Vec3Sub(knot0_pos_b, this_bridle_pos_b, &this_to_knot0_b);

  Vec3 force_local = {
      force_local_xy.x, force_local_xy.y,
      LoadcellForceBZ(force_local_xy.x, force_local_xy.y,
                      CalcCosAngle(&this_to_other_b, &this_to_knot0_b),
                      &this_to_other_local)};

  Mat3Vec3Mult(&loadcell_params->dcm_loadcell2b, &force_local, force_b);
}

COMPILE_ASSERT(NUM_LOADCELL_CHANNELS == 2, NUM_LOADCELL_CHANNELS_must_be_2);

void LoadcellsToTetherForce(const double loadcells[],
                            const WingParams *wing_params,
                            const LoadcellParams loadcell_params[], Vec3 *cart,
                            TetherForceSph *sph, Vec3 *port_force_b,
                            Vec3 *star_force_b) {
  assert(loadcells != NULL && wing_params != NULL && loadcell_params != NULL &&
         cart != NULL && sph != NULL);

  const Vec3 *port_pos_b = &wing_params->bridle_pos[kBridlePort];
  const Vec3 *star_pos_b = &wing_params->bridle_pos[kBridleStar];

  // "knot0" refers to the position of the knot at zero pitch and zero roll.
  Vec3 knot0_pos_b = {
      (port_pos_b->x + star_pos_b->x) / 2.0, wing_params->bridle_y_offset,
      wing_params->bridle_rad + (port_pos_b->z + star_pos_b->z) / 2.0};

  const Vec2 port_channels = {loadcells[kLoadcellSensorPort0],
                              loadcells[kLoadcellSensorPort1]};
  CalcLoadcellForceB(&port_channels, port_pos_b, star_pos_b, &knot0_pos_b,
                     &loadcell_params[kBridlePort], port_force_b);

  const Vec2 star_channels = {loadcells[kLoadcellSensorStarboard0],
                              loadcells[kLoadcellSensorStarboard1]};
  CalcLoadcellForceB(&star_channels, star_pos_b, port_pos_b, &knot0_pos_b,
                     &loadcell_params[kBridleStar], star_force_b);

  Vec3Add(port_force_b, star_force_b, cart);
  TetherForceCartToSph(cart, sph);
}

double VesselHeadingPerchHeadingToPerchAzi(double vessel_heading,
                                           double perch_heading) {
  assert(0.0 <= vessel_heading && vessel_heading < 2.0 * PI);
  assert(0.0 <= perch_heading && perch_heading < 2.0 * PI);

  return remainder(perch_heading - vessel_heading, 2.0 * PI);
}

double WinchPosToDrumAngle(double winch_pos, const WinchParams *params) {
  return winch_pos / params->r_drum;
}

// Incidence angles follow conventions from Etkin (pg. 11):
//
//   alpha = arctan(w / u)
//   beta  = arcsin(v / |V|)
//
// where (u, v, w) are the cartesian components of the body velocity,
// i.e. opposite the apparent wind.
void ApparentWindSphToCart(const ApparentWindSph *sph, Vec3 *cart) {
  cart->x = -sph->v * cos(sph->alpha) * cos(sph->beta);
  cart->y = -sph->v * sin(sph->beta);
  cart->z = -sph->v * sin(sph->alpha) * cos(sph->beta);
}

void ApparentWindCartToSph(const Vec3 *cart, ApparentWindSph *sph) {
  sph->v = Vec3Norm(cart);
  sph->alpha = atan2(-cart->z, -cart->x);
  // The following is equivalent to asin(-cart->y / sph->v) but
  // avoids the potential for division by zero.
  sph->beta = atan2(-cart->y, Vec3XzNorm(cart));
}

void PitotToApparentWindSph(const PitotDifferentialData *diff, const Vec3 *pqr,
                            const PitotParams *params, ApparentWindSph *sph) {
  // The current Pitot tube has a hemispherical tip, and for small
  // angles the expected pressure differences measured between the
  // Pitot ports can be approximated by the pressure distribution on a
  // sphere giving:
  //
  // dyn_press =
  //     dynamic_pressure * (1 - 9/4 * (1 - cos(alpha)^2 * cos(beta)^2)),
  // alpha_press =
  //     dynamic_pressure * (9/2) * sin(2 * port_angle) * cos(beta)^2
  //     * cos(alpha) * sin(alpha),
  // beta_press =
  //     dynamic_pressure * (9/2) * sin(2 * port_angle) * cos(alpha)
  //     * cos(beta) * sin(beta).
  //
  // The wind vector has components:
  //
  //   u = V * cos(alpha) * cos(beta),
  //   v = V * sin(beta),
  //   w = V * cos(beta) * sin(alpha).
  //
  // If we write c = rho * 9/4 * sin(2 * port_angle), then:
  //
  //   a = alpha_press / c = u * w
  //   b = beta_press / c = u * v
  //   d = dyn_press / (0.5 * rho) = u^2 - 5/4 * (v^2 + w^2)
  //                               = u^2 - 5/4 * (a^2 + b^2) / u^2.
  //
  // We can thus solve a quadratic equation for u^2, and then use this
  // to find v and w.
  double c = g_sys.phys->rho * (9.0 / 4.0) * sin(2.0 * params->port_angle);
  double a = diff->alpha_press / c;
  double b = diff->beta_press / c;
  double d = diff->dyn_press / (0.5 * g_sys.phys->rho);

  Vec3 apparent_wind_p;
  apparent_wind_p.x = -Sqrt(0.5 * (d + Sqrt(d * d + 5.0 * (a * a + b * b))));
  apparent_wind_p.y = b / fmin(-1.0, apparent_wind_p.x);
  apparent_wind_p.z = a / fmin(-1.0, apparent_wind_p.x);

  // Transform measurement to body coordinates.
  Vec3 apparent_wind_b;
  Mat3TransVec3Mult(&params->dcm_b2p, &apparent_wind_p, &apparent_wind_b);

  // Correct for offset of Pitot tube.
  Vec3 offset_correction_b;
  Vec3Cross(&params->pos, pqr, &offset_correction_b);
  Vec3Sub(&apparent_wind_b, &offset_correction_b, &apparent_wind_b);

  ApparentWindCartToSph(&apparent_wind_b, sph);
}

void WindWsToWindG(const Vec3 *wind_ws, const Mat3 *dcm_g2p,
                   const Vec3 *omega_g2p, const Vec3 *vel_g,
                   const WindSensorParams *params, Vec3 *wind_g) {
  if (params->on_perch) {
    // Rotate the measured wind velocity into the perch frame.
    Vec3 wind_p;
    Mat3TransVec3Mult(&params->dcm_parent2ws, wind_ws, &wind_p);

    // Remove the apparent wind from the rotating perch.
    Vec3 wind_sensor_vel_p;
    Vec3Cross(omega_g2p, &params->pos_parent, &wind_sensor_vel_p);
    Vec3Add(&wind_p, &wind_sensor_vel_p, &wind_p);

    // Rotate wind from platform to ground frame.
    Mat3TransVec3Mult(dcm_g2p, &wind_p, wind_g);

    // Account for motion of the buoy.
    Vec3Add(wind_g, vel_g, wind_g);
  } else {
    // The parent frame is the ground frame in this case.
    Mat3TransVec3Mult(&params->dcm_parent2ws, wind_ws, wind_g);
  }
}

// TODO: This does not account for the offset between the
// azimuth axis and the elevation axis.  Also, assumes tether length
// is measured from GSG origin.
static void XgsgToXwd(const GsgData *gsg, const Vec3 *X_gsg, Vec3 *X_wd) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  assert(GetSystemParams()->gs_model == kGroundStationModelGSv1 ||
         GetSystemParams()->gs_model == kGroundStationModelTopHat);
  Mat3 dcm_wd2gsg;
  AngleToDcm(PI + gsg->azi, gsg->ele, 0.0, kRotationOrderZyx, &dcm_wd2gsg);
  Mat3TransVec3Mult(&dcm_wd2gsg, X_gsg, X_wd);
  Vec3Add(X_wd, &g_sys.perch->gsg_pos_wd, X_wd);
}

void XgsgToXp(const GsgData *gsg, const Vec3 *X_gsg, double drum_angle,
              Vec3 *Xp) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  assert(GetSystemParams()->gs_model == kGroundStationModelGSv1 ||
         GetSystemParams()->gs_model == kGroundStationModelTopHat);
  Vec3 X_wd;
  XgsgToXwd(gsg, X_gsg, &X_wd);
  WdToP(&X_wd, drum_angle, Xp);
}

static void XwdToGsg(const Vec3 *X_wd, GsgData *gsg) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  assert(GetSystemParams()->gs_model == kGroundStationModelGSv1 ||
         GetSystemParams()->gs_model == kGroundStationModelTopHat);
  Vec3 tether_direction_wd;
  Vec3Sub(X_wd, &g_sys.perch->gsg_pos_wd, &tether_direction_wd);
  TetherDirectionWdToGsgAziEle(&tether_direction_wd, &gsg->azi, &gsg->ele);
}

void XpToGsg(const Vec3 *Xp, double drum_angle, GsgData *gsg) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  assert(GetSystemParams()->gs_model == kGroundStationModelGSv1 ||
         GetSystemParams()->gs_model == kGroundStationModelTopHat);
  Vec3 X_wd;
  PToWd(Xp, drum_angle, &X_wd);
  XwdToGsg(&X_wd, gsg);
}

void TetherDirectionWdToGsgAziEle(const Vec3 *tether_direction_wd, double *azi,
                                  double *ele) {
  *azi =
      Wrap(atan2(tether_direction_wd->y, tether_direction_wd->x) - PI, -PI, PI);
  *ele = atan2(-tether_direction_wd->z, Vec3XyNorm(tether_direction_wd));
}

bool IsLevelwindEngaged(double drum_angle, const LevelwindParams *params) {
  return drum_angle < params->pulley_engage_drum_angle;
}

void CalcDcmWdToGsg0(double detwist_ele, double detwist_pos,
                     Mat3 *dcm_wd2gsg0) {
  AngleToDcm(0.0, PI * 0.5 - detwist_ele, detwist_pos + PI, kRotationOrderXyz,
             dcm_wd2gsg0);
}

// TODO: Move to some GSv2-specific file.
static void CalcDcmPlatformToDrum(double drum_position,
                                  Mat3 *dcm_platform_to_drum) {
  AngleToDcm(0.0, 0.0, drum_position, kRotationOrderZyx, dcm_platform_to_drum);
}

// Calculate the direction at which the tether departs the GSG, in the platform
// frame.  This is independent of vessel attitude and requires only GS encoder
// measurements.
static void CalcTetherVecPFromGsg(double drum_position, double detwist_ele,
                                  double detwist_angle, double gsg_yoke,
                                  double gsg_termination, Vec3 *tether_dir_p) {
  // Frame gsg0 is the detwist frame, and frame gsg2 is the tether termination
  // frame.
  Mat3 dcm_platform_to_drum;
  CalcDcmPlatformToDrum(drum_position, &dcm_platform_to_drum);

  Mat3 dcm_drum_to_gsg0;
  CalcDcmWdToGsg0(detwist_ele, detwist_angle, &dcm_drum_to_gsg0);

  Mat3 dcm_gsg0_to_gsg2;
  AngleToDcm(gsg_yoke, gsg_termination, 0.0, kRotationOrderXyz,
             &dcm_gsg0_to_gsg2);

  Mat3 dcm_drum_to_gsg2, dcm_platform_to_gsg2;
  Mat3Mat3Mult(&dcm_gsg0_to_gsg2, &dcm_drum_to_gsg0, &dcm_drum_to_gsg2);
  Mat3Mat3Mult(&dcm_drum_to_gsg2, &dcm_platform_to_drum, &dcm_platform_to_gsg2);

  const Vec3 tether_dir_gsg2 = {0.0, 0.0, -1.0};
  Mat3TransVec3Mult(&dcm_platform_to_gsg2, &tether_dir_gsg2, tether_dir_p);
}

static void CalcAlignedDetwistAngle(const Vec3 *tether_dir_p,
                                    double drum_position, double detwist_ele,
                                    double hold_cone_half_angle,
                                    double detwist_axis_offset,
                                    double *tether_detwist_angle,
                                    bool *updated) {
  // Project the tether direction vector into the "detwist0" frame, which is the
  // GSG0 frame when the detwist angle is zero.
  Vec3 tether_dir_detwist0;
  {
    Mat3 dcm_drum_to_detwist0;
    CalcDcmWdToGsg0(detwist_ele, 0.0, &dcm_drum_to_detwist0);

    Mat3 dcm_platform_to_drum;
    CalcDcmPlatformToDrum(drum_position, &dcm_platform_to_drum);

    Vec3 tether_dir_drum;
    Mat3Vec3Mult(&dcm_platform_to_drum, tether_dir_p, &tether_dir_drum);
    Mat3Vec3Mult(&dcm_drum_to_detwist0, &tether_dir_drum, &tether_dir_detwist0);
  }

  // The deflection axis is the axis, in the plane of the GSG yoke and
  // termination axes, about which the tether is deflected from alignment with
  // the detwist axis.
  //
  // The parameter detwist_axis_offset is used to offset upwards the physical
  // detwist axis (the z-axis). The sign is chosen so that, with a
  // detwist_axis_offset of 0 deg, the deflection axis will equal the GSG0
  // x-axis if no detwist rotation is required.
  Mat3 dcm_offset_axis;
  AngleToDcm(0., -detwist_axis_offset, 0., kRotationOrderXyz, &dcm_offset_axis);
  Vec3 center_axis;
  Mat3Vec3Mult(&dcm_offset_axis, &kVec3Z, &center_axis);
  Vec3 deflection_axis_detwist0;
  Vec3Cross(&center_axis, &tether_dir_detwist0, &deflection_axis_detwist0);

  // The "tether detwist angle" aligns the deflection axis with the GSG yoke
  // axis. It is only updated if the tether direction is outside the hold cone.
  // Doing so avoids the singularity when the tether departure is aligned with
  // the detwist axis.
  // TODO(b/136996258): This code does not always update the output argument.
  *updated = false;
  if (Asin(Vec3Norm(&deflection_axis_detwist0)) >= hold_cone_half_angle) {
    *tether_detwist_angle =
        Wrap(atan2(deflection_axis_detwist0.y, deflection_axis_detwist0.x), 0.0,
             2.0 * PI);
    *updated = true;
  }
}

// Estimate tether elevation using gsg measurements.
void CalcTetherAnglesFromGsg(
    const Mat3 *dcm_g2p, double drum_position, double detwist_ele,
    double detwist_angle, double gsg_yoke, double gsg_termination,
    double hold_cone_half_angle, double detwist_axis_offset,
    double *tether_elevation_p, double *tether_elevation_g,
    double *tether_detwist_angle, double *last_detwist_angle,
    double *accumulated_detwist_angle) {
  Vec3 tether_dir_p;
  CalcTetherVecPFromGsg(drum_position, detwist_ele, detwist_angle, gsg_yoke,
                        gsg_termination, &tether_dir_p);
  *tether_elevation_p = VecVToElevation(&tether_dir_p);

  // TODO(b/136996258): This code does not always update the output argument.
  bool updated;
  CalcAlignedDetwistAngle(&tether_dir_p, drum_position, detwist_ele,
                          hold_cone_half_angle, detwist_axis_offset,
                          tether_detwist_angle, &updated);

  Vec3 tether_dir_g;
  Mat3TransVec3Mult(dcm_g2p, &tether_dir_p, &tether_dir_g);
  *tether_elevation_g = VecGToElevation(&tether_dir_g);

  // Update the accummulated detwist angle.
  double delta_detwist_angle =
      Wrap(*last_detwist_angle - detwist_angle, -PI, PI);
  *accumulated_detwist_angle += delta_detwist_angle;
  *last_detwist_angle = detwist_angle;
}

void CalcTetherAnglesFromLevelwind(const Mat3 *dcm_ned_to_platform,
                                   double levelwind_ele,
                                   double *tether_elevation_p,
                                   double *tether_elevation_g) {
  Vec3 tether_hat_g, tether_hat_p;
  SphToCart(0.5 * PI, -levelwind_ele, 1.0, &tether_hat_p);
  Mat3TransVec3Mult(dcm_ned_to_platform, &tether_hat_p, &tether_hat_g);
  *tether_elevation_g = VecGToElevation(&tether_hat_g);
  *tether_elevation_p = levelwind_ele;
}

double PressureToAltitude(double pressure, const PhysParams *params) {
  return (params->P_atm - pressure) / (params->rho * params->g);
}

double CalcDryAirDensity(double pressure_pascals, double temperature_celcius) {
  const PhysParams *const phys = g_sys.phys;

  // Absolute temperature [Kelvin].
  const double T = temperature_celcius + 273.15;

  // Density of dry air [kg/m^3].
  const double rho_dry_air = pressure_pascals / (phys->R_dry_air * T);

  return rho_dry_air;
}

// Saturation partial pressure of water vapor [Pa] using Tetens' Formula,
// given temperature in degrees Celcius.
static double CalcSaturationVaporPressure(double temperature_celcius) {
  const double c[] = {6.1078, 7.5, 237.3};
  const double p_sat = 100.0 * c[0] * Exp10((c[1] * temperature_celcius) /
                                            (c[2] + temperature_celcius));
  return p_sat;
}

double CalcAirDensity(double pressure_pascals, double temperature_celcius,
                      double relative_humidity, bool *valid) {
  if (valid != NULL) {
    // Sanity check that the arguments are in a meteorologically
    // plausible range; this function is actually correct over a much
    // larger range of inputs.
    *valid = (0.0 <= relative_humidity && relative_humidity <= 1.0) &&
             (-40.0 <= temperature_celcius && temperature_celcius <= 70.0) &&
             (500e2 <= pressure_pascals && pressure_pascals <= 1500e2);
  }

  const PhysParams *const phys = g_sys.phys;

  // Absolute temperature [Kelvin].
  const double T = temperature_celcius + 273.15;

  // Saturation partial pressure of water vapor [Pa].
  const double p_sat = CalcSaturationVaporPressure(temperature_celcius);

  // Partial pressure of water vapor [Pa].
  const double p_v = relative_humidity * p_sat;

  // Partial pressure of dry air [Pa].
  const double p_d = pressure_pascals - p_v;

  // Density [kg/m^3], accounting for effect of humidity.
  const double rho =
      p_d / (phys->R_dry_air * T) + p_v / (phys->R_water_vapor * T);

  return rho;
}

static Vec3 *CalcGsgPosPlatformAtDrumAngle(Vec3 *gsg_pos_p, double drum_angle) {
  Mat3 dcm_platform2drum;
  AngleToDcm(0.0, 0.0, drum_angle, kRotationOrderZyx, &dcm_platform2drum);
  Vec3 temp;
  Vec3Add(&GetSystemParams()->ground_station.gs02.drum_origin_p,
          Mat3TransVec3Mult(
              &dcm_platform2drum,
              &GetSystemParams()->ground_station.gs02.gsg_pos_drum, &temp),
          gsg_pos_p);
  return gsg_pos_p;
}

void CalcTetherAnchorPoint(double winch_pos, const Mat3 *dcm_g2p,
                           const Vec3 *vessel_g, Vec3 *tether_anchor_g) {
  double drum_angle = winch_pos / GetSystemParams()->winch.r_drum;

  Vec3 tether_anchor_p, temp;
  if (drum_angle <=
      GetSystemParams()->ground_station.gs02.drum_angles.racetrack_high) {
    // Before the tether is fully paid out:
    // Estimate the anchor point at perch and at full payout.
    Vec3 perched_anchor_p =
        GetSystemParams()->ground_station.gs02.drum_origin_p;
    perched_anchor_p.z -= GetSystemParams()->ground_station.gs02.drum_radius;

    Vec3 gsg_pos_farthest_reel_p;
    Vec3 farthest_reel_anchor_p = *CalcGsgPosPlatformAtDrumAngle(
        &gsg_pos_farthest_reel_p,
        GetSystemParams()->ground_station.gs02.drum_angles.racetrack_low);
    farthest_reel_anchor_p.z = perched_anchor_p.z;

    // Crossfade the reel anchor point between perch and farthest reel.
    double perched_drum_angle =
        -g_sys.tether->length / GetSystemParams()->winch.r_drum;
    Vec3 reel_anchor_p;
    CrossfadeVec3(
        &perched_anchor_p, &farthest_reel_anchor_p, drum_angle,
        perched_drum_angle,
        GetSystemParams()->ground_station.gs02.drum_angles.racetrack_low,
        &reel_anchor_p);

    // Crossfade between the reel anchor point to full payout anchor point,
    // to account for the final unwrap during transform.
    Vec3 gsg_pos_p_full_payout;
    CalcGsgPosPlatformAtDrumAngle(
        &gsg_pos_p_full_payout,
        GetSystemParams()->ground_station.gs02.drum_angles.racetrack_high);
    CrossfadeVec3(
        &reel_anchor_p, &gsg_pos_p_full_payout, drum_angle,
        GetSystemParams()->ground_station.gs02.drum_angles.racetrack_low,
        GetSystemParams()->ground_station.gs02.drum_angles.racetrack_high,
        &tether_anchor_p);
  } else {
    // After the tether is fully paid out:
    // Compute the GSG position.
    Mat3 dcm_platform2drum;
    AngleToDcm(0.0, 0.0, drum_angle, kRotationOrderZyx, &dcm_platform2drum);
    Vec3Add(&GetSystemParams()->ground_station.gs02.drum_origin_p,
            Mat3TransVec3Mult(
                &dcm_platform2drum,
                &GetSystemParams()->ground_station.gs02.gsg_pos_drum, &temp),
            &tether_anchor_p);
  }

  InversePoseTransform(dcm_g2p, vessel_g, &tether_anchor_p, tether_anchor_g);
}

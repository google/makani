// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <limits>

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/ground_frame.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"
#include "sim/models/sensors/loadcell.h"

using ::test_util::Rand;

TEST(VesselHeadingPerchHeadingToPerchAzi, Range) {
  for (double vessel_heading = 0.0; vessel_heading < 2.0 * M_PI;
       vessel_heading += 0.1) {
    for (double perch_heading = 0.0; perch_heading < 2.0 * M_PI;
         perch_heading += 0.1) {
      double perch_azi =
          VesselHeadingPerchHeadingToPerchAzi(vessel_heading, perch_heading);
      EXPECT_LE(-PI, perch_azi);
      EXPECT_GE(PI, perch_azi);
      EXPECT_NEAR(
          0.0, remainder(perch_azi - perch_heading + vessel_heading, 2.0 * PI),
          1e-9);
    }
  }
}

#if defined(NDEBUG)

TEST(VesselHeadingPerchHeadingToPerchAzi, OutOfRange) {
  for (double vessel_heading = -6.0 * M_PI; vessel_heading < 6.0 * M_PI;
       vessel_heading += 0.1) {
    for (double perch_heading = -6.0 * M_PI; perch_heading < 2.0 * M_PI;
         perch_heading += 0.1) {
      double perch_azi =
          VesselHeadingPerchHeadingToPerchAzi(vessel_heading, perch_heading);

      EXPECT_LE(-PI, perch_azi);
      EXPECT_GE(PI, perch_azi);
      EXPECT_NEAR(
          0.0, remainder(perch_azi - perch_heading + vessel_heading, 2.0 * PI),
          1e-9);
    }
  }
}

#endif  // defined(NDEBUG)

TEST(PressureToAltitudeTest, Zero0) {
  const double out = PressureToAltitude(g_sys.phys->P_atm, g_sys.phys);
  EXPECT_EQ(out, 0);
}

TEST(ConvSigmaEcefToLocal, SanityCheck_equator) {
  const Vec3 sigma_X_ecef = {2.0, 1.0, 1.0};
  const Vec3 sigma_V_ecef = {2.0, 1.0, 1.0};
  Vec3 gs_pos_ecef = {1e6, 0.0, 0.0};  // At the equator with 0 longitude.
  double g_heading = 22.0;

  Vec3 sigma_Xg, sigma_Vg;
  Mat3 dcm_ecef2g;
  CalcDcmEcefToG(&gs_pos_ecef, g_heading, &dcm_ecef2g);
  ConvSigmaEcefToLocal(&sigma_X_ecef, &sigma_V_ecef, &dcm_ecef2g, &sigma_Xg,
                       &sigma_Vg);

  Vec3 expected_sigma_Xg = {1.0, 1.0, 2.0};
  Vec3 expected_sigma_Vg = {1.0, 1.0, 2.0};
  EXPECT_NEAR_VEC3(sigma_Xg, expected_sigma_Xg, 1e-9);
  EXPECT_NEAR_VEC3(sigma_Vg, expected_sigma_Vg, 1e-9);
}

TEST(ConvSigmaEcefToLocal, SanityCheck_equator_90_long) {
  const Vec3 sigma_X_ecef = {0.0, 2.0, 0.0};
  const Vec3 sigma_V_ecef = {0.0, 2.0, 0.0};
  // At the equator with 90 deg. longitude.
  Vec3 gs_pos_ecef = {0.0, 1e6, 0.0};
  double g_heading = 22.0;

  Vec3 sigma_Xg, sigma_Vg;
  Mat3 dcm_ecef2g;
  CalcDcmEcefToG(&gs_pos_ecef, g_heading, &dcm_ecef2g);
  ConvSigmaEcefToLocal(&sigma_X_ecef, &sigma_V_ecef, &dcm_ecef2g, &sigma_Xg,
                       &sigma_Vg);

  Vec3 expected_sigma_Xg = {0.0, 0.0, 2.0};
  Vec3 expected_sigma_Vg = {0.0, 0.0, 2.0};
  EXPECT_NEAR_VEC3(sigma_Xg, expected_sigma_Xg, 1e-9);
  EXPECT_NEAR_VEC3(sigma_Vg, expected_sigma_Vg, 1e-9);
}

TEST(ApparentWindCartToSph, Recovery) {
  for (int32_t i = 0; i < 22; ++i) {
    ApparentWindSph sph = {Rand(1.0, 100.0), Rand(-PI / 2.0, PI / 2.0),
                           Rand(-PI / 2.0, PI / 2.0)};
    Vec3 V_app_b;
    ApparentWindSphToCart(&sph, &V_app_b);

    ApparentWindSph sph_out;
    ApparentWindCartToSph(&V_app_b, &sph_out);

    EXPECT_NEAR(sph.v, sph_out.v, 1e-9);
    EXPECT_NEAR(sph.alpha, sph_out.alpha, 1e-9);
    EXPECT_NEAR(sph.beta, sph_out.beta, 1e-9);
  }
}

TEST(PitotToApparentWindSph, Recovery) {
  PitotParams pitot_params = GetSystemParams()->pitot;
  pitot_params.dcm_b2p = kMat3Identity;

  ApparentWindSph sph;
  for (sph.v = 10.0; sph.v < 60.0; sph.v += 5.0) {
    for (sph.alpha = -0.34; sph.alpha < 0.34; sph.alpha += 0.02) {
      for (sph.beta = -0.34; sph.beta < 0.34; sph.beta += 0.02) {
        double q = 0.5 * g_sys.phys->rho * sph.v * sph.v;
        double c_alpha = cos(sph.alpha);
        double c_beta = cos(sph.beta);
        PitotDifferentialData diff;
        diff.dyn_press =
            q *
            (1.0 - (9.0 / 4.0) * (1.0 - c_alpha * c_alpha * c_beta * c_beta));
        diff.alpha_press = q * (9.0 / 4.0) *
                           sin(2.0 * pitot_params.port_angle) * c_beta *
                           c_beta * sin(2.0 * sph.alpha);
        diff.beta_press = q * (9.0 / 4.0) * sin(2.0 * pitot_params.port_angle) *
                          c_alpha * sin(2.0 * sph.beta);

        ApparentWindSph sph_out;
        PitotToApparentWindSph(&diff, &kVec3Zero, &pitot_params, &sph_out);
        EXPECT_NEAR(sph.v, sph_out.v, 1e-3);
        EXPECT_NEAR(sph.alpha, sph_out.alpha, 1e-3);
        EXPECT_NEAR(sph.beta, sph_out.beta, 1e-3);
      }
    }
  }
}

TEST(TetherForceCartToSph, Recovery) {
  for (int32_t i = 0; i < 22; ++i) {
    TetherForceSph sph = {Rand(1.0, 100.0), Rand(-PI / 2.0, PI / 2.0),
                          Rand(-PI / 2.0, PI / 2.0)};
    Vec3 Fb_tether;
    TetherForceSphToCart(&sph, &Fb_tether);

    TetherForceSph sph_out;
    TetherForceCartToSph(&Fb_tether, &sph_out);

    EXPECT_NEAR(sph.tension, sph_out.tension, 1e-9);
    EXPECT_NEAR(sph.pitch, sph_out.pitch, 1e-9);
    EXPECT_NEAR(sph.roll, sph_out.roll, 1e-9);
  }
}

TEST(XgsgToXp, Recovery) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) {
    return;
  }

  for (int32_t i = 0; i < 222; ++i) {
    double drum_angle = Rand(-6.3, 6.3);
    GsgData gsg = {Rand(-PI / 2.0, PI / 2.0), Rand(-PI / 2.0, PI / 2.0)};
    GsgData gsg_out;
    Vec3 X_gsg = {g_sys.tether->length + g_sys.wing->bridle_rad, 0.0, 0.0};
    Vec3 Xp;
    XgsgToXp(&gsg, &X_gsg, drum_angle, &Xp);
    XpToGsg(&Xp, drum_angle, &gsg_out);

    EXPECT_NEAR(gsg.azi, gsg_out.azi, 1e-9);
    EXPECT_NEAR(gsg.ele, gsg_out.ele, 1e-9);
  }
}

TEST(XpToGsg, ZeroRot_Xm) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) return;

  const PerchParams *perch_params = &GetSystemParams()->perch;
  Vec3 Xp = {-100.0, 0.0, perch_params->gsg_pos_wd.z};
  GsgData gsg;
  XpToGsg(&Xp, 0.0, &gsg);
  EXPECT_NEAR(gsg.azi, 0.0, 1e-9);
  EXPECT_NEAR(gsg.ele, 0.0, 1e-9);
}

// The following tests assumes that the GSG frame is in a specific
// orientation to the winch and perch frames.
TEST(XgsgToXp, ZeroRot_Xm) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) return;

  GsgData gsg = {0.0, 0.0};
  Vec3 X_gsg = {g_sys.tether->length + g_sys.wing->bridle_rad, 0.0, 0.0};
  Vec3 Xp;
  XgsgToXp(&gsg, &X_gsg, 0.0, &Xp);

  const PerchParams *perch_params = &GetSystemParams()->perch;
  EXPECT_NEAR(Xp.x, -g_sys.tether->length - g_sys.wing->bridle_rad +
                        perch_params->gsg_pos_wd.x +
                        perch_params->winch_drum_origin_p.x,
              1e-9);
  EXPECT_NEAR(Xp.y, 0.0, 1e-9);
  EXPECT_NEAR(Xp.z, perch_params->gsg_pos_wd.z, 1e-9);
}

TEST(XgsgToXp, ZeroRot_Zm) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) return;

  GsgData gsg = {0.0, PI / 2.0};
  Vec3 X_gsg = {g_sys.tether->length + g_sys.wing->bridle_rad, 0.0, 0.0};
  Vec3 Xp;
  XgsgToXp(&gsg, &X_gsg, 0.0, &Xp);

  const PerchParams *perch_params = &GetSystemParams()->perch;
  EXPECT_NEAR(Xp.x,
              perch_params->gsg_pos_wd.x + perch_params->winch_drum_origin_p.x,
              1e-9);
  EXPECT_NEAR(Xp.y, 0.0, 1e-9);
  EXPECT_NEAR(Xp.z, -g_sys.tether->length - g_sys.wing->bridle_rad +
                        perch_params->gsg_pos_wd.z,
              1e-9);
}

TEST(XgsgToXp, ZeroRot_Yp) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) return;

  GsgData gsg = {-PI / 2.0, 0.0};
  Vec3 X_gsg = {g_sys.tether->length + g_sys.wing->bridle_rad, 0.0, 0.0};
  Vec3 Xp;
  XgsgToXp(&gsg, &X_gsg, 0.0, &Xp);

  const PerchParams *perch_params = &GetSystemParams()->perch;
  EXPECT_NEAR(Xp.x,
              perch_params->gsg_pos_wd.x + perch_params->winch_drum_origin_p.x,
              1e-9);
  EXPECT_NEAR(Xp.y, g_sys.tether->length + g_sys.wing->bridle_rad, 1e-9);
  EXPECT_NEAR(Xp.z, perch_params->gsg_pos_wd.z, 1e-9);
}

// The "critical angle", which is a term only used in this file, is the angle
// between the loadcell-to-loadcell vector and the -z-direction of the loadcell
// frame. See \theta as defined in
//     go/makani-doc/controls/tension_estimation.md#tension-estimation.
//
// The geometry of the problem changes based on the relationship between
// the critical angle and the bridle angle. This function lets us ensure
// that we test both configurations, with focus on the port side since we're
// more interested in large bridle offsets in that direction.
bool PortCriticalAngleExceedsBridleAngle(const WingParams &wing_params,
                                         const LoadcellParams *loadcell_params,
                                         double bridle_y_offset) {
  const Vec3 &port_pos_b = wing_params.bridle_pos[kBridlePort];
  const Vec3 &star_pos_b = wing_params.bridle_pos[kBridleStar];
  Vec3 nominal_knot_pos_b = {
      (port_pos_b.x + star_pos_b.x) / 2.0, bridle_y_offset,
      wing_params.bridle_rad + (port_pos_b.z + star_pos_b.z) / 2.0};
  Vec3 star_from_port_b;
  Vec3Sub(&star_pos_b, &port_pos_b, &star_from_port_b);
  Vec3 knot_from_port_b;
  Vec3Sub(&nominal_knot_pos_b, &port_pos_b, &knot_from_port_b);
  double port_bridle_angle =
      Acos(Vec3Dot(&knot_from_port_b, &star_from_port_b) /
           Vec3Norm(&knot_from_port_b) / Vec3Norm(&star_from_port_b));

  Vec3 star_from_port_p;
  Mat3TransVec3Mult(&loadcell_params[kBridlePort].dcm_loadcell2b,
                    &star_from_port_b, &star_from_port_p);
  double port_critical_angle =
      Acos(-star_from_port_p.z / Vec3Norm(&star_from_port_p));

  return port_critical_angle > port_bridle_angle;
}

TEST(LoadcellsToTetherForceTest, TensionRecovery) {
  WingParams wing_params = GetSystemParams()->wing;
  const LoadcellParams *loadcell_params = GetSystemParams()->loadcells;

  // Save the base bridle offset so we can be sure to test this in addition to
  // the offsets defined below.
  double base_y_offset = wing_params.bridle_y_offset;

  // Test the test cases! See descrption of PortCriticalAngleExceedsBridleAngle.
  double small_y_offset = -0.5;
  ASSERT_TRUE(PortCriticalAngleExceedsBridleAngle(wing_params, loadcell_params,
                                                  small_y_offset));
  double large_y_offset = -2.0;
  ASSERT_FALSE(PortCriticalAngleExceedsBridleAngle(wing_params, loadcell_params,
                                                   large_y_offset));

  const double deg_to_rad = M_PI / 180.0;
  double max_tension_exp = 6.0;
  double min_tension_exp = 0.0;
  int32_t num_tensions = 10;

  double max_pitch = 30.0 * deg_to_rad;
  double min_pitch = -30.0 * deg_to_rad;
  int32_t num_pitches = 10;

  double max_roll = 30.0 * deg_to_rad;
  double min_roll = -30.0 * deg_to_rad;
  int32_t num_rolls = 10;

  for (double bridle_y_offset :
       {base_y_offset, small_y_offset, large_y_offset}) {
    wing_params.bridle_y_offset = bridle_y_offset;
    for (int32_t i = 0; i < num_tensions; ++i) {
      for (int32_t j = 0; j < num_pitches; ++j) {
        for (int32_t k = 0; k < num_rolls; ++k) {
          TetherForceSph sph;
          sph.tension = pow(10, min_tension_exp +
                                    i * (max_tension_exp - min_tension_exp) /
                                        (num_tensions - 1));
          sph.roll = min_roll + k * (max_roll - min_roll) / (num_rolls - 1);
          sph.pitch =
              min_pitch + j * (max_pitch - min_pitch) / (num_pitches - 1);

          Vec3 tether_force_b;
          TetherForceSphToCart(&sph, &tether_force_b);

          double loadcell_forces[kNumLoadcellSensors];
          sim::TetherForceToLoadcells(wing_params, loadcell_params,
                                      tether_force_b, loadcell_forces);

          Vec3 est_tether_force_b, est_bridle_port_vector_b,
              est_bridle_star_vector_b;
          TetherForceSph est_sph;
          LoadcellsToTetherForce(loadcell_forces, &wing_params, loadcell_params,
                                 &est_tether_force_b, &est_sph,
                                 &est_bridle_port_vector_b,
                                 &est_bridle_star_vector_b);

          // The accuracy of reconstruction is impacted by the tolerance with
          // which orthonormality is enforced for the basis vectors that form
          // the parameters' dcm_loadcell2b matrices.
          double tol = 1e-5 * sph.tension;

          EXPECT_NEAR_VEC3(tether_force_b, est_tether_force_b, tol);
          EXPECT_NEAR(sph.tension, est_sph.tension, tol);
          EXPECT_NEAR(sph.roll, est_sph.roll, tol);
          EXPECT_NEAR(sph.pitch, est_sph.pitch, tol);
        }
      }
    }
  }
}

TEST(CalcTetherAnglesFromGsg, DeflectionAlreadyInNegativeYokeDirection) {
  // For any detwist angle, if the tether deflection is entirely in the negative
  // yoke direction, then the detwist is already in the appropriate position.
  double drum_angle = 0.0;
  double detwist_ele = 0.0;
  double last_detwist_angle = 0.0;
  double hold_cone_half_angle = 0.0;
  double detwist_axis_offset = 0.0;
  double tether_elevation_g, tether_elevation_p, tether_detwist_angle;
  double integrated_detwist_angle;
  for (double detwist_angle = 0.0; detwist_angle < 2.0 * PI;
       detwist_angle += 0.1) {
    CalcTetherAnglesFromGsg(&kMat3Identity, drum_angle, detwist_ele,
                            detwist_angle, -0.1, 0.0, hold_cone_half_angle,
                            detwist_axis_offset, &tether_elevation_g,
                            &tether_elevation_p, &tether_detwist_angle,
                            &last_detwist_angle, &integrated_detwist_angle);
    EXPECT_NEAR(detwist_angle, tether_detwist_angle, 1e-8);
    EXPECT_NEAR(integrated_detwist_angle, -detwist_angle, 1e-8);
  }
}

TEST(CalcTetherAnglesFromGsg, SimpleCases) {
  // These cases all use simplified geometry, so they're easy to picture and
  // have intuitive answers. Tether deflections are described from the
  // perspective of the GSG, looking along the detwist axis in the direction of
  // the kite.
  double drum_angle = 0.0;
  double detwist_ele = 0.0;
  double detwist_angle = 0.0;
  double last_detwist_angle = 0.0;
  double hold_cone_half_angle = 0.0;
  double detwist_axis_offset = 0.0;
  double tether_elevation_g, tether_elevation_p, tether_detwist_angle;
  double integrated_detwist_angle;

  // The tether is deflected 0.1 rad to the left. Elevation is zero, and the
  // deflection is already fully in the yoke DOF.
  CalcTetherAnglesFromGsg(&kMat3Identity, drum_angle, detwist_ele,
                          detwist_angle, -0.1, 0.0, hold_cone_half_angle,
                          detwist_axis_offset, &tether_elevation_g,
                          &tether_elevation_p, &tether_detwist_angle,
                          &last_detwist_angle, &integrated_detwist_angle);
  EXPECT_NEAR(0.0, tether_elevation_g, 1e-8);
  EXPECT_NEAR(0.0, tether_elevation_p, 1e-8);
  EXPECT_NEAR(0.0, tether_detwist_angle, 1e-8);
  EXPECT_NEAR(0.0, integrated_detwist_angle, 1e-8);

  CalcTetherAnglesFromGsg(&kMat3Identity, drum_angle, detwist_ele,
                          DegToRad(270.0), -0.1, 0.0, hold_cone_half_angle,
                          detwist_axis_offset, &tether_elevation_g,
                          &tether_elevation_p, &tether_detwist_angle,
                          &last_detwist_angle, &integrated_detwist_angle);
  EXPECT_NEAR(0.1, tether_elevation_g, 1e-8);
  EXPECT_NEAR(0.1, tether_elevation_p, 1e-8);
  EXPECT_NEAR(DegToRad(270.0), tether_detwist_angle, 1e-8);
  EXPECT_NEAR(DegToRad(90.0), integrated_detwist_angle, 1e-8);

  // The tether is deflected 0.1 rad up. The elevation is 0.1 rad, and the
  // detwist should rotate to 270 deg to place all deflection in the yoke DOF.
  CalcTetherAnglesFromGsg(&kMat3Identity, drum_angle, detwist_ele,
                          detwist_angle, 0.0, 0.1, hold_cone_half_angle,
                          detwist_axis_offset, &tether_elevation_g,
                          &tether_elevation_p, &tether_detwist_angle,
                          &last_detwist_angle, &integrated_detwist_angle);
  EXPECT_NEAR(0.1, tether_elevation_g, 1e-8);
  EXPECT_NEAR(0.1, tether_elevation_p, 1e-8);
  EXPECT_NEAR(DegToRad(270.0), tether_detwist_angle, 1e-8);
  EXPECT_NEAR(0.0, integrated_detwist_angle, 1e-8);

  // The tether is deflected 0.1 rad to the left and 0.1 rad up. The elevation
  // is 0.1 rad. The ideal detwist angle is approximately 315 deg, but not
  // precisely, due to non-commutativity of rotations.
  CalcTetherAnglesFromGsg(&kMat3Identity, drum_angle, detwist_ele,
                          detwist_angle, -0.1, 0.1, hold_cone_half_angle,
                          detwist_axis_offset, &tether_elevation_g,
                          &tether_elevation_p, &tether_detwist_angle,
                          &last_detwist_angle, &integrated_detwist_angle);
  EXPECT_NEAR(0.1, tether_elevation_g, 1e-8);
  EXPECT_NEAR(0.1, tether_elevation_p, 1e-8);
  EXPECT_NEAR(DegToRad(315.0), tether_detwist_angle, 1e-2);
  EXPECT_NEAR(0.0, integrated_detwist_angle, 1e-8);
}

TEST(CalcTetherAnglesFromGsg, HoldCone) {
  double drum_angle = 0.0;
  double detwist_ele = 0.0;
  double detwist_angle = 0.0;
  double hold_cone_half_angle = 0.05;
  double detwist_axis_offset = 0.0;
  double tether_elevation_g, tether_elevation_p, tether_detwist_angle;
  double integrated_detwist_angle, last_detwist_angle;

  // The tether is deflected straight up, outside the hold cone. The tether
  // detwist angle is 270 deg.
  CalcTetherAnglesFromGsg(
      &kMat3Identity, drum_angle, detwist_ele, detwist_angle, 0.0,
      2.0 * hold_cone_half_angle, hold_cone_half_angle, detwist_axis_offset,
      &tether_elevation_g, &tether_elevation_p, &tether_detwist_angle,
      &last_detwist_angle, &integrated_detwist_angle);
  EXPECT_NEAR(DegToRad(270.0), tether_detwist_angle, 1e-8);

  // The tether now deflects to the left, but inside the hold cone. The tether
  // detwist angle is held.
  CalcTetherAnglesFromGsg(
      &kMat3Identity, drum_angle, detwist_ele, detwist_angle,
      -hold_cone_half_angle / 2.0, 0.0, hold_cone_half_angle,
      detwist_axis_offset, &tether_elevation_g, &tether_elevation_p,
      &tether_detwist_angle, &last_detwist_angle, &integrated_detwist_angle);
  EXPECT_NEAR(DegToRad(270.0), tether_detwist_angle, 1e-8);

  // The tether deflects to the left, outside the hold cone. The tether detwist
  // angle updates to zero.
  CalcTetherAnglesFromGsg(
      &kMat3Identity, drum_angle, detwist_ele, detwist_angle,
      -2.0 * hold_cone_half_angle, 0.0, hold_cone_half_angle,
      detwist_axis_offset, &tether_elevation_g, &tether_elevation_p,
      &tether_detwist_angle, &last_detwist_angle, &integrated_detwist_angle);
  EXPECT_NEAR(0.0, tether_detwist_angle, 1e-8);
}

TEST(CalcDryAirDensity, SampleValues) {
  // Table from https://en.wikipedia.org/wiki/Density_of_air
  EXPECT_NEAR(CalcDryAirDensity(101325.0, 35.0), 1.1455, 1e-4);
  EXPECT_NEAR(CalcDryAirDensity(101325.0, 15.0), 1.2250, 1e-4);
  EXPECT_NEAR(CalcDryAirDensity(101325.0, 0.0), 1.2922, 1e-4);
  EXPECT_NEAR(CalcDryAirDensity(101325.0, -25.0), 1.4224, 1e-4);

  // IUPAC standard temperature and pressure.
  EXPECT_NEAR(CalcDryAirDensity(100e3, 0.0), 1.2754, 1e-4);

  // https://en.wikipedia.org/wiki/Standard_sea_level
  EXPECT_NEAR(CalcDryAirDensity(101325.0, 15.0), 1.2250, 1e-4);

  // Current weather in New Orleans LA.
  EXPECT_NEAR(CalcDryAirDensity(101557.8, 32.8), 1.1564, 1e-4);
}

extern "C" {
double CalcSaturationVaporPressure(double temperature_celcius);
}

TEST(CalcSaturationVaporPressure, SampleValues) {
  // Table from http://biomet.ucdavis.edu/conversions/HumCon.pdf
  EXPECT_NEAR(CalcSaturationVaporPressure(-10.0), 286., 1.);
  EXPECT_NEAR(CalcSaturationVaporPressure(-0.0), 611., 1.);
  EXPECT_NEAR(CalcSaturationVaporPressure(0.0), 611., 1.);
  EXPECT_NEAR(CalcSaturationVaporPressure(10.0), 1228., 1.);
  EXPECT_NEAR(CalcSaturationVaporPressure(20.0), 2338., 1.);
  EXPECT_NEAR(CalcSaturationVaporPressure(30.0), 4243., 1.);
  EXPECT_NEAR(CalcSaturationVaporPressure(40.0), 7376., 2.);
  EXPECT_NEAR(CalcSaturationVaporPressure(49.0), 11737., 2.);
}

TEST(CalcAirDensity, SampleValues) {
  // Table from https://en.wikipedia.org/wiki/Density_of_air
  EXPECT_NEAR(CalcAirDensity(101325.0, 35.0, 0.0, NULL), 1.1455, 1e-4);
  EXPECT_NEAR(CalcAirDensity(101325.0, 15.0, 0.0, NULL), 1.2250, 1e-4);
  EXPECT_NEAR(CalcAirDensity(101325.0, 0.0, 0.0, NULL), 1.2922, 1e-4);
  EXPECT_NEAR(CalcAirDensity(101325.0, -25.0, 0.0, NULL), 1.4224, 1e-4);

  // IUPAC standard temperature and pressure.
  bool valid;
  EXPECT_NEAR(CalcAirDensity(100e3, 0.0, 0.0, &valid), 1.2754, 1e-4);
  EXPECT_TRUE(valid);

  // Current weather in New Orleans LA (91 deg F, 50% humidity).
  EXPECT_NEAR(CalcAirDensity(101557.8, 32.8, 0.5, &valid),
              0.99 * CalcDryAirDensity(101557.8, 32.8), 1e-3);
  EXPECT_TRUE(valid);

  // Check parameter validation.
  CalcAirDensity(0.0, 0.0, 0.0, &valid);
  EXPECT_FALSE(valid);

  CalcAirDensity(101.325, 15.0, 0.0, &valid);
  EXPECT_FALSE(valid);

  CalcAirDensity(101325.0, 15.0, std::numeric_limits<double>::quiet_NaN(),
                 &valid);
  EXPECT_FALSE(valid);
}

TEST(WindWsToWindG, Recovery) {
  // Simple test with the wind sensor located at the origin, not moving.

  WindSensorParams params = WindSensorParams();
  params.on_perch = true;
  params.pos_parent = kVec3Zero;
  params.dcm_parent2ws = kMat3Identity;

  const Vec3 wind_ws = {10.0, 5.5, 2.2};
  const Mat3 dcm_g2p = kMat3Identity;
  const Vec3 omega_g2p = kVec3Zero;
  const Vec3 vel_g = kVec3Zero;

  Vec3 wind_g;
  WindWsToWindG(&wind_ws, &dcm_g2p, &omega_g2p, &vel_g, &params, &wind_g);
  EXPECT_NEAR_VEC3(wind_ws, wind_g, 1e-9);
}

TEST(WindWsToWindG, VesselVelocity) {
  // Simple test with the wind sensor located at the origin, on a moving vessel.

  WindSensorParams params = WindSensorParams();
  params.on_perch = true;
  params.pos_parent = kVec3Zero;
  params.dcm_parent2ws = kMat3Identity;

  const Vec3 wind_ws = kVec3Zero;
  const Mat3 dcm_g2p = kMat3Identity;
  const Vec3 omega_g2p = kVec3Zero;
  const Vec3 vel_g = {3.3, 2.2, 7.7};

  Vec3 wind_g;
  WindWsToWindG(&wind_ws, &dcm_g2p, &omega_g2p, &vel_g, &params, &wind_g);

  // If the wind sensor reads zero, then the vessel motion (vel_g) is
  // equal to the wind velocity.
  EXPECT_NEAR_VEC3(wind_g, vel_g, 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

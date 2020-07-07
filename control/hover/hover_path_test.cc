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

#include <limits.h>
#include <math.h>

#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_playbook.h"
#include "control/crosswind/crosswind_playbook_types.h"
#include "control/ground_frame.h"
#include "control/hover/hover_frame.h"
#include "control/hover/hover_path.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "control/tether_util.h"
#include "lib/util/test_util.h"

extern "C" {

void CalcPayOutReelInPosition(double elevation_cmd, double norm_reel_pos_g,
                              double wind_dir_f, const Vec3 *tether_anchor_g,
                              const Vec3 *fixed_pos_g,
                              const FlightStatus *flight_status,
                              const HoverPathParams *params,
                              HoverPathState *state, Vec3 *reel_pos_g);

void CalcRawPositionCommand(const Vec3 *tether_anchor_g,
                            const Vec3 *vessel_pos_g,
                            const FlightStatus *flight_status,
                            double elevation_cmd, double norm_reel_pos_g,
                            double wind_dir_f, const Vec3 *wing_pos_g,
                            const Vec3 *fixed_pos_g, const Vec3 *perched_pos_g,
                            const Vec3 *accel_start_pos_g,
                            const HoverPathParams *params,
                            HoverPathState *state, Vec3 *wing_pos_g_cmd);

void CalcAccelStartAndCrosswindLoopCenterPosition(const WindEstimate *wind_g,
                                                  const Playbook *playbook,
                                                  const Vec3 *perched_pos_g,
                                                  const HoverPathParams *params,
                                                  Vec3 *accel_start_pos_g,
                                                  Vec3 *loop_center_pos_g);

void SmoothRawPositionCommand(const Vec3 *raw_wing_pos_g_cmd,
                              FlightMode flight_mode,
                              const HoverPathParams *params,
                              HoverPathState *state, Vec3 *wing_pos_g_cmd);

void CalcVelocityCommand(const Vec3 *raw_wing_pos_g_cmd,
                         const Vec3 *wing_pos_g_cmd, double winch_vel,
                         const FlightStatus *flight_status,
                         const Vec3 *wing_vel_g, const HoverPathParams *params,
                         HoverPathState *state, Vec3 *wing_vel_g_cmd);

void CylindricalRateLimit(const Vec3 *wing_pos_g_cmd, double max_tangential_vel,
                          double max_radial_vel, double min_z_vel,
                          double max_z_vel, double ts, Vec3 *wing_pos_g_cmd_z1);

double ConvertArclengthToPosition(double payout, double horizontal_tension_cmd,
                                  double tether_elevation_cmd, double *r_ff,
                                  double *z_ff, double *elevation_ff);

double ConvertPayoutToPosition(double payout, double horizontal_tension_cmd,
                               double tether_elevation_cmd, double *r_ff,
                               double *z_ff, double *elevation_ff);

}  // extern "C"

class AziOffsetOverrider : ::test_util::Overrider<double> {
 public:
  explicit AziOffsetOverrider(double value)
      : Overrider(&GetSystemParamsUnsafe()->ground_station.azi_ref_offset,
                  value) {}
};

class FlightPlanOverrider : ::test_util::Overrider<FlightPlan> {
 public:
  explicit FlightPlanOverrider(FlightPlan value)
      : Overrider(&GetControlParamsUnsafe()->flight_plan, value) {}
};

class AziAllowStartOverrider : ::test_util::Overrider<double> {
 public:
  explicit AziAllowStartOverrider(double value)
      : Overrider(&GetSystemParamsUnsafe()->test_site_params.azi_allow_start,
                  value) {}
};

class AziAllowEndOverrider : ::test_util::Overrider<double> {
 public:
  explicit AziAllowEndOverrider(double value)
      : Overrider(&GetSystemParamsUnsafe()->test_site_params.azi_allow_end,
                  value) {}
};

const Vec3 kWingPosGValues[] = {
    {0.0, 0.0, 0.0},    {-100.0, 0.0, 0.0}, {100.0, 0.0, 0.0},
    {0.0, -100.0, 0.0}, {0.0, 100.0, 0.0},  {0.0, 0.0, -100.0},
    {0.0, 0.0, 100.0},  {10.0, 20.0, 30.0},
};

TEST(HoverPathInit, SameInitialization) {
  HoverPathState state0, state1;
  memset(&state0, 0, sizeof(state0));
  memset(&state1, 0xFF, sizeof(state1));
  HoverPathInit(&kVec3Zero, &kVec3Zero, &GetControlParams()->hover.path,
                &state0);
  HoverPathInit(&kVec3Zero, &kVec3Zero, &GetControlParams()->hover.path,
                &state1);
  EXPECT_TRUE(!memcmp(&state0, &state1, sizeof(state0)));
}

TEST(HoverPathValidateParams, Default) {
  EXPECT_TRUE(HoverPathValidateParams(&GetControlParams()->hover.path));
}

TEST(HoverPathIsAscentComplete, Normal) {
  const HoverPathParams &params = GetControlParams()->hover.path;

  StateEstimate state_est = StateEstimate();

  for (int32_t i = 0; i < kNumFlightModes; ++i) {
    FlightMode flight_mode = static_cast<FlightMode>(i);

    state_est.tether_ground_angles.elevation_valid = true;
    state_est.tether_ground_angles.elevation_p =
        params.target_above_perch_tether_elevation - 1e-5;
    EXPECT_FALSE(HoverPathIsAscentComplete(flight_mode, &params, &state_est));

    state_est.tether_ground_angles.elevation_p =
        params.target_above_perch_tether_elevation;
    if (flight_mode == kFlightModeHoverAscend) {
      EXPECT_TRUE(HoverPathIsAscentComplete(flight_mode, &params, &state_est));
    } else {
      EXPECT_FALSE(HoverPathIsAscentComplete(flight_mode, &params, &state_est));
    }
  }
}

TEST(CalcAccelStartAndCrosswindLoopCenterPosition, DirectlyDownWind) {
  HoverPathParams params = HoverPathParams();
  params.accel_start_elevation = 0.0;

  Playbook playbook = Playbook();
  playbook.num_entries = 1;
  playbook.entries[0].azi_offset = 0.0;
  playbook.entries[0].path_radius_target = 150.0;
  playbook.entries[0].elevation = 0.7;
  playbook.entries[0].wind_speed = 5.0;

  WindEstimate wind_g = WindEstimate();
  wind_g.dir_f = 0.0;
  wind_g.speed_f = 10.0;

  {  // Test the top hat case.
    // GS coodinate system is offset.
    AziOffsetOverrider overrider_offset(M_PI);
    // Clear azimuth allowable limits.
    AziAllowStartOverrider overrider_start(-PI);
    AziAllowEndOverrider overrider_end(PI);
    Vec3 perched_pos_g = {-400.0, 0.0, 0.0};

    Vec3 accel_start_pos_g, crosswind_loop_center_g;
    CalcAccelStartAndCrosswindLoopCenterPosition(
        &wind_g, &playbook, &perched_pos_g, &params, &accel_start_pos_g,
        &crosswind_loop_center_g);

    // Accel start should align with upstroke.
    double half_cone_angle =
        Asin(playbook.entries[0].path_radius_target / g_sys.tether->length);
    double azi_offset = AdjustHalfConeAziOffsetForElevation(
        half_cone_angle, playbook.entries[0].elevation);
    Vec3 expected_accel_start_g;
    SphToVecG(0.0 - azi_offset, 0.0, g_sys.tether->length,
              &expected_accel_start_g);
    EXPECT_NEAR_VEC3(accel_start_pos_g, expected_accel_start_g, 1e-9);

    // Loop center should be located as defined in playbook entry.
    Vec3 expected_loop_center_g;
    SphToVecG(playbook.entries[0].azi_offset, playbook.entries[0].elevation,
              g_sys.tether->length, &expected_loop_center_g);
    EXPECT_NEAR_VEC3(crosswind_loop_center_g, expected_loop_center_g, 1e-9);
  }

  {  // Test the GSv2 case.
    AziOffsetOverrider overrider_offset(0.0);
    // Clear azimuth allowable limits.
    AziAllowStartOverrider overrider_start(-PI);
    AziAllowEndOverrider overrider_end(PI);
    Vec3 unused_perched_pos_g = kVec3Zero;

    Vec3 accel_start_pos_g, crosswind_loop_center_g;
    CalcAccelStartAndCrosswindLoopCenterPosition(
        &wind_g, &playbook, &unused_perched_pos_g, &params, &accel_start_pos_g,
        &crosswind_loop_center_g);
    // Accel start should align with upstroke.
    double half_cone_angle =
        Asin(playbook.entries[0].path_radius_target / g_sys.tether->length);
    double azi_offset = AdjustHalfConeAziOffsetForElevation(
        half_cone_angle, playbook.entries[0].elevation);
    Vec3 expected_accel_start_g = {
        g_sys.tether->length * cos(wind_g.dir_f - azi_offset),
        g_sys.tether->length * sin(wind_g.dir_f - azi_offset), 0.0};
    EXPECT_NEAR_VEC3(accel_start_pos_g, expected_accel_start_g, 1e-9);

    // Loop center should be located as defined in playbook entry.
    Vec3 expected_loop_center_g;
    SphToVecG(playbook.entries[0].azi_offset, playbook.entries[0].elevation,
              g_sys.tether->length, &expected_loop_center_g);
    EXPECT_NEAR_VEC3(crosswind_loop_center_g, expected_loop_center_g, 1e-9);
  }
}

TEST(HoverPathCalcElevationLimits, Normal) {
  const HoverPathParams &params = GetControlParams()->hover.path;
  double elevation_min, elevation_max;
  HoverPathCalcElevationLimits(0.0, &params, &elevation_min, &elevation_max);
  EXPECT_NEAR(elevation_min, params.launch_perch_elevation_min, 1e-9);
  EXPECT_NEAR(elevation_max, params.launch_perch_elevation_max, 1e-9);

  HoverPathCalcElevationLimits(g_sys.tether->length, &params, &elevation_min,
                               &elevation_max);
  EXPECT_NEAR(elevation_min, params.reel_elevation_min, 1e-9);
  EXPECT_NEAR(elevation_max, params.reel_elevation_max, 1e-9);
}

// The reel-in target position should intersect, and be saturated at,
// the ascent target position at the perching length.
TEST(CalcPayOutReelInPosition, AtAscentTarget) {
  // Set params to GSv2 values.
  FlightPlanOverrider flight_plan_overrider(kFlightPlanTurnKey);
  AziOffsetOverrider azi_offset_overrider(0.0);
  // Clear azimuth allowable limits so they don't affect expected position.
  AziAllowStartOverrider overrider_start(-PI);
  AziAllowEndOverrider overrider_end(PI);

  HoverPathState state;
  FlightStatus flight_status;
  flight_status.flight_mode = kFlightModeHoverPayOut;
  flight_status.flight_mode_first_entry = false;

  const HoverPathParams *params = &GetControlParams()->hover.path;

  for (double horizontal_tension_cmd = 5e3; horizontal_tension_cmd < 15e3;
       horizontal_tension_cmd += 1e3) {
    for (double wind_dir_f = -M_PI; wind_dir_f < M_PI; wind_dir_f += 0.1) {
      Vec3 reel_pos_g;
      const double payout = 0.0;
      // A min check on reel_pos_g->z in CalcPayOutReelInPosition ensures we
      // don't command below the perch altitude, so a test with zero payout
      // can't use an anchor point below perch height.
      Vec3 tether_anchor_g = {0.0, 0.0, -5.0};
      double r_ff, z_ff, elevation_ff;
      ConvertPayoutToPosition(payout, horizontal_tension_cmd,
                              params->target_above_perch_tether_elevation,
                              &r_ff, &z_ff, &elevation_ff);
      CalcPayOutReelInPosition(elevation_ff, hypot(r_ff, z_ff), wind_dir_f,
                               &tether_anchor_g, &kVec3Zero, &flight_status,
                               params, &state, &reel_pos_g);
      // The target tether elevation angle at the end of ascent should be
      // approximated by the feed-forwarded reel trajectory.
      EXPECT_NEAR(reel_pos_g.z, tether_anchor_g.z, 1e-1);
    }
  }
}

// Tests that the fully reeled-out azimuth calculated by the
// CalcPayOutReelInPosition matches the transform start position.
TEST(CalcPayOutReelInPosition, ReeledOut) {
  // Set params to GSv2 values.
  FlightPlanOverrider flight_plan_overrider(kFlightPlanTurnKey);
  AziOffsetOverrider azi_offset_overrider(0.0);
  // Clear azimuth allowable limits so they don't affect expected position.
  AziAllowStartOverrider overrider_start(-PI);
  AziAllowEndOverrider overrider_end(PI);
  HoverPathState state;
  FlightStatus flight_status;
  flight_status.flight_mode = kFlightModeHoverPayOut;
  flight_status.flight_mode_first_entry = false;

  const HoverPathParams *params = &GetControlParams()->hover.path;

  for (double elevation_cmd = -1.0; elevation_cmd < 1.0; elevation_cmd += 0.1) {
    for (double wind_dir_f = -M_PI; wind_dir_f < M_PI; wind_dir_f += 0.1) {
      Vec3 reel_pos_g;
      const double payout = 100.0;
      const double horizontal_tension_cmd = 8e3;
      const double tether_elevation_cmd = 0.1;
      double r_ff, z_ff, elevation_ff;
      ConvertPayoutToPosition(payout, horizontal_tension_cmd,
                              tether_elevation_cmd, &r_ff, &z_ff,
                              &elevation_ff);
      CalcPayOutReelInPosition(elevation_cmd, hypot(r_ff, z_ff), wind_dir_f,
                               &kVec3Zero, &kVec3Zero, &flight_status, params,
                               &state, &reel_pos_g);

      double reel_azimuth = VecGToAzimuth(&reel_pos_g);
      EXPECT_NEAR(reel_azimuth, wind_dir_f + params->transform_azimuth_offset,
                  1e-9);
    }
  }
}

// Tests that the lowest point on the tether never goes below the
// ground over the whole payout/reel-in path assuming we maintain the
// minimum tension.
//
// TODO(b/72569714): This test no longer passes after being accidentally
// disabled for a while. Fix it.
//
// TEST(CalcPayOutReelInPosition, TetherDoesNotIntersectGround) {
//   // Set params to GSv2 values.
//   FlightPlanOverrider flight_plan_overrider(kFlightPlanTurnKey);
//   AziOffsetOverrider azi_offset_overrider(0.0);
//
//   const HoverPathParams *params = &GetControlParams()->hover.path;
//   const double minimum_horizontal_tension =
//       GetControlParams()->hover.tension.tension_min_set_point;
//
//   for (double tether_length = 0.0; tether_length < g_sys.tether->length;
//        tether_length += 1.0) {
//     Vec3 wing_pos_g = {-tether_length, 0.0, 0.0};
//     double elevation_cmd =
//     ConvertTensionToElevation(minimum_horizontal_tension,
//                               &wing_pos_g, params);
//     for (double wind_dir_f = -M_PI; wind_dir_f < M_PI; wind_dir_f += 0.1) {
//       Vec3 reel_pos_g;
//       CalcPayOutReelInPosition(elevation_cmd, Vec3Norm(&wing_pos_g),
//                                wind_dir_f, &kVec3Zero, &kVec3Zero,
//                                &kVec3Zero, params, &reel_pos_g);
//       TetherParabola tether_parabola;
//       TensionAndPointToParabola(
//           minimum_horizontal_tension, Vec3XyNorm(&reel_pos_g), -reel_pos_g.z,
//           g_sys.tether->linear_density * g_sys.phys->g, &tether_parabola);
//       EXPECT_GE(ParabolaMinimum(&tether_parabola),
//                 -GetSystemParams()->ground_frame.ground_z);
//     }
//   }
// }

// Tests that the raw position command is equal to the current
// position of the wing during pilot hover in each flight plan.
TEST(CalcRawPositionCommand, kFlightModePilotHover) {
  const HoverPathParams *params = &GetControlParams()->hover.path;
  FlightStatus flight_status;
  flight_status.flight_mode = kFlightModePilotHover;
  flight_status.flight_mode_time = 0.0;
  HoverPathState state;
  Vec3 tether_anchor_g = {0.0, 0.0, -5.0};
  memset(&state, 0, sizeof(state));

  for (int32_t i = 0; i < kNumFlightPlans; ++i) {
    GetSystemParamsUnsafe()->flight_plan = static_cast<FlightPlan>(i);
    const double horizontal_tension_cmd = 5e3;
    const double payout = 0.0;
    const double tether_elevation_cmd = 0.1;
    Vec3 wing_pos_g = {1.0, 2.0, 3.0};
    Vec3 fixed_pos_g = {4.0, 5.0, 6.0};
    Vec3 wing_pos_g_cmd;
    double r_ff, z_ff, elevation_ff;
    ConvertPayoutToPosition(payout, horizontal_tension_cmd,
                            tether_elevation_cmd, &r_ff, &z_ff, &elevation_ff);
    CalcRawPositionCommand(&tether_anchor_g, &kVec3Zero, &flight_status, 0.22,
                           hypot(r_ff, z_ff), -0.22, &wing_pos_g, &fixed_pos_g,
                           &kVec3Zero, &kVec3Zero, params, &state,
                           &wing_pos_g_cmd);
    EXPECT_NEAR_VEC3(wing_pos_g, wing_pos_g_cmd, 1e-9);
  }
}

TEST(CalcRawPositionCommand, ApproximatelyInTetherSphere) {
  const HoverPathParams &params = GetControlParams()->hover.path;
  FlightStatus flight_status;
  HoverPathState state;
  Vec3 tether_anchor_g = {0.0, 0.0, -5.0};
  memset(&state, 0, sizeof(state));

  for (int32_t i = 0; i < kNumFlightPlans; ++i) {
    GetSystemParamsUnsafe()->flight_plan = static_cast<FlightPlan>(i);
    for (int32_t j = 0; j < kNumFlightModes; ++j) {
      flight_status.flight_mode = static_cast<FlightMode>(j);
      flight_status.flight_mode_time = 0.0;
      if (!AnyHoverFlightMode(flight_status.flight_mode)) continue;
      const double horizontal_tension_cmd = 5e3;
      const double tether_elevation_cmd = 0.1;
      const double payout = 3.0;

      Vec3 wing_pos_g = {1.0, 2.0, 3.0};
      Vec3 fixed_pos_g = {4.0, 5.0, 6.0};
      Vec3 accel_start_pos_g = {10.0, 11.0, 12.0};
      Vec3 wing_pos_g_cmd;
      double r_ff, z_ff, elevation_ff;
      ConvertPayoutToPosition(payout, horizontal_tension_cmd,
                              tether_elevation_cmd, &r_ff, &z_ff,
                              &elevation_ff);
      CalcRawPositionCommand(
          &tether_anchor_g, &kVec3Zero, &flight_status, 0.22, hypot(r_ff, z_ff),
          -0.22, &wing_pos_g, &fixed_pos_g, &accel_start_pos_g,
          &accel_start_pos_g, &params, &state, &wing_pos_g_cmd);
      // Due to numerical issues, we only check that the output is
      // approximately within the tether sphere.
      EXPECT_LE(Vec3Norm(&wing_pos_g_cmd) - 1.0, g_sys.tether->length);
    }
  }
}

TEST(SmoothRawPositionCommand, MaxNormalSpeed) {
  const HoverPathParams &params = GetControlParams()->hover.path;
  HoverPathState state = HoverPathState();
  state.wing_pos_g_cmd_z1 = kVec3Zero;

  Vec3 wing_pos_g_z1 = kVec3Zero;
  for (int32_t i = 0; i < kNumFlightModes; ++i) {
    FlightMode flight_mode = static_cast<FlightMode>(i);
    if (AnyAutoHoverFlightMode(flight_mode) &&
        flight_mode != kFlightModeHoverAccel &&
        flight_mode != kFlightModeHoverTransOut) {
      for (int32_t j = 0; j < ARRAYSIZE(kWingPosGValues); ++j) {
        Vec3 wing_pos_g;
        const Vec3 hover_origin_g = kVec3Zero;
        SmoothRawPositionCommand(&kWingPosGValues[j], flight_mode, &params,
                                 &state, &wing_pos_g);

        // Rotate the displacement vector into the hover frame, so we
        // can examine the radial and tangential components.
        Vec3 delta_g, delta_h;
        Vec3Sub(&wing_pos_g, &wing_pos_g_z1, &delta_g);
        RotateGToH(&delta_g, &wing_pos_g, &hover_origin_g, &delta_h);

        EXPECT_LE(fabs(delta_h.y) / *g_sys.ts,
                  params.max_normal_tangential_speed * (1.0 + 1e-3));

        EXPECT_LE(fabs(delta_h.z) / *g_sys.ts,
                  params.max_normal_radial_speed * (1.0 + 1e-3));

        // For the vertical component, we use ground coordinates.
        EXPECT_LE(delta_g.z / *g_sys.ts, params.max_descend_normal_z_speed *
                                             (1.0 + 5.0 * DBL_EPSILON));
        EXPECT_GE(delta_g.z / *g_sys.ts, -params.max_ascend_normal_z_speed *
                                             (1.0 + 5.0 * DBL_EPSILON));

        wing_pos_g_z1 = wing_pos_g;
      }
    }
  }
}

TEST(CalcVelocityCommand, ZeroSteadyStateZVelocity) {
  const HoverPathParams &params = GetControlParams()->hover.path;
  FlightStatus flight_status;
  flight_status.flight_mode = kFlightModeHoverPayOut;
  flight_status.flight_mode_time = 1.0;
  HoverPathState state = HoverPathState();
  state.wing_vel_g_cmd_zs[0] = kVec3Zero;
  state.wing_vel_g_cmd_zs[1] = kVec3Zero;

  Vec3 wing_pos_g_cmd = {-10.0, 20.0, -30.0};
  Vec3 wing_vel_g;
  for (int32_t i = 0; i < 10000.0; ++i) {
    CalcVelocityCommand(&wing_pos_g_cmd, &wing_pos_g_cmd, 22.0, &flight_status,
                        &kVec3Zero, &params, &state, &wing_vel_g);
  }
  EXPECT_NEAR(wing_vel_g.z, 0.0, 1e-2);
}

TEST(HoverPathStep, PositionSameAsInputForPilotHover) {
  const HoverPathParams &params = GetControlParams()->hover.path;
  HoverPathState state = HoverPathState();
  TetherGroundAnglesEstimate tether_ground_angles =
      TetherGroundAnglesEstimate();
  FlightStatus flight_status;

  Playbook playbook = Playbook();
  playbook.num_entries = 1;
  playbook.entries[0].azi_offset = 0.0;
  playbook.entries[0].path_radius_target = 150.0;
  playbook.entries[0].elevation = 0.7;
  playbook.entries[0].wind_speed = 5.0;

  WindEstimate wind_g = WindEstimate();
  wind_g.dir_f = 0.0;
  wind_g.speed_f = 10.0;

  VesselEstimate vessel = VesselEstimate();
  vessel.pos_g = kVec3Zero;

  const double tether_elevation_cmd = 0.1;

  for (int32_t i = 0; i < ARRAYSIZE(kWingPosGValues); ++i) {
    Vec3 wing_pos_g_cmd, wing_vel_g_cmd, crosswind_loop_center_pos_g;
    flight_status.flight_mode = kFlightModePilotHover;
    flight_status.flight_mode_time = 0.0;
    const double horizontal_tension_cmd = 3e3;
    HoverPathStep(&kVec3Zero, &vessel, horizontal_tension_cmd,
                  tether_elevation_cmd, &tether_ground_angles, 100.0,
                  &kWingPosGValues[i], &kVec3Zero, &wind_g, &playbook, 0.0, 0.0,
                  &flight_status, &params, &state, &wing_pos_g_cmd,
                  &wing_vel_g_cmd, &crosswind_loop_center_pos_g);
    EXPECT_NEAR_VEC3(kWingPosGValues[i], wing_pos_g_cmd, 1e-9);
  }
}

TEST(CylindricalRateLimit, RadialPositionChange) {
  Vec3 start = {2.0 * cos(0.1), 2.0 * sin(0.1), -2.0};
  Vec3 finish = {22.0 * cos(0.1), 22.0 * sin(0.1), -2.0};
  assert(fabs(VecGToAzimuth(&start) - VecGToAzimuth(&finish)) < 1e-9);

  Vec3 wing_pos_g_cmd_z1;
  Vec3 wing_pos_g_cmd = start;

  const double max_tangential_vel = 2.0;
  const double max_radial_vel = 5.0;
  const double max_z_vel = 7.0;

  while (Vec3Distance(&wing_pos_g_cmd, &finish) > 1e-3) {
    wing_pos_g_cmd_z1 = wing_pos_g_cmd;
    CylindricalRateLimit(&finish, max_tangential_vel, max_radial_vel,
                         -max_z_vel, max_z_vel, 0.1, &wing_pos_g_cmd);

    // Verify azimuth remains unchanged.
    EXPECT_NEAR(VecGToAzimuth(&wing_pos_g_cmd), VecGToAzimuth(&start), 1e-9);

    // Verify that we obey the radial speed limit
    EXPECT_LE(
        fabs(Vec3XyNorm(&wing_pos_g_cmd) - Vec3XyNorm(&wing_pos_g_cmd_z1)),
        max_radial_vel);

    // Verify z-component unchanged.
    EXPECT_NEAR(wing_pos_g_cmd.z, wing_pos_g_cmd_z1.z, 1e-9);
  }
}

TEST(CylindricalRateLimit, AzimuthalPositionChange) {
  Vec3 start = {100.0, 0.0, -10.0};
  Vec3 finish = {0.0, 100.0, -10.0};
  assert(fabs(Vec3Norm(&start) - Vec3Norm(&finish)) < 1e-9);

  Vec3 wing_pos_g_cmd_z1;
  Vec3 wing_pos_g_cmd = start;

  const double max_tangential_vel = 2.0;
  const double max_radial_vel = 5.0;
  const double max_z_vel = 7.0;
  const double ts = 0.1;

  while (Vec3Distance(&wing_pos_g_cmd, &finish) > 1e-3) {
    wing_pos_g_cmd_z1 = wing_pos_g_cmd;
    CylindricalRateLimit(&finish, max_tangential_vel, max_radial_vel,
                         -max_z_vel, max_z_vel, ts, &wing_pos_g_cmd);

    // Verify tangential speed limit.
    EXPECT_LE(Vec3Distance(&wing_pos_g_cmd, &wing_pos_g_cmd_z1) / ts,
              max_tangential_vel);

    // Verify that the radius is unchanged.
    EXPECT_NEAR(Vec3XyNorm(&wing_pos_g_cmd), Vec3XyNorm(&start), 1e-9);

    // Verify z-component unchanged.
    EXPECT_NEAR(wing_pos_g_cmd.z, wing_pos_g_cmd_z1.z, 1e-9);
  }
}

TEST(CylindricalRateLimit, VerticalPositionChange) {
  Vec3 start = {100.0, 22.0, -10.0};
  Vec3 finish = {100.0, 22.0, -50.0};
  assert(fabs(start.x - finish.x) < 1e-9);
  assert(fabs(start.y - finish.y) < 1e-9);

  Vec3 wing_pos_g_cmd_z1;
  Vec3 wing_pos_g_cmd = start;

  const double max_tangential_vel = 2.0;
  const double max_radial_vel = 5.0;
  const double max_z_vel = 7.0;
  const double ts = 0.1;

  while (Vec3Distance(&wing_pos_g_cmd, &finish) > 1e-3) {
    wing_pos_g_cmd_z1 = wing_pos_g_cmd;
    CylindricalRateLimit(&finish, max_tangential_vel, max_radial_vel,
                         -max_z_vel, max_z_vel, ts, &wing_pos_g_cmd);

    // Verify azimuth remains unchanged.
    EXPECT_NEAR(VecGToAzimuth(&wing_pos_g_cmd), VecGToAzimuth(&start), 1e-9);

    // Verify that the radius is unchanged.
    EXPECT_NEAR(Vec3XyNorm(&wing_pos_g_cmd), Vec3XyNorm(&start), 1e-9);

    // Verify z-component speed limits
    EXPECT_GE(wing_pos_g_cmd.z - wing_pos_g_cmd_z1.z, -max_z_vel);
    EXPECT_LE(wing_pos_g_cmd.z - wing_pos_g_cmd_z1.z, max_z_vel);
  }
}

TEST(CylindricalRateLimit, Wrapping) {
  // This test traverses successive arcs of 10 degrees, incrementing
  // the start azimuth by five degrees each time.  We verify that the
  // length of the resulting path is as expected.  This is to detect
  // any potential angle-wrapping issues. The test is repeated with
  // arcs going in the opposite direction.
  const double angle_of_arc = 10.0 * PI / 180.0;
  const double signs[] = {-1.0, 1.0};

  for (int i = 0; i < ARRAYSIZE(signs); i++) {
    for (double start_azi = -PI; start_azi < PI;
         start_azi += angle_of_arc / 2.0) {
      const double finish_azi =
          Wrap(start_azi + signs[i] * angle_of_arc, -PI, PI);
      const double radius = 100.0;
      Vec3 start, finish;
      CylToVecG(start_azi, radius, -5.0, &start);
      CylToVecG(finish_azi, radius, -5.0, &finish);

      // Do the mini-simulation.
      Vec3 wing_pos_g_cmd_z1;
      Vec3 wing_pos_g_cmd = start;

      const double max_tangential_vel = 2.0;
      const double max_radial_vel = 5.0;
      const double max_z_vel = 7.0;
      const double ts = 0.1;

      double distance = 0.0;
      while (Vec3Distance(&wing_pos_g_cmd, &finish) > 1e-3) {
        wing_pos_g_cmd_z1 = wing_pos_g_cmd;
        CylindricalRateLimit(&finish, max_tangential_vel, max_radial_vel,
                             -max_z_vel, max_z_vel, ts, &wing_pos_g_cmd);
        distance += Vec3Distance(&wing_pos_g_cmd_z1, &wing_pos_g_cmd);
      }

      // Verify that the arclength of the path is as expected.
      EXPECT_NEAR(distance, radius * angle_of_arc, 1e-3);
    }
  }
}

TEST(ConvertPayoutToPosition, SmallPayout) {
  const HoverPathParams &params = GetControlParams()->hover.path;
  // For short payouts and high tensions, the target kite elevation should be
  // very close to the target tether elevation.
  double tension = 20000.0;
  double r, z, elevation;
  ConvertPayoutToPosition(0.0, tension, params.target_reel_tether_elevation, &r,
                          &z, &elevation);
  EXPECT_NEAR(params.target_reel_tether_elevation, elevation, 2e-3);
}

TEST(ConvertPayoutToPosition, CorrectTrend) {
  const HoverPathParams &params = GetControlParams()->hover.path;
  double tension = 8000.0;
  double payout = 300.0;
  double eplison_multiplier = 1.01;
  double r, z, baseline_elevation, new_elevation;

  // Baseline case.
  ConvertPayoutToPosition(payout, tension, params.target_reel_tether_elevation,
                          &r, &z, &baseline_elevation);

  // The kite elevation should always be greater than the tether elevation.
  EXPECT_GT(baseline_elevation, params.target_reel_tether_elevation);

  // Increasing the payout should increase the kite elevation.
  ConvertPayoutToPosition(payout * eplison_multiplier, tension,
                          params.target_reel_tether_elevation, &r, &z,
                          &new_elevation);
  EXPECT_GT(new_elevation, baseline_elevation);

  // Increasing the tension should decrease the kite elevation.
  ConvertPayoutToPosition(payout, tension * eplison_multiplier,
                          params.target_reel_tether_elevation, &r, &z,
                          &new_elevation);
  EXPECT_LT(new_elevation, baseline_elevation);

  // Increasing the tether elevation should increase the kite elevation.
  ConvertPayoutToPosition(
      payout, tension,
      (params.target_reel_tether_elevation * eplison_multiplier), &r, &z,
      &new_elevation);
  EXPECT_GT(new_elevation, baseline_elevation);
}

TEST(ConvertPayoutToPosition, ConsistentDelta) {
  double horizontal_tension_cmd = 1e3;
  double tether_elevation_cmd = 0.1;
  double payout_z1, radial_z1, vertical_z1;

  for (double payout = 0.0; payout < 500.0; payout += 10.0) {
    double radial_position, vertical_position, elevation;
    ConvertPayoutToPosition(payout, horizontal_tension_cmd,
                            tether_elevation_cmd, &radial_position,
                            &vertical_position, &elevation);

    if (payout > 0.0) {
      // Check that hypot(delta_x, delta_y) = delta_s.
      EXPECT_NEAR(
          hypot(radial_position - radial_z1, vertical_position - vertical_z1),
          payout - payout_z1, 1e-3 * fabs(payout - payout_z1));
    }

    // Save the values from this iteration for the next one.
    payout_z1 = payout;
    radial_z1 = radial_position;
    vertical_z1 = vertical_position;
  }
}

TEST(ConvertArclengthToPosition, Recovery) {
  double horizontal_tension_cmd = 1e3;
  double tether_elevation_cmd = 0.1;
  double radial_z1, vertical_z1;
  double measured_arclength = 0.0;

  for (double arclength = 0.0; arclength < 500.0; arclength += 10.0) {
    double radial_position, vertical_position, elevation;
    ConvertArclengthToPosition(arclength, horizontal_tension_cmd,
                               tether_elevation_cmd, &radial_position,
                               &vertical_position, &elevation);

    if (arclength > 0.0) {
      double delta_arclength =
          hypot(radial_position - radial_z1, vertical_position - vertical_z1);
      measured_arclength += delta_arclength;
      EXPECT_NEAR(measured_arclength, arclength, 1e-3 * arclength);
    }

    // Save the values from this iteration for the next one.
    radial_z1 = radial_position;
    vertical_z1 = vertical_position;
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

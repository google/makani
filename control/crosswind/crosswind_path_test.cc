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

#include "common/c_math/geometry.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_path.h"
#include "control/crosswind/crosswind_types.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;

extern "C" {

double CalcApparentWindCurvature(double aero_curvature, const Mat3 *dcm_g2cw,
                                 const Vec3 *wing_vel_cw, double speed,
                                 const Vec3 *wind_g);

double CalcGravityCurvature(const Mat3 *dcm_g2cw, const Vec3 *wing_vel_cw,
                            double speed, LoopDirection loop_dir,
                            const Vec3 *g_g);

void AddCurvatureCorrections(const double aero_curvatures[],
                             int32_t num_aero_curvatures, const Mat3 *dcm_g2cw,
                             const Vec3 *wing_vel_cw, double speed,
                             const Vec3 *wind_g, LoopDirection loop_dir,
                             const Vec3 *g_g, double curvatures[]);

double RemoveCurvatureCorrections(double curvature, const Mat3 *dcm_g2cw,
                                  const Vec3 *wing_vel_cw, double speed,
                                  const Vec3 *wind_g, LoopDirection loop_dir,
                                  const Vec3 *g_g);

}  // extern "C"

// All the weird cases go here.
TEST(CrosswindPathCalcBestHeading, NormalCornerCases) {
  CrosswindPathParams params = CrosswindPathParams();
  params.loop_dir = kLoopDirectionCw;
  params.min_turning_radius = 25.0;
  double path_radius = 47.0;

  Vec3 best_vel_cw;
  Vec3 ans0 = {0.0, 0.0, -1.0};
  CrosswindPathType path_type = kCrosswindPathNormal;
  CrosswindPathCalcBestHeading(&kVec3Zero, path_radius, path_type, &kVec3Zero,
                               &params, &best_vel_cw);
  EXPECT_NEAR_VEC3(best_vel_cw, ans0, 1e-9);
}

// The heading should be continuous everywhere except near the
// singularity.
TEST(CrosswindPathCalcBestHeading, NormalContinuous) {
  CrosswindPathParams params = CrosswindPathParams();
  params.loop_dir = kLoopDirectionCw;
  params.min_turning_radius = 25.0;
  double path_radius = 47.0;

  Vec3 best_vel_cw, best_vel_cw_p, best_vel_cw_m, pos_cw_p, pos_cw_m;
  // Check continuity across path_radius.
  for (int32_t i = 0; i < 22; ++i) {
    Vec3 pos_cw = {0.0, Rand(-250.0, 250.0), Rand(-250.0, 250.0)};
    CrosswindPathType path_type = kCrosswindPathNormal;
    Vec3Scale(&pos_cw, path_radius / Vec3Norm(&pos_cw), &pos_cw);
    Vec3Scale(&pos_cw, 1.0 + 1e-4, &pos_cw_p);
    Vec3Scale(&pos_cw, 1.0 - 1e-4, &pos_cw_m);
    CrosswindPathCalcBestHeading(&pos_cw, path_radius, path_type, &kVec3Zero,
                                 &params, &best_vel_cw);
    CrosswindPathCalcBestHeading(&pos_cw_p, path_radius, path_type, &kVec3Zero,
                                 &params, &best_vel_cw_p);
    CrosswindPathCalcBestHeading(&pos_cw_m, path_radius, path_type, &kVec3Zero,
                                 &params, &best_vel_cw_m);
    EXPECT_NEAR_VEC3(best_vel_cw, best_vel_cw_p, 1e-2);
    EXPECT_NEAR_VEC3(best_vel_cw, best_vel_cw_m, 1e-2);
  }

  // TODO: Check continuity around "bean".
}

// All headings should have unit magnitude.
TEST(CrosswindPathCalcBestHeading, NormalNormalization) {
  CrosswindPathParams params = CrosswindPathParams();
  params.loop_dir = kLoopDirectionCw;
  params.min_turning_radius = 25.0;
  double path_radius = 47.0;
  CrosswindPathType path_type = kCrosswindPathNormal;
  for (int32_t i = 0; i < 22; ++i) {
    Vec3 pos_cw = {0.0, Rand(-250.0, 250.0), Rand(-250.0, 250.0)};
    Vec3 best_vel_cw;
    CrosswindPathCalcBestHeading(&pos_cw, path_radius, path_type, &kVec3Zero,
                                 &params, &best_vel_cw);
    EXPECT_NEAR(Vec3Norm(&best_vel_cw), 1.0, 1e-9);
  }
}

// The normal circular path should be completely anti-symmetric if
// radius is not close to the singularity.
TEST(CrosswindPathCalcBestHeading, NormalSymmetric) {
  CrosswindPathParams params = CrosswindPathParams();
  params.loop_dir = kLoopDirectionCw;
  params.min_turning_radius = 25.0;
  double path_radius = 47.0;
  CrosswindPathType path_type = kCrosswindPathNormal;
  for (int32_t i = 0; i < 22; ++i) {
    Vec3 best_vel_cw_0, best_vel_cw_1;
    Vec3 pos_cw = {0.0, Rand(-250.0, 250.0), Rand(-250.0, 250.0)};
    if (Vec3Norm(&pos_cw) > 1e-4) {
      CrosswindPathCalcBestHeading(&pos_cw, path_radius, path_type, &kVec3Zero,
                                   &params, &best_vel_cw_0);
      CrosswindPathCalcBestHeading(Vec3Scale(&pos_cw, -1.0, &pos_cw),
                                   path_radius, path_type, &kVec3Zero, &params,
                                   &best_vel_cw_1);
      Vec3Scale(&best_vel_cw_1, -1.0, &best_vel_cw_1);
      EXPECT_NEAR_VEC3(best_vel_cw_0, best_vel_cw_1, 1e-9);
    }
  }
}

// The apparent wind curvature should be zero if the inertial speed
// and airspeed are the same.
TEST(CalcApparentWindCurvature, ZeroInNoWind) {
  for (int32_t i = 0; i < 22; ++i) {
    Mat3 dcm_g2cw;
    AngleToDcm(Rand(-0.3, 0.3), Rand(-0.3, 0.3), Rand(-0.3, 0.3),
               kRotationOrderZyx, &dcm_g2cw);
    Vec3 wing_vel_cw = {Rand(-5.0, 5.0), Rand(-50.0, 50.0), Rand(-50.0, 50.0)};
    double apparent_wind_curvature = CalcApparentWindCurvature(
        1.0, &dcm_g2cw, &wing_vel_cw, Vec3Norm(&wing_vel_cw), &kVec3Zero);
    EXPECT_NEAR(apparent_wind_curvature, 0.0, 1e-9);
  }
}

TEST(CalcApparentWindCurvature, SmallAtTopBottomOfLoop) {
  const Vec3 wind_cw = {-5.0, 0.0, 5.0};
  for (int32_t i = 0; i < 22; ++i) {
    Mat3 dcm_g2cw;
    AngleToDcm(Rand(-0.3, 0.3), Rand(-0.3, 0.3), Rand(-0.3, 0.3),
               kRotationOrderZyx, &dcm_g2cw);
    Vec3 wind_g;
    Mat3TransVec3Mult(&dcm_g2cw, &wind_cw, &wind_g);
    Vec3 wing_vel_cw = {0.0, Rand(-50.0, 50.0), 0.0};
    double apparent_wind_curvature = CalcApparentWindCurvature(
        1.0, &dcm_g2cw, &wing_vel_cw, Vec3Norm(&wing_vel_cw), &wind_g);
    double speed_ratio = Vec3Norm(&wind_g) / Vec3Norm(&wing_vel_cw);
    EXPECT_LE(fabs(apparent_wind_curvature), speed_ratio * speed_ratio + 1e-9);
  }
}

TEST(CalcGravityCurvature, ZeroInNoGravity) {
  for (int32_t i = 0; i < 22; ++i) {
    Mat3 dcm_g2cw;
    AngleToDcm(Rand(-0.3, 0.3), Rand(-0.3, 0.3), Rand(-0.3, 0.3),
               kRotationOrderZyx, &dcm_g2cw);
    Vec3 wing_vel_cw = {Rand(-5.0, 5.0), Rand(-50.0, 50.0), Rand(-50.0, 50.0)};
    double gravity_curvature =
        CalcGravityCurvature(&dcm_g2cw, &wing_vel_cw, Vec3Norm(&wing_vel_cw),
                             kLoopDirectionCw, &kVec3Zero);
    EXPECT_NEAR(gravity_curvature, 0.0, 1e-9);
  }
}

TEST(CalcGravityCurvature, ZeroAtSidesOfLoop) {
  const Vec3 g_g = {0.0, 0.0, 9.81};
  for (int32_t i = 0; i < 22; ++i) {
    Mat3 dcm_g2cw;
    AngleToDcm(Rand(-0.3, 0.3), Rand(-0.3, 0.3), Rand(-0.3, 0.3),
               kRotationOrderZyx, &dcm_g2cw);
    Vec3 wing_vel_cw = {Rand(-5.0, 5.0), 0.0, Rand(-50.0, 50.0)};
    double gravity_curvature =
        CalcGravityCurvature(&dcm_g2cw, &wing_vel_cw, Vec3Norm(&wing_vel_cw),
                             kLoopDirectionCw, &g_g);
    EXPECT_NEAR(gravity_curvature, 0.0, 1e-9);
  }
}

TEST(RemoveCurvatureCorrections, Recovery) {
  const Vec3 g_g = {0.0, 0.0, 9.81};
  for (int32_t i = 0; i < 22; ++i) {
    Mat3 dcm_g2cw;
    AngleToDcm(Rand(-0.3, 0.3), Rand(-0.3, 0.3), Rand(-0.3, 0.3),
               kRotationOrderZyx, &dcm_g2cw);
    Vec3 wing_vel_cw = {Rand(-5.0, 5.0), Rand(-50.0, 50.0), Rand(-50.0, 50.0)};
    Vec3 wind_g = {Rand(-5.0, 5.0), Rand(-50.0, 50.0), Rand(-50.0, 50.0)};

    double aero_curvatures[1] = {Rand(-0.02, 0.02)};
    double curvatures[1];
    AddCurvatureCorrections(aero_curvatures, 1, &dcm_g2cw, &wing_vel_cw,
                            Vec3Norm(&wing_vel_cw), &wind_g, kLoopDirectionCw,
                            &g_g, curvatures);
    double aero_curvature_out = RemoveCurvatureCorrections(
        curvatures[0], &dcm_g2cw, &wing_vel_cw, Vec3Norm(&wing_vel_cw), &wind_g,
        kLoopDirectionCw, &g_g);
    EXPECT_NEAR(aero_curvatures[0], aero_curvature_out, 1e-9);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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
#include <stdlib.h>

#include "common/c_math/util.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/tether_util.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;

TEST(TensionAndPointToParabola, Recovery) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    double T0 = Rand(1e-6, 10.0);
    double mu = Rand(0.0, 10.0);
    // The parabola must go through the origin (0,0); specifying a
    // point too close to the origin leads to excess curvature.
    double r = Rand(1e-6, 500.0) * Sign(Rand(-1.0, 1.0));
    double h = Rand(-10.0, 10.0);

    TetherParabola tp;
    TensionAndPointToParabola(T0, r, h, mu, &tp);

    double h_recover = ParabolaHeight(&tp, r);
    double T0_recover = ParabolaHorizontalTension(&tp, mu);
    EXPECT_NEAR(h, h_recover, 1e-6);
    EXPECT_NEAR(T0, T0_recover, 1e-9);
  }
}

TEST(TensionAndAngleToParabola, Recovery) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    double T0 = Rand(0.0, 10.0);
    double mu = Rand(0.0, 10.0);
    double r = Rand(-500.0, 500.0);
    double ang = Rand(-PI / 2.0, PI / 2.0);

    TetherParabola tp;
    TensionAndAngleToParabola(T0, r, ang, mu, &tp);

    double ang_recover = ParabolaAngle(&tp, r);
    double T0_recover = ParabolaHorizontalTension(&tp, mu);
    EXPECT_NEAR(ang, ang_recover, 1e-9);
    EXPECT_NEAR(T0, T0_recover, 1e-9);
  }
}

TEST(TensionAndAngleToParabola, Edge) {
  TetherParabola tp;
  double T0 = 100.0, r = 0.1, mu = 0.1;
  double ang[2] = {-PI / 2.0, PI / 2.0};
  for (int32_t i = 0; i < 2; ++i) {
    TensionAndAngleToParabola(T0, r, ang[i], mu, &tp);
    double ang_recover = ParabolaAngle(&tp, r);
    double T0_recover = ParabolaHorizontalTension(&tp, mu);
    EXPECT_NEAR(T0, T0_recover, 1e-9);
    EXPECT_NEAR(ang[i], ang_recover, 1e-9);
  }
}

// Check whether the parabola height at the vertex is lower than
// nearby points to either side.
TEST(ParabolaVertex, Consistency) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    double T0 = Rand(0.0, 10.0);
    double mu = Rand(0.0, 10.0);
    double r = Rand(-500.0, 500.0);
    double ang = Rand(-PI / 2.0, PI / 2.0);

    TetherParabola tp;
    TensionAndAngleToParabola(T0, r, ang, mu, &tp);

    double r_min = ParabolaVertex(&tp);

    double ha = ParabolaHeight(&tp, r_min - 1e-3);
    double hb = ParabolaHeight(&tp, r_min + 1e-3);
    double h = ParabolaHeight(&tp, r_min);

    EXPECT_LT(h, ha);
    EXPECT_LT(h, hb);
    EXPECT_NEAR(h, ParabolaMinimum(&tp), 1e-9);
  }
}

TEST(ParabolaHorizontalTension, Recovery) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    double T0 = Rand(0.0, 10.0);
    double mu = Rand(0.0, 10.0);
    double r = Rand(-500.0, 500.0);
    double ang = Rand(-PI / 2.0, PI / 2.0);

    TetherParabola tp;
    TensionAndAngleToParabola(T0, r, ang, mu, &tp);

    double T0_recover = ParabolaHorizontalTension(&tp, mu);
    EXPECT_NEAR(T0_recover, T0, 1e-9);
  }
}

TEST(ParabolaVerticalTension, Consistency) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    double T0 = Rand(0.0, 10.0);
    double mu = Rand(0.0, 10.0);
    double r = Rand(-500.0, 500.0);
    double ang = Rand(-PI / 2.0, PI / 2.0);

    TetherParabola tp;
    TensionAndAngleToParabola(T0, r, ang, mu, &tp);

    // There should be no vertical tension at the catenary vertex.
    double r_min = ParabolaVertex(&tp);
    double Ty_vertex = ParabolaVerticalTension(&tp, mu, r_min);
    double Tx_vertex = ParabolaHorizontalTension(&tp, mu);
    double T_vertex = ParabolaTension(&tp, mu, r_min);

    EXPECT_NEAR(Tx_vertex, T0, 1e-9);
    EXPECT_NEAR(Ty_vertex, 0.0, 1e-9);
    EXPECT_NEAR(T_vertex, T0, 1e-9);
  }
}

TEST(ConvertTensionToElevation, ZeroPosition) {
  const HoverPathParams &params = GetControlParams()->hover.path;
  for (double tension = 1.0; tension < 3000.0; tension += 100.0) {
    EXPECT_NEAR(0.0,
                ConvertTensionToElevation(tension, &kVec3Zero,
                                          params.target_reel_tether_elevation),
                1e-9);
  }
}

TEST(ConvertTensionToElevation, SmallPosition) {
  const HoverPathParams &params = GetControlParams()->hover.path;
  for (double tension = 1.0; tension < 3000.0; tension += 100.0) {
    Vec3 small_x = {DBL_EPSILON, 0.0, 0.0};
    EXPECT_NEAR(params.target_reel_tether_elevation,
                ConvertTensionToElevation(tension, &small_x,
                                          params.target_reel_tether_elevation),
                1e-9);

    Vec3 small_neg_x = {-DBL_EPSILON, 0.0, 0.0};
    EXPECT_NEAR(params.target_reel_tether_elevation,
                ConvertTensionToElevation(tension, &small_neg_x,
                                          params.target_reel_tether_elevation),
                1e-9);

    Vec3 small_y = {0.0, DBL_EPSILON, 0.0};
    EXPECT_NEAR(params.target_reel_tether_elevation,
                ConvertTensionToElevation(tension, &small_y,
                                          params.target_reel_tether_elevation),
                1e-9);

    Vec3 small_neg_y = {0.0, -DBL_EPSILON, 0.0};
    EXPECT_NEAR(params.target_reel_tether_elevation,
                ConvertTensionToElevation(tension, &small_neg_y,
                                          params.target_reel_tether_elevation),
                1e-9);

    // Note that pure z displacements are treated the same as a zero
    // position.
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

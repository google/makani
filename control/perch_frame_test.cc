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

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/ground_frame.h"
#include "control/perch_frame.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;

TEST(GToP, Recovery) {
  for (int32_t i = 0; i < 222; ++i) {
    Vec3 Xp = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 Xp_out;
    double perch_azi = Rand(-6.3, 6.3);
    GToP(PToG(&Xp, perch_azi, &Xp_out), perch_azi, &Xp_out);
    EXPECT_NEAR_VEC3(Xp, Xp_out, 1e-9);
  }
}

TEST(GToP, ZeroOrigin) {
  for (int32_t i = 0; i < 222; ++i) {
    Vec3 X = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 X_out, X_out2;
    double perch_azi = Rand(-6.3, 6.3);
    GToP(&X, perch_azi, &X_out);
    RotGToP(&X, perch_azi, &X_out2);
    EXPECT_NEAR_VEC3(X_out, X_out2, 1e-9);

    PToG(&X, perch_azi, &X_out);
    RotPToG(&X, perch_azi, &X_out2);
    EXPECT_NEAR_VEC3(X_out, X_out2, 1e-9);
  }
}

// Conversions between Perch and Winch drum.

TEST(PToWd, Recovery) {
  for (int32_t i = 0; i < 222; ++i) {
    Vec3 X_wd = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 X_wd_out;
    double drum_angle = Rand(-6.3, 6.3);
    PToWd(WdToP(&X_wd, drum_angle, &X_wd_out), drum_angle, &X_wd_out);
    EXPECT_NEAR_VEC3(X_wd, X_wd_out, 1e-9);
  }
}

TEST(PToWd, ZeroOrigin) {
  SystemParams *params = GetSystemParamsUnsafe();

  if (params->gs_model == kGroundStationModelGSv2) {
    params->ground_station.gs02.drum_origin_p = kVec3Zero;
  } else {
    params->perch.winch_drum_origin_p = kVec3Zero;
  }

  for (int32_t i = 0; i < 222; ++i) {
    Vec3 X = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 X_out, X_out2;
    double drum_angle = Rand(-6.3, 6.3);
    PToWd(&X, drum_angle, &X_out);
    RotPToWd(&X, drum_angle, &X_out2);
    EXPECT_NEAR_VEC3(X_out, X_out2, 1e-9);

    WdToP(&X, drum_angle, &X_out);
    RotWdToP(&X, drum_angle, &X_out2);
    EXPECT_NEAR_VEC3(X_out, X_out2, 1e-9);
  }
}

// Conversions between Perch and Levelwind.

TEST(PToLw, Recovery) {
  // TODO(b/109812013): Rethink perch geometry with GSv2.
  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) return;

  Vec3 Xp = kVec3Zero;
  Vec3 X_lw_out = kVec3Zero;

  for (int32_t i = 0; i < 222; ++i) {
    Vec3 X_lw = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    double levelwind_ele = Rand(-PI / 4.0, PI / 2.0);
    double drum_angle = Rand(1.0, 170.0);

    PToLw(LwToP(&X_lw, levelwind_ele, drum_angle, &Xp), levelwind_ele,
          drum_angle, &X_lw_out);

    EXPECT_NEAR_VEC3(X_lw, X_lw_out, 1e-9);
  }
}

// Combinations.

TEST(PToG_GToP_WdToP_PToWd_Test, Recovery) {
  // TODO(b/109812013): Rethink perch geometry with GSv2.
  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) return;

  Vec3 X_wd, Xp, Xg, X_wd_recovered;
  double perch_azi, drum_angle;

  for (int32_t i = 0; i < 222; ++i) {
    drum_angle = Rand(-PI, PI);
    perch_azi = Rand(-PI, PI);
    X_wd.x = Rand(-20.0, 20.0);
    X_wd.y = Rand(-20.0, 20.0);
    X_wd.z = Rand(-20.0, 20.0);

    PToG(WdToP(&X_wd, drum_angle, &Xp), perch_azi, &Xg);
    PToWd(GToP(&Xg, perch_azi, &Xp), drum_angle, &X_wd_recovered);

    EXPECT_NEAR_VEC3(X_wd, X_wd_recovered, 1e-9);
  }
}

// This test confirms that the NED, G, P, and WD coordinate systems
// are aligned (in y and z).
//
// TODO: The NED and G frames should not actually have
// the same altitude.  NED should be on the ground and G should be at
// the bottom of the winch drum.
TEST(NED_G_P_WD, Aligned) {
  // TODO(b/109812013): Rethink perch geometry with GSv2.
  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) return;

  PerchParams *params = &GetSystemParamsUnsafe()->perch;
  params->winch_drum_origin_p = kVec3Zero;
  params->gsg_pos_wd = kVec3Zero;

  Vec3 X_wd = {21.0, 22.0, 23.0};
  Vec3 X_ned, Xp, Xg;
  double g_heading = 0.0, drum_angle = 0.0, perch_azi = 0.0;
  GToNed(PToG(WdToP(&X_wd, drum_angle, &Xp), perch_azi, &Xg), g_heading,
         &X_ned);
  EXPECT_NEAR_VEC3(X_wd, Xp, 1e-9);
  EXPECT_NEAR_VEC3(X_wd, Xg, 1e-9);
  EXPECT_NEAR_VEC3(X_wd, X_ned, 1e-9);
}

TEST(NED_G_P_WD, AntiAligned) {
  // TODO(b/109812013): Rethink perch geometry with GSv2.
  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) return;

  PerchParams *params = &GetSystemParamsUnsafe()->perch;
  params->winch_drum_origin_p = kVec3Zero;
  params->gsg_pos_wd = kVec3Zero;

  Vec3 X_wd = {21.0, 22.0, 23.0};
  Vec3 X_ned, Xp, Xg;
  double g_heading = PI, drum_angle = PI, perch_azi = PI;
  GToNed(PToG(WdToP(&X_wd, drum_angle, &Xp), perch_azi, &Xg), g_heading,
         &X_ned);
  EXPECT_NEAR(X_wd.x, -Xp.x, 1e-9);
  EXPECT_NEAR(X_wd.y, -Xp.y, 1e-9);
  EXPECT_NEAR(X_wd.z, Xp.z, 1e-9);
  EXPECT_NEAR(Xp.x, -Xg.x, 1e-9);
  EXPECT_NEAR(Xp.y, -Xg.y, 1e-9);
  EXPECT_NEAR(Xp.z, Xg.z, 1e-9);
  EXPECT_NEAR(Xg.x, -X_ned.x, 1e-9);
  EXPECT_NEAR(Xg.y, -X_ned.y, 1e-9);
  EXPECT_NEAR(Xg.z, X_ned.z, 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

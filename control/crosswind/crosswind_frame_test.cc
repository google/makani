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
#include "control/crosswind/crosswind_frame.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;

TEST(TransformGToCw, Recovery) {
  for (int32_t i = 0; i < 222; ++i) {
    Vec3 point_g = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 path_center_g = {Rand(-10.0, 10.0), Rand(-10.0, 10.0),
                          Rand(-10.0, 10.0)};
    Vec3 point_g_out;
    TransformCwToG(&point_g, &path_center_g, &point_g_out);
    TransformGToCw(&point_g_out, &path_center_g, &point_g_out);
    EXPECT_NEAR_VEC3(point_g, point_g_out, 1e-9);
  }
}

TEST(TransformGToCw, ZeroOrigin) {
  Vec3 path_center_g = kVec3Zero;

  for (int32_t i = 0; i < 222; ++i) {
    Vec3 point = {Rand(-10.0, 10.0), Rand(-10.0, 10.0), Rand(-10.0, 10.0)};
    Vec3 point_out, point_out2;
    TransformGToCw(&point, &path_center_g, &point_out);
    RotateGToCw(&point, &path_center_g, &point_out2);
    EXPECT_NEAR_VEC3(point_out, point_out2, 1e-9);

    TransformCwToG(&point, &path_center_g, &point_out);
    RotateCwToG(&point, &path_center_g, &point_out2);
    EXPECT_NEAR_VEC3(point_out, point_out2, 1e-9);
  }
}

TEST(CalcDcmFToG, Orthogonality) {
  Vec3 apparent_wind_g;
  Vec3 anchor2kite_g;

  apparent_wind_g.x = -50.0;
  apparent_wind_g.y = 5.0;
  apparent_wind_g.z = 10.0;

  for (double azi_angle = 0.0; azi_angle < 2.0 * PI; azi_angle += 0.1) {
    for (double ele_angle = 0.0; ele_angle < PI / 2.0; ele_angle += 0.1) {
      SphToCart(azi_angle, ele_angle, 425.0, &anchor2kite_g);
      Mat3 dcm_f2g;
      CalcDcmFToG(&apparent_wind_g, &anchor2kite_g, &dcm_f2g);
      EXPECT_TRUE(Mat3IsOrthogonal(&dcm_f2g, 1.0e-9));
    }
  }
}

TEST(CalcDcmFToG, Xaxis) {
  Vec3 apparent_wind_g;
  Vec3 anchor2kite_g;

  apparent_wind_g.x = -50.0;
  apparent_wind_g.y = 5.0;
  apparent_wind_g.z = 10.0;

  for (double azi_angle = 0.0; azi_angle < 2.0 * PI; azi_angle += 0.1) {
    for (double ele_angle = 0.0; ele_angle < PI / 2.0; ele_angle += 0.1) {
      SphToCart(azi_angle, ele_angle, 425.0, &anchor2kite_g);

      Mat3 dcm_f2g;
      CalcDcmFToG(&apparent_wind_g, &anchor2kite_g, &dcm_f2g);

      Vec3 apparent_wind_f;
      Vec3 apparent_wind_f_expected;
      Vec3Scale(&kVec3X, -Vec3Norm(&apparent_wind_g),
                &apparent_wind_f_expected);

      Mat3TransVec3Mult(&dcm_f2g, &apparent_wind_g, &apparent_wind_f);

      EXPECT_NEAR_VEC3(apparent_wind_f, apparent_wind_f_expected, 1e-9);
    }
  }
}

TEST(CalcDcmFToG, Yaxis) {
  Vec3 apparent_wind_g;
  Vec3 anchor2kite_g;

  apparent_wind_g.x = -50.0;
  apparent_wind_g.y = 5.0;
  apparent_wind_g.z = 10.0;
  for (double azi_angle = 0.0; azi_angle < 2.0 * PI; azi_angle += 0.1) {
    for (double ele_angle = 0.0; ele_angle < PI / 2.0; ele_angle += 0.1) {
      SphToCart(azi_angle, ele_angle, 425.0, &anchor2kite_g);

      Mat3 dcm_f2g;
      CalcDcmFToG(&apparent_wind_g, &anchor2kite_g, &dcm_f2g);

      Vec3 anchor2kite_f;

      Mat3TransVec3Mult(&dcm_f2g, &anchor2kite_g, &anchor2kite_f);

      EXPECT_NEAR(anchor2kite_f.y, 0.0, 1e-9);
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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
#include "control/hover/hover_frame.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;

TEST(TransformGToH, NormalWithElevation) {
  const double tether_length = 100.0;
  const double azimuth = PI / 6.0;
  const double elevation = 0.1;
  Vec3 hover_center_g = {-tether_length * cos(elevation) * cos(azimuth),
                         tether_length * cos(elevation) * sin(azimuth),
                         -tether_length * sin(elevation)};
  Vec3 point_h, point_h_ans = {-tether_length * sin(elevation), 0.0,
                               tether_length * cos(elevation)};

  TransformGToH(&kVec3Zero, &hover_center_g, &kVec3Zero, &point_h);
  EXPECT_NEAR_VEC3(point_h_ans, point_h, 1e-9);
}

TEST(TransformGToH, NearSingularity) {
  const double tether_length = 100.0;
  const double azimuth = 0.0;
  const double elevation = PI / 2.0 * (1.0 - DBL_EPSILON);
  Vec3 hover_center_g = {-tether_length * cos(elevation) * cos(azimuth),
                         tether_length * cos(elevation) * sin(azimuth),
                         -tether_length * sin(elevation)};
  Vec3 point_h, point_h_ans = {-tether_length, 0.0, 1.0};
  TransformGToH(&kVec3X, &hover_center_g, &kVec3Zero, &point_h);
  EXPECT_NEAR_VEC3(point_h_ans, point_h, 1e-9);
}

// At the singularity, the answer depends on the behavior of atan2 at
// x = 0.0 and y = 0.0.  This test is here to make sure atan2 returns
// a number and that this number is the same across our target
// systems.
TEST(TransformGToH, Singularity) {
  Vec3 hover_center_g = {0.0, 0.0, -1.0};
  Vec3 point_h, point_h_ans = {-1.0, 0.0, -1.0};
  TransformGToH(&kVec3X, &hover_center_g, &kVec3Zero, &point_h);
  EXPECT_NEAR_VEC3(point_h_ans, point_h, 1e-9);
}

TEST(TransformGToH, IdentityReuseInput) {
  const Vec3 hover_center_g = {-100.0, 50.0, 10.0};
  Vec3 point_h = hover_center_g;
  TransformGToH(&point_h, &hover_center_g, &kVec3Zero, &point_h);
  EXPECT_NEAR_VEC3(kVec3Zero, point_h, 1e-9);
}

TEST(TransformGToH, Recovery) {
  const Vec3 hover_center_g = {-100.0, 50.0, 10.0};
  const Vec3 point_g = {-101.0, 51.0, 9.0};
  Vec3 point_g_out;
  TransformGToH(&point_g, &hover_center_g, &kVec3Zero, &point_g_out);
  TransformHToG(&point_g_out, &hover_center_g, &kVec3Zero, &point_g_out);
  EXPECT_NEAR_VEC3(point_g, point_g_out, 1e-9);
}

TEST(RotateGToH, Recovery) {
  const Vec3 hover_center_g = {-100.0, 50.0, 10.0};
  const Vec3 vector_g = {-1.0, 1.0, -1.0};
  Vec3 vector_g_out;
  RotateGToH(&vector_g, &hover_center_g, &kVec3Zero, &vector_g_out);
  RotateHToG(&vector_g_out, &hover_center_g, &kVec3Zero, &vector_g_out);
  EXPECT_NEAR_VEC3(vector_g, vector_g_out, 1e-9);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

#include "common/c_math/vec2.h"
#include "lib/util/test_util.h"
#include "sim/math/util.h"
#include "sim/models/perch.h"

// Declarations from internal namespace, exposed for testing.
namespace internal {

bool RayIntersectsCircle(const Vec2 &ray_root, const Vec2 &ray_direction,
                         const Vec2 &circle_center, double circle_radius);

}  // namespace internal

namespace {

void CheckRayIntersectsCircle(bool expected, const Vec2 &root,
                              const Vec2 &direction, const Vec2 &center,
                              double radius) {
  for (int32_t i = 0; i < 10; ++i) {
    Vec2 offset = {test_util::RandNormal(), test_util::RandNormal()};
    double angle = 2.0 * M_PI * test_util::Rand();
    Mat2 rotation = {{{cos(angle), sin(angle)}, {-sin(angle), cos(angle)}}};

    Vec2 transformed_root;
    Vec2 transformed_center;
    Vec2 transformed_direction;
    Mat2Vec2Axpby(&rotation, kNoTrans, &root, 1.0, &offset, &transformed_root);
    Mat2Vec2Mult(&rotation, &direction, &transformed_direction);
    Mat2Vec2Axpby(&rotation, kNoTrans, &center, 1.0, &offset,
                  &transformed_center);
    EXPECT_EQ(expected, internal::RayIntersectsCircle(
                            transformed_root, transformed_direction,
                            transformed_center, radius));
  }
}

}  // namespace

TEST(RayIntersectsCircle, Radial) {
  double radius = 2.0;
  Vec2 center = {0.0, 0.0};
  for (double theta = 0.0; theta < 2.0 * M_PI; theta += 0.1) {
    Vec2 root = {1.1 * radius * sin(theta), 1.1 * radius * cos(theta)};
    Vec2 direction;
    Vec2Normalize(&root, &direction);
    CheckRayIntersectsCircle(false, root, direction, center, radius);
    Vec2Scale(&direction, -1.0, &direction);
    CheckRayIntersectsCircle(true, root, direction, center, radius);
  }
}

TEST(RayIntersectsCircle, YAligned) {
  double radius = 2.0;
  Vec2 center = {0.0, 0.0};
  for (double x = -2.0 * radius; x < 2.0 * radius; x += 0.1) {
    Vec2 root = {x, -2.0 * radius};
    Vec2 direction = {0.0, 1.0};
    CheckRayIntersectsCircle(-radius < x && x < radius, root, direction, center,
                             radius);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

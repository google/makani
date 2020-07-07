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

#include "common/c_math/linalg.h"
#include "sim/physics/contactor.h"

TEST(ContactCircle, NoContact) {
  const Vec2 center = {0.0, 0.0};
  Vec2 X_collision;
  const double a = sqrt(2.0) / 2.0;
  EXPECT_FALSE(ContactCircle({1.1, 0.0}, 1.0, center, &X_collision));
  EXPECT_FALSE(ContactCircle({0.0, 1.1}, 1.0, center, &X_collision));
  EXPECT_FALSE(ContactCircle({1.1 * a, 1.1 * a}, 1.0, center, &X_collision));
}

TEST(ContactCircle, Contact) {
  const Vec2 center = {0.0, 0.0};
  Vec2 X_collision;

  EXPECT_TRUE(ContactCircle({0.8, 0.0}, 1.0, center, &X_collision));
  EXPECT_NEAR(1.0, X_collision.x, DBL_EPSILON);
  EXPECT_NEAR(0.0, X_collision.y, DBL_EPSILON);

  const double a = sqrt(2.0) / 2.0;
  EXPECT_TRUE(ContactCircle({0.8 * a, 0.8 * a}, 1.0, center, &X_collision));
  EXPECT_NEAR(a, X_collision.x, DBL_EPSILON);
  EXPECT_NEAR(a, X_collision.y, DBL_EPSILON);
}

TEST(ContactCircle, DeathTest) {
  Vec2 X_collision;
  EXPECT_DEATH(ContactCircle({0.01, 0.0}, 1.0, {0.0, 0.0}, &X_collision), "");
}

TEST(ContactTwoCircles, NoContact) {
  Vec2 first_center = {0.0, 0.0};
  Vec2 second_center = {1.0, 0.0};
  Vec2 X_contactor = {0.0, 22.0};
  Vec2 X_collision;
  EXPECT_FALSE(ContactTwoCircles(X_contactor, 1.0, first_center, 1.0,
                                 second_center, &X_collision));
}

TEST(ContactTwoCircles, ContactFirstCircle) {
  Vec2 first_center = {0.0, 0.0};
  Vec2 second_center = {1.0, 0.0};
  Vec2 X_contactor = {-0.9, 0.0};
  Vec2 X_collision;
  EXPECT_TRUE(ContactTwoCircles(X_contactor, 1.0, first_center, 1.0,
                                second_center, &X_collision));
  EXPECT_NEAR(-1.0, X_collision.x, DBL_EPSILON);
  EXPECT_NEAR(0.0, X_collision.y, DBL_EPSILON);
}

TEST(ContactTwoCircles, ContactSecondCircle) {
  Vec2 first_center = {0.0, 0.0};
  Vec2 second_center = {1.0, 0.0};
  Vec2 X_contactor = {1.0, 0.9};
  Vec2 X_collision;
  EXPECT_TRUE(ContactTwoCircles(X_contactor, 1.0, first_center, 1.0,
                                second_center, &X_collision));
  EXPECT_NEAR(1.0, X_collision.x, DBL_EPSILON);
  EXPECT_NEAR(1.0, X_collision.y, DBL_EPSILON);
}

// The collision point should be at the intersection of the two
// circles closest to the contactor.  For these unit radius circles,
// the intersection point and the circle centers form an equilateral
// triangle, so it is easy to calculate the intersection point.
TEST(ContactTwoCircles, ContactBothCircles) {
  Vec2 first_center = {0.0, 0.0};
  Vec2 second_center = {1.0, 0.0};
  Vec2 X_contactor = {0.5, 0.1};
  Vec2 X_collision;
  EXPECT_TRUE(ContactTwoCircles(X_contactor, 1.0, first_center, 1.0,
                                second_center, &X_collision));
  EXPECT_NEAR(0.5, X_collision.x, DBL_EPSILON);
  EXPECT_NEAR(sqrt(3.0) / 2.0, X_collision.y, DBL_EPSILON);

  X_contactor.y = -0.1;
  EXPECT_TRUE(ContactTwoCircles(X_contactor, 1.0, first_center, 1.0,
                                second_center, &X_collision));
  EXPECT_NEAR(0.5, X_collision.x, DBL_EPSILON);
  EXPECT_NEAR(-sqrt(3.0) / 2.0, X_collision.y, DBL_EPSILON);
}

TEST(ContactTwoCircles, ContactBothCirclesRotated) {
  Vec2 first_center = {0.0, 0.0};
  Vec2 second_center = {sqrt(3.0) / 2.0, 0.5};
  Vec2 X_contactor = {0.1, 0.5};
  Vec2 X_collision;
  EXPECT_TRUE(ContactTwoCircles(X_contactor, 1.0, first_center, 1.0,
                                second_center, &X_collision));
  EXPECT_NEAR(0.0, X_collision.x, DBL_EPSILON);
  EXPECT_NEAR(1.0, X_collision.y, DBL_EPSILON);

  X_contactor.x = 0.5;
  X_contactor.y = 0.0;
  EXPECT_TRUE(ContactTwoCircles(X_contactor, 1.0, first_center, 1.0,
                                second_center, &X_collision));
  EXPECT_NEAR(sqrt(3.0) / 2.0, X_collision.x, DBL_EPSILON);
  EXPECT_NEAR(-0.5, X_collision.y, DBL_EPSILON);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

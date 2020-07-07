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

#include "common/c_math/voting.h"

#include <gtest/gtest.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <cmath>

#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;

extern "C" {

void PermuteQuatSignClosest3(const Quat *q_a, const Quat *q_b, const Quat *q_c,
                             Quat *p_a, Quat *p_b, Quat *p_c);

}  // extern "C"

// Check that the expected answer from Median 3 does not depend on the
// order of the arguments.
static void TestMedian3Perm(double a, double b, double c, double ans) {
  EXPECT_EQ(Median3(a, b, c), ans);
  EXPECT_EQ(Median3(a, c, b), ans);
  EXPECT_EQ(Median3(b, a, c), ans);
  EXPECT_EQ(Median3(b, c, a), ans);
  EXPECT_EQ(Median3(c, a, b), ans);
  EXPECT_EQ(Median3(c, b, a), ans);
}

TEST(Median3, Normal_0) {
  // Normal different values.
  TestMedian3Perm(1.0, 2.0, 3.0, 2.0);
  // Repeated minimum.
  TestMedian3Perm(1.0, 1.0, 3.0, 1.0);
  TestMedian3Perm(-3.0, -3.0, -1.0, -3.0);
  // Repeated maximum.
  TestMedian3Perm(1.0, 3.0, 3.0, 3.0);
  TestMedian3Perm(-3.0, -1.0, -1.0, -1.0);
  // All equal values.
  TestMedian3Perm(1.0, 1.0, 1.0, 1.0);
  TestMedian3Perm(-1.0, -1.0, -1.0, -1.0);
}

TEST(Median3, Infinity_0) {
  TestMedian3Perm(-1.0, 0.0, INFINITY, 0.0);
  TestMedian3Perm(-INFINITY, 0.0, 1.0, 0.0);
  TestMedian3Perm(-INFINITY, INFINITY, INFINITY, INFINITY);
  TestMedian3Perm(-INFINITY, -INFINITY, INFINITY, -INFINITY);
}

TEST(Median3, NotNormal_0) {
  TestMedian3Perm(NAN, NAN, NAN, 0.0);
  TestMedian3Perm(22.0, NAN, NAN, 22.0);
  TestMedian3Perm(8.0, 4.0, NAN, 6.0);
  // Check for a possible NaN.
  TestMedian3Perm(1.0, INFINITY, NAN, INFINITY);
  TestMedian3Perm(1.0, -INFINITY, NAN, -INFINITY);
  TestMedian3Perm(-INFINITY, INFINITY, NAN, 0.0);
  // Check for overflow.
  TestMedian3Perm(DBL_MAX, DBL_MAX, NAN, DBL_MAX);
  TestMedian3Perm(-DBL_MAX, -DBL_MAX, NAN, -DBL_MAX);
  TestMedian3Perm(-DBL_MAX, DBL_MAX, NAN, 0.0);
  // Check for underflow.
  TestMedian3Perm(DBL_MIN, DBL_MIN, NAN, DBL_MIN);
  TestMedian3Perm(-DBL_MIN, -DBL_MIN, NAN, -DBL_MIN);
  TestMedian3Perm(-DBL_MIN, DBL_MIN, NAN, 0.0);
}

#if !defined(NDEBUG)

TEST(Median3Vec3, Null) {
  Vec3 tmp;
  EXPECT_DEATH(Median3Vec3(NULL, &kVec3Zero, &kVec3Zero, &tmp), "");
  EXPECT_DEATH(Median3Vec3(&kVec3Zero, NULL, &kVec3Zero, &tmp), "");
  EXPECT_DEATH(Median3Vec3(&kVec3Zero, &kVec3Zero, NULL, &tmp), "");
  EXPECT_DEATH(Median3Vec3(&kVec3Zero, &kVec3Zero, &kVec3Zero, NULL), "");
}

#endif  // !defined(NDEBUG)

TEST(Median3Quat, Normal) {
  Quat values[3];

  for (int32_t i = 0; i < 2222; ++i) {
    // Generate three random quaternions.
    for (int32_t j = 0; j < 3; ++j) {
      values[j].q0 = Rand(-2.0, 2.0);
      values[j].q1 = Rand(-2.0, 2.0);
      values[j].q2 = Rand(-2.0, 2.0);
      values[j].q3 = Rand(-2.0, 2.0);
      QuatNormalize(&values[j], &values[j]);
    }

    Quat m;
    Median3Quat(&values[0], &values[1], &values[2], &m);
    for (int32_t j = 0; j < 3; ++j) {
      Quat tmp;
      double d_pair = QuatMod(QuatSub(&values[j], &values[(j + 1) % 3], &tmp));
      double d_m = fmin(QuatMod(QuatSub(&values[j], &m, &tmp)),
                        QuatMod(QuatAdd(&values[j], &m, &tmp)));
      // This upper bound can be proved analytically.  See the white paper:
      // Vector and Rotation Median Voting.
      // TODO: Finish the white paper and add a link.
      EXPECT_GE(4.0 * d_pair, d_m);
    }
  }
}

TEST(Medain3Quat, TwoArgumentGriddedAnglesRandomRotation) {
  Quat q_nan = {NAN, 0.0, 0.0, 0.0};
  int32_t kNumAngle = 111;
  for (int32_t i = 0; i < kNumAngle; ++i) {
    double angle;
    if (i < kNumAngle) {
      angle = 0.9 * M_PI * (-1.0 + i / static_cast<double>(kNumAngle));
    } else {
      angle = M_PI + 0.9 * M_PI * (-2.0 + i / static_cast<double>(kNumAngle));
    }

    Quat q = {cos(angle / 2.0), sin(angle / 2.0), 0.0, 0.0};
    Quat ans = {cos(angle / 4.0), sin(angle / 4.0), 0.0, 0.0};
    Quat m1, m2;

    // Iterate over orientations.
    for (int32_t j = 0; j < 10; ++j) {
      Quat p = {Rand(-2.2, 2.2), Rand(-2.2, 2.2), Rand(-2.2, 2.2),
                Rand(-2.2, 2.2)};
      QuatNormalize(&p, &p);

      Quat tmp;
      Median3Quat(&q_nan, &p, QuatMultiply(&q, &p, &tmp), &m1);
      Median3Quat(&q_nan, QuatMultiply(&q, &p, &tmp), &p, &m2);
      QuatMultiply(&ans, &p, &tmp);

      EXPECT_NEAR(fabs(QuatDot(&m1, &m2)), 1.0, 1e-3);
      EXPECT_NEAR(fabs(QuatDot(&tmp, &m2)), 1.0, 1e-3);
    }
  }
}

TEST(Medain3Quat, TwoArgumentOrthogonal) {
  Quat q_nan = {NAN, 0.0, 0.0, 0.0};
  Quat q_i = {0.0, 1.0, 0.0, 0.0};
  Quat m;
  Median3Quat(&q_nan, &kQuatIdentity, &q_i, &m);
  EXPECT_NEAR(fabs(m.q0), 1.0 / sqrt(2.0), 1e-9);
  EXPECT_NEAR(fabs(m.q1), 1.0 / sqrt(2.0), 1e-9);
}

#if !defined(NDEBUG)

TEST(Median3Quat, Null) {
  Quat tmp;
  EXPECT_DEATH(Median3Quat(NULL, &kQuatIdentity, &kQuatIdentity, &tmp), "");
  EXPECT_DEATH(Median3Quat(&kQuatIdentity, NULL, &kQuatIdentity, &tmp), "");
  EXPECT_DEATH(Median3Quat(&kQuatIdentity, &kQuatIdentity, NULL, &tmp), "");
  EXPECT_DEATH(
      Median3Quat(&kQuatIdentity, &kQuatIdentity, &kQuatIdentity, NULL), "");
}

#endif  // !defined(NDEBUG)

TEST(PermuteQuatSignClosest3, SignFlip) {
  Quat values[3];

  for (int32_t i = 0; i < 10; ++i) {
    Quat q = {Rand(-2.0, 2.0), Rand(-2.0, 2.0), Rand(-2.0, 2.0),
              Rand(-2.0, 2.0)};

    double signs[2] = {1.0, 1.0};
    for (int32_t j = 0; j < 3; ++j) {
      if (j % 2 == 0) signs[0] *= -1.0;
      if (j % 2 == 1) signs[1] *= -1.0;

      QuatScale(&q, signs[0], &values[0]);
      QuatScale(&q, signs[1], &values[1]);
      values[2] = q;

      Quat p_values[3];
      PermuteQuatSignClosest3(&values[0], &values[1], &values[2], &p_values[0],
                              &p_values[1], &p_values[2]);
      EXPECT_NEAR_QUAT(values[2], p_values[0], 0.0);
      EXPECT_NEAR_QUAT(values[2], p_values[1], 0.0);
      EXPECT_NEAR_QUAT(values[2], p_values[2], 0.0);
    }
  }
}

TEST(PermuteQuatSignClosest3, ClosestPerm) {
  Quat values[3];

  for (int32_t k = 0; k < 22; ++k) {
    // Generate three random quaternions.
    for (int32_t j = 0; j < 3; ++j) {
      values[j].q0 = Rand(-2.0, 2.0);
      values[j].q1 = Rand(-2.0, 2.0);
      values[j].q2 = Rand(-2.0, 2.0);
      values[j].q3 = Rand(-2.0, 2.0);
    }

    // Compute the sum of moduli for the sign permutation suggested by
    // the function.
    Quat p_values[3];
    PermuteQuatSignClosest3(&values[0], &values[1], &values[2], &p_values[0],
                            &p_values[1], &p_values[2]);

    Quat tmp;
    double d_post = 0.0;
    for (int32_t j = 0; j < 3; ++j) {
      d_post += QuatMod(QuatSub(&p_values[j], &p_values[(j + 1) % 3], &tmp));
    }

    // Compute all four sign permutations and find the minimum sum of moduli.
    double d_min = INFINITY;
    double d_max = 0.0;
    for (int32_t i = 0; i < 2; ++i) {
      for (int32_t j = 0; j < 2; ++j) {
        double d = 0.0;
        double sign0 = i == 1 ? -1.0 : 1.0;
        double sign1 = j == 1 ? -1.0 : 1.0;
        d += QuatMod(QuatLinComb(1.0, &values[2], -sign0, &values[0], &tmp));
        d += QuatMod(QuatLinComb(1.0, &values[2], -sign1, &values[1], &tmp));
        d += QuatMod(QuatLinComb(sign0, &values[0], -sign1, &values[1], &tmp));
        if (d < d_min) d_min = d;
        if (d > d_max) d_max = d;
      }
    }

    EXPECT_NEAR(d_min, d_post, 1e-9);
    // This last test is here just as a sanity check of the relatively
    // complicated test code above.
    EXPECT_GT(d_max, d_post);
  }
}

#if !defined(NDEBUG)

TEST(PermuteQuatSignClosest3, Null) {
  Quat tmp;
  EXPECT_DEATH(PermuteQuatSignClosest3(NULL, &kQuatIdentity, &kQuatIdentity,
                                       &tmp, &tmp, &tmp),
               "");
  EXPECT_DEATH(PermuteQuatSignClosest3(&kQuatIdentity, NULL, &kQuatIdentity,
                                       &tmp, &tmp, &tmp),
               "");
  EXPECT_DEATH(PermuteQuatSignClosest3(&kQuatIdentity, &kQuatIdentity, NULL,
                                       &tmp, &tmp, &tmp),
               "");
  EXPECT_DEATH(PermuteQuatSignClosest3(&kQuatIdentity, &kQuatIdentity,
                                       &kQuatIdentity, NULL, &tmp, &tmp),
               "");
  EXPECT_DEATH(PermuteQuatSignClosest3(&kQuatIdentity, &kQuatIdentity,
                                       &kQuatIdentity, &tmp, NULL, &tmp),
               "");
  EXPECT_DEATH(PermuteQuatSignClosest3(&kQuatIdentity, &kQuatIdentity,
                                       &kQuatIdentity, &tmp, &tmp, NULL),
               "");
}

#endif  // !defined(NDEBUG)

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

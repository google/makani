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
#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;
using ::test_util::RandNormal;

TEST(QuatAdd, Normal) {
  const Quat q1 = {-1.0, 2.0, -3.0, 4.0}, q2 = {1.0, 1.0, 1.0, 1.0};
  Quat q_out, q_ans = {0.0, 3.0, -2.0, 5.0};
  QuatAdd(&q1, &q2, &q_out);
  EXPECT_NEAR_QUAT(q_out, q_ans, DBL_EPSILON);
}

TEST(QuatAdd, ReuseInput1) {
  Quat q1 = {-1.0, 2.0, -3.0, 4.0}, q2 = {1.0, 1.0, 1.0, 1.0};
  Quat q_ans = {0.0, 3.0, -2.0, 5.0};
  QuatAdd(&q1, &q2, &q1);
  EXPECT_NEAR_QUAT(q1, q_ans, DBL_EPSILON);
}

TEST(QuatAdd, ReuseInput2) {
  Quat q1 = {-1.0, 2.0, -3.0, 4.0}, q2 = {1.0, 1.0, 1.0, 1.0};
  Quat q_ans = {0.0, 3.0, -2.0, 5.0};
  QuatAdd(&q1, &q2, &q2);
  EXPECT_NEAR_QUAT(q2, q_ans, DBL_EPSILON);
}

TEST(QuatSub, Normal) {
  const Quat q1 = {-1.0, 2.0, -3.0, 4.0}, q2 = {-1.0, -1.0, -1.0, -1.0};
  Quat q_out, q_ans = {0.0, 3.0, -2.0, 5.0};
  QuatSub(&q1, &q2, &q_out);
  EXPECT_NEAR_QUAT(q_out, q_ans, DBL_EPSILON);
}

TEST(QuatSub, ReuseInput1) {
  Quat q1 = {-1.0, 2.0, -3.0, 4.0}, q2 = {-1.0, -1.0, -1.0, -1.0};
  Quat q_ans = {0.0, 3.0, -2.0, 5.0};
  QuatSub(&q1, &q2, &q1);
  EXPECT_NEAR_QUAT(q1, q_ans, DBL_EPSILON);
}

TEST(QuatSub, ReuseInput2) {
  Quat q1 = {-1.0, 2.0, -3.0, 4.0}, q2 = {-1.0, -1.0, -1.0, -1.0};
  Quat q_ans = {0.0, 3.0, -2.0, 5.0};
  QuatSub(&q1, &q2, &q2);
  EXPECT_NEAR_QUAT(q2, q_ans, DBL_EPSILON);
}

TEST(QuatScale, Normal) {
  const Quat q = {-1.0, 2.0, -3.0, 4.0};
  Quat q_out, q_ans = {-2.0, 4.0, -6.0, 8.0};
  QuatScale(&q, 2.0, &q_out);
  EXPECT_NEAR_QUAT(q_out, q_ans, DBL_EPSILON);
}

TEST(QuatScale, ReuseInput) {
  Quat q = {-1.0, 2.0, -3.0, 4.0}, q_ans = {-2.0, 4.0, -6.0, 8.0};
  QuatScale(&q, 2.0, &q);
  EXPECT_NEAR_QUAT(q, q_ans, DBL_EPSILON);
}

TEST(QuatLinComb, Normal) {
  const Quat q1 = {-1.0, 2.0, -3.0, 4.0}, q2 = {1.0, 1.0, 1.0, 1.0};
  Quat q_out, q_ans;

  QuatLinComb(1.0, &q1, 0.0, &q2, &q_out);
  EXPECT_NEAR_QUAT(q1, q_out, DBL_EPSILON);

  QuatLinComb(0.0, &q1, 1.0, &q2, &q_out);
  EXPECT_NEAR_QUAT(q2, q_out, DBL_EPSILON);

  QuatAdd(&q1, &q2, &q_ans);
  QuatLinComb(1.0, &q1, 1.0, &q2, &q_out);
  EXPECT_NEAR_QUAT(q_out, q_ans, DBL_EPSILON);

  QuatSub(&q1, &q2, &q_ans);
  QuatLinComb(1.0, &q1, -1.0, &q2, &q_out);
  EXPECT_NEAR_QUAT(q_out, q_ans, DBL_EPSILON);
}

TEST(QuatLinComb, ReuseInput1) {
  Quat q_ans, q1 = {-1.0, 2.0, -3.0, 4.0}, q2 = {1.0, 1.0, 1.0, 1.0};

  QuatAdd(&q1, &q2, &q_ans);
  QuatLinComb(1.0, &q1, 1.0, &q2, &q1);
  EXPECT_NEAR_QUAT(q1, q_ans, DBL_EPSILON);
}

TEST(QuatLinComb, ReuseInput2) {
  Quat q_ans, q1 = {-1.0, 2.0, -3.0, 4.0}, q2 = {1.0, 1.0, 1.0, 1.0};

  QuatAdd(&q1, &q2, &q_ans);
  QuatLinComb(1.0, &q1, 1.0, &q2, &q2);
  EXPECT_NEAR_QUAT(q2, q_ans, DBL_EPSILON);
}

TEST(QuatLinComb3, Normal) {
  const Quat q1 = {-1.0, 2.0, -3.0, 4.0};
  const Quat q2 = {1.0, 1.0, 1.0, 1.0};
  const Quat q3 = {-5.0, 0.0, 0.0, 5.0};
  Quat q_out, q_ans;

  QuatLinComb3(1.0, &q1, 0.0, &q2, 0.0, &q3, &q_out);
  EXPECT_NEAR_QUAT(q1, q_out, DBL_EPSILON);

  QuatLinComb3(0.0, &q1, 1.0, &q2, 0.0, &q3, &q_out);
  EXPECT_NEAR_QUAT(q2, q_out, DBL_EPSILON);

  QuatLinComb3(0.0, &q1, 0.0, &q2, 1.0, &q3, &q_out);
  EXPECT_NEAR_QUAT(q3, q_out, DBL_EPSILON);

  QuatAdd(QuatAdd(&q1, &q2, &q_ans), &q3, &q_ans);
  QuatLinComb3(1.0, &q1, 1.0, &q2, 1.0, &q3, &q_out);
  EXPECT_NEAR_QUAT(q_out, q_ans, DBL_EPSILON);

  QuatAdd(QuatSub(&q1, &q2, &q_ans), &q3, &q_ans);
  QuatLinComb3(1.0, &q1, -1.0, &q2, 1.0, &q3, &q_out);
  EXPECT_NEAR_QUAT(q_out, q_ans, DBL_EPSILON);
}

TEST(QuatLinComb3, ReuseInput1) {
  Quat q_ans, q1 = {-1.0, 2.0, -3.0, 4.0};
  const Quat q2 = {1.0, 1.0, 1.0, 1.0};
  const Quat q3 = {-5.0, 0.0, 0.0, 5.0};

  QuatAdd(QuatAdd(&q1, &q2, &q_ans), &q3, &q_ans);
  QuatLinComb3(1.0, &q1, 1.0, &q2, 1.0, &q3, &q1);
  EXPECT_NEAR_QUAT(q1, q_ans, DBL_EPSILON);
}

TEST(QuatLinComb3, ReuseInput2) {
  const Quat q1 = {-1.0, 2.0, -3.0, 4.0};
  Quat q_ans, q2 = {1.0, 1.0, 1.0, 1.0};
  const Quat q3 = {-5.0, 0.0, 0.0, 5.0};

  QuatAdd(QuatAdd(&q1, &q2, &q_ans), &q3, &q_ans);
  QuatLinComb3(1.0, &q1, 1.0, &q2, 1.0, &q3, &q2);
  EXPECT_NEAR_QUAT(q2, q_ans, DBL_EPSILON);
}

TEST(QuatLinComb3, ReuseInput3) {
  const Quat q1 = {-1.0, 2.0, -3.0, 4.0};
  const Quat q2 = {1.0, 1.0, 1.0, 1.0};
  Quat q_ans, q3 = {-5.0, 0.0, 0.0, 5.0};

  QuatAdd(QuatAdd(&q1, &q2, &q_ans), &q3, &q_ans);
  QuatLinComb3(1.0, &q1, 1.0, &q2, 1.0, &q3, &q3);
  EXPECT_NEAR_QUAT(q3, q_ans, DBL_EPSILON);
}

TEST(QuatConj, Normal) {
  const Quat q = {1.0, 2.0, 3.0, 4.0};
  Quat q_out, q_ans = {1.0, -2.0, -3.0, -4.0};

  QuatConj(&q, &q_out);
  EXPECT_NEAR_QUAT(q_out, q_ans, DBL_EPSILON);
}

TEST(QuatConj, ReuseInput) {
  Quat q = {1.0, 2.0, 3.0, 4.0};
  Quat q_ans = {1.0, -2.0, -3.0, -4.0};

  QuatConj(&q, &q);
  EXPECT_NEAR_QUAT(q, q_ans, DBL_EPSILON);
}

#if defined(NDEBUG)
TEST(QuatInv, Zero) {
  Quat q_zero = {0.0, 0.0, 0.0, 0.0};
  Quat q_inv;
  QuatInv(&q_zero, &q_inv);

  // We have defined the inverse of the zero quaternion to be as follows,
  // to avoid Inf or NaN.
  EXPECT_EQ(q_inv.q0, DBL_MAX);
  EXPECT_EQ(q_inv.q1, 0.0);
  EXPECT_EQ(q_inv.q2, 0.0);
  EXPECT_EQ(q_inv.q3, 0.0);
}
#endif  // defined(NDEBUG)

TEST(QuatInv, Small) {
  Quat q_small0 = {DBL_MIN, 0.0, 0.0, 0.0};
  Quat q_ans0 = {1.0 / DBL_MIN, 0.0, 0.0, 0.0};
  Quat q_inv;
  QuatInv(&q_small0, &q_inv);
  EXPECT_NEAR(q_inv.q0 / q_ans0.q0, 1.0, DBL_EPSILON);
  EXPECT_EQ(q_inv.q1, 0.0);
  EXPECT_EQ(q_inv.q2, 0.0);
  EXPECT_EQ(q_inv.q3, 0.0);

  Quat q_small0123 = {DBL_MIN, DBL_MIN, DBL_MIN, DBL_MIN};
  Quat q_ans0123 = {1.0 / (4.0 * DBL_MIN), -1.0 / (4.0 * DBL_MIN),
                    -1.0 / (4.0 * DBL_MIN), -1.0 / (4.0 * DBL_MIN)};
  QuatInv(&q_small0123, &q_inv);
  EXPECT_NEAR_QUAT(q_inv, q_ans0123, DBL_EPSILON);
}

TEST(QuatInv, ReuseInputSmall) {
  Quat q_small0 = {DBL_MIN, 0.0, 0.0, 0.0};
  Quat q_ans0 = {1.0 / DBL_MIN, 0.0, 0.0, 0.0};
  QuatInv(&q_small0, &q_small0);
  EXPECT_NEAR(q_small0.q0 / q_ans0.q0, 1.0, DBL_EPSILON);
  EXPECT_EQ(q_small0.q1, 0.0);
  EXPECT_EQ(q_small0.q2, 0.0);
  EXPECT_EQ(q_small0.q3, 0.0);

  Quat q_small0123 = {DBL_MIN, DBL_MIN, DBL_MIN, DBL_MIN};
  Quat q_ans0123 = {1.0 / (4.0 * DBL_MIN), -1.0 / (4.0 * DBL_MIN),
                    -1.0 / (4.0 * DBL_MIN), -1.0 / (4.0 * DBL_MIN)};
  QuatInv(&q_small0123, &q_small0123);
  EXPECT_NEAR_QUAT(q_small0123, q_ans0123, DBL_EPSILON);
}

TEST(QuatInv, Large) {
  Quat q_large0 = {DBL_MAX, 0.0, 0.0, 0.0};
  Quat q_ans0 = {1.0 / DBL_MAX, 0.0, 0.0, 0.0};
  Quat q_inv;
  QuatInv(&q_large0, &q_inv);
  EXPECT_NEAR(q_inv.q0 / q_ans0.q0, 1.0, DBL_EPSILON);
  EXPECT_EQ(q_inv.q1, 0.0);
  EXPECT_EQ(q_inv.q2, 0.0);
  EXPECT_EQ(q_inv.q3, 0.0);

  Quat q_large0123 = {DBL_MAX / 4.0, DBL_MAX / 4.0, DBL_MAX / 4.0,
                      DBL_MAX / 4.0};
  Quat q_ans0123 = {1.0 / DBL_MAX, -1.0 / DBL_MAX, -1.0 / DBL_MAX,
                    -1.0 / DBL_MAX};
  QuatInv(&q_large0123, &q_inv);
  EXPECT_NEAR_QUAT(q_inv, q_ans0123, DBL_EPSILON);
}

TEST(QuatMultiply, CompareToMATLAB) {
  const Quat q1 = {-1.0, 2.0, -3.0, 4.0};
  const Quat q2 = {1.0, 1.0, 1.0, 1.0};
  Quat q_out, q_ans = {-4.0, -6.0, -2.0, 8.0};

  QuatMultiply(&q1, &q2, &q_out);
  EXPECT_NEAR_QUAT(q_out, q_ans, DBL_EPSILON);
}

TEST(QuatMultiply, ReuseInput1) {
  Quat q1 = {-1.0, 2.0, -3.0, 4.0};
  const Quat q2 = {1.0, 1.0, 1.0, 1.0};
  Quat q_ans = {-4.0, -6.0, -2.0, 8.0};

  QuatMultiply(&q1, &q2, &q1);
  EXPECT_NEAR_QUAT(q1, q_ans, DBL_EPSILON);
}

TEST(QuatMultiply, ReuseInput2) {
  const Quat q1 = {-1.0, 2.0, -3.0, 4.0};
  Quat q2 = {1.0, 1.0, 1.0, 1.0};
  Quat q_ans = {-4.0, -6.0, -2.0, 8.0};

  QuatMultiply(&q1, &q2, &q2);
  EXPECT_NEAR_QUAT(q2, q_ans, DBL_EPSILON);
}

TEST(QuatDivide, CompareToMATLAB) {
  Quat q1 = {1.0, 0.0, 1.0, 0.0};
  Quat q2 = {1.0, 0.5, 0.5, 0.75};
  Quat q_out;

  QuatDivide(&q1, &q2, &q_out);

  EXPECT_NEAR(q_out.q0, 0.727272727272727, 3.0 * DBL_EPSILON);
  EXPECT_NEAR(q_out.q1, 0.121212121212121, 3.0 * DBL_EPSILON);
  EXPECT_NEAR(q_out.q2, 0.242424242424242, 3.0 * DBL_EPSILON);
  EXPECT_NEAR(q_out.q3, -0.606060606060606, 3.0 * DBL_EPSILON);
}

TEST(QuatDivide, ReuseInput1) {
  Quat q1 = {1.0, 0.0, 1.0, 0.0};
  const Quat q2 = {1.0, 0.5, 0.5, 0.75};

  QuatDivide(&q1, &q2, &q1);

  EXPECT_NEAR(q1.q0, 0.727272727272727, 3.0 * DBL_EPSILON);
  EXPECT_NEAR(q1.q1, 0.121212121212121, 3.0 * DBL_EPSILON);
  EXPECT_NEAR(q1.q2, 0.242424242424242, 3.0 * DBL_EPSILON);
  EXPECT_NEAR(q1.q3, -0.606060606060606, 3.0 * DBL_EPSILON);
}

TEST(QuatDivide, ReuseInput2) {
  const Quat q1 = {1.0, 0.0, 1.0, 0.0};
  Quat q2 = {1.0, 0.5, 0.5, 0.75};

  QuatDivide(&q1, &q2, &q2);

  EXPECT_NEAR(q2.q0, 0.727272727272727, 3.0 * DBL_EPSILON);
  EXPECT_NEAR(q2.q1, 0.121212121212121, 3.0 * DBL_EPSILON);
  EXPECT_NEAR(q2.q2, 0.242424242424242, 3.0 * DBL_EPSILON);
  EXPECT_NEAR(q2.q3, -0.606060606060606, 3.0 * DBL_EPSILON);
}

// TODO: Find a nice way to get a random distribution of
// rotations and use it here.
TEST(QuatDivide, CompareToDCMDiff) {
  Quat q1 = {1.0 / sqrt(2.0), 0.0, 1.0 / sqrt(2.0), 0.0};
  Quat q2 = {1.0 / sqrt(2.0), 0.0, -1.0 / sqrt(2.0), 0.0};
  Quat q_out_dcm, q_out_q;
  Mat3 dcm1, dcm2, dcm_out;

  QuatToDcm(&q1, &dcm1);
  QuatToDcm(&q2, &dcm2);

  Mat3Mult(&dcm2, kTrans, &dcm1, kNoTrans, &dcm_out);
  DcmToQuat(&dcm_out, &q_out_dcm);

  QuatDivide(&q1, &q2, &q_out_q);

  EXPECT_NEAR(q_out_q.q0, q_out_dcm.q0, 1e-9);
  EXPECT_NEAR(q_out_q.q1, q_out_dcm.q1, 1e-9);
  EXPECT_NEAR(q_out_q.q2, q_out_dcm.q2, 1e-9);
  EXPECT_NEAR(q_out_q.q3, q_out_dcm.q3, 1e-9);
}

TEST(QuatMaxAbs, Normal) {
  const Quat q1 = {-5.0, 2.0, -3.0, 2.0};
  const Quat q2 = {-1.0, 10.0, -3.0, 2.0};
  const Quat q3 = {-1.0, 2.0, -3.0, 2.0};
  const Quat q4 = {-1.0, 2.0, -3.0, 4.0};

  EXPECT_NEAR(QuatMaxAbs(&q1), 5.0, DBL_EPSILON);
  EXPECT_NEAR(QuatMaxAbs(&q2), 10.0, DBL_EPSILON);
  EXPECT_NEAR(QuatMaxAbs(&q3), 3.0, DBL_EPSILON);
  EXPECT_NEAR(QuatMaxAbs(&q4), 4.0, DBL_EPSILON);
}

TEST(QuatNorm, CompareToQuatMod) {
  const Quat q = {-1.0, 2.0, -3.0, 4.0};
  EXPECT_NEAR(QuatModSquared(&q), QuatMod(&q) * QuatMod(&q), DBL_EPSILON);
}

TEST(QuatMod, Zero) {
  Quat q_zero = {0.0, 0.0, 0.0, 0.0};
  double quat_mod = QuatMod(&q_zero);
  EXPECT_EQ(quat_mod, 0.0);
}

TEST(QuatMod, Small) {
  Quat q_small0 = {DBL_MIN, 0.0, 0.0, 0.0};
  double quat_mod0 = QuatMod(&q_small0);
  EXPECT_EQ(quat_mod0, DBL_MIN);

  // Check with a value whose reciprocal is infinite.
  double very_small = 5.0e-324;  // Smallest representable double.
  Quat q_smaller0 = {very_small, 0.0, 0.0, 0.0};
  double quat_mod_smaller0 = QuatMod(&q_smaller0);
  EXPECT_EQ(quat_mod_smaller0, very_small);

  Quat q_small0123 = {DBL_MIN, DBL_MIN, DBL_MIN, DBL_MIN};
  double quat_mod0123 = QuatMod(&q_small0123);
  EXPECT_EQ(quat_mod0123, 2.0 * DBL_MIN);
}

TEST(QuatMod, Unit) {
  Quat q_unit = {0.5, 0.5, 0.5, 0.5};
  double quat_mod = QuatMod(&q_unit);
  EXPECT_NEAR(quat_mod, 1.0, DBL_EPSILON);
}

TEST(QuatMod, Large) {
  Quat q_large0 = {DBL_MAX, 0.0, 0.0, 0.0};
  double quat_mod0 = QuatMod(&q_large0);
  EXPECT_NEAR(quat_mod0 / DBL_MAX, 1.0, DBL_EPSILON);

  Quat q_large0123 = {DBL_MAX / 4.0, DBL_MAX / 4.0, DBL_MAX / 4.0,
                      DBL_MAX / 4.0};
  double quat_mod0123 = QuatMod(&q_large0123);
  EXPECT_NEAR(quat_mod0123 / (DBL_MAX / 2.0), 1.0, DBL_EPSILON);
}

TEST(QuatMod, InfsAndNaNs) {
  Quat q_infs01 = {INFINITY, INFINITY, 0.0, 0.0};
  double quat_mod01 = QuatMod(&q_infs01);
  EXPECT_EQ(quat_mod01, INFINITY);

  Quat q_infs_nans = {INFINITY, INFINITY, NAN, 0.0};
  double quat_mod_infs_nans = QuatMod(&q_infs_nans);
  EXPECT_EQ(quat_mod_infs_nans, INFINITY);
}

TEST(QuatNormalize, Zero) {
  Quat q_norm = {1.0, 1.0, 1.0, 1.0};
  Quat q_zero = {0.0, 0.0, 0.0, 0.0};
  Quat q_ans = {1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q_zero, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans, DBL_EPSILON);
}

TEST(QuatNormalize, ReuseInputZero) {
  Quat q_zero = {0.0, 0.0, 0.0, 0.0};
  Quat q_ans = {1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q_zero, &q_zero);
  EXPECT_NEAR_QUAT(q_zero, q_ans, DBL_EPSILON);
}

TEST(QuatNormalize, Small) {
  Quat q_norm = {1.0, 1.0, 1.0, 1.0};

  Quat q_min0 = {DBL_MIN, 0.0, 0.0, 0.0};
  Quat q_ans0 = {1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q_min0, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans0, DBL_EPSILON);

  Quat q_min1 = {0.0, DBL_MIN, 0.0, 0.0};
  Quat q_ans1 = {0.0, 1.0, 0.0, 0.0};
  QuatNormalize(&q_min1, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans1, DBL_EPSILON);

  Quat q_min2 = {0.0, 0.0, DBL_MIN, 0.0};
  Quat q_ans2 = {0.0, 0.0, 1.0, 0.0};
  QuatNormalize(&q_min2, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans2, DBL_EPSILON);

  Quat q_min3 = {0.0, 0.0, 0.0, DBL_MIN};
  Quat q_ans3 = {0.0, 0.0, 0.0, 1.0};
  QuatNormalize(&q_min3, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans3, DBL_EPSILON);

  double very_small = 5.0e-324;  // Smallest representable double.
  Quat q_min01 = {very_small, very_small, 0.0, 0.0};
  Quat q_ans01 = {sqrt(2.0) / 2.0, sqrt(2.0) / 2.0, 0.0, 0.0};
  QuatNormalize(&q_min01, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans01, DBL_EPSILON);

  Quat q_min0123 = {DBL_MIN, DBL_MIN, DBL_MIN, DBL_MIN};
  Quat q_ans0123 = {0.5, 0.5, 0.5, 0.5};
  QuatNormalize(&q_min0123, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans0123, DBL_EPSILON);

  Quat q_min0n12n3 = {DBL_MIN, -DBL_MIN, DBL_MIN, -DBL_MIN};
  Quat q_ans0n12n3 = {0.5, -0.5, 0.5, -0.5};
  QuatNormalize(&q_min0n12n3, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans0n12n3, DBL_EPSILON);

  Quat q_small_x = {1.0, DBL_MIN, 0.0, 0.0};
  QuatNormalize(&q_small_x, &q_norm);
  EXPECT_EQ(q_norm.q1, q_small_x.q1);

  Quat q_small_y = {1.0, 0.0, DBL_MIN, 0.0};
  QuatNormalize(&q_small_y, &q_norm);
  EXPECT_EQ(q_norm.q2, q_small_y.q2);

  Quat q_small_z = {1.0, 0.0, 0.0, DBL_MIN};
  QuatNormalize(&q_small_z, &q_norm);
  EXPECT_EQ(q_norm.q3, q_small_z.q3);
}

TEST(QuatNormalize, ReuseInputSmall) {
  Quat q_min0 = {DBL_MIN, 0.0, 0.0, 0.0};
  Quat q_ans0 = {1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q_min0, &q_min0);
  EXPECT_NEAR_QUAT(q_min0, q_ans0, DBL_EPSILON);

  Quat q_min1 = {0.0, DBL_MIN, 0.0, 0.0};
  Quat q_ans1 = {0.0, 1.0, 0.0, 0.0};
  QuatNormalize(&q_min1, &q_min1);
  EXPECT_NEAR_QUAT(q_min1, q_ans1, DBL_EPSILON);

  Quat q_min2 = {0.0, 0.0, DBL_MIN, 0.0};
  Quat q_ans2 = {0.0, 0.0, 1.0, 0.0};
  QuatNormalize(&q_min2, &q_min2);
  EXPECT_NEAR_QUAT(q_min2, q_ans2, DBL_EPSILON);

  Quat q_min3 = {0.0, 0.0, 0.0, DBL_MIN};
  Quat q_ans3 = {0.0, 0.0, 0.0, 1.0};
  QuatNormalize(&q_min3, &q_min3);
  EXPECT_NEAR_QUAT(q_min3, q_ans3, DBL_EPSILON);

  Quat q_min01 = {DBL_MIN, DBL_MIN, 0.0, 0.0};
  Quat q_ans01 = {sqrt(2.0) / 2.0, sqrt(2.0) / 2.0, 0.0, 0.0};
  QuatNormalize(&q_min01, &q_min01);
  EXPECT_NEAR_QUAT(q_min01, q_ans01, DBL_EPSILON);

  Quat q_min0123 = {DBL_MIN, DBL_MIN, DBL_MIN, DBL_MIN};
  Quat q_ans0123 = {0.5, 0.5, 0.5, 0.5};
  QuatNormalize(&q_min0123, &q_min0123);
  EXPECT_NEAR_QUAT(q_min0123, q_ans0123, DBL_EPSILON);

  Quat q_min0n12n3 = {DBL_MIN, -DBL_MIN, DBL_MIN, -DBL_MIN};
  Quat q_ans0n12n3 = {0.5, -0.5, 0.5, -0.5};
  QuatNormalize(&q_min0n12n3, &q_min0n12n3);
  EXPECT_NEAR_QUAT(q_min0n12n3, q_ans0n12n3, DBL_EPSILON);

  Quat q_small_x = {1.0, DBL_MIN, 0.0, 0.0};
  QuatNormalize(&q_small_x, &q_small_x);
  EXPECT_EQ(q_small_x.q1, q_small_x.q1);

  Quat q_small_y = {1.0, 0.0, DBL_MIN, 0.0};
  QuatNormalize(&q_small_y, &q_small_y);
  EXPECT_EQ(q_small_y.q2, q_small_y.q2);

  Quat q_small_z = {1.0, 0.0, 0.0, DBL_MIN};
  QuatNormalize(&q_small_z, &q_small_z);
  EXPECT_EQ(q_small_z.q3, q_small_z.q3);
}

TEST(QuatNormalize, Large) {
  Quat q_norm = {1.0, 1.0, 1.0, 1.0};

  Quat q_max0 = {DBL_MAX, 0.0, 0.0, 0.0};
  Quat q_ans0 = {1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q_max0, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans0, DBL_EPSILON);

  Quat q_max1 = {0.0, DBL_MAX, 0.0, 0.0};
  Quat q_ans1 = {0.0, 1.0, 0.0, 0.0};
  QuatNormalize(&q_max1, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans1, DBL_EPSILON);

  Quat q_max2 = {0.0, 0.0, DBL_MAX, 0.0};
  Quat q_ans2 = {0.0, 0.0, 1.0, 0.0};
  QuatNormalize(&q_max2, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans2, DBL_EPSILON);

  Quat q_max3 = {0.0, 0.0, 0.0, DBL_MAX};
  Quat q_ans3 = {0.0, 0.0, 0.0, 1.0};
  QuatNormalize(&q_max3, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans3, DBL_EPSILON);

  Quat q_max01 = {DBL_MAX, DBL_MAX, 0.0, 0.0};
  Quat q_ans01 = {sqrt(2.0) / 2.0, sqrt(2.0) / 2.0, 0.0, 0.0};
  QuatNormalize(&q_max01, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans01, DBL_EPSILON);

  Quat q_max0123 = {DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
  Quat q_ans0123 = {0.5, 0.5, 0.5, 0.5};
  QuatNormalize(&q_max0123, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans0123, DBL_EPSILON);

  Quat q_max0n12n3 = {DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX};
  Quat q_ans0n12n3 = {0.5, -0.5, 0.5, -0.5};
  QuatNormalize(&q_max0n12n3, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans0n12n3, DBL_EPSILON);
}

TEST(QuatNormalize, ReuseInputLarge) {
  Quat q_max0 = {DBL_MAX, 0.0, 0.0, 0.0};
  Quat q_ans0 = {1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q_max0, &q_max0);
  EXPECT_NEAR_QUAT(q_max0, q_ans0, DBL_EPSILON);

  Quat q_max1 = {0.0, DBL_MAX, 0.0, 0.0};
  Quat q_ans1 = {0.0, 1.0, 0.0, 0.0};
  QuatNormalize(&q_max1, &q_max1);
  EXPECT_NEAR_QUAT(q_max1, q_ans1, DBL_EPSILON);

  Quat q_max2 = {0.0, 0.0, DBL_MAX, 0.0};
  Quat q_ans2 = {0.0, 0.0, 1.0, 0.0};
  QuatNormalize(&q_max2, &q_max2);
  EXPECT_NEAR_QUAT(q_max2, q_ans2, DBL_EPSILON);

  Quat q_max3 = {0.0, 0.0, 0.0, DBL_MAX};
  Quat q_ans3 = {0.0, 0.0, 0.0, 1.0};
  QuatNormalize(&q_max3, &q_max3);
  EXPECT_NEAR_QUAT(q_max3, q_ans3, DBL_EPSILON);

  Quat q_max01 = {DBL_MAX, DBL_MAX, 0.0, 0.0};
  Quat q_ans01 = {sqrt(2.0) / 2.0, sqrt(2.0) / 2.0, 0.0, 0.0};
  QuatNormalize(&q_max01, &q_max01);
  EXPECT_NEAR_QUAT(q_max01, q_ans01, DBL_EPSILON);

  Quat q_max0123 = {DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
  Quat q_ans0123 = {0.5, 0.5, 0.5, 0.5};
  QuatNormalize(&q_max0123, &q_max0123);
  EXPECT_NEAR_QUAT(q_max0123, q_ans0123, DBL_EPSILON);

  Quat q_max0n12n3 = {DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX};
  Quat q_ans0n12n3 = {0.5, -0.5, 0.5, -0.5};
  QuatNormalize(&q_max0n12n3, &q_max0n12n3);
  EXPECT_NEAR_QUAT(q_max0n12n3, q_ans0n12n3, DBL_EPSILON);
}

TEST(QuatNormalize, InfsAndNaNs) {
  Quat q_norm = {1.0, 1.0, 1.0, 1.0};

  Quat q_inf0 = {1.0 / 0.0, 0.0, 0.0, 0.0};
  Quat q_ans0 = {1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q_inf0, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans0, DBL_EPSILON);

  Quat q_inf1 = {0.0, 1.0 / 0.0, 0.0, 0.0};
  Quat q_ans1 = {0.0, 1.0, 0.0, 0.0};
  QuatNormalize(&q_inf1, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans1, DBL_EPSILON);

  Quat q_inf01 = {1.0 / 0.0, 1.0 / 0.0, 0.0, 0.0};
  Quat q_ans01 = {sqrt(2.0) / 2.0, sqrt(2.0) / 2.0, 0.0, 0.0};
  QuatNormalize(&q_inf01, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans01, DBL_EPSILON);

  Quat q_nan1 = {0.0, 0.0 / 0.0, 0.0, 0.0};
  Quat q_ans_nan1 = {1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q_nan1, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans_nan1, DBL_EPSILON);

  Quat q_nan_inf = {0.0 / 0.0, 0.0 / 0.0, 1.0 / 0.0, 0.0};
  Quat q_ans_nan_inf = {1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q_nan_inf, &q_norm);
  EXPECT_NEAR_QUAT(q_norm, q_ans_nan_inf, DBL_EPSILON);
}

TEST(QuatNormalize, ReuseInputInfsAndNaNs) {
  Quat q_inf0 = {1.0 / 0.0, 0.0, 0.0, 0.0};
  Quat q_ans0 = {1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q_inf0, &q_inf0);
  EXPECT_NEAR_QUAT(q_inf0, q_ans0, DBL_EPSILON);

  Quat q_inf1 = {0.0, 1.0 / 0.0, 0.0, 0.0};
  Quat q_ans1 = {0.0, 1.0, 0.0, 0.0};
  QuatNormalize(&q_inf1, &q_inf1);
  EXPECT_NEAR_QUAT(q_inf1, q_ans1, DBL_EPSILON);

  Quat q_inf01 = {1.0 / 0.0, 1.0 / 0.0, 0.0, 0.0};
  Quat q_ans01 = {sqrt(2.0) / 2.0, sqrt(2.0) / 2.0, 0.0, 0.0};
  QuatNormalize(&q_inf01, &q_inf01);
  EXPECT_NEAR_QUAT(q_inf01, q_ans01, DBL_EPSILON);

  Quat q_nan1 = {0.0, 0.0 / 0.0, 0.0, 0.0};
  Quat q_ans_nan1 = {1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q_nan1, &q_nan1);
  EXPECT_NEAR_QUAT(q_nan1, q_ans_nan1, DBL_EPSILON);

  Quat q_nan_inf = {0.0 / 0.0, 0.0 / 0.0, 1.0 / 0.0, 0.0};
  Quat q_ans_nan_inf = {1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q_nan_inf, &q_nan_inf);
  EXPECT_NEAR_QUAT(q_nan_inf, q_ans_nan_inf, DBL_EPSILON);
}

TEST(QuatNormalize, NoChangeSign) {
  Quat q_out;
  Quat q0 = {-1.0, 0.0, 0.0, 0.0};
  QuatNormalize(&q0, &q_out);
  EXPECT_NEAR_QUAT(q_out, q0, 1e-9);

  Quat q1 = {0.0, -1.0, 0.0, 0.0};
  QuatNormalize(&q1, &q_out);
  EXPECT_NEAR_QUAT(q_out, q1, 1e-9);

  Quat q2 = {0.0, 0.0, -1.0, 0.0};
  QuatNormalize(&q2, &q_out);
  EXPECT_NEAR_QUAT(q_out, q2, 1e-9);

  Quat q3 = {0.0, 0.0, 0.0, -1.0};
  QuatNormalize(&q3, &q_out);
  EXPECT_NEAR_QUAT(q_out, q3, 1e-9);
}

// This uses the mathematical definition of quaternion rotation:
//
//   v_out = q^-1 * v_in * q
//
// as a check against the DCM calculation.
TEST(QuatRotate, CompareWithQuatMultiply) {
  for (int32_t i = 0; i < 222; ++i) {
    Vec3 v = {Rand(-1.0, 1.0), Rand(-1.0, 1.0), Rand(-1.0, 1.0)};
    Quat q = {Rand(-1.0, 1.0), Rand(-1.0, 1.0), Rand(-1.0, 1.0),
              Rand(-1.0, 1.0)};
    QuatNormalize(&q, &q);
    Quat q_inv, q_out, q_v = {0.0, v.x, v.y, v.z};
    QuatInv(&q, &q_inv);
    QuatMultiply(QuatMultiply(&q_inv, &q_v, &q_out), &q, &q_out);
    Vec3 v_mult = {q_out.q1, q_out.q2, q_out.q3};

    Vec3 v_rot;
    QuatRotate(&q, &v, &v_rot);

    // I did not prove the 10 * DBL_EPSILON limit here, but these
    // values should be "close".
    EXPECT_NEAR_VEC3(v_rot, v_mult, 10.0 * DBL_EPSILON);
  }
}

TEST(QuatToDcm, CompareToMATLAB) {
  const Quat q = {0.697035155263178, -0.060588926576927, 0.686790776520073,
                  -0.196950256639710};
  Mat3 dcm;
  Mat3 dcm_ans = {{{-0.020941948606985, -0.357786337296536, -0.933568621812276},
                   {0.191338673567395, 0.915079156771617, -0.354992463074382},
                   {0.981300640367677, -0.186062015699273, 0.049294822526421}}};
  QuatToDcm(&q, &dcm);
  EXPECT_NEAR_MAT3(dcm, dcm_ans, 1e-9);
}

#if defined(NDEBUG)
TEST(QuatToDcm, SameAsNormalized) {
  Quat q_unit, q_non_unit = {1.0, -2.0, 3.0, -4.0};
  QuatNormalize(&q_non_unit, &q_unit);

  Mat3 dcm_non_unit, dcm_unit;
  QuatToDcm(&q_non_unit, &dcm_non_unit);
  QuatToDcm(&q_unit, &dcm_unit);

  EXPECT_NEAR_MAT3(dcm_non_unit, dcm_unit, 2.0 * DBL_EPSILON);
}
#endif  // defined(NDEBUG)

TEST(DcmToQuat, Recovery) {
  for (int32_t i = 0; i < 222; ++i) {
    Quat q = {Rand(-1.0, 1.0), Rand(-1.0, 1.0), Rand(-1.0, 1.0),
              Rand(-1.0, 1.0)};
    QuatNormalize(&q, &q);

    Quat q_out;
    Mat3 dcm;
    DcmToQuat(QuatToDcm(&q, &dcm), &q_out);
    if (Sign(q.q0) != Sign(q_out.q0)) QuatScale(&q_out, -1.0, &q_out);

    EXPECT_NEAR_QUAT(q, q_out, 2.0 * DBL_EPSILON);
  }
}

TEST(DcmToQuat, RecoverySmall) {
  Quat q = {1.0, DBL_MIN, 0.0, 0.0};
  QuatNormalize(&q, &q);

  Quat q_out;
  Mat3 dcm;
  DcmToQuat(QuatToDcm(&q, &dcm), &q_out);
  if (Sign(q.q0) != Sign(q_out.q0)) QuatScale(&q_out, -1.0, &q_out);

  EXPECT_NEAR_QUAT(q, q_out, 2.0 * DBL_EPSILON);
}

TEST(QuatToAxisAngle, Recovery) {
  // If the magnitude of the rotation is greater than PI, then there
  // is necessarily a way of representing the rotation with a shorter
  // rotation.  1.813 is the max chosen because PI^2 = 1.813^2 +
  // 1.813^2 + 1.813^2.
  double angs[7] = {-1.813, -1.0, -0.5, 0.0, 0.5, 1.0, 1.813};

  for (int32_t i = 0; i < 7; ++i) {
    for (int32_t j = 0; j < 7; ++j) {
      for (int32_t k = 0; k < 7; ++k) {
        if (angs[i] == 0.0 && angs[j] == 0.0 && angs[k] == 0.0) continue;
        Quat q;
        Vec3 ax, ax_ang_final, ax_ang_start = {angs[i], angs[j], angs[k]};
        double ang = QuatToAxisAngle(AxisToQuat(&ax_ang_start, &q), &ax);
        Vec3Scale(&ax, ang, &ax_ang_final);

        EXPECT_NEAR(ax_ang_start.x, ax_ang_final.x, 1e-9);
        EXPECT_NEAR(ax_ang_start.y, ax_ang_final.y, 1e-9);
        EXPECT_NEAR(ax_ang_start.z, ax_ang_final.z, 1e-9);
      }
    }
  }
}

#if defined(NDEBUG)
TEST(QuatToAxisAngle, Recovery0) {
  Quat q;
  Vec3 axis_angle_final, axis_angle_start = kVec3Zero;
  double angle =
      QuatToAxisAngle(AxisToQuat(&axis_angle_start, &q), &axis_angle_final);
  Vec3Scale(&axis_angle_final, angle, &axis_angle_final);

  EXPECT_NEAR_VEC3(axis_angle_start, axis_angle_final, DBL_EPSILON);
}
#endif  // defined(NDEBUG)

TEST(QuatToAxisAngle, SingleAxisRotations) {
  const int32_t num_angles = 8;
  double angles[num_angles] = {
      -PI,      -3.0 * PI / 4.0, -PI / 2.0,      -PI / 4.0,
      PI / 4.0, PI / 2.0,        3.0 * PI / 4.0, PI};
  Mat3 dcm;
  Quat q;
  double angle;
  Vec3 axis_angle_ans, axis_angle;

  for (int32_t i = 0; i < num_angles; i++) {
    // Check x rotations.
    AngleToDcm(angles[i], 0.0, 0.0, kRotationOrderXyz, &dcm);
    angle = QuatToAxisAngle(DcmToQuat(&dcm, &q), &axis_angle);
    Vec3Scale(&axis_angle, angle, &axis_angle);
    Vec3Scale(&kVec3X, angles[i], &axis_angle_ans);
    EXPECT_NEAR_VEC3(axis_angle_ans, axis_angle, DBL_EPSILON);

    // Check y rotations.
    AngleToDcm(0.0, angles[i], 0.0, kRotationOrderXyz, &dcm);
    angle = QuatToAxisAngle(DcmToQuat(&dcm, &q), &axis_angle);
    Vec3Scale(&axis_angle, angle, &axis_angle);
    Vec3Scale(&kVec3Y, angles[i], &axis_angle_ans);
    EXPECT_NEAR_VEC3(axis_angle_ans, axis_angle, DBL_EPSILON);

    // Check z rotations.
    AngleToDcm(0.0, 0.0, angles[i], kRotationOrderXyz, &dcm);
    angle = QuatToAxisAngle(DcmToQuat(&dcm, &q), &axis_angle);
    Vec3Scale(&axis_angle, angle, &axis_angle);
    Vec3Scale(&kVec3Z, angles[i], &axis_angle_ans);
    EXPECT_NEAR_VEC3(axis_angle_ans, axis_angle, DBL_EPSILON);
  }
}

#if defined(NDEBUG)
TEST(QuatToAxisAngle, ZeroRotation) {
  Vec3 axis;
  double ang = QuatToAxisAngle(&kQuatIdentity, &axis);
  EXPECT_EQ(ang, 0.0);
  EXPECT_EQ(axis.x, 0.0);
  EXPECT_EQ(axis.y, 0.0);
  EXPECT_EQ(axis.z, 0.0);
}
#endif  // defined(NDEBUG)

TEST(QuatToAxisAngle, NoSignChange) {
  Vec3 axis;
  for (int32_t sign0 = -1; sign0 <= 1; sign0 += 2) {
    for (int32_t sign1 = -1; sign1 <= 1; sign1 += 2) {
      for (int32_t sign2 = -1; sign2 <= 1; sign2 += 2) {
        for (int32_t sign3 = -1; sign3 <= 1; sign3 += 2) {
          Quat q = {sign0 * 0.5, sign1 * 0.5, sign2 * 0.5, sign3 * 0.5};
          QuatToAxisAngle(&q, &axis);
          EXPECT_GT(axis.x * q.q1, 0.0);
          EXPECT_GT(axis.y * q.q2, 0.0);
          EXPECT_GT(axis.z * q.q3, 0.0);
        }
      }
    }
  }
}

TEST(QuatToAxisAngle, SmallRotation) {
  Vec3 axis;
  double ang;

  Quat q_small_x = {1.0, DBL_MIN, 0.0, 0.0};
  ang = QuatToAxisAngle(&q_small_x, &axis);
  EXPECT_EQ(ang, 2.0 * DBL_MIN);
  EXPECT_NEAR_VEC3(axis, kVec3X, DBL_EPSILON);

  Quat q_small_y = {1.0, 0.0, DBL_MIN, 0.0};
  ang = QuatToAxisAngle(&q_small_y, &axis);
  EXPECT_EQ(ang, 2.0 * DBL_MIN);
  EXPECT_NEAR_VEC3(axis, kVec3Y, DBL_EPSILON);

  Quat q_small_z = {1.0, 0.0, 0.0, DBL_MIN};
  ang = QuatToAxisAngle(&q_small_z, &axis);
  EXPECT_EQ(ang, 2.0 * DBL_MIN);
  EXPECT_NEAR_VEC3(axis, kVec3Z, DBL_EPSILON);
}

// This is an older version of the QuatToAxisAngle function.  The
// newer version is more numerically stable for small angles, but it
// is nice to compare against this version for "normal" angles.
static double OldQuatToAxisAngle(const Quat *q, Vec3 *axis) {
  double angle = 2.0 * Acos(q->q0);
  double sine_ang2 = sin(angle / 2.0);

  if (fabs(sine_ang2) < DBL_TOL) {
    *axis = kVec3Zero;
  } else {
    Vec3 tmp = {q->q1, q->q2, q->q3};
    Vec3Scale(&tmp, 1.0 / sine_ang2, axis);
  }

  if (angle > PI) {
    return angle - 2.0 * PI;
  } else {
    return angle;
  }
}

TEST(QuatToAxisAngle, CompareToOldVersion) {
  for (int32_t k = 0; k < 2222; ++k) {
    Quat q = {RandNormal(), RandNormal(), RandNormal(), RandNormal()};
    QuatNormalize(&q, &q);
    Vec3 axis_old, axis_new;
    double angle_old = OldQuatToAxisAngle(&q, &axis_old);
    double angle_new = QuatToAxisAngle(&q, &axis_new);
    EXPECT_NEAR(angle_old, angle_new, 1e-9);
    EXPECT_NEAR_VEC3(axis_old, axis_new, 1e-9);
  }
}

TEST(AxisAngleToQuat, Recovery) {
  for (int32_t i = 0; i < 222; ++i) {
    Quat q = {Rand(-2.2, 2.2), Rand(-2.2, 2.2), Rand(-2.2, 2.2),
              Rand(-2.2, 2.2)};
    QuatNormalize(&q, &q);

    Vec3 axis;
    double angle = QuatToAxisAngle(&q, &axis);

    Quat p;
    AxisAngleToQuat(&axis, angle, &p);

    EXPECT_NEAR(1.0, fabs(QuatDot(&q, &p)), 1e-9);
  }
}

TEST(AxisAngleToQuat, Identity) {
  Vec3 axis = {1.0, 2.0, 3.0};
  double ang = 0.0;
  Quat q_ans = {1.0, 0.0, 0.0, 0.0};
  Quat q;
  AxisAngleToQuat(&axis, ang, &q);
  EXPECT_NEAR_QUAT(q, q_ans, 1e-9);
}

TEST(AxisAngleToQuat, RotX90) {
  double ang = PI / 2.0;
  Quat q_ans = {sqrt(2.0) / 2.0, sqrt(2.0) / 2.0, 0.0, 0.0};
  Quat q;
  AxisAngleToQuat(&kVec3X, ang, &q);
  EXPECT_NEAR_QUAT(q, q_ans, 1e-9);
}

TEST(AxisAngleToQuat, RotXn90) {
  double ang = -PI / 2.0;
  Quat q_ans = {sqrt(2.0) / 2.0, -sqrt(2.0) / 2.0, 0.0, 0.0};
  Quat q;
  AxisAngleToQuat(&kVec3X, ang, &q);
  EXPECT_NEAR_QUAT(q, q_ans, 1e-9);
}

// The transpose is necessary here because the DCM represents a change
// of basis rather than a rotation.
TEST(AxisAngleToQuat, MATLAB_0) {
  Vec3 axis = {1.0, 2.0, 3.0};
  double ang = 0.22;
  Mat3 dcm_ans = {{{0.977619060092705, -0.171529738735821, 0.121813472459646},
                   {0.178416181784219, 0.982783892379004, -0.047994655514076},
                   {-0.111483807887048, 0.068653984659271, 0.991391946189502}}};
  Mat3 dcm_trans_ans;
  Mat3Trans(&dcm_ans, &dcm_trans_ans);
  Mat3 dcm;
  Quat q;
  AxisAngleToQuat(&axis, ang, &q);
  QuatToDcm(&q, &dcm);
  EXPECT_NEAR_MAT3(dcm, dcm_trans_ans, 1e-9);
}

TEST(QuatToAxis, Rotation90Deg) {
  Vec3 axis;

  Quat q_x90 = {sqrt(2.0) / 2.0, sqrt(2.0) / 2.0, 0.0, 0.0};
  Vec3 axis_x90_ans = {PI / 2.0, 0.0, 0.0};
  QuatToAxis(&q_x90, &axis);
  EXPECT_NEAR_VEC3(axis_x90_ans, axis, DBL_EPSILON);

  Quat q_xn90 = {sqrt(2.0) / 2.0, -sqrt(2.0) / 2.0, 0.0, 0.0};
  Vec3 axis_xn90_ans = {-PI / 2.0, 0.0, 0.0};
  QuatToAxis(&q_xn90, &axis);
  EXPECT_NEAR_VEC3(axis_xn90_ans, axis, DBL_EPSILON);

  Quat q_y90 = {sqrt(2.0) / 2.0, 0.0, sqrt(2.0) / 2.0, 0.0};
  Vec3 axis_y90_ans = {0.0, PI / 2.0, 0.0};
  QuatToAxis(&q_y90, &axis);
  EXPECT_NEAR_VEC3(axis_y90_ans, axis, DBL_EPSILON);

  Quat q_yn90 = {sqrt(2.0) / 2.0, 0.0, -sqrt(2.0) / 2.0, 0.0};
  Vec3 axis_yn90_ans = {0.0, -PI / 2.0, 0.0};
  QuatToAxis(&q_yn90, &axis);
  EXPECT_NEAR_VEC3(axis_yn90_ans, axis, DBL_EPSILON);

  Quat q_z90 = {sqrt(2.0) / 2.0, 0.0, 0.0, sqrt(2.0) / 2.0};
  Vec3 axis_z90_ans = {0.0, 0.0, PI / 2.0};
  QuatToAxis(&q_z90, &axis);
  EXPECT_NEAR_VEC3(axis_z90_ans, axis, DBL_EPSILON);

  Quat q_zn90 = {sqrt(2.0) / 2.0, 0.0, 0.0, -sqrt(2.0) / 2.0};
  Vec3 axis_zn90_ans = {0.0, 0.0, -PI / 2.0};
  QuatToAxis(&q_zn90, &axis);
  EXPECT_NEAR_VEC3(axis_zn90_ans, axis, DBL_EPSILON);
}

TEST(QuatToAxis, Recovery) {
  Quat q;
  Vec3 axis_out, axis = {1.0, 2.0, -0.5};
  QuatToAxis(AxisToQuat(&axis, &q), &axis_out);
  EXPECT_NEAR_VEC3(axis_out, axis, DBL_EPSILON);

  QuatToAxis(AxisToQuat(&axis_out, &q), &axis_out);
  EXPECT_NEAR_VEC3(axis_out, axis, DBL_EPSILON);
}

TEST(AxisToQuat, Rotation90Deg) {
  Quat q;

  Quat q_x90_ans = {sqrt(2.0) / 2.0, sqrt(2.0) / 2.0, 0.0, 0.0};
  Vec3 axis_x90 = {PI / 2.0, 0.0, 0.0};
  AxisToQuat(&axis_x90, &q);
  EXPECT_NEAR_QUAT(q_x90_ans, q, DBL_EPSILON);

  Quat q_xn90_ans = {sqrt(2.0) / 2.0, -sqrt(2.0) / 2.0, 0.0, 0.0};
  Vec3 axis_xn90 = {-PI / 2.0, 0.0, 0.0};
  AxisToQuat(&axis_xn90, &q);
  EXPECT_NEAR_QUAT(q_xn90_ans, q, DBL_EPSILON);

  Quat q_y90_ans = {sqrt(2.0) / 2.0, 0.0, sqrt(2.0) / 2.0, 0.0};
  Vec3 axis_y90 = {0.0, PI / 2.0, 0.0};
  AxisToQuat(&axis_y90, &q);
  EXPECT_NEAR_QUAT(q_y90_ans, q, DBL_EPSILON);

  Quat q_yn90_ans = {sqrt(2.0) / 2.0, 0.0, -sqrt(2.0) / 2.0, 0.0};
  Vec3 axis_yn90 = {0.0, -PI / 2.0, 0.0};
  AxisToQuat(&axis_yn90, &q);
  EXPECT_NEAR_QUAT(q_yn90_ans, q, DBL_EPSILON);

  Quat q_z90_ans = {sqrt(2.0) / 2.0, 0.0, 0.0, sqrt(2.0) / 2.0};
  Vec3 axis_z90 = {0.0, 0.0, PI / 2.0};
  AxisToQuat(&axis_z90, &q);
  EXPECT_NEAR_QUAT(q_z90_ans, q, DBL_EPSILON);

  Quat q_zn90_ans = {sqrt(2.0) / 2.0, 0.0, 0.0, -sqrt(2.0) / 2.0};
  Vec3 axis_zn90 = {0.0, 0.0, -PI / 2.0};
  AxisToQuat(&axis_zn90, &q);
  EXPECT_NEAR_QUAT(q_zn90_ans, q, DBL_EPSILON);
}

TEST(AxisToQuat, Recovery) {
  Quat q_out, q = {sqrt(2.0) / 2.0, 0.0, sqrt(2.0) / 2.0, 0.0};
  Vec3 axis;
  AxisToQuat(QuatToAxis(&q, &axis), &q_out);
  EXPECT_NEAR_QUAT(q_out, q, DBL_EPSILON);

  AxisToQuat(QuatToAxis(&q_out, &axis), &q_out);
  EXPECT_NEAR_QUAT(q_out, q, DBL_EPSILON);
}

TEST(QuatToAngle, kRotationOrderZyx) {
  double r1, r2, r3;
  Quat q = {0.182574185835055, 0.365148371670111, 0.547722557505166,
            0.730296743340221};
  QuatToAngle(&q, kRotationOrderZyx, &r1, &r2, &r3);
  EXPECT_NEAR(r1, 2.356194490192345, 1e-9);
  EXPECT_NEAR(r2, -0.339836909454122, 1e-9);
  EXPECT_NEAR(r3, 1.428899272190733, 1e-9);
}

#if defined(NDEBUG)
TEST(QuatToAngle, kRotationOrderZyx_Unnormalized) {
  double r1, r2, r3;
  Quat q = {1.0, 2.0, 3.0, 4.0};
  QuatToAngle(&q, kRotationOrderZyx, &r1, &r2, &r3);
  EXPECT_NEAR(r1, 2.356194490192345, 1e-9);
  EXPECT_NEAR(r2, -0.339836909454122, 1e-9);
  EXPECT_NEAR(r3, 1.428899272190733, 1e-9);
}
#endif  // defined(NDEBUG)

TEST(QuatToAngle, CompareToDCMToAngle) {
  Quat q = {1.0, 2.0, 3.0, 4.0};
  QuatNormalize(&q, &q);
  Mat3 dcm;
  double r_quat_1, r_quat_2, r_quat_3, r_dcm_1, r_dcm_2, r_dcm_3;
  for (int32_t i = 0; i < kNumRotationOrders; ++i) {
    RotationOrder order = static_cast<RotationOrder>(i);
    QuatToAngle(&q, order, &r_quat_1, &r_quat_2, &r_quat_3);
    DcmToAngle(QuatToDcm(&q, &dcm), order, &r_dcm_1, &r_dcm_2, &r_dcm_3);
    EXPECT_NEAR(r_dcm_1, r_quat_1, 1e-9);
    EXPECT_NEAR(r_dcm_2, r_quat_2, 1e-9);
    EXPECT_NEAR(r_dcm_3, r_quat_3, 1e-9);
  }
}

TEST(AngleToQuat, Recovery) {
  for (int32_t i = 0; i < kNumRotationOrders; ++i) {
    RotationOrder order = static_cast<RotationOrder>(i);
    for (int32_t j = 0; j < 10; ++j) {
      double r1_in = Rand(0.0, PI / 2.0);
      double r2_in = Rand(0.0, PI / 2.0);
      double r3_in = Rand(0.0, PI / 2.0);
      Quat q;
      double r1_out, r2_out, r3_out;
      AngleToQuat(r1_in, r2_in, r3_in, order, &q);
      QuatToAngle(&q, order, &r1_out, &r2_out, &r3_out);
      EXPECT_NEAR(r1_in, r1_out, 1e-9);
      EXPECT_NEAR(r2_in, r2_out, 1e-9);
      EXPECT_NEAR(r3_in, r3_out, 1e-9);
    }
  }
}

TEST(QuatToMrp, Normal) {
  const Quat q_pos = {0.788333398130528, -0.143880418517924, 0.351515804100355,
                      0.484009832572400};
  Vec3 mrp;
  QuatToMrp(&q_pos, &mrp);
  EXPECT_NEAR(mrp.x, -0.0804550307388614, 1e-9);
  EXPECT_NEAR(mrp.y, 0.1965605543506705, 1e-9);
  EXPECT_NEAR(mrp.z, 0.2706485452200185, 1e-9);

  const Quat q_neg = {-0.788333398130528, 0.143880418517924, -0.351515804100355,
                      -0.484009832572400};
  QuatToMrp(&q_neg, &mrp);
  EXPECT_NEAR(mrp.x, -0.0804550307388614, 1e-9);
  EXPECT_NEAR(mrp.y, 0.1965605543506705, 1e-9);
  EXPECT_NEAR(mrp.z, 0.2706485452200185, 1e-9);
}

TEST(QuatToMrp, DivideByZero) {
  // Test for possible divide by zero condition.
  const Quat q_pos = {1.0, 0.0, 0.0, 0.0};
  Vec3 mrp;
  QuatToMrp(&q_pos, &mrp);
  EXPECT_NEAR(mrp.x, 0.0, 1e-9);
  EXPECT_NEAR(mrp.y, 0.0, 1e-9);
  EXPECT_NEAR(mrp.z, 0.0, 1e-9);

  const Quat q_neg = {-1.0, 0.0, 0.0, 0.0};
  QuatToMrp(&q_neg, &mrp);
  EXPECT_NEAR(mrp.x, 0.0, 1e-9);
  EXPECT_NEAR(mrp.y, 0.0, 1e-9);
  EXPECT_NEAR(mrp.z, 0.0, 1e-9);
}

TEST(MrpToQuat, Normal) {
  const Vec3 mrp = {-0.0804550307388614, 0.1965605543506705,
                    0.2706485452200185};
  Quat q;
  MrpToQuat(&mrp, &q);
  EXPECT_NEAR(q.q0, 0.788333398130528, 1e-9);
  EXPECT_NEAR(q.q1, -0.143880418517924, 1e-9);
  EXPECT_NEAR(q.q2, 0.351515804100355, 1e-9);
  EXPECT_NEAR(q.q3, 0.484009832572400, 1e-9);
}

TEST(Vec3Vec3ToDcm, Recovery) {
  // TODO(b/111406468): Set the random number seed to make this test
  // deterministic.
  for (int i = 0; i < 100; i++) {
    Vec3 a = {Rand(-2.2, 2.2), Rand(-2.2, 2.2), Rand(-2.2, 2.2)};
    Vec3 b = {Rand(-2.2, 2.2), Rand(-2.2, 2.2), Rand(-2.2, 2.2)};

    Mat3 dcm_a2b;
    Vec3Vec3ToDcm(&a, &b, &dcm_a2b);

    Vec3 b_out;
    Mat3Vec3Mult(&dcm_a2b, &a, &b_out);

    // Verify that b_out is in the same direction as b.
    double angle_between_vectors = Vec3ToAxisAngle(&b, &b_out, NULL);
    EXPECT_NEAR(angle_between_vectors, 0.0, 1e-6);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

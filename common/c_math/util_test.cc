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

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <cmath>

#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;
using ::test_util::RandNormal;
using ::test_util::RandNormalf;

TEST(IsApproximatelyEqualVec3, Basic) {
  const Vec3 v0 = {0.6, 0.8, 0.0};
  const Vec3 dv_dir = {-0.6, 0.8, 0.0};
  Vec3 dv, v1;

  Vec3Scale(&dv_dir, DBL_EPSILON * 0.9, &dv);
  Vec3Add(&v0, &dv, &v1);
  EXPECT_TRUE(IsApproximatelyEqualVec3(&v0, &v1));

  Vec3Scale(&dv_dir, -DBL_EPSILON * 0.9, &dv);
  Vec3Add(&v0, &dv, &v1);
  EXPECT_TRUE(IsApproximatelyEqualVec3(&v0, &v1));

  Vec3Scale(&dv_dir, DBL_EPSILON * 1.1, &dv);
  Vec3Add(&v0, &dv, &v1);
  EXPECT_FALSE(IsApproximatelyEqualVec3(&v0, &v1));

  Vec3Scale(&dv_dir, -DBL_EPSILON * 1.1, &dv);
  Vec3Add(&v0, &dv, &v1);
  EXPECT_FALSE(IsApproximatelyEqualVec3(&v0, &v1));
}

TEST(MinMaxSignTest, Normal0) {
  EXPECT_EQ(-22, MinInt32(-22, 0));
  EXPECT_EQ(-22, MinInt32(-22, -22));
  EXPECT_EQ(22, MinInt32(22, 222));

  EXPECT_EQ(22, MaxInt32(22, 0));
  EXPECT_EQ(0, MaxInt32(0, 0));
  EXPECT_EQ(22, MaxInt32(-222, 22));

  EXPECT_EQ(1, SignInt32(22));
  EXPECT_EQ(-1, SignInt32(-22));
  EXPECT_EQ(0, SignInt32(0));

  EXPECT_EQ(1.0, Sign(22.0));
  EXPECT_EQ(-1.0, Sign(-22.0));
  EXPECT_EQ(0.0, Sign(0.0));

  double a[] = {-5.0, 3.2, 4.8, -1.2, 0.0};
  int32_t a_ind;
  double a_max, a_min;

  a_max = MaxArray(a, ARRAYSIZE(a), &a_ind);
  EXPECT_EQ(4.8, a_max);
  EXPECT_EQ(2, a_ind);
  a_max = MaxArray(a, ARRAYSIZE(a), nullptr);
  EXPECT_EQ(4.8, a_max);

  a_min = MinArray(a, ARRAYSIZE(a), &a_ind);
  EXPECT_EQ(-5.0, a_min);
  EXPECT_EQ(0, a_ind);
  a_min = MinArray(a, ARRAYSIZE(a), nullptr);
  EXPECT_EQ(-5.0, a_min);

  int32_t b[] = {-5, 3, 4, -1, 0};
  int32_t b_ind;
  int32_t b_max;
  b_max = MaxArrayInt32(b, ARRAYSIZE(b), &b_ind);
  EXPECT_EQ(4, b_max);
  EXPECT_EQ(2, b_ind);
  b_max = MaxArrayInt32(b, ARRAYSIZE(b), nullptr);
  EXPECT_EQ(4, b_max);

  uint32_t c[] = {5u, 3u, 4u, 1u, 10u};
  uint32_t c_max;
  int32_t c_ind;
  c_max = MaxArrayUint32(c, ARRAYSIZE(c), &c_ind);
  EXPECT_EQ(10u, c_max);
  EXPECT_EQ(4, c_ind);
  c_max = MaxArrayUint32(c, ARRAYSIZE(c), nullptr);
  EXPECT_EQ(10u, c_max);

  double d[] = {2.0, 1.0, 4.0, 0, 33.3, 4.0};
  int32_t d_ind;
  double d_max, d_min;

  d_max = MaxArray(d, ARRAYSIZE(d), &d_ind);
  EXPECT_EQ(33.3, d_max);
  EXPECT_EQ(4, d_ind);
  d_max = MaxArray(d, ARRAYSIZE(d), nullptr);
  EXPECT_EQ(33.3, d_max);

  d_min = MinArray(d, ARRAYSIZE(d), &d_ind);
  EXPECT_EQ(0.0, d_min);
  EXPECT_EQ(3, d_ind);
  d_min = MinArray(d, ARRAYSIZE(d), nullptr);
  EXPECT_EQ(0.0, d_min);

  int64_t e[] = {-5, 3, 4, -1, 0};
  int32_t e_ind;
  int64_t e_max;
  e_max = MaxArrayInt64(e, ARRAYSIZE(e), &e_ind);
  EXPECT_EQ(4, e_max);
  EXPECT_EQ(2, e_ind);
  e_max = MaxArrayInt64(e, ARRAYSIZE(e), nullptr);
  EXPECT_EQ(4, e_max);
}

TEST(MinMaxUintTest, SampleValues) {
  EXPECT_EQ(0U, MinUint32(0U, 22U));
  EXPECT_EQ(22U, MaxUint32(0U, 22U));

  EXPECT_EQ(0U, MinUint32(22U, 0U));
  EXPECT_EQ(22U, MaxUint32(22U, 0U));

  EXPECT_EQ(22U, MinUint32(22U, 22U));
  EXPECT_EQ(22U, MaxUint32(22U, 22U));

  EXPECT_EQ(0U, MinUint32(0U, UINT32_MAX));
  EXPECT_EQ(UINT32_MAX, MaxUint32(0U, UINT32_MAX));

  EXPECT_EQ(0U, MinUint32(UINT32_MAX, 0U));
  EXPECT_EQ(UINT32_MAX, MaxUint32(UINT32_MAX, 0U));

  EXPECT_EQ(UINT32_MAX - 20U, MinUint32(UINT32_MAX - 10U, UINT32_MAX - 20U));
  EXPECT_EQ(UINT32_MAX - 10U, MaxUint32(UINT32_MAX - 10U, UINT32_MAX - 20U));

  EXPECT_EQ(UINT32_MAX - 20U, MinUint32(UINT32_MAX - 20U, UINT32_MAX - 10U));
  EXPECT_EQ(UINT32_MAX - 10U, MaxUint32(UINT32_MAX - 20U, UINT32_MAX - 10U));
}

TEST(MeanPair, Normal0) {
  for (int32_t i = 0; i < 22; ++i) {
    double x = Rand(-10.0, 10.0), y = Rand(-10.0, 10.0);
    EXPECT_NEAR((x + y) / 2.0, MeanPair(x, y), 1e-14);
  }

  for (int32_t i = 0; i < 200; ++i) {
    EXPECT_EQ(MeanPair(0.0, 2.0 * static_cast<double>(i)),
              static_cast<double>(i));
    EXPECT_EQ(MeanPair(2.0 * static_cast<double>(i), 0.0),
              static_cast<double>(i));
  }
}

// A simple two-pass variance implementation to compare
// to VarArray.
static double VarArrayTwoPass(const double *x_buf, int32_t n) {
  if (n == 0) return 0.0;
  assert(x_buf != nullptr);

  double mean = 0.0;
  for (int32_t i = 0; i < n; ++i) {
    mean += x_buf[i];
  }
  mean = mean / n;

  double var = 0.0;
  for (int32_t i = 0; i < n; ++i) {
    var += (x_buf[i] - mean) * (x_buf[i] - mean);
  }
  return var / n;
}

TEST(VarArray, AgreesWtihTwoPass) {
  const int32_t kLength = 100;
  double x_buf[kLength];

  for (int32_t i = 0; i < kLength; ++i) {
    x_buf[i] = Rand(1e-3, 10.0);
  }

  EXPECT_NEAR(VarArray(x_buf, kLength), VarArrayTwoPass(x_buf, kLength), 1e-10);

  for (int32_t i = 0; i < kLength; ++i) {
    x_buf[i] += 1e8;
  }

  EXPECT_NEAR(VarArray(x_buf, kLength), VarArrayTwoPass(x_buf, kLength), 1e-6);
}

TEST(VarArray, ZeroLength) { EXPECT_EQ(VarArray(nullptr, 0), 0.0); }

TEST(VarArray, SinglePass) {
  double x[3] = {1e8, 1e8 + 1.0, 1e8 + 2.0};
  // The simplest single-pass variance formula reports 2.0 from
  // this calculation.
  EXPECT_EQ(VarArray(x, 3), 2.0 / 3.0);
}

TEST(VarArray, Constant) {
  double c[] = {0.0, 1.0, 1e8, -1.0};
  const int32_t kLength = 100;
  double y[kLength];

  for (int32_t k = 0; k < ARRAYSIZE(c); ++k) {
    for (int32_t i = 0; i < kLength; ++i) {
      y[i] = c[k];
    }
    EXPECT_NEAR(VarArray(y, kLength), 0.0, DBL_EPSILON);
  }
}

TEST(VarArray, Alternating) {
  double c[] = {0.0, 0.5, 1e8, -1.0};
  const int32_t kLength = 100;  // This needs to be even.
  double y[kLength];

  for (int32_t k = 0; k < ARRAYSIZE(c); ++k) {
    for (int32_t i = 0; i < kLength; ++i) {
      y[i] = (i % 2 == 0) ? c[k] : -c[k];
    }
    EXPECT_NEAR(VarArray(y, kLength), c[k] * c[k], DBL_EPSILON);
  }
}

TEST(MeanPair, Boundaries_0) {
  EXPECT_EQ(MeanPair(DBL_MAX, DBL_MAX), DBL_MAX);
  EXPECT_EQ(MeanPair(-DBL_MAX, -DBL_MAX), -DBL_MAX);
  EXPECT_EQ(MeanPair(DBL_MIN, DBL_MIN), DBL_MIN);
  EXPECT_EQ(MeanPair(-DBL_MIN, -DBL_MIN), -DBL_MIN);
  EXPECT_EQ(MeanPair(INFINITY, 1.0), INFINITY);
  EXPECT_EQ(MeanPair(1.0, INFINITY), INFINITY);
  EXPECT_EQ(MeanPair(-INFINITY, 1.0), -INFINITY);
  EXPECT_EQ(MeanPair(1.0, -INFINITY), -INFINITY);
  EXPECT_TRUE(std::isnan(MeanPair(-INFINITY, INFINITY)));
}

TEST(MeanArrayTest, Normal0) {
  double arr[100];
  for (int32_t i = 0; i < 100; ++i) {
    arr[i] = i;
  }
  double mean = MeanArray(arr, 100);
  EXPECT_NEAR(mean, 49.5, 1e-9);
}

TEST(SwapInPlace, Normal) {
  double a = RandNormal();
  double b = RandNormal();

  double a_swap = a;
  double b_swap = b;

  SwapInPlace(&a, &b);
  EXPECT_EQ(a, b_swap);
  EXPECT_EQ(b, a_swap);
}

TEST(SwapInPlacef, Normal) {
  float a = RandNormalf();
  float b = RandNormalf();

  float a_swap = a;
  float b_swap = b;

  SwapInPlacef(&a, &b);
  EXPECT_EQ(a, b_swap);
  EXPECT_EQ(b, a_swap);
}

TEST(SaturateTest, Consistency_0) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    double a, b, x = RandNormal(), y = RandNormal();
    if (y < x) {
      a = x;
      x = y;
      y = a;
    }
    a = RandNormal();

    b = Saturate(a, x, y);
    if (a >= y) {
      EXPECT_EQ(b, y);
      EXPECT_TRUE(IsSaturated(b, x, y));
    } else if (a <= x) {
      EXPECT_EQ(b, x);
      EXPECT_TRUE(IsSaturated(b, x, y));
    } else {
      EXPECT_EQ(a, b);
      EXPECT_FALSE(IsSaturated(b, x, y));
    }
  }
}

TEST(SaturateWrappedTest, Consistency0) {
  // Various normal and edge case tests of SaturateWrapped.
  //
  // In range, range not wrapped.
  EXPECT_NEAR(0.5, SaturateWrapped(0.5, 0.25, 0.75, 0.0, 1.0), 1e-9);
  // In range before end, range wrapped.
  EXPECT_NEAR(0.1, SaturateWrapped(0.1, 0.75, 0.25, 0.0, 1.0), 1e-9);
  // In range after start, range wrapped.
  EXPECT_NEAR(0.9, SaturateWrapped(0.9, 0.75, 0.25, 0.0, 1.0), 1e-9);
  // Out of range, range not wrapped, closest to start.
  EXPECT_NEAR(0.25, SaturateWrapped(0.2, 0.25, 0.75, 0.0, 1.0), 1e-9);
  // Out of range, range not wrapped, closest to end around wrap.
  EXPECT_NEAR(0.9, SaturateWrapped(0.1, 0.8, 0.9, 0.0, 1.0), 1e-9);
  // Out of range, range wrapped, closest to end.
  EXPECT_NEAR(0.1, SaturateWrapped(0.2, 0.9, 0.1, 0.0, 1.0), 1e-9);
  // Out of range, range wrapped, closest to start.
  EXPECT_NEAR(0.9, SaturateWrapped(0.8, 0.9, 0.1, 0.0, 1.0), 1e-9);
  // Equal to range start, range wrapped.
  EXPECT_NEAR(0.75, SaturateWrapped(0.75, 0.75, 0.25, 0.0, 1.0), 1e-9);
  // In range, equal to range start, range wrapped.
  EXPECT_NEAR(0.75, SaturateWrapped(0.75, 0.75, 0.25, 0.0, 1.0), 1e-9);
  // In range, equal to range end, range wrapped.
  EXPECT_NEAR(0.25, SaturateWrapped(0.25, 0.75, 0.25, 0.0, 1.0), 1e-9);
  // In range, equal to wrap right, which should wrap to wrap left.
  EXPECT_NEAR(0.0, SaturateWrapped(1.0, 0.75, 0.25, 0.0, 1.0), 1e-9);
  // In range, range equal to wrap.
  EXPECT_NEAR(0.5, SaturateWrapped(0.5, 0.0, 1.0, 0.0, 1.0), 1e-9);
  // In range at wrap end, range equal to wrap, should wrap to wrap left.
  EXPECT_NEAR(0.0, SaturateWrapped(1.0, 0.0, 1.0, 0.0, 1.0), 1e-9);
  // Out of range, equidistant from ends should choose start.
  EXPECT_NEAR(0.25, SaturateWrapped(0.0, 0.25, 0.75, 0.0, 1.0), 1e-9);
}

TEST(SaturateVecTest, Normal0) {
  for (int32_t t = 0; t < test_util::kNumTests; t++) {
    const int32_t len =
        rand() % VEC_MAX_ELEMENTS;  // NOLINT(runtime/threadsafe_fn)
    VEC(len, v);
    VEC(len, v_low);
    VEC(len, v_high);
    VEC(len, v_out);
    for (int32_t i = 0; i < len; i++) {
      double a = Rand(-1e9, 1e9);
      double b = Rand(-1e9, 1e9);
      if (a > b) SwapInPlace(&a, &b);
      *VecPtr(&v, i) = Rand(-1e9, 1e9);
      *VecPtr(&v_low, i) = a;
      *VecPtr(&v_high, i) = b;
      *VecPtr(&v_out, i) = 0.0;
    }
    SaturateVec(&v, &v_low, &v_high, &v_out);
    for (int32_t i = 0; i < len; i++) {
      EXPECT_EQ(
          Saturate(*VecPtr(&v, i), *VecPtr(&v_low, i), *VecPtr(&v_high, i)),
          *VecPtr(&v_out, i));
    }
  }

  const Vec2 v2_low = {0.1, 0.2}, v2_high = {0.2, 0.5};
  const Vec2 v2 = {RandNormal(), RandNormal()};
  Vec2 v2_out;
  SaturateVec2(&v2, &v2_low, &v2_high, &v2_out);
  EXPECT_NEAR(v2_out.x, Saturate(v2.x, v2_low.x, v2_high.x), 1e-9);
  EXPECT_NEAR(v2_out.y, Saturate(v2.y, v2_low.y, v2_high.y), 1e-9);

  const Vec3 v3_low = {0.1, 0.2, 0.3}, v3_high = {0.2, 0.5, 0.9};
  const Vec3 v3 = {RandNormal(), RandNormal(), RandNormal()};
  Vec3 v3_out;
  SaturateVec3(&v3, &v3_low, &v3_high, &v3_out);
  EXPECT_NEAR(v3_out.x, Saturate(v3.x, v3_low.x, v3_high.x), 1e-9);
  EXPECT_NEAR(v3_out.y, Saturate(v3.y, v3_low.y, v3_high.y), 1e-9);
  EXPECT_NEAR(v3_out.z, Saturate(v3.z, v3_low.z, v3_high.z), 1e-9);
}

TEST(FabsVec3, Consistency0) {
  Vec3 v3 = {-2.0, 100.0, -1.0e6};
  FabsVec3(&v3, &v3);
  EXPECT_EQ(2.0, v3.x);
  EXPECT_EQ(100.0, v3.y);
  EXPECT_EQ(1.0e6, v3.z);

  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    const Vec3 v_in = {RandNormal(), RandNormal(), RandNormal()};
    double norm_in = Vec3Norm(&v_in);
    Vec3 v_out;
    FabsVec3(&v_in, &v_out);
    EXPECT_EQ(fabs(v_in.x), v_out.x);
    EXPECT_EQ(fabs(v_in.y), v_out.y);
    EXPECT_EQ(fabs(v_in.z), v_out.z);
    EXPECT_EQ(Vec3Norm(&v_out), norm_in);
  }
}

TEST(SaturateVec3ByScalar, Consistency0) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    Vec3 a, b;
    double tmp, l = RandNormal(), h = RandNormal();
    if (h < l) {
      tmp = l;
      l = h;
      h = tmp;
    }
    a.x = RandNormal();
    a.y = RandNormal();
    a.z = RandNormal();

    SaturateVec3ByScalar(&a, l, h, &b);
    if (a.x >= h) {
      EXPECT_EQ(b.x, h);
    } else if (a.x <= l) {
      EXPECT_EQ(b.x, l);
    } else {
      EXPECT_EQ(b.x, a.x);
    }

    if (a.y >= h) {
      EXPECT_EQ(b.y, h);
    } else if (a.y <= l) {
      EXPECT_EQ(b.y, l);
    } else {
      EXPECT_EQ(b.y, a.y);
    }

    if (a.z >= h) {
      EXPECT_EQ(b.z, h);
    } else if (a.z <= l) {
      EXPECT_EQ(b.z, l);
    } else {
      EXPECT_EQ(b.z, a.z);
    }
  }
}

TEST(SaturateArrayByScalar, Consistency0) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    double a[4], b[4];
    double tmp, l = RandNormal(), h = RandNormal();
    if (h < l) {
      tmp = l;
      l = h;
      h = tmp;
    }
    a[0] = RandNormal();
    a[1] = RandNormal();
    a[2] = RandNormal();

    SaturateArrayByScalar(a, ARRAYSIZE(a), l, h, b);

    for (int32_t j = 0; j < ARRAYSIZE(a); ++j) {
      if (a[j] >= h) {
        EXPECT_EQ(b[j], h);
      } else if (a[j] <= l) {
        EXPECT_EQ(b[j], l);
      } else {
        EXPECT_EQ(b[j], a[j]);
      }
    }
  }
}

TEST(Crossfade, Normal) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    double y_lower = RandNormal();
    double y_upper = RandNormal();
    double x_lower = RandNormal();
    double x_upper = RandNormal();

    if (x_lower > x_upper) SwapInPlace(&x_lower, &x_upper);
    if (fabs(x_lower - x_upper) < DBL_EPSILON) x_upper += 1.0;

    EXPECT_EQ(y_lower,
              Crossfade(y_lower, y_upper, x_lower - 1.0, x_lower, x_upper));

    EXPECT_EQ(y_lower, Crossfade(y_lower, y_upper, x_lower, x_lower, x_upper));

    EXPECT_NEAR(0.5 * (y_lower + y_upper),
                Crossfade(y_lower, y_upper, 0.5 * (x_lower + x_upper), x_lower,
                          x_upper),
                DBL_EPSILON * (fabs(y_lower) + fabs(y_upper)) *
                    (fabs(x_lower) + fabs(x_upper)) / (x_upper - x_lower));

    EXPECT_EQ(y_upper, Crossfade(y_lower, y_upper, x_upper, x_lower, x_upper));

    EXPECT_EQ(y_upper,
              Crossfade(y_lower, y_upper, x_upper + 1.0, x_lower, x_upper));
  }
}

TEST(CrossfadeTest, ZeroLowZeroHigh) {
  EXPECT_EQ(0.0, Crossfade(0.0, 1.0, 0.0, 0.0, 0.0));
  EXPECT_EQ(0.0, Crossfade(0.0, 1.0, -1.0, 0.0, 0.0));
  EXPECT_EQ(1.0, Crossfade(0.0, 1.0, 1.0, 0.0, 0.0));
}

#if !defined(NDEBUG)
TEST(CrossfadeDeathTest, LowGreaterThanHigh) {
  EXPECT_DEATH(Crossfade(0.0, 1.0, 0.0, 1.0, 0.0), "");
}
#endif  // !defined(NDEBUG)

TEST(CrossfadeVecTest, Normal0) {
  double fade = 0.4, fade_low = 0.1, fade_high = 0.9;

  const Vec2 v2_a = {RandNormal(), RandNormal()};
  const Vec2 v2_b = {RandNormal(), RandNormal()};
  Vec2 v2_out;
  CrossfadeVec2(&v2_a, &v2_b, fade, fade_low, fade_high, &v2_out);
  EXPECT_NEAR(v2_out.x, Crossfade(v2_a.x, v2_b.x, fade, fade_low, fade_high),
              1e-9);
  EXPECT_NEAR(v2_out.y, Crossfade(v2_a.y, v2_b.y, fade, fade_low, fade_high),
              1e-9);

  const Vec3 v3_a = {RandNormal(), RandNormal(), RandNormal()};
  const Vec3 v3_b = {RandNormal(), RandNormal(), RandNormal()};
  Vec3 v3_out;
  CrossfadeVec3(&v3_a, &v3_b, fade, fade_low, fade_high, &v3_out);
  EXPECT_NEAR(v3_out.x, Crossfade(v3_a.x, v3_b.x, fade, fade_low, fade_high),
              1e-9);
  EXPECT_NEAR(v3_out.y, Crossfade(v3_a.y, v3_b.y, fade, fade_low, fade_high),
              1e-9);
  EXPECT_NEAR(v3_out.z, Crossfade(v3_a.z, v3_b.z, fade, fade_low, fade_high),
              1e-9);
}

TEST(CrossfadeMat3Test, Normal0) {
  Mat3 m1 = kMat3Zero;
  Mat3 m2 = {{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}}};
  Mat3 m_out, m_compare;

  CrossfadeMat3(&m1, &m2, -100.0, 0.0, 100.0, &m_out);
  EXPECT_NEAR_MAT3(m1, m_out, 1e-9);

  CrossfadeMat3(&m1, &m2, 200.0, 0.0, 100.0, &m_out);
  EXPECT_NEAR_MAT3(m2, m_out, 1e-9);

  CrossfadeMat3(&m1, &m2, 50.0, 0.0, 100.0, &m_out);
  Mat3Scale(&m2, 0.5, &m_compare);
  EXPECT_NEAR_MAT3(m_compare, m_out, 1e-9);
}

TEST(Interp1Test, Normal0) {
  double x[] = {-5.2, -3.2, 0.1, 7.7, 22.2};
  double y[] = {-144.0, 12.2, -34.2, 23.0, 24.2};
  double xi[] = {-10.0, -6.0, -4.0, -3.2, 0.0, 2.3, 7.7, 8.8, 100.0};

  // These values were calculated with MATLAB's:
  // interp1(x, y, xi, 'linear', 'extrap')
  double ans[] = {-518.88, -206.48,           -50.28,
                  12.2,    -32.7939393939394, -17.6421052631579,
                  23.0,    23.0910344827586,  30.6386206896552};
  double yi;
  for (int32_t i = 0; i < ARRAYSIZE(xi); ++i) {
    yi = Interp1(x, y, ARRAYSIZE(x), xi[i], kInterpOptionDefault);
    EXPECT_NEAR(ans[i], yi, 1e-9);
  }
}

TEST(Interp2, SameAtDefinedValues) {
  double x[] = {-1.1111, 2.5};
  double y[] = {10.0, 20.999, 30.0};
  const int32_t nx = ARRAYSIZE(x);
  const int32_t ny = ARRAYSIZE(y);

  const double z[ny][nx] = {{0., 1.2222}, {2.5, PI}, {4., -5.1}};

  // Check exact recovery of tabulated data.
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      InterpOption opts[] = {kInterpOptionDefault, kInterpOptionSaturate};
      for (int k = 0; k < ARRAYSIZE(opts); ++k) {
        EXPECT_NEAR(
            Interp2(x, y, nx, ny, (const double *)z, x[i], y[j], opts[k]),
            z[j][i], 1e-9);
      }
    }
  }
}

TEST(Interp2, Extrapolation) {
  const double x[] = {1.0, 2.0};
  const double y[] = {10.0, 20.0, 30.0};
  const int32_t nx = ARRAYSIZE(x);
  const int32_t ny = ARRAYSIZE(y);

  const double z[ny][nx] = {{0., 1.}, {2., 3.}, {4., 5.}};

  // Check extrapolation options.
  EXPECT_NEAR(
      Interp2(x, y, nx, ny, (const double *)z, 1.0, 0.0, kInterpOptionSaturate),
      0.0, 1e-9);
  EXPECT_NEAR(
      Interp2(x, y, nx, ny, (const double *)z, 1.0, 0.0, kInterpOptionDefault),
      -2.0, 1e-9);
  EXPECT_NEAR(Interp2(x, y, nx, ny, (const double *)z, 1.0, 40.0,
                      kInterpOptionSaturate),
              4.0, 1e-9);
  EXPECT_NEAR(
      Interp2(x, y, nx, ny, (const double *)z, 1.0, 40.0, kInterpOptionDefault),
      6.0, 1e-9);

  EXPECT_NEAR(Interp2(x, y, nx, ny, (const double *)z, 0.0, 30.0,
                      kInterpOptionSaturate),
              4.0, 1e-9);
  EXPECT_NEAR(
      Interp2(x, y, nx, ny, (const double *)z, 0.0, 30.0, kInterpOptionDefault),
      3.0, 1e-9);
  EXPECT_NEAR(Interp2(x, y, nx, ny, (const double *)z, 3.0, 30.0,
                      kInterpOptionSaturate),
              5.0, 1e-9);
  EXPECT_NEAR(
      Interp2(x, y, nx, ny, (const double *)z, 3.0, 30.0, kInterpOptionDefault),
      6.0, 1e-9);
}

TEST(Interp2, MATLAB) {
  // Test data computed using the following MATLAB script:
  //
  // nx = 5;
  // ny = 3;
  //
  // x = linspace(1.0, 5.0, nx);
  // y = linspace(10.0, 30.0, ny);
  //
  // A = [100.0, 20.0, 13.0, 1.0, 19.0;
  //      110.0, -20.0, 1000.0, 2.0, 5.0;
  //      100.0, 20.5, 22.0, -8.7, 200.6];
  //
  // n_tests = 10;
  // test_x = (x(end) - x(1)) * rand(1, n_tests) + x(1);
  // test_y = (y(end) - y(1)) * rand(1, n_tests) + y(1);
  //
  // test_z = interp2(x, y, A, test_x, test_y, 'linear');
  //
  // testdata = [test_x.', test_y.', test_z.']
  //
  // Note: MATLAB's interp2 function does not support extrapolation.

  const int nx = 5;
  const int ny = 3;

  const double x[nx] = {1.0, 2.0, 3.0, 4.0, 5.0};
  const double y[ny] = {10.0, 20.0, 30.0};
  const double z[ny][nx] = {{100.0, 20.0, 13.0, 1.0, 19.0},
                            {110.0, -20.0, 1000.0, 2.0, 5.0},
                            {100.0, 20.5, 22.0, -8.7, 200.6}};

  const double testdata[10][3] = {
      {2.03225878364827, 18.4833351942761, 13.9459428036958},
      {2.63487938445021, 20.1571656932224, 618.050772011439},
      {3.37958429603446, 11.7103159418009, 113.241157328705},
      {2.04884699112338, 15.2496446939667, 24.9947863019506},
      {3.41137235752833, 26.0202924553948, 240.225552273432},
      {3.84486312173473, 10.5844055512429, 11.8594403863746},
      {1.88698693606896, 28.5770827895609, 24.5338054602825},
      {1.46967060342322, 24.6066172571091, 55.2623474676474},
      {2.18670349287331, 19.7721794760716, 166.980511873494},
      {2.27511320770353, 21.5705012204688, 222.97011754129}};

  // Because we are not extrapolating, the results should be the same
  // with either extrapolation option.
  const InterpOption opts[] = {kInterpOptionDefault, kInterpOptionSaturate};
  int tests = 0;
  for (int i = 0; i < ARRAYSIZE(opts); ++i) {
    for (int j = 0; j < ARRAYSIZE(testdata); ++j) {
      EXPECT_NEAR(Interp2(x, y, nx, ny, (const double *)z, testdata[j][0],
                          testdata[j][1], opts[i]),
                  testdata[j][2], 1e-9);
      tests++;
    }
  }

  // Make sure we actually ran the expected number of tests.
  EXPECT_EQ(tests, 20);
}

TEST(Interp1WarpY, Normal) {
  double x[] = {0.0, 1.0, 2.0, 3.0, 4.0};
  double y[] = {0.0, 1.0, 2.0, 3.0, 4.0};
  double xi[] = {0.1, 1.2, 2.2, 3.3, 3.5};
  double yi_ans[] = {sqrt(0.1), sqrt(1.0 + 0.2 * 3.0), sqrt(4.0 + 0.2 * 5.0),
                     sqrt(9.0 + 0.3 * 7.0), sqrt(9.0 + 0.5 * 7.0)};
  for (int32_t i = 0; i < ARRAYSIZE(x); ++i) {
    double yi = Interp1WarpY(x, y, ARRAYSIZE(x), xi[i], kInterpOptionDefault,
                             &Square, &sqrt);
    EXPECT_NEAR(yi_ans[i], yi, DBL_EPSILON);

    yi = Interp1WarpY(x, y, ARRAYSIZE(x), xi[i], kInterpOptionSaturate, &Square,
                      &sqrt);
    EXPECT_NEAR(yi_ans[i], yi, DBL_EPSILON);
  }
}

TEST(Interp1WarpY, SameAtDefinedValues) {
  double x[] = {0.0, 1.0, 2.0, 3.0, 4.0};
  double y[] = {0.0, 1.0, 2.0, 3.0, 4.0};

  for (int32_t i = 0; i < ARRAYSIZE(x); ++i) {
    double yi = Interp1WarpY(x, y, ARRAYSIZE(x), x[i], kInterpOptionDefault,
                             &Square, &sqrt);
    EXPECT_NEAR(y[i], yi, DBL_EPSILON);
  }
}

TEST(CircularInterp1Test, Normal0) {
  double x[] = {0.0, 1.0, 2.0, 2.7, PI};
  double y[] = {1.0, -3.0, 7.5, 14.6, 1.0};
  double xi[] = {-1.0, 0.5, 1.0, 2.5, PI, 4.0};

  double ans[] = {8.93615406, -1.0, -3.0, 12.57142857, 1.0, -2.43362939};
  double yi;
  for (int32_t i = 0; i < ARRAYSIZE(xi); ++i) {
    yi = CircularInterp1(x, y, ARRAYSIZE(x), xi[i]);
    EXPECT_NEAR(ans[i], yi, 1e-7);
  }
}

TEST(Interp1Vec3Test, Normal0) {
  double t[22], vx[22], vy[22], vz[22];
  Vec3 v[22], vo;
  for (int32_t i = 0; i < ARRAYSIZE(t); ++i) {
    if (i == 0) {
      t[i] = Rand();
    } else {
      t[i] = t[i - 1] + Rand();
    }
    vx[i] = RandNormal();
    vy[i] = RandNormal();
    vz[i] = RandNormal();
    v[i].x = vx[i];
    v[i].y = vy[i];
    v[i].z = vz[i];
  }

  double vox = Interp1(t, vx, ARRAYSIZE(t), t[10] + 2.2, kInterpOptionDefault);
  double voy = Interp1(t, vy, ARRAYSIZE(t), t[10] + 2.2, kInterpOptionDefault);
  double voz = Interp1(t, vz, ARRAYSIZE(t), t[10] + 2.2, kInterpOptionDefault);
  Interp1Vec3(t, v, ARRAYSIZE(t), t[10] + 2.2, kInterpOptionDefault, &vo);
  EXPECT_NEAR(vox, vo.x, 1e-9);
  EXPECT_NEAR(voy, vo.y, 1e-9);
  EXPECT_NEAR(voz, vo.z, 1e-9);
}

TEST(Sigmoid, Limits) {
  double ws[] = {0.1, 1.0, 10.0, 100.0};

  for (int32_t i = 0; i < ARRAYSIZE(ws); ++i) {
    double w = ws[i];

    EXPECT_EQ(0.0, Sigmoid(-INFINITY, w));
    EXPECT_EQ(1.0, Sigmoid(INFINITY, w));
    EXPECT_NEAR(0.5, Sigmoid(0.0, w), 1e-12);

    EXPECT_NEAR(0.0, Sigmoid(-10.0 * w, w), 1e-2);
    EXPECT_NEAR(1.0, Sigmoid(10.0 * w, w), 1e-2);

    EXPECT_NEAR(0.05, Sigmoid(-w / 2.0, w), 1e-3 * 0.05);
    EXPECT_NEAR(0.95, Sigmoid(w / 2.0, w), 1e-3 * 0.05);
  }
}

TEST(PolyFit2, Normal0) {
  double x[3] = {1.0, 2.0, 3.0};
  double y[3] = {3.001, 5.998, 11.002};
  double ans[3] = {1.003499999999999, -0.013499999999993, 2.010999999999990};
  double coeff[3];

  PolyFit2(x, y, coeff);

  for (int32_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(coeff[i], ans[i], 1e-9);
  }
}

TEST(PolyVal, X1) {
  double c[] = {2.0, 3.0, 4.0};
  double y = PolyVal(c, 1.0, ARRAYSIZE(c));
  EXPECT_NEAR(y, 2 + 3 + 4, 1e-9);
}

TEST(PolyVal, Rand3) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    double c[] = {Rand(), Rand(), Rand()};
    double x = Rand();
    double y = PolyVal(c, x, ARRAYSIZE(c));
    EXPECT_NEAR(y, c[0] * x * x + c[1] * x + c[2], 1e-9);
  }
}

TEST(PolyVal, Rand4) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    double c[] = {RandNormal(), RandNormal(), RandNormal(), RandNormal()};
    double x = RandNormal();
    double y = PolyVal(c, x, ARRAYSIZE(c));
    double y_check = c[0] * x * x * x + c[1] * x * x + c[2] * x + c[3];
    EXPECT_NEAR(y / y_check, 1.0, 1e-9);
  }
}

TEST(PolyDer, Ones) {
  double c[] = {1.0, 1.0, 1.0, 1.0, 1.0};
  double dc[ARRAYSIZE(c) - 1];
  PolyDer(c, ARRAYSIZE(dc), dc);
  EXPECT_NEAR(dc[0], 4.0, 1e-9);
  EXPECT_NEAR(dc[1], 3.0, 1e-9);
  EXPECT_NEAR(dc[2], 2.0, 1e-9);
  EXPECT_NEAR(dc[3], 1.0, 1e-9);
}

TEST(ApplyCalInvertCal, Invert) {
  CalParams cal_params;
  double raw_in, raw_out, cal;

  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    cal_params.scale = Rand(-1e3, 1e4);
    cal_params.bias = Rand(-1e3, 1e3);
    cal_params.bias_count = static_cast<int32_t>(Rand(-1e5, 1e5));
    raw_in = Rand(-1e5, 1e5);

    cal = ApplyCal(raw_in, &cal_params);
    raw_out = InvertCal(cal, &cal_params);

    EXPECT_NEAR(raw_in, raw_out, 1e-9);
  }
}

TEST(ApplyEncoderCalInvertEncoderCal, Invert) {
  EncoderCalParams params;
  int32_t raw_in, raw_out;

  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    params.cal.scale = 2.0 * PI / 4096.0;
    params.cal.bias = 0.0;
    params.cal.bias_count = static_cast<int32_t>(Rand(-4096.0, 4096.0));
    params.encoder_counts = 4096;
    params.cal_val_center = Rand(0.0, 1.0);

    raw_in = static_cast<int32_t>(Rand(0.0, 4096.0));
    raw_out = InvertEncoderCal(ApplyEncoderCal(raw_in, &params), &params);

    // TODO: Apply and invert can result in an error of one
    // count.
    EXPECT_LE(abs((raw_in % (params.encoder_counts - 1)) -
                  (raw_out % (params.encoder_counts - 1))),
              1);
  }
}

TEST(Wrap, NegPiToPi) {
  double x[] = {-4.0 * PI - 0.01,
                -4.0 * PI,
                -4.0 * PI + 0.01,
                -3.0 * PI,
                -1.5 * PI,
                -PI,
                -0.1,
                0.0,
                0.1,
                PI,
                1.5 * PI,
                3.0 * PI,
                4.0 * PI - 0.01,
                4.0 * PI,
                4.0 * PI + 0.01};
  double ans[] = {-0.01, 0.0, 0.01,      -PI, 0.5 * PI, -PI, -0.1, 0.0,
                  0.1,   -PI, -0.5 * PI, -PI, -0.01,    0.0, 0.01};
  for (int32_t i = 0; i < ARRAYSIZE(x); ++i) {
    EXPECT_NEAR(Wrap(x[i], -PI, PI), ans[i], 1e-9);
  }
}

TEST(Wrap, ZeroTo2Pi) {
  double x[] = {-4.0 * PI - 0.01,
                -4.0 * PI,
                -4.0 * PI + 0.01,
                -3.0 * PI,
                -1.5 * PI,
                -PI,
                -0.1,
                0.0,
                0.1,
                PI,
                1.5 * PI,
                3.0 * PI,
                4.0 * PI - 0.01,
                4.0 * PI,
                4.0 * PI + 0.01};
  double ans[] = {2.0 * PI - 0.01, 0.0, 0.01, PI, 0.5 * PI, PI,
                  2.0 * PI - 0.1,  0.0, 0.1,  PI, 1.5 * PI, PI,
                  2.0 * PI - 0.01, 0.0, 0.01};
  for (int32_t i = 0; i < ARRAYSIZE(x); ++i) {
    EXPECT_NEAR(Wrap(x[i], 0.0, 2.0 * PI), ans[i], 1e-9);
  }
}

TEST(Wrap, Other) {
  double x[] = {-10.9, -0.9, 0.1, 10.1};
  for (int32_t i = 0; i < ARRAYSIZE(x); ++i) {
    EXPECT_NEAR(Wrap(x[i], 0.0, 1.0), 0.1, 1e-9);
    EXPECT_NEAR(Wrap(x[i], 1.0, 2.0), 1.1, 1e-9);
    EXPECT_NEAR(Wrap(x[i], -1.0, 0.0), -0.9, 1e-9);
    EXPECT_NEAR(Wrap(x[i], -2.0, -1.0), -1.9, 1e-9);
  }
}

TEST(WrapInt32, StdInt) {
  int32_t x[] = {-129, -385, -641, -128, -384, -640, 127, 383, 639,
                 128,  384,  640,  0,    -100, -200, 100, 200};
  int32_t ans[] = {127,  127,  127,  -128, -128, -128, 127, 127, 127,
                   -128, -128, -128, 0,    -100, 56,   100, -56};
  int32_t bits = 8;
  int32_t low = -(1 << (bits - 1));
  int32_t high = low + (1 << bits);

  for (int32_t i = 0; i < ARRAYSIZE(x); ++i) {
    EXPECT_EQ(WrapInt32(x[i], low, high), ans[i]);
  }
}

TEST(WrapInt32, Arbitrary) {
  int32_t x[] = {-14, -111, -208, -13, -110, -207, 83, 180, 277,
                 84,  181,  278,  0,   -10,  -20,  10, 100};
  int32_t ans[] = {83,  83,  83,  -13, -13, -13, 83, 83, 83,
                   -13, -13, -13, 0,   -10, 77,  10, 3};
  int32_t low = -13;
  int32_t high = low + 97;

  for (int32_t i = 0; i < ARRAYSIZE(x); ++i) {
    EXPECT_EQ(WrapInt32(x[i], low, high), ans[i]);
  }
}

TEST(InsideRange, Basic) {
  EXPECT_TRUE(InsideRange(0.5, 0.0, 1.0));
  EXPECT_FALSE(InsideRange(-1.0, -0.5, 0.0));
  EXPECT_FALSE(InsideRange(2.0, -0.1, 1.0));
}

TEST(InsideRangeWrapped, Normal) {
  EXPECT_TRUE(InsideRangeWrapped(2, 0, 10, 1, 3));
  EXPECT_FALSE(InsideRangeWrapped(0, 0, 10, 1, 3));
  EXPECT_FALSE(InsideRangeWrapped(4, 0, 10, 1, 3));
  EXPECT_TRUE(InsideRangeWrapped(1, 0, 10, 1, 3));
  EXPECT_FALSE(InsideRangeWrapped(3, 0, 10, 1, 3));
}

TEST(InsideRangeWrapped, RangeLimitsEqual) {
  EXPECT_TRUE(InsideRangeWrapped(0, 0, 10, 1, 1));
  EXPECT_TRUE(InsideRangeWrapped(1, 0, 10, 1, 1));
  EXPECT_TRUE(InsideRangeWrapped(5, 0, 10, 1, 1));
}

TEST(InsideRangeWrapped, RangeWrapped) {
  EXPECT_TRUE(InsideRangeWrapped(0, 0, 10, 4, 2));
  EXPECT_FALSE(InsideRangeWrapped(3, 0, 10, 4, 2));
  EXPECT_TRUE(InsideRangeWrapped(8, 0, 10, 4, 2));
  EXPECT_TRUE(InsideRangeWrapped(4, 0, 10, 4, 2));
  EXPECT_FALSE(InsideRangeWrapped(2, 0, 10, 4, 2));
}

TEST(InsideRangeWrapped, NegativeLimits) {
  EXPECT_FALSE(InsideRangeWrapped(-9, -10, -5, -8, -6));
  EXPECT_TRUE(InsideRangeWrapped(-7, -10, -5, -8, -6));
  EXPECT_FALSE(InsideRangeWrapped(-5.5, -10, -5, -8, -6));
  EXPECT_TRUE(InsideRangeWrapped(-8, -10, -5, -8, -6));
  EXPECT_FALSE(InsideRangeWrapped(-6, -10, -5, -8, -6));
}

TEST(InsideRangeWrapped, ZeroCrossingLimits) {
  EXPECT_FALSE(InsideRangeWrapped(-8, -10, 10, -3, 2));
  EXPECT_TRUE(InsideRangeWrapped(-1, -10, 10, -3, 2));
  EXPECT_TRUE(InsideRangeWrapped(0, -10, 10, -3, 2));
  EXPECT_TRUE(InsideRangeWrapped(1, -10, 10, -3, 2));
  EXPECT_FALSE(InsideRangeWrapped(7, -10, 10, -3, 2));
}

TEST(InsideRangeWrapped, ZeroCrossingLimitsRangeWrapped) {
  EXPECT_TRUE(InsideRangeWrapped(-7, -10, 10, 9, -4));
  EXPECT_FALSE(InsideRangeWrapped(-3, -10, 10, 9, -4));
  EXPECT_FALSE(InsideRangeWrapped(0, -10, 10, 9, -4));
  EXPECT_FALSE(InsideRangeWrapped(6, -10, 10, 9, -4));
  EXPECT_TRUE(InsideRangeWrapped(9.5, -10, 10, 9, -4));
}

TEST(SplitVec3Arr, Recovery4) {
  double xs_in[] = {RandNormal(), RandNormal(), RandNormal(), RandNormal()};
  double ys_in[] = {RandNormal(), RandNormal(), RandNormal(), RandNormal()};
  double zs_in[] = {RandNormal(), RandNormal(), RandNormal(), RandNormal()};
  double xs_out[ARRAYSIZE(xs_in)];
  double ys_out[ARRAYSIZE(ys_in)];
  double zs_out[ARRAYSIZE(zs_in)];
  Vec3 vs[ARRAYSIZE(xs_in)];

  JoinVec3Arr(xs_out, ys_out, zs_out, ARRAYSIZE(vs), vs);
  SplitVec3Arr(vs, ARRAYSIZE(vs), xs_in, ys_in, zs_in);

  for (int32_t i = 0; i < ARRAYSIZE(vs); ++i) {
    EXPECT_EQ(xs_in[i], xs_out[i]);
    EXPECT_EQ(ys_in[i], ys_out[i]);
    EXPECT_EQ(zs_in[i], zs_out[i]);
  }
}

TEST(SplitVec3Arr, Recovery1) {
  double xs_in[] = {RandNormal()};
  double ys_in[] = {RandNormal()};
  double zs_in[] = {RandNormal()};
  double xs_out[ARRAYSIZE(xs_in)];
  double ys_out[ARRAYSIZE(ys_in)];
  double zs_out[ARRAYSIZE(zs_in)];
  Vec3 vs[ARRAYSIZE(xs_in)];

  JoinVec3Arr(xs_out, ys_out, zs_out, ARRAYSIZE(vs), vs);
  SplitVec3Arr(vs, ARRAYSIZE(vs), xs_in, ys_in, zs_in);

  for (int32_t i = 0; i < ARRAYSIZE(vs); ++i) {
    EXPECT_EQ(xs_in[i], xs_out[i]);
    EXPECT_EQ(ys_in[i], ys_out[i]);
    EXPECT_EQ(zs_in[i], zs_out[i]);
  }
}

TEST(DegToRad, StandardAngles) {
  const double kTolerance = 1e-15;
  EXPECT_NEAR(0.0, DegToRad(0.0), kTolerance);
  EXPECT_NEAR(M_PI / 6.0, DegToRad(30.0), kTolerance);
  EXPECT_NEAR(M_PI / 4.0, DegToRad(45.0), kTolerance);
  EXPECT_NEAR(M_PI / 3.0, DegToRad(60.0), kTolerance);
  EXPECT_NEAR(M_PI / 2.0, DegToRad(90.0), kTolerance);
  EXPECT_NEAR(M_PI, DegToRad(180.0), kTolerance);
  EXPECT_NEAR(2.0 * M_PI, DegToRad(360.0), kTolerance);
}

TEST(RadToDeg, StandardAngles) {
  const double kTolerance = 1e-15;
  EXPECT_NEAR(30.0, RadToDeg(M_PI / 6.0), kTolerance);
  EXPECT_NEAR(45.0, RadToDeg(M_PI / 4.0), kTolerance);
  EXPECT_NEAR(60.0, RadToDeg(M_PI / 3.0), kTolerance);
  EXPECT_NEAR(90.0, RadToDeg(M_PI / 2.0), kTolerance);
  EXPECT_NEAR(180.0, RadToDeg(M_PI), kTolerance);
  EXPECT_NEAR(360.0, RadToDeg(2.0 * M_PI), kTolerance);
}

TEST(RadToDeg, InverseOfDegToRad) {
  const double kTolerance = 1e-13;
  for (int32_t i = 0; i < 100; ++i) {
    double deg = Rand(-360.0, 360.0);
    EXPECT_NEAR(deg, RadToDeg(DegToRad(deg)), kTolerance);
  }
  for (int32_t i = 0; i < 100; ++i) {
    double rad = Rand(-2.0 * M_PI, 2.0 * M_PI);
    EXPECT_NEAR(rad, DegToRad(RadToDeg(rad)), kTolerance);
  }
}

TEST(Exp10, CompareToPow) {
  EXPECT_NEAR(Exp10(22.0), 1e22, 1e8);
  for (int32_t i = 0; i < 100; ++i) {
    double x = Rand(-12.0, 12.0);
    EXPECT_NEAR(pow(10.0, x), Exp10(x), 1e-9 * pow(10.0, x));
  }
}

TEST(Square, CompareToPow) {
  EXPECT_NEAR(Square(22.0), 484.0, 1e-9);
  for (int32_t i = 0; i < 100; ++i) {
    double x = Rand(-1e6, 1e6);
    EXPECT_NEAR(pow(x, 2.0), Square(x), 1e-9);
  }
}

TEST(ThirdPower, CompareToPow) {
  EXPECT_NEAR(ThirdPower(22.0), 10648.0, 1e-9);
  for (int32_t i = 0; i < 100; ++i) {
    double x = Rand(-1e6, 1e6);
    EXPECT_NEAR(pow(x, 3.0), ThirdPower(x), 1e-12 * fabs(pow(x, 3.0)));
  }
}

TEST(FourthPower, CompareToPow) {
  EXPECT_NEAR(FourthPower(22.0), 234256.0, 1e-9);
  for (int32_t i = 0; i < 100; ++i) {
    double x = Rand(-1e6, 1e6);
    EXPECT_NEAR(pow(x, 4.0), FourthPower(x), 1e-12 * pow(x, 4.0));
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

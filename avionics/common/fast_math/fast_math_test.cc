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
#include <math.h>

#include "common/macros.h"
#include "lib/util/test_util.h"

extern "C" {

#include "avionics/common/fast_math/fast_math.h"

}  // extern "C"

using ::test_util::RandNormalf;

TEST(WrapAngle, Normal) {
  for (double theta = -15.0; theta < 15.0; theta += 1.1e-1) {
    double ans = fmod(theta + PI_F, 2.0 * PI_F) - PI_F;
    if (ans < -PI_F) ans += 2.0 * PI_F;
    EXPECT_NEAR(WrapAngle((float)theta), (float)ans, 20.0f * FLT_EPSILON);
  }
}

TEST(ModAngle, Normal) {
  for (double theta = -30.0; theta < 30.0; theta += 1.1e-1) {
    double ans = fmod(theta + PI_F, 2.0 * PI_F) - PI_F;
    if (ans < -PI_F) ans += 2.0 * PI_F;
    EXPECT_NEAR(ModAngle((float)theta), (float)ans, 20.0f * FLT_EPSILON);
  }
}

TEST(Signf, Normal) {
  for (float x = -5.0f; x < 5.0f; x += 0.1234f) {
    float sign_x = Signf(x);

    if (x < 0.0f) {
      EXPECT_EQ(sign_x, -1.0f);
    } else if (x > 0.0f) {
      EXPECT_EQ(sign_x, 1.0f);
    } else {
      EXPECT_EQ(sign_x, 0.0f);
    }
  }

  EXPECT_EQ(Signf(0.0f), 0.0f);
  EXPECT_EQ(Signf(-0.0f), 0.0f);
}

TEST(ISignf, Normal) {
  for (float x = -5.0f; x < 5.0f; x += 0.1234f) {
    int32_t sign_x = ISignf(x);

    if (x < 0.0f) {
      EXPECT_EQ(sign_x, -1);
    } else if (x > 0.0f) {
      EXPECT_EQ(sign_x, 1);
    } else {
      EXPECT_EQ(sign_x, 0);
    }
  }

  EXPECT_EQ(ISignf(0.0f), 0);
  EXPECT_EQ(ISignf(-0.0f), 0);
}

TEST(IRoundf, Normal) {
  EXPECT_EQ(-2, IRoundf(-1.5f - FLT_EPSILON));
  EXPECT_EQ(-2, IRoundf(-1.5f));
  EXPECT_EQ(-1, IRoundf(-1.5f + FLT_EPSILON));

  EXPECT_EQ(-1, IRoundf(-1.0f));

  EXPECT_EQ(-1, IRoundf(-0.5f - FLT_EPSILON));
  EXPECT_EQ(-1, IRoundf(-0.5f));
  EXPECT_EQ(0, IRoundf(-0.5f + FLT_EPSILON));

  EXPECT_EQ(0, IRoundf(-FLT_EPSILON));
  EXPECT_EQ(0, IRoundf(0.0f));
  EXPECT_EQ(0, IRoundf(FLT_EPSILON));

  EXPECT_EQ(0, IRoundf(0.5f - FLT_EPSILON));
  EXPECT_EQ(1, IRoundf(0.5f));
  EXPECT_EQ(1, IRoundf(0.5f + FLT_EPSILON));

  EXPECT_EQ(1, IRoundf(1.0f));

  EXPECT_EQ(1, IRoundf(1.5f - FLT_EPSILON));
  EXPECT_EQ(2, IRoundf(1.5f));
  EXPECT_EQ(2, IRoundf(1.5f + FLT_EPSILON));
}

TEST(IRoundf, Large) {
  EXPECT_EQ(-1001, IRoundf(-1000.5f - 1000.0f * FLT_EPSILON));
  EXPECT_EQ(-1001, IRoundf(-1000.5f));
  EXPECT_EQ(-1000, IRoundf(-1000.5f + 1000.0f * FLT_EPSILON));

  EXPECT_EQ(1000, IRoundf(1000.5f - 1000.0f * FLT_EPSILON));
  EXPECT_EQ(1001, IRoundf(1000.5f));
  EXPECT_EQ(1001, IRoundf(1000.5f + 1000.0f * FLT_EPSILON));
}

TEST(Crossfadef, Normal) {
  for (int32_t i = 0; i < test_util::kNumTests; ++i) {
    float y_lower = RandNormalf();
    float y_upper = RandNormalf();
    float x_lower = RandNormalf();
    float x_upper = RandNormalf();

    if (x_lower > x_upper) Swapf(&x_lower, &x_upper);
    if (fabsf(x_lower - x_upper) < FLT_EPSILON) x_upper += 1.0f;

    float tol = 2.0f * FLT_EPSILON * (fabsf(y_lower) + fabsf(y_upper))
        * (fabsf(x_lower) + fabsf(x_upper)) / (x_upper - x_lower);

    EXPECT_EQ(y_lower,
              Crossfadef(y_lower, y_upper, x_lower - 1.0f, x_lower, x_upper));

    EXPECT_EQ(y_lower,
              Crossfadef(y_lower, y_upper, x_lower, x_lower, x_upper));

    EXPECT_NEAR(0.75f * y_lower + 0.25f * y_upper,
                Crossfadef(y_lower, y_upper, 0.75f * x_lower + 0.25f * x_upper,
                           x_lower, x_upper), tol);

    EXPECT_NEAR(0.5f * (y_lower + y_upper),
                Crossfadef(y_lower, y_upper, 0.5f * x_lower + 0.5f * x_upper,
                           x_lower, x_upper), tol);

    EXPECT_NEAR(0.25f * y_lower + 0.75f * y_upper,
                Crossfadef(y_lower, y_upper, 0.25f * x_lower + 0.75f * x_upper,
                           x_lower, x_upper), tol);

    EXPECT_EQ(y_upper,
              Crossfadef(y_lower, y_upper, x_upper, x_lower, x_upper));

    EXPECT_EQ(y_upper,
              Crossfadef(y_lower, y_upper, x_upper + 1.0f, x_lower, x_upper));
  }
}

TEST(Atan2Lookup, Normal) {
  for (float x = -100.0f; x < 100.0f; x += 1.1e-1f) {
    for (float y = -100.0f; y < 100.0f; y += 1.3e-1f) {
      EXPECT_NEAR(Atan2Lookup(y, x), atan2f(y, x), 10.0f * FLT_EPSILON);
    }
  }
}

TEST(Atan2Lookup, MagnitudeLessThanKPi) {
  EXPECT_LE(Atan2Lookup(0.0f, -FLT_MAX), PI_F);
  EXPECT_LE(Atan2Lookup(0.0f, -INFINITY), PI_F);
  EXPECT_GE(Atan2Lookup(-FLT_MIN, -FLT_MAX), -PI_F);
  EXPECT_GE(Atan2Lookup(-FLT_MIN, -INFINITY), -PI_F);
}

TEST(Atan2Lookup, Small) {
  EXPECT_NEAR(Atan2Lookup(FLT_MIN, 0.0f), PI_F / 2.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-FLT_MIN, 0.0f), -PI_F / 2.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(FLT_MIN, FLT_MIN), PI_F / 4.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-FLT_MIN, FLT_MIN), -PI_F / 4.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(FLT_MIN, -FLT_MIN), 3.0f * PI_F / 4.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-FLT_MIN, -FLT_MIN),
              -3.0f * PI_F / 4.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(0.0f, FLT_MIN), 0.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(0.0f, -FLT_MIN), PI_F, FLT_EPSILON);
}

TEST(Atan2Lookup, Large) {
  EXPECT_NEAR(Atan2Lookup(FLT_MAX, 0.0f), PI_F / 2.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-FLT_MAX, 0.0f), -PI_F / 2.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(FLT_MAX, FLT_MAX), PI_F / 4.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-FLT_MAX, FLT_MAX), -PI_F / 4.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(FLT_MAX, -FLT_MAX), 3.0f * PI_F / 4.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-FLT_MAX, -FLT_MAX),
              -3.0f * PI_F / 4.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(0.0f, FLT_MAX), 0.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(0.0f, -FLT_MAX), PI_F, FLT_EPSILON);
}

TEST(Atan2Lookup, SmallLarge) {
  EXPECT_NEAR(Atan2Lookup(FLT_MAX, FLT_MIN), PI_F / 2.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-FLT_MAX, FLT_MIN), -PI_F / 2.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(FLT_MAX, -FLT_MIN), PI_F / 2.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-FLT_MAX, -FLT_MIN), -PI_F / 2.0f, FLT_EPSILON);

  EXPECT_NEAR(Atan2Lookup(FLT_MIN, FLT_MAX), 0.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-FLT_MIN, FLT_MAX), 0.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(FLT_MIN, -FLT_MAX), PI_F, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-FLT_MIN, -FLT_MAX), -PI_F, FLT_EPSILON);
}

TEST(Atan2Lookup, EdgeCases) {
  EXPECT_NEAR(Atan2Lookup(0.1f, 0.0f), PI_F / 2.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-0.1f, 0.0f), -PI_F / 2.0f, FLT_EPSILON);

  // EXPECT_NEAR(Atan2Lookup(0.0f, -0.0f), PI_F, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(0.0f, -0.1f), PI_F, FLT_EPSILON);
  // EXPECT_NEAR(Atan2Lookup(-0.0f, -0.0f), -PI_F, FLT_EPSILON);
  // EXPECT_NEAR(Atan2Lookup(-0.0f, -0.1f), -PI_F, FLT_EPSILON);

  EXPECT_EQ(Atan2Lookup(0.0f, 0.0f), 0.0f);
  EXPECT_TRUE(copysignf(1.0f, Atan2Lookup(0.0f, 0.0f)) > 0.0f);

  EXPECT_EQ(Atan2Lookup(0.0f, 0.1f), 0.0f);
  EXPECT_TRUE(copysignf(1.0f, Atan2Lookup(0.0f, 0.1f)) > 0.0f);

  EXPECT_EQ(Atan2Lookup(-0.0f, 0.0f), -0.0f);
  // EXPECT_TRUE(copysignf(1.0f, Atan2Lookup(-0.0f, 0.0f)) < 0.0f);

  EXPECT_EQ(Atan2Lookup(-0.0f, 0.1f), -0.0f);
  // EXPECT_TRUE(copysignf(1.0f, Atan2Lookup(-0.0f, 0.1f)) < 0.0f);

  EXPECT_NEAR(Atan2Lookup(INFINITY, 0.1f), PI_F / 2.0f, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-INFINITY, 0.1f), -PI_F / 2.0f, FLT_EPSILON);
  // EXPECT_NEAR(Atan2Lookup(INFINITY, -INFINITY),
  //                         3.0f * PI_F / 4.0f, FLT_EPSILON);
  // EXPECT_NEAR(Atan2Lookup(-INFINITY, -INFINITY),
  //                         -3.0f * PI_F / 4.0f, FLT_EPSILON);
  // EXPECT_NEAR(Atan2Lookup(INFINITY, INFINITY), PI_F / 4.0f, FLT_EPSILON);
  // EXPECT_NEAR(Atan2Lookup(-INFINITY, INFINITY), -PI_F / 4.0f, FLT_EPSILON);

  EXPECT_NEAR(Atan2Lookup(0.1f, -INFINITY), PI_F, FLT_EPSILON);
  EXPECT_NEAR(Atan2Lookup(-0.1f, -INFINITY), -PI_F, FLT_EPSILON);
  EXPECT_EQ(Atan2Lookup(0.1f, INFINITY), 0.0f);
  EXPECT_TRUE(copysignf(1.0f, Atan2Lookup(0.1f, INFINITY)) > 0.0f);
  EXPECT_EQ(Atan2Lookup(-0.1f, INFINITY), -0.0f);
  EXPECT_TRUE(copysignf(1.0f, Atan2Lookup(-0.1f, INFINITY)) < 0.0f);
}

TEST(SinLookup, Normal) {
  for (float theta = -6.0f * PI_F; theta < 6.0f * PI_F; theta += 0.001f) {
    EXPECT_NEAR(SinLookup(theta), sinf(theta), 20.0f * FLT_EPSILON);
  }
}

TEST(SinCosLookup, Normal) {
  for (float theta = -6.0f * PI_F; theta < 6.0f * PI_F; theta += 0.001f) {
    float sin_val, cos_val;
    SinCosLookup(theta, &sin_val, &cos_val);
    EXPECT_NEAR(sin_val, sinf(theta), 20.0f * FLT_EPSILON);
    EXPECT_NEAR(cos_val, cosf(theta), 20.0f * FLT_EPSILON);
  }
}

// This test confirms a trigonometry identity used in motor_foc.c as
// an optimization.
TEST(SinCosLookup, CompareToTrigIdentity) {
  for (float theta = -6.0f * PI_F; theta < 6.0f * PI_F; theta += 0.001f) {
    float sin_theta, cos_theta;
    SinCosLookup(theta, &sin_theta, &cos_theta);

    float sin_m_2pi3, cos_m_2pi3;
    SinCosLookup(theta - 2.0f * PI_F / 3.0f, &sin_m_2pi3, &cos_m_2pi3);

    float sin_p_2pi3, cos_p_2pi3;
    SinCosLookup(theta + 2.0f * PI_F / 3.0f, &sin_p_2pi3, &cos_p_2pi3);

    // Calculate sin(theta +/- 2 pi / 3) and cos(theta +/- 2 pi / 3).
    float sin_cos_2pi3 = sin_theta * cosf(2.0f * PI_F / 3.0f);
    float sin_sin_2pi3 = sin_theta * sinf(2.0f * PI_F / 3.0f);
    float cos_sin_2pi3 = cos_theta * sinf(2.0f * PI_F / 3.0f);
    float cos_cos_2pi3 = cos_theta * cosf(2.0f * PI_F / 3.0f);
    float cos_m_2pi3_new = cos_cos_2pi3 + sin_sin_2pi3;
    float cos_p_2pi3_new = cos_cos_2pi3 - sin_sin_2pi3;
    float sin_m_2pi3_new = sin_cos_2pi3 - cos_sin_2pi3;
    float sin_p_2pi3_new = sin_cos_2pi3 + cos_sin_2pi3;

    EXPECT_NEAR(cos_m_2pi3, cos_m_2pi3_new, 20.0f * FLT_EPSILON);
    EXPECT_NEAR(cos_p_2pi3, cos_p_2pi3_new, 20.0f * FLT_EPSILON);
    EXPECT_NEAR(sin_m_2pi3, sin_m_2pi3_new, 20.0f * FLT_EPSILON);
    EXPECT_NEAR(sin_p_2pi3, sin_p_2pi3_new, 20.0f * FLT_EPSILON);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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
#include <stdio.h>

#include <algorithm>

#include "common/macros.h"

extern "C" {
#include "avionics/common/fast_math/filter.h"
}  // extern "C"

// Test the function FirstOrderFilterInit using a combination of finite and
// infinite zeros coupled with finite and zero poles. A Tustin tranformation to
// convert between the s and z-plane is assumed.
TEST(FirstOrderFilterInitTest, Normal) {
  FirstOrderFilterParams params;
  FirstOrderFilterState state;
  float gain[] = {0.1f, 2.0f};
  float s_poles[] = {0.0f, 0.0f, 0.0f,
                     -0.5f, -1.0f, -10.0f,
                     -1.0f, -1.0f, -1.0f};
  float s_zeros[] = {-1.0f, -10.0f, -INFINITY,
                     -INFINITY, -INFINITY, -INFINITY,
                     -0.5f, -5.0f, -20.0};
  float dt[] = {0.2f, 1.0f, 20.0f};
  float y0 = 1;
  float s, z, threshold;

  threshold = 20.0f * FLT_EPSILON;

  for (int32_t i_gain = 0; i_gain < ARRAYSIZE(gain); ++i_gain) {
    for (int32_t i_dt = 0; i_dt < ARRAYSIZE(dt); ++i_dt) {
      for (int32_t i_s = 0; i_s < ARRAYSIZE(s_poles); ++i_s) {
        // Purely real pole and zero.
        FirstOrderFilterInit(gain[i_gain], s_zeros[i_s], s_poles[i_s], dt[i_dt],
                             y0, &params, &state);

        // Filter is to be normalized such that a_0 == 1.
        EXPECT_NEAR(params.a[0], 1.0f, FLT_EPSILON);

        // Recalculate the continuous time pole from the filter coefficients.
        z = -params.a[1] / params.a[0];
        s = (2.0f / dt[i_dt]) * (z - 1.0f) / (z + 1.0f);
        if (s == -INFINITY) {
          EXPECT_EQ(s, s_poles[i_s]);
        } else {
          EXPECT_NEAR(s, s_poles[i_s],
                      threshold / fmaxf(fabsf(z + 1.0f), 0.01f));
        }

        // Recalculate the continuous time zero from the filter coefficients.
        z = -params.b[1] / params.b[0];
        s = (2.0f / dt[i_dt]) * (z - 1.0f) / (z + 1.0f);
        if (s == -INFINITY) {
          EXPECT_EQ(s, s_zeros[i_s]);
        } else {
          EXPECT_NEAR(s, s_zeros[i_s],
                      threshold / fmaxf(fabsf(z + 1.0f), 0.01f));
        }

        if (fabs(s_poles[i_s]) < FLT_EPSILON) {
          EXPECT_NEAR((params.b[0] + params.b[1]) / dt[i_dt],
                      gain[i_gain], threshold);
        } else {
          EXPECT_NEAR((params.b[0] + params.b[1]) / (params.a[0] + params.a[1]),
                      gain[i_gain], 5.0f * threshold
                      / fminf(fabsf(params.a[0] + params.a[1]), 1.0f));
        }
      }
    }
  }
}

// TODO: Add FirstOrderFilterInit test for derivative and constant
// gain.

// Test the FirstOrderFilterReset function for various filters and inputs.
TEST(FirstOrderFilterResetTest, Normal) {
  float gain[] = {0.1f, 2.0f};
  float s_poles[] = {-1.0f,  -1.0f,     -10.0f};
  float s_zeros[] = {-10.0f, -INFINITY, -1.0f};
  float dt[] = {0.02f, 1.0f, 20.0f};
  float y0[] = {0.0f, 5.0f};
  float x0;
  float tol_ref;

  FirstOrderFilterParams params;
  FirstOrderFilterState state;

  for (int32_t i_gain = 0; i_gain < ARRAYSIZE(gain); ++i_gain) {
    for (int32_t i_pole = 0; i_pole < ARRAYSIZE(s_poles); ++i_pole) {
      for (int32_t i_zero = 0; i_zero < ARRAYSIZE(s_zeros); ++i_zero) {
        for (int32_t i_dt = 0; i_dt < ARRAYSIZE(dt); ++i_dt) {
          FirstOrderFilterInit(gain[i_gain], s_zeros[i_zero], s_poles[i_pole],
                               dt[i_dt], -1.0f, &params, &state);

          for (int32_t i_y0 = 0; i_y0 < ARRAYSIZE(y0); ++i_y0) {
            tol_ref = 2 * FLT_EPSILON * y0[i_y0]
                * fmaxf(fabsf(params.b[0]), fabsf(params.b[1]))
                / fabsf(params.b[0] + params.b[1]);
            x0 = y0[i_y0] * (params.a[0] + params.a[1])
                / (params.b[0] + params.b[1]);
            FirstOrderFilter(500.0f, &params, &state);
            FirstOrderFilterReset(y0[i_y0], &params, &state);
            EXPECT_NEAR(FirstOrderFilter(x0, &params, &state),
                        y0[i_y0], tol_ref);
          }
        }
      }
    }
  }
}

// TODO: Add FirstOrderFilterReset test for integrator,
//  derivative, and constant gain.

// Test the linear step response of a filter and whether it is normalized
// correctly to be insensitive to differing time steps.
TEST(FirstOrderFilter, LinearStep) {
  // Create two structs whose time steps differ by an order of magnitude.

  const int32_t kNumCycle = 5;
  const int32_t kTimeFactor = 3;
  const float kInput = 2.0f;
  const float kGain = 10.0f;
  const float kThreshold
      = 2.0f * sqrtf(kTimeFactor) * kInput * kGain * FLT_EPSILON;

  float y0 = 0.0f;
  float s_pole = -1.0f;
  float dt2 = 0.001f, dt1 = kTimeFactor * dt2;

  FirstOrderFilterParams params1, params2;
  FirstOrderFilterState state1, state2;

  FirstOrderFilterInit(kGain, -2.0f / dt1, s_pole, dt1, y0, &params1, &state1);
  FirstOrderFilterInit(kGain, -2.0f / dt2, s_pole, dt2, y0, &params2, &state2);

  float y1, y2;
  for (int i = 0; i < kNumCycle; i++) {  // Outer loop running at dt1.
    y1 = FirstOrderFilter(kInput, &params1, &state1);
    for (int j = 0; j < kTimeFactor; j++) {  // Inner loop running at dt2.
      y2 = FirstOrderFilter(kInput, &params2, &state2);
    }
    // The numeric roundoff error will roughly follow a Bernoulli distribution.
    // With a large number of samples, this will approach a normal distribution
    // with an error growing roughly as sqrt(n_samples) where n_samples =
    // kTimeFactor * (i+1). The kTimeFactor has already been taken care of in
    // kThreshold. Note that this does not represent the worst case error.
    EXPECT_NEAR(y1, y2, kThreshold * sqrtf((float)(i + 1)));
  }

  float dt_total = kNumCycle * dt1;
  EXPECT_NEAR(y1, kGain * (1.0 - expf(s_pole * dt_total)) * kInput,
              4.0f * FLT_EPSILON * kInput * kGain);
  EXPECT_NEAR(y2, kGain * (1.0 - expf(s_pole * dt_total)) * kInput,
              40.0f * FLT_EPSILON * kInput * kGain);
}

// Test asymptotic decay to the windup limit. The input to a filter is chosen so
// that it naturally asymptotes to ylim. Windup is then applied to change the
// pole location and cause the system to converge to ylim at a different rate.
TEST(FirstOrderWindup, AsymptoteToLim) {
  FirstOrderFilterParams filter, filter_ref;
  FirstOrderFilterState state, state_ref;

  const int32_t kNumCycle = 50;
  const float tolerance = 1e-5f, dt = 0.001f, y0 = 10.0f;
  float gain[] = {1.0f, 2.0f};
  float s_zero[] = {-INFINITY, -2.0f / dt};
  float s_pole[] = {-1.0f, -2.0f};
  float s_windup[] = {-2.0f, -10.0f, -50.0f};
  float y_lim[] = {0.0f, 5.0f};

  float y_ref, y, u_limit;

  for (int i_gain = 0; i_gain < ARRAYSIZE(gain); ++i_gain) {
    for (int i_zero = 0; i_zero < ARRAYSIZE(s_zero); ++i_zero) {
      for (int i_pole = 0; i_pole < ARRAYSIZE(s_pole); ++i_pole) {
        for (int i_windup = 0; i_windup < ARRAYSIZE(s_windup); ++i_windup) {
          // Main filter.
          FirstOrderFilterInit(gain[i_gain], s_zero[i_zero], s_pole[i_pole],
                               dt, y0, &filter, &state);
          FirstOrderWindupInit(s_windup[i_windup], dt, &filter);

          // Reference filter for inputs past saturation.
          float gain_aw = (1.0f - filter.k_aw) * gain[i_gain]
              * (filter.a[0] + filter.a[1])
              / (filter.a[0] + filter.a[1] * (1.0f - filter.k_aw));
          FirstOrderFilterInit(gain_aw, s_zero[i_zero], s_windup[i_windup],
                               dt, 0.0f, &filter_ref, &state_ref);

          for (int i_lim = 0; i_lim < ARRAYSIZE(y_lim); ++i_lim) {
            u_limit = y_lim[i_lim] / gain[i_gain];

            // Manually set the initial state to deal with differing gains
            // producing different delayed inputs.
            state.x = u_limit;
            state.y = y0;
            state_ref.x = 0.0f;
            state_ref.y = y0 - y_lim[i_lim];

            for (int i = 0; i < kNumCycle; ++i) {
              // Run the reference filter using the anti-windup pole.
              y_ref = FirstOrderFilter(0.0f, &filter_ref,
                                       &state_ref) + y_lim[i_lim];

              // Run the main filter and then apply windup.
              FirstOrderFilter(u_limit, &filter, &state);
              y = FirstOrderWindup(-y_lim[i_lim], y_lim[i_lim],
                                   &filter, &state);

              EXPECT_NEAR(y, y_ref, tolerance * (y_ref - y_lim[i_lim]));
            }
          }
        }
      }
    }
  }
}

// Test the function RateLimit.
TEST(RateLimitTest, Normal) {
  const float ts[] = {0.1f, 0.2f, 2.0f};
  const float upper_limits[] = {-1.0f, 1.0f, 0.0f, 5.0f};
  const float lower_limits[] = {-5.0f, -1.0f, 0.0f, 1.0f};
  const float rates[] = {-2.0f, -1.0f, 1.0f, 2.0f};

  float u, y = 0.0f, y0, lower_limit, upper_limit;
  float input_step, upper_step, lower_step;

  for (int i_ts = 0; i_ts < ARRAYSIZE(ts); ++i_ts) {
    for (int i_up = 0; i_up < ARRAYSIZE(upper_limits); ++i_up) {
      for (int i_low = 0; i_low < ARRAYSIZE(lower_limits); ++i_low) {
        for (int i_rate = 0; i_rate < ARRAYSIZE(rates); ++i_rate) {
          upper_limit = upper_limits[i_up];
          lower_limit = lower_limits[i_low];
          if (upper_limit < lower_limit) {
            std::swap(upper_limit, lower_limit);
          }

          input_step = rates[i_rate] * ts[i_ts];
          upper_step = upper_limit * ts[i_ts];
          lower_step = lower_limit * ts[i_ts];

          y0 = y;
          u = y0 + input_step;
          RateLimitf(u, lower_limit, upper_limit, ts[i_ts], &y);

          if (input_step > upper_step) {
            EXPECT_NEAR(y, y0 + upper_step, 1e-7);
          } else if (input_step < lower_step) {
            EXPECT_NEAR(y, y0 + lower_step, 1e-7);
          } else {
            EXPECT_NEAR(y, u, 1e-7);
          }
        }
      }
    }
  }
}

TEST(TustinSToZ, Normal) {
  EXPECT_NEAR(TustinSToZ(0.0f, 1.0f), 1.0f, FLT_EPSILON);
  EXPECT_NEAR(TustinSToZ(-1.0f, 1.0f), 0.5f / 1.5f, 2.0f * FLT_EPSILON);
  EXPECT_NEAR(TustinSToZ(-2.0f, 1.0f), 0.0f / 2.0f, 2.0f * FLT_EPSILON);
  EXPECT_NEAR(TustinSToZ(-3.0f, 1.0f), -0.5f / 2.5f, 2.0f * FLT_EPSILON);

  EXPECT_NEAR(TustinSToZ(0.0f, 2.0f), 1.0f, FLT_EPSILON);
  EXPECT_NEAR(TustinSToZ(-0.5f, 2.0f), 0.5f / 1.5f, 2.0f * FLT_EPSILON);
  EXPECT_NEAR(TustinSToZ(-1.0f, 2.0f), 0.0f / 2.0f, 2.0f * FLT_EPSILON);
  EXPECT_NEAR(TustinSToZ(-1.5f, 2.0f), -0.5f / 2.5f, 2.0f * FLT_EPSILON);
}

TEST(TustinSToZ, NegativeInfinity) {
  EXPECT_NEAR(TustinSToZ(-INFINITY, 1.0f), -1.0f, FLT_EPSILON);
  EXPECT_NEAR(TustinSToZ(-INFINITY, 2.0f), -1.0f, FLT_EPSILON);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

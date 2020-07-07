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
#include <gsl/gsl_errno.h>
#include <gsl/gsl_odeiv2.h>
#include <math.h>
#include <stdlib.h>

#include <algorithm>
#include <string>

#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "lib/util/random_matrix.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;
using ::test_util::RandNormal;

TEST(RunningVar, Zero) {
  double x[] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 5.0,
                5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
  double buf[12] = {0.0};
  int32_t ind = 0;
  double var;
  for (int32_t i = 0; i < ARRAYSIZE(x); ++i) {
    var = RunningVar(x[i], ARRAYSIZE(buf), buf, &ind);
  }
  EXPECT_NEAR(var, 0.0, 1e-9);
}

// Compare to MATLAB var([0, 1, ..., 5, 5], 1).  Note that this
// normalizes to N rather than N-1.
TEST(RunningVar, Normal) {
  double x[] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 5.0,
                5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
  double buf[22] = {0.0};
  int32_t ind = 0;
  double var;
  for (int32_t i = 0; i < ARRAYSIZE(x); ++i) {
    var = RunningVar(x[i], 13, buf, &ind);
  }
  EXPECT_NEAR(var, 1.136094674556214, 1e-9);

  ind = 0;
  for (int32_t i = 0; i < ARRAYSIZE(x); ++i) {
    var = RunningVar(x[i], 22, buf, &ind);
  }
  EXPECT_NEAR(var, 3.811983471074383, 1e-9);
}

// This function is deprecated.  Do not use in new code.
static int64_t convscale(double newgain, double basegain) {
  return static_cast<int64_t>(ceil(exp(basegain - newgain)));
}

TEST(LpfHpf, ConvergenceConsistency) {
  const int32_t num_tests = test_util::kNumTests;
  const double kConvergenceTol = 1e-5;
  for (int32_t i = 0; i < num_tests; ++i) {
    const double in = Rand(1e-9, 1e9);
    const double fc = Rand(1e-3, 10.0);
    double state_lp = Rand(1e-9, 1e9);
    double state_hp = Rand(1e-9, 1e9);
    double in_z1 = in;

    // Low pass filter test.
    for (int32_t j = 0; j < test_util::kNumTests * convscale(fc, 18); ++j) {
      state_lp = Lpf(in, fc, 0.01, &state_lp);
      if (j % 22 == 0) {
        if ((fabs(in - state_lp) < kConvergenceTol * fabs(in))) break;
      }
    }
    EXPECT_NEAR(in, state_lp, kConvergenceTol * fabs(in));

    // High pass filter test.
    for (int32_t j = 0; j < test_util::kNumTests * convscale(fc, 18); ++j) {
      Hpf(in, fc, 0.01, &state_hp, &in_z1);
      if (j % 22 == 0) {
        if ((fabs(0.0 - state_hp) < kConvergenceTol * fabs(in))) break;
      }
    }
    EXPECT_NEAR(0.0, state_hp, kConvergenceTol * fabs(in));
  }
}

TEST(Diff, Normal) {
  double u_z1 = 0.0;
  EXPECT_NEAR(Diff(1.0, 1.0, &u_z1), 1.0, 1e-9);
  EXPECT_NEAR(Diff(1.0, 1.0, &u_z1), 0.0, 1e-9);
  EXPECT_NEAR(Diff(-0.1, 1.0, &u_z1), -1.1, 1e-9);
}

TEST(DiffVec3, Normal) {
  Vec3 u_z1 = kVec3Zero;
  Vec3 out;
  DiffVec3(&kVec3Ones, 1.0, &out, &u_z1);
  EXPECT_NEAR_VEC3(out, kVec3Ones, 1e-9);

  DiffVec3(&kVec3Ones, 1.0, &out, &u_z1);
  EXPECT_NEAR_VEC3(out, kVec3Zero, 1e-9);

  Vec3 u = {-0.1, -0.1, -0.1};
  Vec3 ans = {-1.1, -1.1, -1.1};
  DiffVec3(&u, 1.0, &out, &u_z1);
  EXPECT_NEAR_VEC3(out, ans, 1e-9);
}

TEST(DiffCircular, BasicChecks) {
  double ts = 0.004;
  double u[] = {0.0, 2.0 * M_PI, 0.0,  0.1, 2.0 * M_PI - 0.1,
                0.0, -M_PI,      M_PI, 0.1, -M_PI};
  double ans[] = {
      0.0,        0.0, 0.1 / ts,           -0.2 / ts,        0.1 / ts,
      -M_PI / ts, 0.0, -(M_PI - 0.1) / ts, (M_PI - 0.1) / ts};
  double u_z1 = 0.0;
  for (int32_t i = 1; i < ARRAYSIZE(u); ++i) {
    double du = DiffCircular(u[i], 2.0 * M_PI, ts, &u_z1);
    EXPECT_NEAR(du, ans[i - 1], 1e-9);
  }
}

// Design 2nd order complementary filters.
//
// MATLAB code used to generate these filters:
//
//   format long;
//   z = 0.3;
//   w = 10;
//   s = tf('s');
//   Hs_lp = w^2/(s^2 + 2*z*w*s + w^2);
//   Hz_lp = c2d(Hs_lp, 0.004, 'tustin');
//   Hs_hp = (s^2 + 2*z*w*s)/(s^2 + 2*z*w*s + w^2);
//   Hz_hp = c2d(Hs_hp, 0.004, 'tustin');
//
//   minreal(Hz_lp+Hz_hp)
//   Hz_lp.num{1}
//   Hz_lp.den{1}
//   Hz_hp.num{1}
//   Hz_hp.den{1}
//
TEST(ComplementaryTest, StepResponse) {
  const int32_t num_filt_coef = 3;
  const double den[num_filt_coef] = {1.0, -1.974713551955749,
                                     0.976293954958514};
  const double num_lp[num_filt_coef] = {
      0.000395100750691426, 0.000790201501382853, 0.000395100750691426};
  const double num_hp[num_filt_coef] = {0.999604899249309, -1.975503753457132,
                                        0.975898854207823};
  double state_lp[num_filt_coef - 1] = {0, 0};
  double state_hp[num_filt_coef - 1] = {0, 0};
  double y_lp = 0;
  double y_hp = 0;
  double u;
  double tol;

  const int32_t num_step_resp = 9;
  double u_tests[num_step_resp] = {-1e8, -2.2e5, -2.22e3, -1e-6, 0.0,
                                   1e-4, 2.2,    6.6e4,   8.8e6};

  for (int32_t j = 0; j < num_step_resp; ++j) {
    u = u_tests[j];
    for (int32_t i = 0; i < 1000; ++i) {
      y_lp = Filter(u, num_filt_coef, den, num_lp, state_lp);
      y_hp = Filter(u, num_filt_coef, den, num_hp, state_hp);
      tol = fmax(fabs(1e-8 * u), 1e-8);
      EXPECT_NEAR(y_lp + y_hp, u, tol);
    }
  }
}

TEST(RateLimit, Difference) {
  double out = RandNormal() * 0.01;
  for (int32_t i = 0; i < 100; ++i) {
    double in = RandNormal() * 0.01;
    double low = RandNormal();
    double high = low + Rand();
    double out_z1 = out;

    // If the high and low arguments to saturate are reversed (i.e. high<low),
    // "high" is output.  That's a bit weird for RateLimit b/c when high<low,
    // then instead of holding the signal steady, it follows the high rate
    // limit.  I have no argument to say one is better than the other.
    out = RateLimit(in, low, high, 0.01, &out);

    double step_in = in - out_z1;
    double step_out = out - out_z1;
    double low_ts = low * 0.01;
    double high_ts = high * 0.01;
    out_z1 = out;

    if (low_ts < step_in && step_in < high_ts) {
      EXPECT_NEAR(step_out, step_in, 1e-9);
    } else {
      if (step_in >= high_ts) {
        EXPECT_NEAR(step_out, high_ts, 1e-9);
      } else if (step_in <= low_ts) {
        EXPECT_NEAR(step_out, low_ts, 1e-9);
      } else {
        EXPECT_EQ(0, 1);
      }
    }
  }
}

TEST(RateLimitInt32, Difference) {
  int32_t out = (int32_t)floor(Rand(-1000.0, 1000.0) * 0.01);

  for (int32_t i = 0; i < 100; ++i) {
    int32_t in = (int32_t)floor(Rand(-1000.0, 1000.0) * 0.01);
    int32_t low = static_cast<int32_t>(Rand(-1000.0, 1000.0));
    int32_t high = low + static_cast<int32_t>(Rand(0.0, 1000.0));
    int32_t out_z1 = out;

    out = RateLimitInt32(in, low, high, 0.01, &out);

    int32_t step_in = in - out_z1;
    int32_t step_out = out - out_z1;
    int32_t low_ts = (int32_t)floor((double)low * 0.01);
    int32_t high_ts = (int32_t)ceil((double)high * 0.01);
    out_z1 = out;

    if (low_ts < step_in && step_in < high_ts) {
      EXPECT_EQ(step_out, step_in);
    } else {
      if (step_in >= high_ts) {
        EXPECT_EQ(step_out, high_ts);
      } else if (step_in <= low_ts) {
        EXPECT_EQ(step_out, low_ts);
      } else {
        EXPECT_EQ(0, 1);
      }
    }
  }
}

TEST(RateLimitCircular, SameAsRateLimitNotAcrossBoundary) {
  double y0_z1 = 0.0;
  double y1_z1 = 0.0;
  for (int32_t i = 0; i < 22; ++i) {
    double u = Rand(-M_PI, M_PI);
    double y0 = RateLimit(u, -0.1, 0.1, 0.01, &y0_z1);
    double y1 = RateLimitCircular(u, -0.1, 0.1, -M_PI, M_PI, 0.01, &y1_z1);
    EXPECT_NEAR(y0, y1, 1e-9);
  }
}

TEST(RateLimitCircular, NoRateLimit) {
  double theta_z1 = 0.0;
  for (double theta = 0.0; theta < 4.0 * M_PI; theta += 0.1) {
    double theta_out =
        RateLimitCircular(theta, -0.2, 0.2, -M_PI, M_PI, 1.0, &theta_z1);
    EXPECT_NEAR(Wrap(theta, -M_PI, M_PI), theta_out, 1e-9);
  }
}

TEST(RateLimitCircular, WithRateLimit) {
  double theta_z1 = 0.0;
  double expected_theta = -0.1;
  for (double theta = 0.0; theta < 4.0 * M_PI; theta += 0.11) {
    double theta_out =
        RateLimitCircular(theta, -0.1, 0.1, -M_PI, M_PI, 1.0, &theta_z1);
    // We are careful to not increase theta so much faster that the
    // rate limited theta should go backwards.
    expected_theta += 0.1;
    EXPECT_NEAR(Wrap(expected_theta, -M_PI, M_PI), theta_out, 1e-9);
  }
}

TEST(RateLimitCircular, SameAsRateLimitWhenWrappedAcrossBoundary) {
  double y0_z1 = 0.0;
  double y1_z1 = 0.0;
  for (double theta = M_PI - 0.3; theta < M_PI + 0.3; theta += 0.01) {
    double u = theta + Rand(-0.3, 0.3);
    double y0 = RateLimit(u, -0.1, 0.1, 1.0, &y0_z1);
    double y1 = RateLimitCircular(u, -0.1, 0.1, -M_PI, M_PI, 1.0, &y1_z1);
    EXPECT_NEAR(Wrap(y0, -M_PI, M_PI), y1, 1e-9);
  }
}

TEST(FilterCircularBuffer, CompareToFilter) {
  double a[] = {1.0, 0.56, -0.22, 0.78, -0.1};
  double b[] = {0.1, 0.22, 1.5, 0.34, 0.88};

  int32_t ind = 0;
  double z_old[ARRAYSIZE(a) - 1] = {0};
  double z_new[ARRAYSIZE(a) - 1] = {0};
  for (int32_t i = 0; i < 1000; ++i) {
    double u = RandNormal();
    EXPECT_EQ(Filter(u, ARRAYSIZE(a), a, b, z_old),
              FilterCircularBuffer(u, ARRAYSIZE(a), a, b, z_new, &ind));
  }
}

TEST(HoldMax, Random) {
  double x[10000];
  const double ts = 0.004;
  int32_t hold_num = 3;
  HoldData hold_data = {-DBL_MAX, 0};
  for (int32_t i = 0; i < ARRAYSIZE(x); ++i) {
    x[i] = Rand(-1e3, 1e3);
    double held = HoldMax(x[i], hold_num * ts, ts, &hold_data);
    double max = x[i];
    for (int32_t j = 0; j <= hold_num - (int32_t)(hold_data.timer / ts); ++j) {
      if (i - j < 0) break;
      if (x[i - j] > max) {
        max = x[i - j];
      }
    }
    EXPECT_NEAR(held, max, 1e-9);
  }
}

// Test LatchOn on trivial cases.
TEST(LatchOn, NormalTrivial) {
  const int32_t u[] = {1, 1, 0, 0};
  double hold_time = 0.5;
  int32_t counter[] = {0, 120, 0, 300};
  int32_t solution_latch[] = {1, 1, 0, 1};
  int32_t solution_counter[] = {125, 125, 0, 299};
  int32_t latch;
  for (int32_t i = 0; i < ARRAYSIZE(u); ++i) {
    latch = LatchOn(u[i], hold_time, 0.004, &counter[i]);
    EXPECT_EQ(solution_latch[i], latch);
    EXPECT_EQ(solution_counter[i], counter[i]);
  }
}

TEST(Zoh, Normal) {
  double t[] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0,
                1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1};
  double u[] = {0.0,  1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0,
                8.0,  9.0,  10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0, 17.0, 18.0, 19.0, 20.0, 21.0};
  double ans[] = {0.0,  0.0,  0.0,  3.0,  3.0,  3.0,  6.0,  6.0,
                  6.0,  9.0,  9.0,  9.0,  12.0, 12.0, 12.0, 15.0,
                  15.0, 15.0, 18.0, 18.0, 18.0, 21.0};
  double y, t_z1 = t[0], u_z1 = u[0];
  for (int32_t i = 0; i < ARRAYSIZE(t); ++i) {
    y = Zoh(t[i], u[i], 0.3, &t_z1, &u_z1);
    EXPECT_NEAR(y, ans[i], 1e-9);
  }
}

// TODO: The following type and function should
// eventually be moved into a filter metrics library.

typedef struct { double tau, dc_gain, overshoot, settling_time; } FilterMetrics;

// Find filter metrics of a time series of data when subjected to a
// step input from zero to u_final.  Note that the n-1 element of the
// array is assumed to be the final value.  If the time was not
// sufficiently long for the response to settle, then all of the
// metrics will have errors.
static void GetFilterMetrics(double t[], double y[], int32_t n, double u_final,
                             FilterMetrics *metrics) {
  double y_max = -INFINITY, y_min = INFINITY;
  metrics->settling_time = INFINITY;
  metrics->tau = INFINITY;

  // Assume y[n-1] is the DC output of the filter when exposed to
  // step, u_final.
  double y_final = y[n - 1];
  assert(u_final != 0);
  metrics->dc_gain = y_final / u_final;

  // Find the minimum and maximum output.
  for (int32_t i = 0; i < n; ++i) {
    if (y[i] > y_max) {
      y_max = y[i];
    }
    if (y[i] < y_min) {
      y_min = y[i];
    }
  }

  // Chose a scale for the filter.  Use x_final for low pass filters
  // and y_max - y_final for high pass filters.
  double scale = fmax(fmax(fabs(y_final), y_max - y_final), y_final - y_min);
  double settle_band = 0.05 * scale;
  double tau_band = 0.33 * scale;

  if (y_final == 0.0) {
    metrics->overshoot = INFINITY;
  } else if (y_final > 0.0) {
    metrics->overshoot = y_max / y_final;
  } else {
    metrics->overshoot = y_min / y_final;
  }

  // Zip backward in time to find settling_time and time constant,
  // tau.
  for (int32_t i = n - 1; i >= 0; --i) {
    // Store the time stamp for the first time the output is ouside of
    // the settle_band.
    if (metrics->settling_time == INFINITY &&
        (y[i] < y_final - settle_band || y[i] > y_final + settle_band)) {
      metrics->settling_time = t[i];
    }

    // Store the time stamp for the first time the output is outside
    // of the tau_band.
    if (y[i] < y_final - tau_band || y[i] > y_final + tau_band) {
      if (metrics->tau == INFINITY) {
        metrics->tau = t[i];
      }
    } else {
      // If an oscillation causes the output to re-enter the tau_band,
      // then clear tau and watch for the next time it exits the band.
      metrics->tau = INFINITY;
    }
  }
}

// Tests design of 2nd order complementary filter. Use the following
// MATLAB code to generate the filter coefficients:
//
//   format long;
//   z = 0.3; w = 10;
//   s = tf('s');
//   Hs_lp = w^2 / (s^2 + 2*z*w*s + w^2);
//   Hz_lp = c2d(Hs_lp, 0.004, 'tustin');
//   Hs_hp = (s^2 + 2*z*w*s) / (s^2 + 2*z*w*s + w^2);
//   Hz_hp = c2d(Hs_hp, 0.004, 'tustin');
//
//   minreal(Hz_lp + Hz_hp)
//   Hz_lp.num{1}
//   Hz_lp.den{1}
//   Hz_hp.num{1}
//   Hz_hp.den{1}
//
TEST(Filter, CheckFilterMetrics) {
  const int32_t num_filt_coef = 3;
  const double den[num_filt_coef] = {1.0, -1.974713551955749,
                                     0.976293954958514};
  const double num_lp[num_filt_coef] = {
      0.000395100750691426, 0.000790201501382853, 0.000395100750691426};
  const double num_hp[num_filt_coef] = {0.999604899249309, -1.975503753457132,
                                        0.975898854207823};
  double state_lp[num_filt_coef - 1] = {0.0, 0.0};
  double state_hp[num_filt_coef - 1] = {0.0, 0.0};
  double u;
  FilterMetrics metrics_lp = {0.0, 0.0, 0.0, 0.0};
  FilterMetrics metrics_hp = {0.0, 0.0, 0.0, 0.0};

  const int32_t num_loops = 2000;
  double t[num_loops] = {0};
  double y_lp[num_loops] = {0};
  double y_hp[num_loops] = {0};

  const int32_t num_step_resp = 9;
  double u_tests[num_step_resp] = {-1e8, -2.2e5, -2.22e3, -1e-6, 1e-6,
                                   1e-4, 2.2,    6.6e4,   8.8e6};

  for (int32_t j = 0; j < num_step_resp; ++j) {
    u = u_tests[j];
    state_lp[0] = 0.0;
    state_lp[1] = 0.0;
    state_hp[0] = 0.0;
    state_hp[1] = 0.0;

    for (int32_t i = 0; i < num_loops; ++i) {
      if (i != 0) {
        t[i] = t[i - 1] + 0.004;
      }
      y_lp[i] = Filter(u, num_filt_coef, den, num_lp, state_lp);
      y_hp[i] = Filter(u, num_filt_coef, den, num_hp, state_hp);
    }

    GetFilterMetrics(t, y_lp, num_loops, u, &metrics_lp);
    GetFilterMetrics(t, y_hp, num_loops, u, &metrics_hp);

    // No matter the step input size, these filter metrics should
    // remain constant.
    EXPECT_NEAR(metrics_lp.tau, 0.14, 1e-9);
    EXPECT_NEAR(metrics_lp.dc_gain, 1.0, 1e-9);
    EXPECT_NEAR(metrics_lp.overshoot, 1.372342491505, 1e-9);
    EXPECT_NEAR(metrics_lp.settling_time, 1.012, 1e-9);

    EXPECT_NEAR(metrics_hp.tau, 0.14, 1e-9);
    EXPECT_NEAR(metrics_hp.dc_gain, 0.0, 1e-9);
    EXPECT_NEAR(metrics_hp.settling_time, 1.012, 1e-9);
  }
}

// A simple model used to evaluate the step response characteristics
// of the PID function.  The model has the equivalent transfer
// function:
//
//             1
//   P(s) = -------
//          s^2 - 3
//
static int SimpleModel(double /*t*/, const double *states, double *deriv,
                       void *params) {
  double u = *(double *)params;
  deriv[0] = states[1];
  deriv[1] = 3.0 * states[0] + u;
  return GSL_SUCCESS;
}

// Compare the step response to an equivalent step response in MATLAB.
// The following MATLAB commands generate the expected step response
// characteristics:
//
//   model = tf([1], [1, 0, -3]);
//   compensator = tf([10.4, 69.4, 13.7], [1, 0]);
//   stepinfo(feedback(model * compensator, 1))
//
TEST(Pid, StepResponse) {
  PidParams pid_params;
  pid_params.kp = 69.4;
  pid_params.ki = 13.7;
  pid_params.kd = 10.4;
  pid_params.int_output_min = -1e6;
  pid_params.int_output_max = 1e6;

  // Build ODE system.
  double u = 0.0;
  double t_step = 1e-2;
  gsl_odeiv2_system ode_system = {SimpleModel, nullptr, 2, &u};
  gsl_odeiv2_driver *ode_driver = gsl_odeiv2_driver_alloc_y_new(
      &ode_system, gsl_odeiv2_step_rkf45, t_step, 1e-6, 1e-6);

  // Initialize step response statistics.
  double t_rise_10pct = 0.0;
  double t_rise_90pct = 0.0;
  double t_settle_2pct = 0.0;
  double peak = 0.0;
  double t_peak = 0.0;

  // Initialize system state.
  double int_output = 0.0;
  double x[2] = {0.0, 0.0};
  double error_z1 = 0.0;
  double t = 0.0;
  while (t < 10.0) {
    double error = 1.0 - x[0];
    double deriv_error = (error - error_z1) / t_step;
    u = Pid(error, deriv_error, t_step, kIntegratorModeIntegrate, &pid_params,
            &int_output);
    error_z1 = error;
    int32_t status = gsl_odeiv2_driver_apply(ode_driver, &t, t + t_step, x);
    EXPECT_EQ(status, GSL_SUCCESS);

    // Calculate step response statistics.
    if (t_rise_10pct <= 0.0 && x[0] >= 0.1) {
      t_rise_10pct = t;
    }

    if (t_rise_90pct <= 0.0 && x[0] >= 0.9) {
      t_rise_90pct = t;
    }

    if (x[0] > peak) {
      peak = x[0];
      t_peak = t;
    }

    if (fabs(x[0] - 1.0) > 0.02) {
      t_settle_2pct = t;
    }
  }
  gsl_odeiv2_driver_free(ode_driver);

  // Check that the step response characteristics are close to those
  // calculated in MATLAB.
  EXPECT_NEAR(0.1047, t_rise_90pct - t_rise_10pct, 5.0 * t_step);
  EXPECT_NEAR(4.0628, t_settle_2pct, 5.0 * t_step);
  EXPECT_NEAR(0.2893, t_peak, 5.0 * t_step);
  EXPECT_NEAR(1.2803, peak, 5.0 * t_step);
}

TEST(PidAntiWindup, SameAsPidWhenNoTrackingError) {
  PidParams pid_params;
  pid_params.kp = 69.4;
  pid_params.ki = 13.7;
  pid_params.kd = 10.4;
  pid_params.int_output_min = -10.0;
  pid_params.int_output_max = 5.0;

  double int_output_0 = 0.0;
  double int_output_1 = 0.0;
  IntegratorMode modes[] = {kIntegratorModeIntegrate, kIntegratorModeHold,
                            kIntegratorModeReset};
  for (double error = -100.0; error < 100.0; error += 5.0) {
    for (double deriv_error = -100.0; deriv_error < 100.0; deriv_error += 5.0) {
      for (int32_t i = 0; i < ARRAYSIZE(modes); ++i) {
        double u0 =
            Pid(error, deriv_error, 1e-2, modes[i], &pid_params, &int_output_0);
        double u1 = PidAntiWindup(error, deriv_error, 0.0, 1e-2, modes[i],
                                  &pid_params, &int_output_1);
        EXPECT_EQ(u0, u1);
        EXPECT_EQ(int_output_0, int_output_1);
      }
    }
  }
}

TEST(CrossfadePidParams, Normal) {
  PidParams p0 = {1.0, 2.0, 3.0, 4.0, 5.0};
  PidParams p1 = {3.0, 4.0, 5.0, 6.0, 7.0};
  PidParams p_ans = {2.0, 3.0, 4.0, 5.0, 6.0};
  PidParams p_out;
  CrossfadePidParams(&p0, &p1, 0.0, 0.0, 1.0, &p_out);
  EXPECT_NEAR(p0.kp, p_out.kp, 1e-9);
  EXPECT_NEAR(p0.ki, p_out.ki, 1e-9);
  EXPECT_NEAR(p0.kd, p_out.kd, 1e-9);
  EXPECT_NEAR(p0.int_output_min, p_out.int_output_min, 1e-9);
  EXPECT_NEAR(p0.int_output_max, p_out.int_output_max, 1e-9);

  CrossfadePidParams(&p0, &p1, 1.0, 0.0, 1.0, &p_out);
  EXPECT_NEAR(p1.kp, p_out.kp, 1e-9);
  EXPECT_NEAR(p1.ki, p_out.ki, 1e-9);
  EXPECT_NEAR(p1.kd, p_out.kd, 1e-9);
  EXPECT_NEAR(p1.int_output_min, p_out.int_output_min, 1e-9);
  EXPECT_NEAR(p1.int_output_max, p_out.int_output_max, 1e-9);

  CrossfadePidParams(&p0, &p1, 0.5, 0.0, 1.0, &p_out);
  EXPECT_NEAR(p_ans.kp, p_out.kp, 1e-9);
  EXPECT_NEAR(p_ans.ki, p_out.ki, 1e-9);
  EXPECT_NEAR(p_ans.kd, p_out.kd, 1e-9);
  EXPECT_NEAR(p_ans.int_output_min, p_out.int_output_min, 1e-9);
  EXPECT_NEAR(p_ans.int_output_max, p_out.int_output_max, 1e-9);
}

TEST(Lpf2Init, Recovery) {
  const double fc = 1.0;
  const double zeta = 1.0;
  const double u0 = 22.0;
  const double ts = 0.01;

  double zs[2];
  Lpf2Init(u0, fc, zeta, ts, zs);
  double y = Lpf2(u0, fc, zeta, ts, zs);

  EXPECT_NEAR(u0, y, 1e-9 * u0);
}

TEST(Lpf2Vec3Init, Recovery) {
  const double fc = 1.5;
  const double zeta = 0.2;
  const Vec3 u0 = {-3.1415, 22.0, 2.718};
  const double ts = 0.01;

  Vec3 zs[2];
  Lpf2Vec3Init(&u0, fc, zeta, ts, zs);

  Vec3 y = kVec3Zero;
  Lpf2Vec3(&u0, fc, zeta, ts, &y, zs);

  EXPECT_NEAR_VEC3(u0, y, 1e-9 * Vec3Norm(&u0));
}

TEST(Lpf2, CompareToMatlab) {
  double z[] = {0.0, 0.0};
  double u[] = {0.21, 0.48, 0.42, 0.86, 0.17, 0.34, 0.27, 0.69, 0.22, 0.81};
  double y_ans[] = {0.093901861491, 0.342094859344, 0.477429827156,
                    0.618045630939, 0.566055782162, 0.234627483456,
                    0.267391513705, 0.495945064330, 0.478437557604,
                    0.487619036676};
  for (int32_t i = 0; i < ARRAYSIZE(u); ++i) {
    double y = Lpf2(u[i], 60.0, 0.9, 0.01, z);
    EXPECT_NEAR(y, y_ans[i], 1e-9);
  }
}

TEST(Lpf2, Step) {
  double z[] = {0.0, 0.0};
  double u = 10.0;
  double y = 0.0;

  for (int32_t i = 0; i < 100; ++i) {
    y = Lpf2(u, 10.0, 0.9, 0.01, z);
  }
  EXPECT_NEAR(y, u, 1e-9);
}

TEST(BandPass2, CompareToMatlab) {
  double z[] = {0.0, 0.0};
  double u[] = {0.21, 0.48, 0.42, 0.86, 0.17, 0.34, 0.27, 0.69, 0.22, 0.81};
  double y_ans[] = {0.0896696725305, 0.147337182165,  -0.0181018075621,
                    0.152380009056,  -0.202026657597, -0.11446405452,
                    0.145751398739,  0.0725011653206, -0.0892196027409,
                    0.0979872694158};

  for (int32_t i = 0; i < ARRAYSIZE(u); ++i) {
    double y = BandPass2(u[i], 60.0, 0.9, 0.01, z);
    EXPECT_NEAR(y, y_ans[i], 1e-9);
  }
}

TEST(DiffLpf2, CompareToMatlabDiff) {
  double z[] = {0.0, 0.0};
  double u[] = {0.21, 0.48, 0.42, 0.86, 0.17, 0.34, 0.27, 0.69, 0.22, 0.81};
  double y_ans[] = {18.780372298104, 30.858227272584,  -3.791233710248,
                    31.914394466957, -42.312364222394, -23.973295518778,
                    30.526101568545, 15.184608556526,  -18.686109901794,
                    20.522405716132};

  for (int32_t i = 0; i < ARRAYSIZE(u); ++i) {
    double y = DiffLpf2(u[i], 60.0, 0.9, 0.01, z);
    EXPECT_NEAR(y, y_ans[i], 1e-9);
  }
}

TEST(Lpf2Vec3, CompareToMatlab) {
  Vec3 z[] = {kVec3Zero, kVec3Zero};
  double u[] = {0.21, 0.48, 0.42, 0.86, 0.17, 0.34, 0.27, 0.69, 0.22, 0.81};
  double y_ans[] = {0.093901861491, 0.342094859344, 0.477429827156,
                    0.618045630939, 0.566055782162, 0.234627483456,
                    0.267391513705, 0.495945064330, 0.478437557604,
                    0.487619036676};

  for (int32_t i = 0; i < ARRAYSIZE(u); ++i) {
    Vec3 u_v3 = {0.1 * u[i], 10.0 * u[i], -100.0 * u[i]};
    Vec3 y_ans_v3 = {0.1 * y_ans[i], 10.0 * y_ans[i], -100.0 * y_ans[i]};
    Vec3 y;
    Lpf2Vec3(&u_v3, 60.0, 0.9, 0.01, &y, z);
    EXPECT_NEAR_VEC3(y, y_ans_v3, 1e-9);
  }
}

TEST(BandPass2Vec3, CompareToMatlab) {
  Vec3 z[] = {kVec3Zero, kVec3Zero};
  double u[] = {0.21, 0.48, 0.42, 0.86, 0.17, 0.34, 0.27, 0.69, 0.22, 0.81};
  double y_ans[] = {0.0896696725305, 0.147337182165,  -0.0181018075621,
                    0.152380009056,  -0.202026657597, -0.11446405452,
                    0.145751398739,  0.0725011653206, -0.0892196027409,
                    0.0979872694158};

  for (int32_t i = 0; i < ARRAYSIZE(u); ++i) {
    Vec3 u_v3 = {0.1 * u[i], 10.0 * u[i], -100.0 * u[i]};
    Vec3 y_ans_v3 = {0.1 * y_ans[i], 10.0 * y_ans[i], -100.0 * y_ans[i]};
    Vec3 y;
    BandPass2Vec3(&u_v3, 60.0, 0.9, 0.01, &y, z);
    EXPECT_NEAR_VEC3(y, y_ans_v3, 1e-9);
  }
}

TEST(DiffLpf2Vec3, CompareToMatlab) {
  Vec3 z[] = {kVec3Zero, kVec3Zero};
  double u[] = {0.21, 0.48, 0.42, 0.86, 0.17, 0.34, 0.27, 0.69, 0.22, 0.81};
  double y_ans[] = {18.780372298104, 30.858227272584,  -3.791233710248,
                    31.914394466957, -42.312364222394, -23.973295518778,
                    30.526101568545, 15.184608556526,  -18.686109901794,
                    20.522405716132};
  Vec3 y;
  for (int32_t i = 0; i < ARRAYSIZE(u); ++i) {
    Vec3 u_v3 = {0.1 * u[i], 10.0 * u[i], -100.0 * u[i]};
    Vec3 y_ans_v3 = {0.1 * y_ans[i], 10.0 * y_ans[i], -100.0 * y_ans[i]};
    DiffLpf2Vec3(&u_v3, 60.0, 0.9, 0.01, &y, z);
    EXPECT_NEAR_VEC3(y, y_ans_v3, 1e-9);
  }
}

TEST(SecondOrderFilterCoeff, CompareToMatlabLowPass) {
  double fc = 2.2;
  double zeta = 0.707;
  double ts = 0.004;
  double a_ans[] = {1.000000000000, -1.921873645765, 0.924813677214};
  double b_ans[] = {0.000735007862, 0.001470015724, 0.000735007862};
  double a[3], b[3];

  SecondOrderFilterCoeff(fc, zeta, ts, kFilterTypeLowPass, a, b);
  for (int32_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(a[i], a_ans[i], 1e-9);
    EXPECT_NEAR(b[i], b_ans[i], 1e-9);
  }
}

TEST(SecondOrderFilterCoeff, CompareToMatlabHighPass) {
  double fc = 2.2;
  double zeta = 0.707;
  double ts = 0.004;
  double a_ans[] = {1.000000000000, -1.921873645765, 0.924813677214};
  double b_ans[] = {0.961671830745, -1.923343661490, 0.961671830745};
  double a[3], b[3];

  SecondOrderFilterCoeff(fc, zeta, ts, kFilterTypeHighPass, a, b);
  for (int32_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(a[i], a_ans[i], 1e-9);
    EXPECT_NEAR(b[i], b_ans[i], 1e-9);
  }
}

TEST(SecondOrderFilterCoeff, CompareToMatlabBandPass) {
  double fc = 2.2;
  double zeta = 0.707;
  double ts = 0.004;
  double a_ans[] = {1.000000000000, -1.921873645765, 0.924813677214};
  double b_ans[] = {0.037593161393, 0.000000000000, -0.037593161393};
  double a[3], b[3];

  SecondOrderFilterCoeff(fc, zeta, ts, kFilterTypeBandPass, a, b);
  for (int32_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(a[i], a_ans[i], 1e-9);
    EXPECT_NEAR(b[i], b_ans[i], 1e-9);
  }
}

TEST(SecondOrderFilterCoeff, CompareToMatlabBandStop) {
  double fc = 2.2;
  double zeta = 0.707;
  double ts = 0.004;
  double a_ans[] = {1.000000000000, -1.921873645765, 0.924813677214};
  double b_ans[] = {0.962406838607, -1.921873645765, 0.962406838607};
  double a[3], b[3];

  SecondOrderFilterCoeff(fc, zeta, ts, kFilterTypeBandStop, a, b);
  for (int32_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(a[i], a_ans[i], 1e-9);
    EXPECT_NEAR(b[i], b_ans[i], 1e-9);
  }
}

TEST(SecondOrderFilterCoeff, CompareToMatlabDiffAndLowPass) {
  double fc = 2.2;
  double zeta = 0.707;
  double ts = 0.004;
  double a_ans[] = {1.000000000000, -1.921873645765, 0.924813677214};
  double b_ans[] = {0.367503931043, 0.000000000000, -0.367503931043};
  double a[3], b[3];

  SecondOrderFilterCoeff(fc, zeta, ts, kFilterTypeDiffAndLowPass, a, b);
  for (int32_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(a[i], a_ans[i], 1e-9);
    EXPECT_NEAR(b[i], b_ans[i], 1e-9);
  }
}

static double PeakDetectorComparison(double u, double y, double fc_down,
                                     double ts, double *y_z1) {
  y = Lpf(u, fc_down, ts, y_z1);
  if (u > y) {
    y = u;
    *y_z1 = y;
  }
  return y;
}

TEST(PeakDetector, CompareToOtherVersion) {
  double y1_z1 = 0.0, y2_z1 = 0.0;
  double u = 0.0;
  double ts = 0.01;
  for (int32_t i = 0; i < 2222; ++i) {
    u += Rand(-1.0, 1.0);
    double fc_down = Rand(0.0, 1.0 / (2.0 * M_PI * ts));
    double y1 = PeakDetector(u, fc_down, ts, &y1_z1);
    double y2 = PeakDetectorComparison(u, y2_z1, fc_down, ts, &y2_z1);
    EXPECT_NEAR(y1, y2, 1e-9);
  }
}

TEST(Backlash, CompareToMatlab) {
  double x_z1_ans[] = {
      0.000000000000,   0.000000000000,   0.000000000000,   0.000000000000,
      0.000000000000,   0.000000000000,   0.000000000000,   0.000000000000,
      0.000000000000,   0.000000000000,   0.000000000000,   0.000000000000,
      0.000000000000,   0.000000000000,   0.000000000000,   0.000000000000,
      0.000000000000,   -0.257646077702,  -0.352751729331,  -0.352751729331,
      -0.352751729331,  -0.124800970186,  0.733174335849,   0.828279987479,
      0.828279987479,   0.828279987479,   -0.169091655960,  -1.208702593997,
      -1.303808245626,  -1.303808245626,  -1.303808245626,  0.462984282107,
      1.684230852144,   1.779336503774,   1.779336503774,   1.359150000000,
      -0.756876908253,  -2.159759110292,  -2.254864761922,  -2.254864761922,
      -1.359150000000,  1.050769534399,   2.635287368440,   2.730393020069,
      2.730393020069,   1.359150000000,   -1.344662160545,  -3.110815626587,
      -3.205921278217,  -3.205921278217,  -1.359150000000,  1.638554786692,
      3.586343884735,   3.681449536364,   3.681449536364,   1.359150000000,
      -1.932447412838,  -4.061872142882,  -4.156977794512,  -4.156977794512,
      -1.359150000000,  2.226340038984,   4.537400401030,   4.632506052659,
      4.632506052659,   1.359150000000,   -2.520232665130,  -5.012928659178,
      -5.108034310807,  -5.108034310807,  -1.359150000000,  2.814125291277,
      5.488456917325,   5.583562568955,   5.583562568955,   1.359150000000,
      -3.108017917423,  -5.963985175473,  -6.059090827102,  -6.002653493111,
      -1.359150000000,  3.401910543569,   6.439513433620,   6.534619085250,
      6.296546119257,   1.359150000000,   -3.695803169715,  -6.915041691768,
      -7.010147343397,  -6.590438745403,  -1.359150000000,  3.989695795861,
      7.390569949915,   7.485675601545,   6.884331371549,   1.359150000000,
      -4.283588422008,  -7.866098208063,  -7.961203859692,  -7.178223997696,
      -1.359150000000,  4.577481048154,   8.341626466211,   8.436732117840,
      7.472116623842,   1.359150000000,   -4.871373674300,  -8.817154724358,
      -8.912260375988,  -7.766009249988,  -1.359150000000,  5.165266300446,
      9.292682982506,   9.387788634135,   8.059901876134,   1.359150000000,
      -5.459158926593,  -9.768211240653,  -9.863316892283,  -8.353794502280,
      -1.359150000000,  5.753051552739,   10.243739498801,  10.338845150430,
      8.647687128427,   1.359150000000,   -6.046944178885,  -10.719267756948,
      -10.814373408578, -8.941579754573,  -1.359150000000,  6.340836805031,
      11.194796015096,  11.289901666726,  9.235472380719,   1.359150000000,
      -6.634729431178,  -11.670324273244, -11.765429924873, -9.529365006865,
      -1.359150000000,  6.928622057324,   12.145852531391,  12.240958183021,
      9.823257633012,   1.359150000000,   -7.222514683470,  -12.621380789539};

  double t = 0.0, x = 0.0, x_z1 = 0.0;
  for (int32_t i = 1; i < 30; ++i) {
    t += 0.1;
    x = t * sin(2.0 * M_PI * t);
    x_z1 = Backlash(x, 2.7183, &x_z1);

    EXPECT_NEAR(x_z1, x_z1_ans[i], 1e-9);
  }
}

TEST(Delay, BasicCheck0) {
  double buf[] = {0.0};
  int32_t ind = 0;
  double x_in[] = {2.0, 4.0, 6.0, 8.0, 10.0, 9.0, 7.0, 5.0, 3.0, 1.0};
  double x_ans[] = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 9.0, 7.0, 5.0, 3.0};

  for (int32_t i = 0; i < ARRAYSIZE(x_in); ++i) {
    double x_returned = Delay(x_in[i], ARRAYSIZE(buf), buf, &ind);
    EXPECT_EQ(x_returned, x_ans[i]);
  }
}

TEST(Delay, BasicCheck1) {
  double buf[] = {0.0, 0.0, 0.0, 0.0};
  int32_t ind = 0;
  double x_in[] = {2.0, 4.0, 6.0, 8.0, 10.0, 9.0, 7.0, 5.0, 3.0, 1.0};
  double x_ans[] = {0.0, 0.0, 0.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 9.0};

  for (int32_t i = 0; i < ARRAYSIZE(x_in); ++i) {
    double x_returned = Delay(x_in[i], ARRAYSIZE(buf), buf, &ind);
    EXPECT_EQ(x_returned, x_ans[i]);
  }
}

TEST(Integrator, IntegrateTriangle) {
  const double dx = 0.01;

  // Test kIntegratorModeIntegrate functionality by integrating a
  // triangular section.
  double sum_out, sum = 0.0;
  for (double y = 0.0; y < 1.0; y += dx) {
    sum_out =
        Integrator(y, -INFINITY, INFINITY, dx, kIntegratorModeIntegrate, &sum);
  }
  EXPECT_NEAR(0.5, sum, dx);
  EXPECT_EQ(sum, sum_out);

  // Test kIntegratorModeHold functionality.
  for (double y = 0.0; y < 1.0; y += dx) {
    Integrator(y, -INFINITY, INFINITY, dx, kIntegratorModeHold, &sum);
  }
  EXPECT_EQ(sum, sum_out);

  // Test kIntegratorModeReset functionality.
  Integrator(22.0, -INFINITY, INFINITY, dx, kIntegratorModeReset, &sum);
  EXPECT_EQ(0.0, sum);
}

TEST(Integrator, Saturate) {
  const double dx = 0.01;
  double sat_min = -0.4;
  double sat_max = 0.5;

  // Test upper saturation.
  double sum = 0.0;
  for (double x = 0.0; x < 1.0; x += dx) {
    Integrator(1.0, sat_min, sat_max, dx, kIntegratorModeIntegrate, &sum);
  }
  EXPECT_EQ(sat_max, sum);

  // Test lower saturation.
  sum = 0.0;
  for (double x = 0.0; x < 1.0; x += dx) {
    Integrator(-1.0, sat_min, sat_max, dx, kIntegratorModeIntegrate, &sum);
  }
  EXPECT_EQ(sat_min, sum);
}

TEST(UpdateCircularAveragingBuffer, Normal) {
  double error_margin = 1e-12;
  CircularAveragingBuffer buffer;
  constexpr int32_t kArraySize = 10;
  double array[kArraySize];
  double values[2 * kArraySize] = {0};
  std::generate_n(values, ARRAYSIZE(values), RandNormal);

  InitCircularAveragingBuffer(array, kArraySize, &buffer);
  double avg;
  for (int32_t i = 0; i < ARRAYSIZE(values); ++i) {
    avg = UpdateCircularAveragingBuffer(values[i], &buffer);
    EXPECT_EQ((i + 1) % kArraySize, buffer.next_idx);
    EXPECT_EQ(buffer.full, i + 1 >= kArraySize);
    double actual_sum = 0.0;
    int32_t start_idx = MaxInt32(0, i - kArraySize + 1);
    int32_t array_length = MinInt32(i + 1, kArraySize);
    for (int32_t j = start_idx; j < start_idx + array_length; ++j) {
      actual_sum += values[j];
    }
    EXPECT_NEAR(actual_sum, buffer.sum, error_margin);
    double actual_avg = actual_sum / static_cast<double>(array_length);
    EXPECT_NEAR(actual_avg, avg, error_margin);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

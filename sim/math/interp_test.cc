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

#include <vector>

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "lib/util/test_util.h"
#include "sim/math/interp.h"

TEST(InterpTest, Interp) {
  auto data_func = [](double t) -> double { return exp(t); };

  std::vector<double> t_vals;
  double dt = 0.1;
  for (int32_t i = 0; i < 11; ++i) {
    t_vals.push_back(static_cast<double>(i) * dt);
  }
  sim::TimeseriesInterpolator interp(t_vals);

  std::vector<double> data;
  for (double t : t_vals) {
    data.push_back(data_func(t));
  }

  double t = 0.0;
  do {
    // Approximate the error bound as the max of |1/2 f''(t) dt**2|.
    EXPECT_NEAR(data_func(t), interp.Interp(t, data), exp(1.0) / 2.0 * dt * dt);
    // Sample at the off-grid points.
    t += dt * 0.7;
  } while (t < *t_vals.rbegin());
}

TEST(InterpTest, WrapInterpUpperLimit) {
  std::vector<double> t_vals = {0.0, 1.0};
  std::vector<double> data = {0.9, -0.9};
  sim::TimeseriesInterpolator interp(t_vals);

  EXPECT_NEAR(0.95, interp.WrapInterp(0.25, data, -1.0, 1.0), 1e-8);
  EXPECT_NEAR(-0.95, interp.WrapInterp(0.75, data, -1.0, 1.0), 1e-8);
}

TEST(InterpTest, WrapInterpLowerLimit) {
  std::vector<double> t_vals = {0.0, 1.0};
  std::vector<double> data = {-0.9, 0.9};
  sim::TimeseriesInterpolator interp(t_vals);

  EXPECT_NEAR(-0.95, interp.WrapInterp(0.25, data, -1.0, 1.0), 1e-8);
  EXPECT_NEAR(0.95, interp.WrapInterp(0.75, data, -1.0, 1.0), 1e-8);
}

TEST(InterpTest, InterpVec3) {
  auto data_func = [](double t) -> Vec3 {
    return {sin(t), cos(t), cos(2.0 * t)};
  };

  std::vector<double> t_vals;
  double dt = 2.0 * PI / 20.0;
  for (int32_t i = 0; i < 21; ++i) {
    t_vals.push_back(static_cast<double>(i) * dt);
  }
  sim::TimeseriesInterpolator interp(t_vals);

  std::vector<Vec3> data;
  for (double t : t_vals) {
    data.push_back(data_func(t));
  }

  double t = 0.0;
  do {
    // Approximate the error bound as the max of |1/2 f''(t) dt**2|
    // for each of the component functions.
    EXPECT_NEAR_VEC3(data_func(t), interp.InterpVec3(t, data), 2.0 * dt * dt);
    // Sample at the off-grid points.
    t += dt * 0.7;
  } while (t < 2.0 * PI);
}

TEST(InterpTest, WrapInterpVec3) {
  std::vector<double> t_vals = {0.0, 1.0};
  std::vector<Vec3> data = {{0.9, -0.9, 0.4}, {-0.9, 0.9, 0.6}};
  sim::TimeseriesInterpolator interp(t_vals);

  // The x-component should increase around the wraparound point, while the
  // y-component should decrease around it. The z-component should not wrap.
  EXPECT_NEAR_VEC3(Vec3({0.95, -0.95, 0.45}),
                   interp.WrapInterpVec3(0.25, data, -1.0, 1.0), 1e-8);
  EXPECT_NEAR_VEC3(Vec3({-0.95, 0.95, 0.55}),
                   interp.WrapInterpVec3(0.75, data, -1.0, 1.0), 1e-8);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

#include <glog/logging.h>
#include <math.h>

#include "common/c_math/vec3.h"
#include "control/sensor_util.h"
#include "lib/util/test_util.h"
#include "sim/models/sensors/pitot.h"
#include "sim/physics/aero_frame.h"

// Declarations from internal namespace, exposed for testing.
namespace internal {

void CalcFivePortPressures(double air_density, double port_angle,
                           const Vec3 &apparent_wind_p, double *p_dynamic,
                           double *p_alpha, double *p_beta);

void CalcInflowFromRotorsAtPitot(double air_density, double thrust,
                                 double freestream_vel,
                                 const PitotSimParams &params, Vec3 *inflow_b);

}  // namespace internal

TEST(CalcFivePortPressures, Normal) {
  double air_density = 1.2;
  double airspeed = 20.0;
  double dynamic_pressure = 0.5 * air_density * airspeed * airspeed;
  Vec3 apparent_wind_w = {-airspeed, 0.0, 0.0};
  for (double alpha = -0.5; alpha <= 0.5; alpha += 0.1) {
    for (double beta = -0.5; alpha <= 0.5; alpha += 0.1) {
      Vec3 apparent_wind_b;
      RotWToB(&apparent_wind_w, alpha, beta, &apparent_wind_b);
      double p_dynamic, p_alpha, p_beta;
      internal::CalcFivePortPressures(air_density, PI / 4.0, apparent_wind_b,
                                      &p_dynamic, &p_alpha, &p_beta);
      EXPECT_NEAR(p_dynamic,
                  dynamic_pressure * (1.0 -
                                      (9.0 / 4.0) * (sin(alpha) * sin(alpha) *
                                                         cos(beta) * cos(beta) +
                                                     sin(beta) * sin(beta))),
                  1e-9);
      EXPECT_NEAR(p_alpha, dynamic_pressure * (9.0 / 4.0) * cos(beta) *
                               cos(beta) * sin(2.0 * alpha),
                  1e-9);
      EXPECT_NEAR(p_beta,
                  dynamic_pressure * (9.0 / 4.0) * cos(alpha) * sin(2.0 * beta),
                  1e-9);
    }
  }
}

// Tests that we calculate a reasonable inflow velocity during hover.
TEST(CalcInflowFromRotorsAtPitot, Hover) {
  const PitotSimParams &params = GetSimParams()->pitots_sim[0];

  // Calculate the inflow velocity under typical hover conditions.
  Vec3 inflow_b;
  internal::CalcInflowFromRotorsAtPitot(1.176, 22000.0, 20.0, params,
                                        &inflow_b);

  // This is a rough estimate of what the inflow should be based on
  // CFD simulations which were correlated to Pitot data from the
  // 2016-08-16 hover test.
  Vec3 expected_inflow_b = {-2.8, 0.0, 0.0};

  EXPECT_NEAR_VEC3(expected_inflow_b, inflow_b, 0.2);
}

TEST(CalcInflowFromRotorsAtPitot, NegativeXInflowDuringThrustingClimb) {
  const PitotSimParams &params = GetSimParams()->pitots_sim[0];
  Vec3 inflow_b;
  internal::CalcInflowFromRotorsAtPitot(1.2, 17000.0, 10.0, params, &inflow_b);
  EXPECT_LT(inflow_b.x, 0.0);
}

TEST(CalcInflowFromRotorsAtPitot, NegativeXInflowDuringThrustingFall) {
  const PitotSimParams &params = GetSimParams()->pitots_sim[0];
  Vec3 inflow_b;
  internal::CalcInflowFromRotorsAtPitot(1.2, 17000.0, -10.0, params, &inflow_b);
  EXPECT_LT(inflow_b.x, 0.0);
}

TEST(CalcInflowFromRotorsAtPitot, PositiveXInflowDuringDraggingClimb) {
  const PitotSimParams &params = GetSimParams()->pitots_sim[0];
  Vec3 inflow_b;
  internal::CalcInflowFromRotorsAtPitot(1.2, -17000.0, 10.0, params, &inflow_b);
  EXPECT_GT(inflow_b.x, 0.0);
}

TEST(CalcInflowFromRotorsAtPitot, PositiveXInflowDuringDraggingFall) {
  const PitotSimParams &params = GetSimParams()->pitots_sim[0];
  Vec3 inflow_b;
  internal::CalcInflowFromRotorsAtPitot(1.2, -17000.0, -10.0, params,
                                        &inflow_b);
  EXPECT_GT(inflow_b.x, 0.0);
}

// Tests that the inflow of the rotors does not have a large effect at
// high crosswind speeds.
TEST(CalcInflowFromRotorsAtPitot, Crosswind) {
  const PitotSimParams &params = GetSimParams()->pitots_sim[0];

  // Calculate the inflow velocity under typical crosswind conditions.
  Vec3 inflow_b;
  internal::CalcInflowFromRotorsAtPitot(1.2, 5000.0, 60.0, params, &inflow_b);

  // We expect relatively low inflows under crosswind conditions.
  EXPECT_NEAR_VEC3(kVec3Zero, inflow_b, 1.0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

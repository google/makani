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

#include "common/c_math/util.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"
#include "sim/models/ground_station_util.h"
#include "sim/sim_params.h"

class RacetrackTetherIntegratorTest : public ::testing::Test {
 protected:
  RacetrackTetherIntegratorTest()
      : drum_angles_(GetSystemParams()->ground_station.gs02.drum_angles),
        integrator_(GetSystemParams()->ground_station.gs02,
                    GetSimParams()->gs02_sim) {}

  double Integrand(double tau) {
    return Sqrt(integrator_.a_ * Square(tau) + integrator_.b_ * tau +
                integrator_.c_);
  }

  void CompareToNumerical(double drum_angle, int32_t num_intervals) {
    const double tau = drum_angles_.racetrack_high - drum_angle;
    const double dtau = tau / static_cast<double>(num_intervals);
    double numerical_integral = 0.0;
    double midpoint = dtau / 2.0;
    for (int32_t i = 0; i < num_intervals; ++i) {
      numerical_integral += Integrand(midpoint) * dtau;
      midpoint += dtau;
    }

    EXPECT_NEAR(numerical_integral, integrator_.WrappedLength(drum_angle),
                Square(dtau));
  }
  const Gs02DrumAngles drum_angles_;
  const RacetrackTetherIntegrator integrator_;
};

// Compare the fully-wrapped tether length against arc lengths of helices that
// provide upper and lower bounds.
//
// The upper bound is provided by a helix with radius equal to the outer radius
// of the racetrack (i.e. the drum radius). The lower bound is provided by a
// helix with radius equal to the inner radius of the racetrack.
TEST_F(RacetrackTetherIntegratorTest, SanityChecks) {
  const Gs02Params &gs02_params = GetSystemParams()->ground_station.gs02;
  const Gs02SimParams &gs02_sim_params = GetSimParams()->gs02_sim;

  double dtheta = drum_angles_.racetrack_high - drum_angles_.racetrack_low;
  double dx_dtheta =
      (gs02_params.gsg_pos_drum.x - gs02_sim_params.wrap_start_posx_drum) /
      dtheta;
  double full_length = integrator_.WrappedLength(drum_angles_.racetrack_low);

  EXPECT_LT(full_length,
            Sqrt(Square(dx_dtheta) + Square(gs02_params.drum_radius)) * dtheta);

  double inner_radius = gs02_params.gsg_pos_drum.z;
  EXPECT_GT(full_length,
            Sqrt(Square(dx_dtheta) + Square(inner_radius)) * dtheta);
}

// Use numerical integration to validate the analytical integral used by
// RacetrackTetherIntegrator.
TEST_F(RacetrackTetherIntegratorTest, CompareToNumerical) {
  for (int32_t i = 0; i < 100; ++i) {
    double drum_angle = test_util::Rand(drum_angles_.racetrack_low,
                                        drum_angles_.racetrack_high);
    int32_t num_intervals =
        static_cast<int32_t>(test_util::Rand(0.1, 1.0) * 100.0);
    CompareToNumerical(drum_angle, num_intervals);
  }

  CompareToNumerical(drum_angles_.racetrack_low, 100);
}

// Check that the wrapped lenght saturates approprately for drum angles to
// either side of the racetrack.
TEST_F(RacetrackTetherIntegratorTest, Saturations) {
  EXPECT_NEAR(integrator_.WrappedLength(drum_angles_.racetrack_high + 0.1), 0.0,
              DBL_EPSILON);
  EXPECT_NEAR(integrator_.WrappedLength(drum_angles_.racetrack_low - 0.1),
              integrator_.WrappedLength(drum_angles_.racetrack_low),
              DBL_EPSILON);
}

// This test effectively forces racetrack_tether_length to be derived from
// RacetrackTetherIntegrator. Ideally this would be done automatically in
// the build, but doing so would introduce a circular dependency that's
// not worth the effort to resolve.
TEST_F(RacetrackTetherIntegratorTest, Length) {
  EXPECT_NEAR(GetSystemParams()->ground_station.gs02.racetrack_tether_length,
              integrator_.WrappedLength(drum_angles_.racetrack_low), 0.01);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

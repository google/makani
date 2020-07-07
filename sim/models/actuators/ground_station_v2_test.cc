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
#include "sim/models/actuators/actuator.h"
#include "sim/models/actuators/ground_station_v2.h"
#include "sim/sim_params.h"

class GroundStationV2Test : public ::testing::Test {
 protected:
  GroundStationV2Test()
      : ned_frame_(),
        vessel_frame_(),
        params_(GetSystemParams()->ground_station.gs02),
        sim_params_(GetSimParams()->gs02_sim),
        angles_(GetSystemParams()->ground_station.gs02.drum_angles),
        gs02_(ned_frame_, vessel_frame_, params_, GetSimParams()->gs02_sim) {}

  // Deal with the boilerplate necessary to set the drum angle and angular
  // velocity.
  void SetDrum(double theta, double omega) {
    gs02_.ClearContinuousStates();
    gs02_.ClearDerivedValues();

    gs02_.platform_azi_.set_val(0.0);
    gs02_.platform_azi_vel_.set_val(0.0);
    gs02_.drum_angle_.set_val(theta);
    gs02_.drum_omega_.set_val(omega);
    gs02_.detwist_angle_.set_val(0.0);

    gs02_.UpdateDerivedStates();
  }

  double CalcFreeLength(double theta) {
    SetDrum(theta, 0.0);
    return gs02_.CalcTetherFreeLength();
  }

  // Check that the tether free length changes at expected rates.
  void TestLengthDerivative() {
    const double dtheta = 0.01;

    // Main wrapping.
    double dl_dtheta_expected = Sqrt(Square(sim_params_.dx_dtheta_main_wrap) +
                                     Square(params_.drum_radius));
    for (double theta = -40.0; theta < angles_.wide_wrap_low - 1.5 * dtheta;
         theta += dtheta) {
      double dl_dtheta =
          (CalcFreeLength(theta + dtheta) - CalcFreeLength(theta)) / dtheta;
      EXPECT_NEAR(dl_dtheta_expected, dl_dtheta, 1e-8);
      theta += dtheta;
    }

    // Wide wrapping.
    dl_dtheta_expected = Sqrt(Square(sim_params_.dx_dtheta_wide_wrap) +
                              Square(params_.drum_radius));
    for (double theta = angles_.wide_wrap_low + dtheta / 2.0;
         theta < angles_.racetrack_low - 1.5 * dtheta; theta += dtheta) {
      double dl_dtheta =
          (CalcFreeLength(theta + dtheta) - CalcFreeLength(theta)) / dtheta;
      EXPECT_NEAR(dl_dtheta_expected, dl_dtheta, 1e-8);
      theta += dtheta;
    }

    // Racetrack. dl_dtheta is not constant in this case; we'll check that it
    // stays bounded.
    double dr_dtheta = (hypot(params_.gsg_pos_drum.y, params_.gsg_pos_drum.z) -
                        params_.drum_radius) /
                       (angles_.racetrack_high - angles_.racetrack_low);
    double dl_dtheta_max =
        Sqrt(Square(sim_params_.wrap_start_posx_drum - params_.gsg_pos_drum.x) +
             Square(params_.drum_radius) + Square(dr_dtheta));

    for (double theta = angles_.racetrack_low + dtheta / 2.0;
         theta < angles_.racetrack_high - 1.5 * dtheta; theta += dtheta) {
      double dl_dtheta =
          (CalcFreeLength(theta + dtheta) - CalcFreeLength(theta)) / dtheta;
      EXPECT_LE(dl_dtheta, dl_dtheta_max);
      theta += dtheta;
    }
  }

  ReferenceFrame ned_frame_;
  ReferenceFrame vessel_frame_;
  const Gs02Params &params_;
  const Gs02SimParams &sim_params_;
  const Gs02DrumAngles &angles_;
  GroundStationV2 gs02_;
};

TEST_F(GroundStationV2Test, LengthDerivative) { TestLengthDerivative(); }

// TODO: Add tests for the following:
//  - Check that anchor position matches inner radius when on the GSG.
//  - Check that anchor position is on top of the drum when on the helical
//    wrapping sections.
//  - Compare numerical derivative of anchor position with the calculated
//    velocity.

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

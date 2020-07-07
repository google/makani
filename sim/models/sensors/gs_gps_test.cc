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

#include "avionics/common/tether_message_types.h"
#include "common/c_math/mat3.h"
#include "control/system_params.h"
#include "sim/models/environment.h"
#include "sim/models/sensors/gs_gps.h"
#include "sim/sim_messages.h"
#include "sim/sim_params.h"

class GsGpsTest : public ::testing::Test {
 protected:
  const SystemParams &system_params_;
  const SimParams &sim_params_;

  std::unique_ptr<Environment> environment_;
  std::unique_ptr<GroundFrame> ground_frame_;
  std::unique_ptr<GsGps> gs_gps_;
  bool has_compass_;

  GsGpsTest();
  void CheckCompassPitchHeading(double expected_pitch, double expected_heading);
  const GsGpsParams *GetParams() { return &gs_gps_->gs_gps_params_; }
};

GsGpsTest::GsGpsTest()
    : system_params_(*GetSystemParams()),
      sim_params_(*GetSimParams()),
      environment_(nullptr),
      ground_frame_(nullptr),
      gs_gps_(nullptr),
      has_compass_(true) {
  environment_.reset(new Environment(
      sim_params_.iec_sim, sim_params_.phys_sim, system_params_.phys,
      system_params_.wind_sensor, system_params_.ground_frame));
  ground_frame_.reset(new GroundFrame(environment_->ned_frame(),
                                      system_params_.ground_frame,
                                      sim_params_.ground_frame_sim));
  gs_gps_.reset(new GsGps(*ground_frame_, system_params_.gs_gps, has_compass_,
                          sim_params_.gs_gps_sim));
}

void GsGpsTest::CheckCompassPitchHeading(double expected_pitch,
                                         double expected_heading) {
  Vec3 X_ecef_gps, V_ecef_gps;
  double heading, pitch, length;
  gs_gps_->CalcGpsPositionVelocityAngles(&X_ecef_gps, &V_ecef_gps, &heading,
                                         &pitch, &length);
  EXPECT_NEAR(heading, Wrap(expected_heading, 0.0, 2.0 * PI), 1e-6);
  EXPECT_NEAR(pitch, Wrap(expected_pitch, -PI, PI), 1e-6);
}

TEST_F(GsGpsTest, CompassPitchHeadingTest) {
  // Compute compass heading and pitch angles when the vessel frame is aligned
  // with the ground frame.
  Mat3 dcm_g2p = kMat3Identity;
  ReferenceFrame parent_frame__(*ground_frame_, kVec3Zero, dcm_g2p);
  gs_gps_->set_parent_frame(parent_frame__);
  Vec3 baseline_pos_ecef, baseline_vel_ecef;
  double baseline_heading, baseline_pitch, baseline_length;
  gs_gps_->CalcGpsPositionVelocityAngles(&baseline_pos_ecef, &baseline_vel_ecef,
                                         &baseline_heading, &baseline_pitch,
                                         &baseline_length);

  // Impose vessel rotations. Compute compass heading and pitch angles.
  double heading = 3 * PI / 4.0;
  double pitch = -PI / 10.0;
  AngleToDcm(heading, pitch, 0.0, kRotationOrderZyx, &dcm_g2p);
  parent_frame__ = ReferenceFrame(*ground_frame_, kVec3Zero, dcm_g2p);
  gs_gps_->clear_parent_frame();
  gs_gps_->set_parent_frame(parent_frame__);

  // Compute the expected compass heading and pitch angle.
  Vec3 d_gps_antennae_p, d_gps_antennae_g;
  Vec3Sub(&GetParams()->secondary_antenna_p.pos,
          &GetParams()->primary_antenna_p.pos, &d_gps_antennae_p);
  Mat3TransVec3Mult(&dcm_g2p, &d_gps_antennae_p, &d_gps_antennae_g);

  double compass_heading, compass_pitch, compass_length;
  CartToSph(&d_gps_antennae_g, &compass_heading, &compass_pitch,
            &compass_length);

  // Pitch angle sign is opposite to the returned by CartToSph by our
  // convention.
  CheckCompassPitchHeading(-compass_pitch, compass_heading);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

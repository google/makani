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

#include "sim/models/rigid_bodies/buoy.h"
#include <gtest/gtest.h>
#include "control/system_params.h"
#include "lib/util/test_util.h"
#include "sim/models/environment.h"
#include "sim/models/sea.h"
#include "sim/sim_params.h"

class BuoyTest : public ::testing::Test {
 protected:
  const SystemParams &system_params_;
  const SimParams &sim_params_;
  std::unique_ptr<Environment> environment_;
  std::unique_ptr<GroundFrame> ground_frame_;

  std::unique_ptr<Sea> sea_;
  std::unique_ptr<Buoy> buoy_;

  BuoyTest();
  ~BuoyTest() {}
};

BuoyTest::BuoyTest()
    : system_params_(*GetSystemParams()),
      sim_params_(*GetSimParams()),
      environment_(nullptr),
      ground_frame_(nullptr),
      sea_(nullptr),
      buoy_(nullptr) {
  environment_.reset(new Environment(
      sim_params_.iec_sim, sim_params_.phys_sim, system_params_.phys,
      system_params_.wind_sensor, system_params_.ground_frame));
  ground_frame_.reset(new GroundFrame(environment_->ned_frame(),
                                      system_params_.ground_frame,
                                      sim_params_.ground_frame_sim));
  sea_.reset(new Sea(*environment_, *ground_frame_, sim_params_.sea_sim,
                     sim_params_.buoy_sim.msl_pos_z_g));
  buoy_.reset(new Buoy(*environment_, *ground_frame_, system_params_.buoy,
                       sim_params_.buoy_sim, sea_.get()));
}

// Tests that the initial buoyancy force equals the buoy weight, +/- 10% to
// accommodate waves and buoy initial conditions.
TEST_F(BuoyTest, WetVolumeTest) {
  double wet_volume = buoy_->wet_volume();
  double mass = system_params_.buoy.mass;
  double water_density = sea_->GetWaterDensity();
  EXPECT_NEAR(mass, wet_volume * water_density, mass / 10.0);
}

// Test that the buoy is stable; i.e.: the buoyancy center is above the center
// of mass.
TEST_F(BuoyTest, StabilityTest) {
  Vec3 buoyancy_center_v = buoy_->buoyancy_center_v();
  Vec3 buoy_cm_v = system_params_.buoy.center_of_mass_pos;
  EXPECT_LT(buoyancy_center_v.z, buoy_cm_v.z);
}

// Test properties of the inertia matrix.
TEST_F(BuoyTest, InertiaMatrixTest) {
  Mat3 I = system_params_.buoy.inertia_tensor;

  // Check for symmetry.
  EXPECT_NEAR_RELATIVE(I.d[0][1], I.d[1][0], 1e-6);
  EXPECT_NEAR_RELATIVE(I.d[0][2], I.d[2][0], 1e-6);
  EXPECT_NEAR_RELATIVE(I.d[1][2], I.d[2][1], 1e-6);

  // Check for positive diagonal elements.
  for (int i = 0; i < 3; i++) EXPECT_GT(I.d[i][i], 0.);

  // Check semi-positive definiteness. For reference see:
  // https://en.wikipedia.org/wiki/Diagonally_dominant_matrix
  EXPECT_GE(I.d[0][0], fabs(I.d[0][1]) + fabs(I.d[0][2]));
  EXPECT_GE(I.d[1][1], fabs(I.d[1][0]) + fabs(I.d[1][2]));
  EXPECT_GE(I.d[2][2], fabs(I.d[2][0]) + fabs(I.d[2][1]));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

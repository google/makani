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

#include "avionics/common/avionics_messages.h"
#include "common/c_math/vec3.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/ground_station_v2.h"
#include "sim/models/full_system.h"
#include "sim/models/sensors/ground_station_v2_sensors.h"
#include "sim/models/tether.h"
#include "sim/sim_params.h"

namespace internal {

bool CalcCurvature(const Vec3 &v1, const Vec3 &v2, Vec3 *curvature);
}

class TetherTest : public ::testing::Test {
 protected:
  const SystemParams &system_params_;
  const SimParams &sim_params_;

  std::unique_ptr<Sea> sea_;
  std::unique_ptr<Environment> environment_;
  std::unique_ptr<GroundFrame> ground_frame_;
  std::unique_ptr<Buoy> buoy_;
  std::unique_ptr<Tether> tether_;
  std::unique_ptr<GroundStationV2Base> gs02_;

  TetherTest();
  void CheckStartAzimuthElevation(double azi, double ele);
};

TetherTest::TetherTest()
    : system_params_(*GetSystemParams()),
      sim_params_(*GetSimParams()),
      sea_(nullptr),
      environment_(nullptr),
      ground_frame_(nullptr),
      buoy_(nullptr),
      tether_(nullptr),
      gs02_(nullptr) {
  environment_.reset(new Environment(
      sim_params_.iec_sim, sim_params_.phys_sim, system_params_.phys,
      system_params_.wind_sensor, system_params_.ground_frame));
  ground_frame_.reset(new GroundFrame(environment_->ned_frame(),
                                      system_params_.ground_frame,
                                      sim_params_.ground_frame_sim));
  buoy_.reset(new Buoy(*environment_, *ground_frame_, system_params_.buoy,
                       sim_params_.buoy_sim, sea_.get()));
  tether_.reset(
      new Tether(*environment_, system_params_.tether, sim_params_.tether_sim));
  gs02_.reset(new GroundStationV2(
      environment_->ned_frame(), buoy_->vessel_frame(),
      system_params_.ground_station.gs02, sim_params_.gs02_sim));
}

void TetherTest::CheckStartAzimuthElevation(double azi, double ele) {
  tether_->UpdateTetherStartDirG();
  GroundStationV2Sensors *gs02_sensors = new GroundStationV2Sensors();
  FullSystem::SetGsgAngles(*buoy_, *tether_, gs02_.get(), gs02_sensors,
                           &sim_telem);
  EXPECT_NEAR(sim_telem.tether.Xv_start_elevation, ele, DBL_EPSILON);
  EXPECT_NEAR(sim_telem.tether.Xv_start_azimuth, azi, DBL_EPSILON);
}

TEST(CalcCurvature, SimpleNonsingular) {
  Vec3 v1 = {10.0 / Sqrt(2.0) - 10.0, 10.0 / Sqrt(2.0), 0.0};
  Vec3 v2 = {-10.0 / Sqrt(2.0), 10.0 - 10.0 / Sqrt(2.0), 0.0};
  Vec3 curvature;
  EXPECT_TRUE(internal::CalcCurvature(v1, v2, &curvature));
  Vec3 expected = {0.0, 0.0, 0.1};
  EXPECT_NEAR_VEC3(expected, curvature, DBL_TOL);
}

TEST(CalcCurvature, ParallelSameDirection) {
  Vec3 v1 = {1.0, 2.0, 3.0};
  Vec3 v2 = {2.0, 4.0, 6.0};
  Vec3 curvature;
  EXPECT_TRUE(internal::CalcCurvature(v1, v2, &curvature));
  EXPECT_NEAR_VEC3(kVec3Zero, curvature, DBL_TOL);
}

TEST(CalcCurvatureDeath, ParallelOppositeDirections) {
  Vec3 v1 = {1.0, 2.0, 3.0};
  Vec3 v2 = {-2.0, -4.0, -6.0};
  Vec3 curvature;
  EXPECT_FALSE(internal::CalcCurvature(v1, v2, &curvature));
}

TEST(CalcCurvature, ZeroVectorInput) {
  Vec3 v1 = {0.0, 0.0, 0.0};
  Vec3 v2 = {2.0, 4.0, 6.0};
  Vec3 curvature;
  EXPECT_FALSE(internal::CalcCurvature(v1, v2, &curvature));
}

// Set the tether direction to be horizontal in the g-frame, rotate the buoy
// around the pitch axis, expect the tether elevation to equal the opposite of
// the buoy pitch angle.
TEST_F(TetherTest, BuoyPitchToTetherEleTest) {
  tether_->set_Xg_start(kVec3Zero);
  tether_->set_start_ind(sim_params_.tether_sim.num_nodes);
  tether_->set_Xg_end({1.0, 0.0, 0.0});

  Mat3 dcm_g2v__;
  AngleToDcm(0.0, PI / 3.0, 0.0, kRotationOrderXyz, &dcm_g2v__);
  buoy_->clear_dcm_g2v();
  buoy_->set_dcm_g2v(dcm_g2v__);

  CheckStartAzimuthElevation(0.0, -PI / 3.0);
}

// Set the tether direction to be horizontal in the g-frame, rotate the buoy
// around the yaw axis, expect the tether azimuth to equal the opposite of
// the buoy yaw angle.
TEST_F(TetherTest, BuoyYawToTetherAziTest) {
  tether_->set_Xg_start(kVec3Zero);
  tether_->set_start_ind(sim_params_.tether_sim.num_nodes);
  tether_->set_Xg_end({1.0, 0.0, 0.0});

  Mat3 dcm_g2v__;
  AngleToDcm(0.0, 0.0, PI / 3.0, kRotationOrderXyz, &dcm_g2v__);
  buoy_->clear_dcm_g2v();
  buoy_->set_dcm_g2v(dcm_g2v__);

  CheckStartAzimuthElevation(-PI / 3.0, 0.0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

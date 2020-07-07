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

#include "common/c_math/vec3.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/models/sensors/wind_sensor.h"
#include "sim/sim_messages.h"
#include "sim/sim_types.h"

class WindSensorTest : public ::testing::Test {
 public:
  WindSensorTest() {}
  virtual ~WindSensorTest(){};
  void TestIdentityOrientationNotRotating();
  void TestIdentityOrientationRotating();
};

void WindSensorTest::TestIdentityOrientationNotRotating() {
  WindSensorParams wind_sensor_params = GetSystemParams()->wind_sensor;
  wind_sensor_params.dcm_parent2ws = kMat3Identity;

  FaultSchedule faults(nullptr);
  WindSensor wind_sensor(wind_sensor_params, GetSimParams()->wind_sensor_sim,
                         &faults);

  Vec3 wind_g = {-22.2, 2.2, 0.1};
  wind_sensor.set_parent_frame(ReferenceFrame());
  wind_sensor.set_ground_frame(ReferenceFrame());
  wind_sensor.UpdateDerivedStates();
  wind_sensor.set_wind_g(wind_g);

  wind_sensor.DiscreteUpdate(0.0);

  SimSensorMessage sensor_message;
  TetherUpMessage tether_up;
  wind_sensor.UpdateSensorOutputs(&sensor_message, &tether_up);
  TetherWind &wind = tether_up.wind;

  EXPECT_NEAR(wind.velocity[0], wind_g.x, 1e-6);
  EXPECT_NEAR(wind.velocity[1], wind_g.y, 1e-6);
  EXPECT_NEAR(wind.velocity[2], wind_g.z, 1e-6);
}

void WindSensorTest::TestIdentityOrientationRotating() {
  WindSensorParams wind_sensor_params = GetSystemParams()->wind_sensor;
  wind_sensor_params.pos_parent = {0.0, 1.0, 0.0};
  wind_sensor_params.dcm_parent2ws = kMat3Identity;

  FaultSchedule faults(nullptr);
  WindSensor wind_sensor(wind_sensor_params, GetSimParams()->wind_sensor_sim,
                         &faults);

  ReferenceFrame ground_frame = ReferenceFrame();
  Vec3 wind_g = {-10.0, 0.0, 0.0};
  wind_sensor.set_parent_frame(ReferenceFrame(
      ground_frame, kVec3Zero, kVec3Zero, kMat3Identity, {0.0, 0.0, 1.0}));
  wind_sensor.set_ground_frame(ground_frame);
  wind_sensor.UpdateDerivedStates();
  wind_sensor.set_wind_g(wind_g);

  wind_sensor.DiscreteUpdate(0.0);

  SimSensorMessage sensor_message;
  TetherUpMessage tether_up;
  wind_sensor.UpdateSensorOutputs(&sensor_message, &tether_up);
  TetherWind &wind = tether_up.wind;

  EXPECT_NEAR(wind.velocity[0], wind_g.x + 1.0, 1e-6);
  EXPECT_NEAR(wind.velocity[1], wind_g.y, 1e-6);
  EXPECT_NEAR(wind.velocity[2], wind_g.z, 1e-6);
}

TEST_F(WindSensorTest, IdentityOrientationNotRotating) {
  TestIdentityOrientationNotRotating();
}

TEST_F(WindSensorTest, IdentityOrientationRotating) {
  TestIdentityOrientationRotating();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

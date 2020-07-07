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

#include <stdint.h>

#include <memory>
#include <vector>

#include "avionics/motor/firmware/flags.h"
#include "common/c_math/vec3.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "lib/util/test_util.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/actuator.h"
#include "sim/models/actuators/rotor.h"
#include "sim/models/environment.h"
#include "sim/models/sensors/rotor_sensor.h"
#include "sim/sim_params.h"
#include "sim/sim_types.h"
#include "system/labels.h"

using ::test_util::RandNormal;

const SystemParams *system_params = GetSystemParams();
const SimParams *sim_params = GetSimParams();

class RotorSensorTest : public ::testing::Test {
 public:
  RotorSensorTest() : faults_(nullptr), rotor_sensor_(), rotors_() {
    packets_.command_message.motor_command = kMotorCommandRun;
    packets_.motor_statuses.resize(kNumMotors);

    environment_.reset(new Environment(
        sim_params->iec_sim, sim_params->phys_sim, system_params->phys,
        system_params->wind_sensor, system_params->ground_frame));
    for (int32_t i = 0; i < kNumMotors; ++i) {
      // HitlRotor is pretty close to a bare-bones "set the motor speed, then
      // read it" interface, which is all we need.
      rotors_.emplace_back(
          new HitlRotor(static_cast<MotorLabel>(i), system_params->rotors[i],
                        sim_params->rotor_sim, system_params->rotor_sensors[i],
                        *environment_, system_params->ts));
    }
    rotor_sensor_.reset(
        new RotorSensor(rotors_, system_params->rotor_sensors, &faults_));
  }

 protected:
  FaultSchedule faults_;
  std::unique_ptr<RotorSensor> rotor_sensor_;
  std::vector<std::unique_ptr<RotorBase>> rotors_;
  AvionicsPackets packets_;
  std::unique_ptr<Environment> environment_;

  // Checks that the rotor sensor angular velocity is reporting the
  // same value as the actual rotor angular velocity.
  virtual void RotorSensorUpdateTest_RotorVel() {
    double omega[kNumMotors];
    for (int32_t i = 0; i < kNumMotors; ++i) {
      omega[i] = 1000.0 * RandNormal();
      packets_.motor_statuses[i].omega = static_cast<float>(
          InvertCal(omega[i], &system_params->rotor_sensors[i].omega_cal));
      rotors_[i]->SetStateFromVector({});
      rotors_[i]->SetFromAvionicsPackets(packets_);
    }

    rotor_sensor_->DiscreteUpdate(0.0);

    for (int32_t i = 0; i < kNumMotors; ++i) {
      EXPECT_NEAR(rotor_sensor_->rotor_speeds(i), omega[i], 1e-4);
    }
  }
};

TEST_F(RotorSensorTest, RotorSensorUpdateTest_RotorVel) {
  RotorSensorUpdateTest_RotorVel();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  test_util::SetRunfilesDir();
  return RUN_ALL_TESTS();
}

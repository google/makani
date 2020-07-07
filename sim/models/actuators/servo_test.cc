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

#include "common/c_math/util.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/models/actuators/servo.h"
#include "sim/sim_params.h"
#include "sim/sim_types.h"
#include "system/labels.h"

const SystemParams *system_params = GetSystemParams();
const SimParams *sim_params = GetSimParams();

class ServoTest : public ::testing::Test {
 public:
  ServoTest()
      : faults_(nullptr),
        servo_a1_(kServoA1, system_params->servos[kServoA1],
                  sim_params->servos_sim[kServoA1], &faults_) {}

 protected:
  FaultSchedule faults_;
  Servo servo_a1_;

  // This test checks that all of the outputs are zero when all of the inputs
  // are zero. Kind of trivial, but it might provide a template for future
  // tests.
  void CalcDerivTest_Normal_0() {
    // Set continuous state.
    servo_a1_.motor_angular_vel_.Clear();
    servo_a1_.motor_angular_vel_.set_val(0.0);
    servo_a1_.motor_angle_.Clear();
    servo_a1_.motor_angle_.set_val(0.0);
    servo_a1_.ref_model_shaft_angle_.Clear();
    servo_a1_.ref_model_shaft_angle_.set_val(0.0);

    // Set derived values.
    servo_a1_.external_shaft_torque_.Clear();
    servo_a1_.external_shaft_torque_.set_val(0.0);

    // Clear (but don't set) derived values that are outputs.
    servo_a1_.motor_power_.Clear();

    // Run x_dot = f(x, u).
    servo_a1_.CalcDerivHelper(0.0);

    EXPECT_NEAR(0.0, servo_a1_.motor_angular_vel_.deriv(), DBL_TOL);
    EXPECT_NEAR(0.0, servo_a1_.motor_angle_.deriv(), DBL_TOL);
    EXPECT_NEAR(0.0, servo_a1_.ref_model_shaft_angle_.deriv(), DBL_TOL);
  }

  void CheckServoCommandSaturation() {
    AvionicsPackets packets;
    ControllerCommandMessage &command_message = packets.command_message;

    // Command the servo to 1.0 + max_angle_command, then check to see that
    // the command is properly saturated.
    command_message.servo_angle[kServoA1] =
        1.0f + float(sim_params->servos_sim[kServoA1]
                         .servo_drive.ref_model_max_position_limit);
    servo_a1_.shaft_angle_cmd_.Clear();
    servo_a1_.SetFromAvionicsPackets(packets);
    EXPECT_NEAR(servo_a1_.shaft_angle_cmd(),
                sim_params->servos_sim[kServoA1]
                    .servo_drive.ref_model_max_position_limit,
                DBL_TOL);

    // Command the servo to -1.0 + min_angle_command, then check to see that
    // the command is properly saturated.
    command_message.servo_angle[kServoA1] =
        -1.0f + float(sim_params->servos_sim[kServoA1]
                          .servo_drive.ref_model_min_position_limit);
    servo_a1_.shaft_angle_cmd_.Clear();
    servo_a1_.SetFromAvionicsPackets(packets);
    EXPECT_NEAR(servo_a1_.shaft_angle_cmd(),
                sim_params->servos_sim[kServoA1]
                    .servo_drive.ref_model_min_position_limit,
                DBL_TOL);
  }
};

TEST_F(ServoTest, CalcDerivTest_Normal_0) { CalcDerivTest_Normal_0(); }

TEST_F(ServoTest, CheckServoCommandSaturation) {
  CheckServoCommandSaturation();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

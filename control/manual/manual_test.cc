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

#include <float.h>
#include <math.h>
#include <string.h>

#include "control/control_params.h"
#include "control/control_types.h"
#include "control/manual/manual.h"
#include "control/manual/manual_types.h"
#include "lib/util/test_util.h"
#include "system/labels.h"

class ManualStepTest : public testing::Test {
 public:
  ManualStepTest()
      : params_(GetControlParams()->manual),
        flight_status_(),
        state_est_(),
        state_() {
    state_est_.joystick.valid = true;
  }

 protected:
  const ManualParams &params_;
  FlightStatus flight_status_;
  StateEstimate state_est_;
  ManualState state_;
};

// Confirms that ManualStep sets all fields of the
// ControlOutput structure.
TEST_F(ManualStepTest, SetAllValues) {
  ControlOutput control_output_0;
  memset(&control_output_0, 0xFF, sizeof(control_output_0));
  ControlOutput control_output = control_output_0;
  ManualStep(&flight_status_, &state_est_, &params_, &state_, &control_output);

  // Note that this check relies on the output not having an actual
  // value composed of 0xFF.
  EXPECT_NE(control_output.sync.sequence, control_output_0.sync.sequence);
  EXPECT_NE(control_output.sync.sequence, control_output_0.sync.flight_mode);
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    EXPECT_NE(control_output.flaps[i], control_output_0.flaps[i]);
  }
  for (int32_t i = 0; i < kNumMotors; ++i) {
    EXPECT_NE(control_output.motor_speed_upper_limit[i],
              control_output_0.motor_speed_upper_limit[i]);
    EXPECT_NE(control_output.motor_speed_lower_limit[i],
              control_output_0.motor_speed_lower_limit[i]);
    EXPECT_NE(control_output.motor_torque[i], control_output_0.motor_torque[i]);
  }
  EXPECT_NE(control_output.winch_vel_cmd, control_output_0.winch_vel_cmd);
  EXPECT_NE(control_output.detwist_cmd, control_output_0.detwist_cmd);
  EXPECT_NE(control_output.stop_motors, control_output_0.stop_motors);
  EXPECT_NE(control_output.run_motors, control_output_0.run_motors);
  EXPECT_NE(control_output.tether_release, control_output_0.tether_release);
  EXPECT_NE(control_output.gs_azi_cmd.target,
            control_output_0.gs_azi_cmd.target);
  EXPECT_NE(control_output.gs_azi_cmd.dead_zone,
            control_output_0.gs_azi_cmd.dead_zone);
  EXPECT_NE(control_output.gs_unpause_transform,
            control_output_0.gs_unpause_transform);
  EXPECT_NE(control_output.hold_gs_azi_cmd, control_output_0.hold_gs_azi_cmd);
}

TEST_F(ManualStepTest, CheckRollGains) {
  state_est_.joystick.throttle_f = 0.5;
  state_est_.joystick.data.release = false;

  state_est_.joystick.data.roll = -1.0;
  auto control_output = ControlOutput();
  ManualStep(&flight_status_, &state_est_, &params_, &state_, &control_output);
  EXPECT_NEAR(params_.output.lower_flap_limits[kFlapA1],
              control_output.flaps[kFlapA1], DBL_EPSILON);
  EXPECT_NEAR(params_.output.upper_flap_limits[kFlapA8],
              control_output.flaps[kFlapA8], DBL_EPSILON);

  state_est_.joystick.data.roll = 1.0;
  control_output = ControlOutput();
  ManualStep(&flight_status_, &state_est_, &params_, &state_, &control_output);
  EXPECT_NEAR(params_.output.upper_flap_limits[kFlapA1],
              control_output.flaps[kFlapA1], DBL_EPSILON);
  EXPECT_NEAR(params_.output.lower_flap_limits[kFlapA8],
              control_output.flaps[kFlapA8], DBL_EPSILON);
}

TEST_F(ManualStepTest, CheckPitchGains) {
  state_est_.joystick.throttle_f = 0.5;
  state_est_.joystick.data.release = false;

  // Check appropriate saturation of the elevator deflection command.
  // The joystick pitch command must be large enough to saturate the elevator
  // deflection command.
  state_est_.joystick.data.pitch = -1.5;
  auto control_output = ControlOutput();
  ManualStep(&flight_status_, &state_est_, &params_, &state_, &control_output);
  EXPECT_NEAR(params_.output.upper_flap_limits[kFlapEle],
              control_output.flaps[kFlapEle], DBL_EPSILON);

  state_est_.joystick.data.pitch = 1.0;
  control_output = ControlOutput();
  ManualStep(&flight_status_, &state_est_, &params_, &state_, &control_output);
  EXPECT_NEAR(params_.output.lower_flap_limits[kFlapEle],
              control_output.flaps[kFlapEle], DBL_EPSILON);
}

TEST_F(ManualStepTest, CheckYawGains) {
  state_est_.joystick.throttle_f = 0.5;
  state_est_.joystick.data.release = false;

  state_est_.joystick.data.yaw = -1.0;
  auto control_output = ControlOutput();
  ManualStep(&flight_status_, &state_est_, &params_, &state_, &control_output);
  EXPECT_NEAR(params_.output.upper_flap_limits[kFlapRud],
              control_output.flaps[kFlapRud], DBL_EPSILON);

  state_est_.joystick.data.yaw = 1.0;
  control_output = ControlOutput();
  ManualStep(&flight_status_, &state_est_, &params_, &state_, &control_output);
  EXPECT_NEAR(params_.output.lower_flap_limits[kFlapRud],
              control_output.flaps[kFlapRud], DBL_EPSILON);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

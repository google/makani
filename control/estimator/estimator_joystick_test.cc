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

#include "control/estimator/estimator_joystick.h"

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <math.h>
#include <stdlib.h>

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"

using ::test_util::Rand;
using ::test_util::RandNormal;
using ::test_util::LpfIterationTolerance;

TEST(EstimatorJoystickStep, Consistency) {
  const EstimatorJoystickParams *params =
      &GetControlParams()->estimator.joystick;
  FaultMask fault;
  ClearAllFaults(&fault);

  for (int32_t j = 0; j < test_util::kNumTests; j++) {
    JoystickData joystick_in;
    joystick_in.throttle = Rand(0, 1);

    EstimatorJoystickState state;
    EstimatorJoystickInit(&state);
    JoystickEstimate joystick;
    int32_t N = 1000;
    for (int32_t i = 1; i < N; i++) {
      EstimatorJoystickStep(&joystick_in, &fault,
                            &GetControlParams()->estimator.joystick, &state,
                            &joystick);
    }
    EXPECT_NEAR_RELATIVE(
        joystick_in.throttle, joystick.throttle_f,
        LpfIterationTolerance(params->fc_throttle, *g_sys.ts, N));
  }
}

TEST(EstimatorJoystickStep, DebounceJoystick) {
  const JoystickData joystick_in = {
      Rand(0.0, 1.0), 0.0, 0.0, 0.0, kJoystickSwitchPositionUp, false, false};
  EstimatorJoystickState state;
  EstimatorJoystickInit(&state);

  FaultMask fault;
  ClearAllFaults(&fault);
  JoystickEstimate joystick;
  EstimatorJoystickStep(&joystick_in, &fault,
                        &GetControlParams()->estimator.joystick, &state,
                        &joystick);

  EXPECT_EQ(joystick_in.throttle, joystick.data.throttle);
  EXPECT_EQ(joystick_in.roll, joystick.data.roll);
  EXPECT_EQ(joystick_in.pitch, joystick.data.pitch);
  EXPECT_EQ(joystick_in.yaw, joystick.data.yaw);
  EXPECT_EQ(joystick_in.switch_position, joystick.data.switch_position);
  EXPECT_EQ(joystick_in.release, joystick.data.release);
}

TEST(EstimatorJoystickStep, DebounceJoystick_Transitions) {
  const JoystickSwitchPositionLabel switch_transitions[][2] = {
      {kJoystickSwitchPositionUp, kJoystickSwitchPositionDown},
      {kJoystickSwitchPositionUp, kJoystickSwitchPositionMiddle},
      {kJoystickSwitchPositionMiddle, kJoystickSwitchPositionDown},
      {kJoystickSwitchPositionMiddle, kJoystickSwitchPositionUp},
      {kJoystickSwitchPositionDown, kJoystickSwitchPositionMiddle},
      {kJoystickSwitchPositionDown, kJoystickSwitchPositionUp}};

  const EstimatorJoystickParams *params =
      &GetControlParams()->estimator.joystick;
  FaultMask fault;
  ClearAllFaults(&fault);
  for (int32_t i = 0; i < ARRAYSIZE(switch_transitions); i++) {
    JoystickData joystick_in = {0.0,   0.0,  0.0, 0.0, switch_transitions[i][1],
                                false, false};
    EstimatorJoystickState state;
    EstimatorJoystickInit(&state);
    state.switch_position_z1 = switch_transitions[i][0];
    JoystickEstimate joystick;
    EstimatorJoystickStep(&joystick_in, &fault,
                          &GetControlParams()->estimator.joystick, &state,
                          &joystick);
    EXPECT_TRUE(joystick.valid);
    for (int32_t j = 0; j < params->joystick_num_debounce - 1; j++) {
      if (j > 0) {
        EXPECT_EQ(switch_transitions[i][0], joystick.data.switch_position);
      }
      EstimatorJoystickStep(&joystick_in, &fault,
                            &GetControlParams()->estimator.joystick, &state,
                            &joystick);
      EXPECT_TRUE(joystick.valid);
    }
    EXPECT_EQ(joystick_in.switch_position, joystick.data.switch_position);
    EXPECT_TRUE(joystick.valid);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

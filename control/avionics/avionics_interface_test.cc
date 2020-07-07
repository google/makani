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

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <stdint.h>
#include <string.h>

#include <vector>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/network/aio_labels.h"
#include "common/macros.h"
#include "control/avionics/avionics_interface.h"
#include "control/avionics/avionics_interface_types.h"
#include "control/control_params.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_params.h"
#include "system/labels.h"

// CVT state persists across all tests. These should be incremented before any
// CvtPut operation, but they can then be safely reused across several distinct
// CVT entries.
static uint16_t g_aio_sequence = 1U;
static int64_t g_aio_timestamp = 1L;

TEST(AvionicsInterfaceInit, SameInitialization) {
  AvionicsInterfaceState state0, state1;
  memset(&state0, 0, sizeof(state0));
  memset(&state1, 0xFF, sizeof(state1));
  AvionicsInterfaceInit(&state0);
  AvionicsInterfaceInit(&state1);
  EXPECT_TRUE(!memcmp(&state0, &state1, sizeof(state0)));
}

class GuardJoystickReleaseTest : public ::testing::Test {
 protected:
  GuardJoystickReleaseTest()
      : state_(),
        loadcell_messages_(kNumLoadcellNodes, LoadcellMessage()),
        loadcells_updated_(kNumLoadcellNodes, false),
        joystick_status_(),
        input_messages_(),
        control_input_(),
        faults_() {
    // Utility methods depend on the order of node labels.
    LoadcellNodeLabel ordered_labels[] = {
        kLoadcellNodePortA, kLoadcellNodePortB, kLoadcellNodeStarboardA,
        kLoadcellNodeStarboardB};
    for (int32_t i = 0; i < ARRAYSIZE(ordered_labels); ++i) {
      CHECK_EQ(i, static_cast<int32_t>(ordered_labels[i]));
    }

    AvionicsInterfaceInit(&state_);
  }

  void SetArmed(std::vector<bool> values) {
    CHECK_EQ(loadcell_messages_.size(), values.size());
    for (int32_t i = 0; static_cast<size_t>(i) < values.size(); ++i) {
      loadcell_messages_[i].tether_release_fully_armed = values[i];
    }
  }

  void SetUpdated(std::vector<bool> values) {
    CHECK_EQ(loadcell_messages_.size(), values.size());
    for (int32_t i = 0; static_cast<size_t>(i) < values.size(); ++i) {
      loadcells_updated_[i] = values[i];
    }
  }

  void PopulateCvt() {
    for (int32_t i = 0; i < kNumLoadcellNodes; ++i) {
      ++g_aio_sequence;
      ++g_aio_timestamp;

      auto node = static_cast<LoadcellNodeLabel>(i);
      if (loadcells_updated_[node]) {
        CvtPutLoadcellMessage(LoadcellNodeLabelToLoadcellNodeAioNode(node),
                              &loadcell_messages_[node], g_aio_sequence,
                              g_aio_timestamp);
      }
    }

    CvtPutJoystickStatusMessage(kAioNodeJoystickA, &joystick_status_,
                                g_aio_sequence, g_aio_timestamp);
  }

  AvionicsInterfaceState state_;
  std::vector<LoadcellMessage> loadcell_messages_;
  std::vector<bool> loadcells_updated_;
  JoystickStatusMessage joystick_status_;
  ControlInputMessages input_messages_;
  ControlInput control_input_;
  FaultMask faults_[kNumSubsystems];
};

TEST_F(GuardJoystickReleaseTest, OnlyPortArmed_NoRelease) {
  SetArmed({true, false, false, false});
  SetUpdated({true, true, true, true});
  joystick_status_.momentary_switch = kJoystickSwitchPositionUp;
  PopulateCvt();

  ConvertAvionicsToControl(GetSystemParams(),
                           &GetControlParams()->sensor_limits,
                           &GetControlParams()->fault_detection, &state_,
                           &input_messages_, &control_input_, faults_);

  EXPECT_FALSE(control_input_.joystick.release);
}

TEST_F(GuardJoystickReleaseTest, OnlyStarboardArmed_NoRelease) {
  SetArmed({false, false, false, true});
  SetUpdated({true, true, true, true});
  joystick_status_.momentary_switch = kJoystickSwitchPositionUp;
  PopulateCvt();

  ConvertAvionicsToControl(GetSystemParams(),
                           &GetControlParams()->sensor_limits,
                           &GetControlParams()->fault_detection, &state_,
                           &input_messages_, &control_input_, faults_);

  EXPECT_FALSE(control_input_.joystick.release);
}

TEST_F(GuardJoystickReleaseTest, PortNotUpdated_NoRelease) {
  // Initialization of state_ ensures that not updating the CVT for a given node
  // wil trigger a no-update fault.
  SetArmed({true, true, true, true});
  SetUpdated({false, false, true, true});
  joystick_status_.momentary_switch = kJoystickSwitchPositionUp;
  PopulateCvt();

  ConvertAvionicsToControl(GetSystemParams(),
                           &GetControlParams()->sensor_limits,
                           &GetControlParams()->fault_detection, &state_,
                           &input_messages_, &control_input_, faults_);

  EXPECT_TRUE(HasFault(kFaultTypeNoUpdate, &faults_[kSubsysTetherRelease]));
  EXPECT_FALSE(control_input_.joystick.release);
}

TEST_F(GuardJoystickReleaseTest, SuccessfulRelease) {
  SetArmed({true, false, false, true});
  SetUpdated({true, false, false, true});
  joystick_status_.momentary_switch = kJoystickSwitchPositionUp;
  PopulateCvt();

  ConvertAvionicsToControl(GetSystemParams(),
                           &GetControlParams()->sensor_limits,
                           &GetControlParams()->fault_detection, &state_,
                           &input_messages_, &control_input_, faults_);

  EXPECT_TRUE(control_input_.joystick.release);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

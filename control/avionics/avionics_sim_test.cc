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

#include "control/avionics/avionics_sim.h"

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <string.h>

#include <algorithm>
#include <cstdlib>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/gps_receiver.h"
#include "avionics/common/tether_message.h"
#include "control/avionics/avionics_interface.h"
#include "control/control_params.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/pack_sim_messages.h"
#include "sim/sim_messages.h"
#include "system/labels_util.h"

namespace {

#if defined(NDEBUG)

// Places random data in the input SimSensorMessages.
void PrepareInput(SimSensorMessage *in) {
  uint8_t buffer[PACK_SIMSENSORMESSAGE_SIZE];
  std::generate_n(buffer, sizeof(buffer), std::rand);
  UnpackSimSensorMessage(buffer, 1U, in);
}

// Calls PrepareInput, then copies GPS message fields that will *not* be handled
// to the output ControlInputMessages.
void PrepareInputAndOutput(SimSensorMessage *in, ControlInputMessages *out) {
  PrepareInput(in);

  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    auto gps_label = static_cast<WingGpsReceiverLabel>(i);
    GpsReceiverType gps_type = WingGpsReceiverLabelToGpsReceiverType(gps_label);
    CHECK(gps_type == kGpsReceiverTypeNovAtel ||
          gps_type == kGpsReceiverTypeSeptentrio)
        << "Invalid GPS type: " << static_cast<int32_t>(gps_type);
    if (gps_type != kGpsReceiverTypeNovAtel) {
      out->wing_gps_novatel[i] = in->control_input_messages.wing_gps_novatel[i];
    }
    if (gps_type != kGpsReceiverTypeSeptentrio) {
      out->wing_gps_septentrio[i] =
          in->control_input_messages.wing_gps_septentrio[i];
    }
  }

  // The ground estimate must also be ignored here because it is not
  // populated from SimSensorsMessage.
  // TODO: Consider other ways around this issue.
  out->ground_estimate = in->control_input_messages.ground_estimate;
}

#endif  // defined(NDEBUG)

}  // namespace

// Due to statefulness of the CVT, this needs to be shared across test cases. It
// must be incremented before each call to
// UpdateControllerCvtFromSimSensorMessage to
// prevent deduplication.
static uint16_t g_seq = 0U;

#if defined(NDEBUG)

// Test that UpdateControllerCvtFromSimSensorMessage and
// ConvertAvionicsToControl
// both handle all fields in ControlInputMessages.
TEST(UpdateControllerCvtFromSimSensorMessage, Recovery) {
  HitlConfiguration hitl_config;
  hitl_config.use_software_joystick = true;
  FaultMask faults[kNumSubsystems];
  memset(faults, 0, sizeof(faults));

  std::srand(22);
  for (int32_t i = 0; i < 22; ++i) {
    // Create random input data.
    SimSensorMessage in;
    memset(&in, 0, sizeof(in));

    ControlInputMessages out;
    ControlInput control_input;
    memset(&out, 0, sizeof(out));

    // Random input data.
    PrepareInputAndOutput(&in, &out);
    // Set all updated flags to true.
    memset(&in.control_input_messages_updated, 0xFF,
           sizeof(in.control_input_messages_updated));

    // Make sure that the data does not currently match.
    EXPECT_NE(0, memcmp(&in.control_input_messages, &out, sizeof(out)));

    UpdateControllerCvtFromSimSensorMessage(&in, &hitl_config, ++g_seq, 0);
    AvionicsInterfaceState state = AvionicsInterfaceState();
    ConvertAvionicsToControl(GetSystemParams(),
                             &GetControlParams()->sensor_limits,
                             &GetControlParams()->fault_detection, &state, &out,
                             &control_input, faults);
    EXPECT_EQ(0, memcmp(&in.control_input_messages, &out, sizeof(out)));

    // Generate more random data.
    PrepareInputAndOutput(&in, &out);
    // Set all updated flags to false.
    memset(&in.control_input_messages_updated, 0x00,
           sizeof(in.control_input_messages_updated));

    // Check that the updated flags being false correctly leaves data untouched.
    ControlInputMessages out_prev = out;

    UpdateControllerCvtFromSimSensorMessage(&in, &hitl_config, ++g_seq, 0);
    state = AvionicsInterfaceState();
    ConvertAvionicsToControl(GetSystemParams(),
                             &GetControlParams()->sensor_limits,
                             &GetControlParams()->fault_detection, &state, &out,
                             &control_input, faults);
    EXPECT_EQ(0, memcmp(&out, &out_prev, sizeof(out)));

    // Set all updated flags to true, and confirm the update.
    // Note that we don't increment the sequence number as it was
    // not actually applied on the previous step.
    memset(&in.control_input_messages_updated, 0xFF,
           sizeof(in.control_input_messages_updated));
    UpdateControllerCvtFromSimSensorMessage(&in, &hitl_config, g_seq, 0);
    state = AvionicsInterfaceState();
    ConvertAvionicsToControl(GetSystemParams(),
                             &GetControlParams()->sensor_limits,
                             &GetControlParams()->fault_detection, &state, &out,
                             &control_input, faults);
    EXPECT_NE(0, memcmp(&out, &out_prev, sizeof(out)));
    EXPECT_EQ(0, memcmp(&in.control_input_messages, &out, sizeof(out)));
  }
}

// Test that UpdateGroundEstimatorCvtFromSimSensorMessage and
// ConvertGroundvionicsToGroundEstimator
// both handle all fields in GroundEstimatorInputMessages.
TEST(UpdateGroundEstimatorCvtFromSimSensorMessage, Recovery) {
  FaultMask faults[kNumSubsystems];
  memset(faults, 0, sizeof(faults));

  std::srand(22);
  for (int32_t i = 0; i < 22; ++i) {
    // Create random input data.
    SimSensorMessage in;
    memset(&in, 0, sizeof(in));

    GroundEstimatorInputMessages out;
    GroundEstimatorInput ground_input;
    memset(&out, 0, sizeof(out));

    // Random input data.
    PrepareInput(&in);

    // Zero out GroundCompass message.

    // Set all updated flags to true.
    memset(&in.ground_input_messages_updated, 0xFF,
           sizeof(in.ground_input_messages_updated));

    // Make sure that the data does not currently match.
    EXPECT_NE(0, memcmp(&in.ground_input_messages, &out, sizeof(out)));

    UpdateGroundEstimatorCvtFromSimSensorMessage(&in, ++g_seq, 0);
    GroundvionicsInterfaceState state = GroundvionicsInterfaceState();

    ConvertGroundvionicsToGroundEstimator(
        GetSystemParams(), &GetControlParams()->ground_sensor_limits,
        &GetControlParams()->fault_detection, &state, &out, &ground_input,
        faults);
    EXPECT_EQ(0, memcmp(&in.ground_input_messages, &out, sizeof(out)));

    // Generate more random data.
    PrepareInput(&in);

    // Zero out GroundCompass message.

    // Set all updated flags to false.
    memset(&in.ground_input_messages_updated, 0x00,
           sizeof(in.ground_input_messages_updated));

    // Check that the updated flags being false correctly leaves data untouched.
    GroundEstimatorInputMessages out_prev = out;

    UpdateGroundEstimatorCvtFromSimSensorMessage(&in, ++g_seq, 0);
    state = GroundvionicsInterfaceState();
    ConvertGroundvionicsToGroundEstimator(
        GetSystemParams(), &GetControlParams()->ground_sensor_limits,
        &GetControlParams()->fault_detection, &state, &out, &ground_input,
        faults);

    EXPECT_EQ(0, memcmp(&out, &out_prev, sizeof(out)));

    // Set all updated flags to true, and confirm the update.
    // Note that we don't increment the sequence number as it was
    // not actually applied on the previous step.
    memset(&in.ground_input_messages_updated, 0xFF,
           sizeof(in.ground_input_messages_updated));
    UpdateGroundEstimatorCvtFromSimSensorMessage(&in, g_seq, 0);
    state = GroundvionicsInterfaceState();
    ConvertGroundvionicsToGroundEstimator(
        GetSystemParams(), &GetControlParams()->ground_sensor_limits,
        &GetControlParams()->fault_detection, &state, &out, &ground_input,
        faults);

    EXPECT_NE(0, memcmp(&out, &out_prev, sizeof(out)));
    EXPECT_EQ(0, memcmp(&in.ground_input_messages, &out, sizeof(out)));
  }
}

#endif  // defined(NDEBUG)

TEST(UpdateControllerCvtFromSimSensorMessageTest, HitlOptions) {
  // Initialize SimSensorMessage with all updated flags set to true.
  SimSensorMessage sim_sensors;
  memset(&sim_sensors, 0, sizeof(sim_sensors));
  memset(&sim_sensors.control_input_messages_updated, 0xFF,
         sizeof(sim_sensors.control_input_messages_updated));

  HitlConfiguration hitl_config;
  hitl_config.use_software_joystick = true;

  JoystickStatusMessage joystick;
  TetherUpMessage tether_up;

  UpdateControllerCvtFromSimSensorMessage(&sim_sensors, &hitl_config, ++g_seq,
                                          0);
  EXPECT_TRUE(CvtGetJoystickStatusMessage(kAioNodeJoystickA, &joystick, nullptr,
                                          nullptr));
  EXPECT_TRUE(CvtGetTetherUpMessage(kAioNodeCsA, &tether_up, nullptr, nullptr));
  EXPECT_TRUE(TetherIsNoUpdateCountValid(tether_up.joystick.no_update_count));

  hitl_config.use_software_joystick = false;
  UpdateControllerCvtFromSimSensorMessage(&sim_sensors, &hitl_config, ++g_seq,
                                          0);
  EXPECT_FALSE(CvtGetJoystickStatusMessage(kAioNodeJoystickA, &joystick,
                                           nullptr, nullptr));
  EXPECT_TRUE(CvtGetTetherUpMessage(kAioNodeCsA, &tether_up, nullptr, nullptr));
  EXPECT_FALSE(TetherIsNoUpdateCountValid(tether_up.joystick.no_update_count));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

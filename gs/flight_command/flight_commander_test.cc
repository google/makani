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

#include <vector>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/safety_codes.h"
#include "avionics/linux/aio.h"
#include "avionics/linux/cvt_util.h"
#include "gs/flight_command/flight_commander.h"
#include "sim/cvt_sim_messages.h"
#include "sim/sim_messages.h"

uint16_t g_sequence = 0U;
int64_t g_timestamp = 0LL;

bool FoundFlightCommandMessage(FlightCommandMessage *command) {
  return CvtGetNextUpdate(kAioNodeOperator, CvtGetFlightCommandMessage, 0.5,
                          command);
}

// Use a shorter timeout (0.01s) if we don't expect to find a message. That's a
// bit circular, but otherwise we'd spend an excessive amount of time waiting.
bool DidntFindFlightCommandMessage(FlightCommandMessage *command) {
  return !CvtGetNextUpdate(kAioNodeOperator, CvtGetFlightCommandMessage, 0.01,
                           command);
}

bool HoverAccelCommanded(FlightCommandMessage *command) {
  return command->force_hover_accel &&
         command->safety_code == FLIGHT_COMMAND_SIGNAL;
}

bool ExperimentCommanded(FlightCommandMessage *command,
                         ExperimentType experiment_type,
                         uint8_t experiment_case_id) {
  return command->experiment_type == experiment_type &&
         command->experiment_case_id == experiment_case_id &&
         command->safety_code == FLIGHT_COMMAND_SIGNAL;
}

class StandardFlightCommanderTest : public ::testing::Test {
 public:
  StandardFlightCommanderTest()
      : commander_(true, kFlightModeHoverAccel, false, kGroundStationModeReel,
                   false, false, kNumFlightModes, false, 0, 0),
        command_(),
        tether_down_() {}

  // Put a fake TetherDownMessage in the CVT, with core switch A's
  // control_telemetry.flight_mode set to a specified value.
  void PlantTetherControlTelemetry(FlightMode flight_mode) {
    tether_down_.control_telemetry.flight_mode =
        static_cast<uint8_t>(flight_mode);
    CvtPutTetherDownMessage(kAioNodeCsA, &tether_down_, g_sequence++,
                            g_timestamp++);
  }

 protected:
  FlightCommander commander_;
  FlightCommandMessage command_;
  TetherDownMessage tether_down_;
};

TEST_F(StandardFlightCommanderTest, SuccessOnFirstTry) {
  PlantTetherControlTelemetry(kFlightModeHoverFullLength);
  EXPECT_TRUE(commander_.RunOnce());
  EXPECT_TRUE(FoundFlightCommandMessage(&command_));
  EXPECT_TRUE(HoverAccelCommanded(&command_));

  PlantTetherControlTelemetry(kFlightModeHoverAccel);
  EXPECT_FALSE(commander_.RunOnce());
  EXPECT_TRUE(DidntFindFlightCommandMessage(&command_));
}

TEST_F(StandardFlightCommanderTest, AFewTriesBeforeSuccess) {
  for (int32_t i = 0; i < 3; ++i) {
    PlantTetherControlTelemetry(kFlightModeHoverFullLength);
    EXPECT_TRUE(commander_.RunOnce());
    EXPECT_TRUE(FoundFlightCommandMessage(&command_));
    EXPECT_TRUE(HoverAccelCommanded(&command_));
  }

  PlantTetherControlTelemetry(kFlightModeHoverAccel);
  EXPECT_FALSE(commander_.RunOnce());
  EXPECT_TRUE(DidntFindFlightCommandMessage(&command_));
}

TEST(FlightCommanderTest, SimulatedTetherDown) {
  FlightCommander commander(true, kFlightModeHoverAccel, false,
                            kGroundStationModeReel, false, false, 10, true, 0,
                            0);

  auto sim_tether_down = SimTetherDownMessage();
  sim_tether_down.messages[kTetherDownSourceCsA].control_telemetry.flight_mode =
      static_cast<uint8_t>(kFlightModeHoverFullLength);
  sim_tether_down.updated[kTetherDownSourceCsA] = true;

  CvtPutSimTetherDownMessage(kAioNodeSimulator, &sim_tether_down, g_sequence++,
                             g_timestamp++);

  EXPECT_TRUE(commander.RunOnce());

  auto command = FlightCommandMessage();
  EXPECT_TRUE(FoundFlightCommandMessage(&command));
  EXPECT_TRUE(HoverAccelCommanded(&command));
}

TEST(FlightCommanderTest, SimulatedExperimentCommand) {
  ExperimentType experiment_type = kExperimentTypeHoverElevator;
  uint8_t experiment_case_id = 1;
  FlightCommander commander(false, kFlightModeHoverFullLength, false,
                            kNumGroundStationModes, false, false, 10, true,
                            experiment_type, experiment_case_id);

  auto sim_tether_down = SimTetherDownMessage();
  sim_tether_down.messages[kTetherDownSourceCsA].control_telemetry.flight_mode =
      static_cast<uint8_t>(kFlightModeHoverFullLength);
  sim_tether_down.messages[kTetherDownSourceCsA]
      .control_telemetry.experiment_type = experiment_type;
  sim_tether_down.messages[kTetherDownSourceCsA]
      .control_telemetry.experiment_case_id = experiment_case_id;
  sim_tether_down.updated[kTetherDownSourceCsA] = true;

  CvtPutSimTetherDownMessage(kAioNodeSimulator, &sim_tether_down, g_sequence++,
                             g_timestamp++);

  EXPECT_TRUE(commander.RunOnce());
  auto command = FlightCommandMessage();
  EXPECT_TRUE(FoundFlightCommandMessage(&command));
  EXPECT_TRUE(
      ExperimentCommanded(&command, experiment_type, experiment_case_id));
}

int main(int argc, char **argv) {
  std::vector<MessageType> subscribe_types = {kMessageTypeFlightCommand};
  AioSetup(kAioNodeOperator, UDP_PORT_AIO, subscribe_types.data(),
           static_cast<int32_t>(subscribe_types.size()));

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

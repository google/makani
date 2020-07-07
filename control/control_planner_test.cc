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

#include <algorithm>
#include <cstdlib>

#include "common/c_math/util.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "system/labels.h"

extern "C" {

bool IsInWindWindow(const WindWindow *window, const WindEstimate *wind_est);

FlightMode ArbitrateFlightMode(ControllerLabel controller_label,
                               const ControlSyncData cont_sync_in[],
                               FlightMode proposed_flight_mode,
                               const FaultMask *faults, PlannerState *state,
                               ControlSyncData *cont_sync_out);

}  // extern "C"

// Convert windward heading to a leeward wind direction as used in the wind
// estimate in the controller.
static double WindToHeadingRad(double from_heading_deg) {
  return (from_heading_deg - 180.0) * PI / 180.0;
}

TEST(IsInWindWindow, Normal) {
  WindWindow window = {5.0, 10.0, 15.0 * PI / 180.0, 200.0 * PI / 180.0};
  WindEstimate est = {};
  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(20.0);
  EXPECT_TRUE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(90.0);
  EXPECT_TRUE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(150.0);
  EXPECT_TRUE(IsInWindWindow(&window, &est));

  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(10.0);
  EXPECT_FALSE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(230.0);
  EXPECT_FALSE(IsInWindWindow(&window, &est));

  est.speed_f_playbook = 2.5;
  est.dir_f_playbook = WindToHeadingRad(40.0);
  EXPECT_FALSE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 12.5;
  est.dir_f_playbook = WindToHeadingRad(40.0);
  EXPECT_FALSE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 12.5;
  est.dir_f_playbook = WindToHeadingRad(215.0);
  EXPECT_FALSE(IsInWindWindow(&window, &est));
}

TEST(IsInWindWindow, CrossesZero) {
  WindWindow window = {5.0, 10.0, 315.0 * PI / 180.0, 80.0 * PI / 180.0};
  WindEstimate est = {};
  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(0.0);
  EXPECT_TRUE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(30.0);
  EXPECT_TRUE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(330.0);
  EXPECT_TRUE(IsInWindWindow(&window, &est));

  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(90.0);
  EXPECT_FALSE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(270.0);
  EXPECT_FALSE(IsInWindWindow(&window, &est));

  est.speed_f_playbook = 2.5;
  est.dir_f_playbook = WindToHeadingRad(40.0);
  EXPECT_FALSE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 12.5;
  est.dir_f_playbook = WindToHeadingRad(40.0);
  EXPECT_FALSE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 12.5;
  est.dir_f_playbook = WindToHeadingRad(200.0);
  EXPECT_FALSE(IsInWindWindow(&window, &est));
}

TEST(IsInWindWindow, NoLimit) {
  WindWindow window = {5.0, 10.0, 10.0 * PI / 180.0, 10.0 * PI / 180.0};
  WindEstimate est = {};
  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(0.0);
  EXPECT_TRUE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(10.0);
  EXPECT_TRUE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 7.5;
  est.dir_f_playbook = WindToHeadingRad(200.0);
  EXPECT_TRUE(IsInWindWindow(&window, &est));

  est.speed_f_playbook = 2.5;
  est.dir_f_playbook = WindToHeadingRad(200.0);
  EXPECT_FALSE(IsInWindWindow(&window, &est));
  est.speed_f_playbook = 12.5;
  est.dir_f_playbook = WindToHeadingRad(200.0);
  EXPECT_FALSE(IsInWindWindow(&window, &est));
}

TEST(ArbitrateFlightMode, Failover) {
  PlannerState state = PlannerState();
  state.controller_sync_sequence = 0U;

  ControlSyncData cont_sync_out;
  for (int32_t i = 0; i < kNumFlightModes; ++i) {
    FaultMask faults[kNumSubsystems];
    std::generate_n(reinterpret_cast<uint8_t *>(&faults), sizeof(faults),
                    std::rand);
    ClearAllFaults(&faults[kSubsysControllerA]);
    ClearAllFaults(&faults[kSubsysControllerB]);
    ClearAllFaults(&faults[kSubsysControllerC]);

    ControlSyncData cont_sync_in[kNumControllers] = {
        {0U, (i + 1) % kNumFlightModes},
        {0U, (i + 2) % kNumFlightModes},
        {0U, (i + 3) % kNumFlightModes}};

    FlightMode decision;
    // A has no fault so:
    //   - A chooses the proposed mode.
    //   - B and C choose A.
    decision = ArbitrateFlightMode(kControllerA, cont_sync_in,
                                   static_cast<FlightMode>(i), faults, &state,
                                   &cont_sync_out);
    EXPECT_EQ(i, decision);
    decision = ArbitrateFlightMode(kControllerB, cont_sync_in,
                                   static_cast<FlightMode>(i), faults, &state,
                                   &cont_sync_out);
    EXPECT_EQ(cont_sync_in[kControllerA].flight_mode, decision);
    decision = ArbitrateFlightMode(kControllerC, cont_sync_in,
                                   static_cast<FlightMode>(i), faults, &state,
                                   &cont_sync_out);
    EXPECT_EQ(cont_sync_in[kControllerA].flight_mode, decision);

    // A has a fault and B has no fault so:
    //   - A and B choose the proposed mode.
    //   - C chooses B.
    SetFault(kFaultTypeNoUpdate, true, &faults[kSubsysControllerA]);
    decision = ArbitrateFlightMode(kControllerA, cont_sync_in,
                                   static_cast<FlightMode>(i), faults, &state,
                                   &cont_sync_out);
    EXPECT_EQ(i, decision);
    decision = ArbitrateFlightMode(kControllerB, cont_sync_in,
                                   static_cast<FlightMode>(i), faults, &state,
                                   &cont_sync_out);
    EXPECT_EQ(i, decision);
    decision = ArbitrateFlightMode(kControllerC, cont_sync_in,
                                   static_cast<FlightMode>(i), faults, &state,
                                   &cont_sync_out);
    EXPECT_EQ(cont_sync_in[kControllerB].flight_mode, decision);

    // A and B have faults so everyone chooses their proposed mode.
    SetFault(kFaultTypeNoUpdate, true, &faults[kSubsysControllerB]);
    decision = ArbitrateFlightMode(kControllerA, cont_sync_in,
                                   static_cast<FlightMode>(i), faults, &state,
                                   &cont_sync_out);
    EXPECT_EQ(i, decision);
    decision = ArbitrateFlightMode(kControllerB, cont_sync_in,
                                   static_cast<FlightMode>(i), faults, &state,
                                   &cont_sync_out);
    EXPECT_EQ(i, decision);
    decision = ArbitrateFlightMode(kControllerC, cont_sync_in,
                                   static_cast<FlightMode>(i), faults, &state,
                                   &cont_sync_out);
    EXPECT_EQ(i, decision);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

#include "control/estimator/estimator_ground_station.h"

#include <gtest/gtest.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/plc_messages.h"
#include "control/estimator/estimator_types.h"

extern "C" {

GroundStationMode DebounceGroundStationMode(GroundStationMode candidate,
                                            bool candidate_valid,
                                            int32_t num_debounce,
                                            EstimatorGroundStationState *state);
}

class DebounceGroundStationModeTest : public ::testing::Test {
 protected:
  DebounceGroundStationModeTest() : num_debounce_(10), state_() {
    EstimatorGroundStationInit(&state_);
  }

  const int32_t num_debounce_;
  EstimatorGroundStationState state_;
};

TEST_F(DebounceGroundStationModeTest, SimpleDebounce) {
  for (int32_t i = 1; i < 2 * num_debounce_; ++i) {
    GroundStationMode mode = DebounceGroundStationMode(
        kGroundStationModeReel, true, num_debounce_, &state_);
    EXPECT_EQ(
        i < num_debounce_ ? kGroundStationModeManual : kGroundStationModeReel,
        mode);
  }
}

TEST_F(DebounceGroundStationModeTest, RampDownOnDifferentMode) {
  // Get one iteration away from switching the mode.
  for (int32_t i = 1; i <= num_debounce_ - 1; ++i) {
    GroundStationMode mode = DebounceGroundStationMode(
        kGroundStationModeReel, true, num_debounce_, &state_);
    EXPECT_EQ(kGroundStationModeManual, mode);
  }

  // Ramp down for 3 iterations.
  for (int32_t i = 1; i <= 3; ++i) {
    GroundStationMode mode = DebounceGroundStationMode(
        kGroundStationModeHighTension, true, num_debounce_, &state_);
    EXPECT_EQ(kGroundStationModeManual, mode);
  }

  // Ramp back up for 3 iterations.
  for (int32_t i = 1; i <= 3; ++i) {
    GroundStationMode mode = DebounceGroundStationMode(
        kGroundStationModeReel, true, num_debounce_, &state_);
    EXPECT_EQ(kGroundStationModeManual, mode);
  }

  // Mode switch should occur here.
  GroundStationMode mode = DebounceGroundStationMode(
      kGroundStationModeReel, true, num_debounce_, &state_);
  EXPECT_EQ(kGroundStationModeReel, mode);
}

TEST_F(DebounceGroundStationModeTest, RampDownOnInvalidMode) {
  // Get one iteration away from switching the mode.
  for (int32_t i = 1; i <= num_debounce_ - 1; ++i) {
    GroundStationMode mode = DebounceGroundStationMode(
        kGroundStationModeReel, true, num_debounce_, &state_);
    EXPECT_EQ(kGroundStationModeManual, mode);
  }

  // Ramp down for 3 iterations.
  for (int32_t i = 1; i <= 3; ++i) {
    GroundStationMode mode = DebounceGroundStationMode(
        kGroundStationModeReel, false, num_debounce_, &state_);
    EXPECT_EQ(kGroundStationModeManual, mode);
  }

  // Ramp back up for 3 iterations.
  for (int32_t i = 1; i <= 3; ++i) {
    GroundStationMode mode = DebounceGroundStationMode(
        kGroundStationModeReel, true, num_debounce_, &state_);
    EXPECT_EQ(kGroundStationModeManual, mode);
  }

  // Mode switch should occur here.
  GroundStationMode mode = DebounceGroundStationMode(
      kGroundStationModeReel, true, num_debounce_, &state_);
  EXPECT_EQ(kGroundStationModeReel, mode);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

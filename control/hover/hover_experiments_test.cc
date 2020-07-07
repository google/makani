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

#include "common/c_math/filter.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/hover/hover.h"
#include "control/hover/hover_experiments.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"

TEST(HoverInjectInit, Nothing) {
  HoverExperimentState hover_experiment_state = HoverExperimentState();
  HoverExperimentsInit(&GetControlParams()->hover.experiments,
                       &hover_experiment_state);
}

TEST(HoverElevatorExperimentIsEnabled, Default) {
  FlightStatus flight_status = FlightStatus();
  ExperimentState experiment_state = ExperimentState();

  flight_status.flight_mode = kFlightModeHoverTransformGsUp;
  experiment_state.active_type = kExperimentTypeHoverElevator;

  EXPECT_TRUE(
      HoverElevatorExperimentIsEnabled(&flight_status, &experiment_state));

  flight_status.flight_mode = kFlightModeHoverTransformGsDown;
  EXPECT_TRUE(
      HoverElevatorExperimentIsEnabled(&flight_status, &experiment_state));
}

TEST(HoverElevatorExperimentIsEnabled, NotEnabledDifferentFlightMode) {
  FlightStatus flight_status = FlightStatus();
  ExperimentState experiment_state = ExperimentState();

  flight_status.flight_mode = kFlightModePilotHover;
  experiment_state.active_type = kExperimentTypeHoverElevator;

  EXPECT_FALSE(
      HoverElevatorExperimentIsEnabled(&flight_status, &experiment_state));
}

TEST(HoverElevatorExperimentIsEnabled, NotEnabledDifferentActiveType) {
  FlightStatus flight_status = FlightStatus();
  ExperimentState experiment_state = ExperimentState();

  flight_status.flight_mode = kFlightModePilotHover;
  experiment_state.active_type = kExperimentTypeCrosswindSpoiler;

  EXPECT_FALSE(
      HoverElevatorExperimentIsEnabled(&flight_status, &experiment_state));
}

TEST(HoverElevatorExperimentsOutputSignal, ExperimentEnabled) {
  const HoverExperiments *hover_experiments =
      &GetControlParams()->hover.experiments;

  // The experiment is intended to be executed in TransformUp
  FlightStatus flight_status = FlightStatus();
  flight_status.flight_mode = kFlightModeHoverTransformGsUp;
  HoverExperimentState hover_experiment_state = HoverExperimentState();
  StateEstimate state_est = StateEstimate();

  double flap_value = 1.234;
  for (uint8_t i = 0; i < ARRAYSIZE(hover_experiments->hover_elevator); ++i) {
    ControlOutput control_output = ControlOutput();
    state_est.experiment.case_id = i;
    state_est.experiment.active_type = kExperimentTypeHoverElevator;
    control_output.flaps[kFlapEle] = flap_value;

    hover_experiment_state.elevator_cmd_z1 = 0.0;
    HoverElevatorExperimentsOutputSignal(
        &flight_status, &control_output, &state_est, hover_experiments,
        &hover_experiment_state, &control_output);

    EXPECT_NEAR(control_output.flaps[kFlapEle],
                hover_experiment_state.elevator_cmd_z1, 1e-9);
  }
}

TEST(HoverElevatorExperimentsOutputSignal, ExperimentDisabled) {
  const HoverExperiments *hover_experiments =
      &GetControlParams()->hover.experiments;

  // The experiment should never be enabled in PilotHover
  FlightStatus flight_status = FlightStatus();
  flight_status.flight_mode = kFlightModePilotHover;
  HoverExperimentState hover_experiment_state = HoverExperimentState();
  StateEstimate state_est = StateEstimate();

  double flap_value = 1.234;
  for (uint8_t i = 0; i < ARRAYSIZE(hover_experiments->hover_elevator); ++i) {
    ControlOutput control_output = ControlOutput();
    state_est.experiment.case_id = i;
    state_est.experiment.active_type = kExperimentTypeHoverElevator;

    control_output.flaps[kFlapEle] = flap_value;

    hover_experiment_state.elevator_cmd_z1 = 0.0;
    HoverElevatorExperimentsOutputSignal(
        &flight_status, &control_output, &state_est, hover_experiments,
        &hover_experiment_state, &control_output);

    EXPECT_NEAR(control_output.flaps[kFlapEle], flap_value, 1e-9);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

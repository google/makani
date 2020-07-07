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

// Commands the flight controller to force a flight mode transition, regardless
// of gates.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "avionics/linux/aio.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/experiments/experiment_util.h"
#include "gs/flight_command/flight_commander.h"
#include "lib/util/operator_confirmation.h"

DEFINE_bool(hover_accel, false, "Force HoverAccel flight mode.");
DEFINE_bool(gs_reel, false,
            "Command the GS to reel mode, if in pilot "
            "hover.");
DEFINE_bool(gs_ht, false,
            "Command the GS to high tension mode, if in "
            "pilot hover.");
DEFINE_bool(gs_unpause, false, "Command the GS to unpause during a transform.");
DEFINE_bool(detwist_turn_once, false, "Command the detwist to turn once.");
DEFINE_bool(sim, false, "Whether to operate with simulator support.");
DEFINE_int32(test, 0, "Set the experiment type. Must be greater than 0");
DEFINE_int32(case, -1, "Set the experiment case ID. Must be greater than -1");

namespace {

bool AioCallback(void *ptr) {
  return static_cast<FlightCommander *>(ptr)->RunOnce();
}

std::string GetExperimentCaseDescription(ExperimentType experiment_type,
                                         uint8_t case_id) {
  std::ostringstream experiment;
  const ControlParams *control_params = GetControlParams();
  switch (experiment_type) {
    case kExperimentTypeHoverElevator:
      experiment << "Set 'elevator' to "
                 << RadToDeg(control_params->hover.experiments
                                 .hover_elevator[case_id]
                                 .elevator)
                 << " degrees.";
      break;
    case kExperimentTypeCrosswindSpoiler:
      experiment
          << "Set 'target' to "
          << control_params->crosswind.experiments.crosswind_spoiler[case_id]
                 .target
          << "\nSet 'start_loop_angle' to "
          << control_params->crosswind.experiments.crosswind_spoiler[case_id]
                 .start_loop_angle
          << "\nSet 'end_loop_angle' to "
          << control_params->crosswind.experiments.crosswind_spoiler[case_id]
                 .end_loop_angle
          << "\nSet 'always_on' to "
          << control_params->crosswind.experiments.crosswind_spoiler[case_id]
                 .always_on;
      break;
    default:
    case kExperimentTypeNoTest:
    case kNumExperimentTypes:
      assert(false);
      return "<Unknown>";
  }
  return experiment.str();
}

std::string GetActionString() {
  std::ostringstream action;
  if (FLAGS_hover_accel) {
    action << "Force HoverAccel right now!  ARE YOU SURE??";
  } else if (FLAGS_gs_reel) {
    action << "Force GS to reel mode (in pilot hover).  ARE YOU SURE??";
  } else if (FLAGS_gs_ht) {
    action << "Force GS to high tension mode (in pilot hover).  ARE YOU SURE??";
  } else if (FLAGS_gs_unpause) {
    action << "Unpause the GS transform (if it is currently paused).";
  } else if (FLAGS_detwist_turn_once) {
    action << "Command the detwist to make one rotation (if in hover).";
  } else if (FLAGS_test > 0 && FLAGS_case > -1) {
    ExperimentType experiment_type = static_cast<ExperimentType>(FLAGS_test);
    action << "Set the test '" << ExperimentTypeToString(experiment_type)
           << "' and case id '" << FLAGS_case << "':\n"
           << GetExperimentCaseDescription(experiment_type,
                                           (uint8_t)FLAGS_case);
  }
  return action.str();
}

}  // namespace

int main(int argc, char *argv[]) {
  FLAGS_stderrthreshold = 0;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage("Command a flight mode transition.");

  bool set_flight_mode = false;
  FlightMode flight_mode;
  bool set_ground_station_mode = false;
  bool gs_unpause_transform = false;
  bool force_detwist_turn_once = false;
  GroundStationMode ground_station_mode = kNumGroundStationModes;
  const ControlParams *control_params = GetControlParams();

  if ((FLAGS_hover_accel + FLAGS_gs_reel + FLAGS_gs_ht + FLAGS_gs_unpause +
       FLAGS_detwist_turn_once + (FLAGS_test != 0 && FLAGS_case != -1)) != 1) {
    LOG(ERROR) << "Must specify exactly one flight mode or GS command, "
               << "or a valid experiment config (test > 0 and case >= 0).";
    google::ShowUsageWithFlagsRestrict(argv[0], "flight_command");
    return EXIT_FAILURE;
  } else if (FLAGS_hover_accel) {
    set_flight_mode = true;
    flight_mode = kFlightModeHoverAccel;
  } else if (FLAGS_gs_reel) {
    set_ground_station_mode = true;
    ground_station_mode = kGroundStationModeReel;
  } else if (FLAGS_gs_ht) {
    set_ground_station_mode = true;
    ground_station_mode = kGroundStationModeHighTension;
  } else if (FLAGS_gs_unpause) {
    gs_unpause_transform = true;
  } else if (FLAGS_detwist_turn_once) {
    force_detwist_turn_once = true;
  } else if (FLAGS_test != 0) {
    if (FLAGS_test < 0 || FLAGS_case < -1) {
      LOG(ERROR) << "`test` must be greater than 0 and `case` must be greater "
                    "than -1.";
      google::ShowUsageWithFlagsRestrict(argv[0], "flight_command");
      return EXIT_FAILURE;
    } else if (FLAGS_test >= kNumExperimentTypes) {
      LOG(ERROR) << "Invalid test type: " << FLAGS_test;
      return EXIT_FAILURE;
    } else if (FLAGS_case >
               GetNumberOfExperimentCases(
                   static_cast<ExperimentType>(FLAGS_test), control_params)) {
      LOG(ERROR) << "Invalid case id: " << FLAGS_case;
      return EXIT_FAILURE;
    }
  }

  CHECK(OperatorConfirmationPrompt(GetActionString()))
      << "The string you entered does not match.";

  FlightCommander commander(
      set_flight_mode, flight_mode, set_ground_station_mode,
      ground_station_mode, gs_unpause_transform, force_detwist_turn_once, 10,
      FLAGS_sim, (uint8_t)FLAGS_test, (uint8_t)FLAGS_case);
  std::vector<MessageType> subscribe_types = {kMessageTypeTetherDown};
  if (FLAGS_sim) {
    subscribe_types.push_back(kMessageTypeSimTetherDown);
  }

  const double ts = 0.1;  // Loop period [s] for AIO callback (10 Hz).
  AioLoopStart(kAioNodeOperator, UDP_PORT_AIO, subscribe_types.data(),
               static_cast<int32_t>(subscribe_types.size()), AioCallback,
               &commander, static_cast<int32_t>(1000000 * ts));
  AioClose();

  return EXIT_SUCCESS;
}

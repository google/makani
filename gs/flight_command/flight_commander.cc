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

#include "gs/flight_command/flight_commander.h"

#include <glog/logging.h>
#include <stdio.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/safety_codes.h"
#include "avionics/common/tether_convert.h"
#include "avionics/common/tether_message.h"
#include "avionics/linux/aio.h"
#include "avionics/network/message_type.h"
#include "control/control_params.h"
#include "control/experiments/experiment_types.h"
#include "control/experiments/experiment_util.h"
#include "control/hover/hover_types.h"
#include "sim/cvt_sim_messages.h"
#include "sim/sim_messages.h"

FlightCommander::FlightCommander(
    bool set_flight_mode, FlightMode commanded_mode,
    bool set_ground_station_mode, GroundStationMode ground_station_mode,
    bool gs_unpause_transform, bool force_detwist_turn_once,
    int32_t num_attempts, bool support_sim, uint8_t experiment_type,
    uint8_t experiment_case_id)
    : set_flight_mode_(set_flight_mode),
      commanded_mode_(commanded_mode),
      set_ground_station_mode_(set_ground_station_mode),
      ground_station_mode_(ground_station_mode),
      gs_unpause_transform_(gs_unpause_transform),
      force_detwist_turn_once_(force_detwist_turn_once),
      attempts_left_(num_attempts),
      experiment_type_(experiment_type),
      experiment_case_id_(experiment_case_id),
      support_sim_(support_sim),
      controller_heard_(false),
      last_flight_mode_(kFlightModeForceSigned),
      last_ground_station_mode_(kNumGroundStationModes),
      last_gs_unpause_transform_(false),
      last_force_detwist_turn_once_(false),
      tether_down_merge_state_() {
  int32_t num_operations =
      (set_flight_mode != 0) + (set_ground_station_mode != 0) +
      (gs_unpause_transform != 0) + (force_detwist_turn_once != 0) +
      (experiment_type > 0);
  CHECK_NE(num_operations, 0) << "No operation selected.";
  CHECK_EQ(num_operations, 1) << "Only one operation is supported at a time";
  if (set_flight_mode) {
    CHECK_EQ(commanded_mode_, kFlightModeHoverAccel)
        << "Only HoverAccel is currently supported.";
  }
  TetherDownMergeStateInit(&tether_down_merge_state_);
}

bool FlightCommander::RunOnce() {
  if (support_sim_) {
    SimTetherDownMessage sim_tether_down;
    uint16_t sequence;
    int64_t timestamp;
    if (CvtGetSimTetherDownMessage(kAioNodeSimulator, &sim_tether_down,
                                   &sequence, &timestamp)) {
      for (int32_t i = 0; i < kNumTetherDownSources; ++i) {
        if (sim_tether_down.updated[i]) {
          CvtPutTetherDownMessage(
              TetherDownSourceToAioNode(static_cast<TetherDownSource>(i)),
              &sim_tether_down.messages[i], sequence, timestamp);
        }
      }
    }
  }

  attempts_left_--;
  bool should_continue = attempts_left_ > 0;

  const TetherDownMessage *m = TetherDownMergeCvtGet(&tether_down_merge_state_);
  const TetherControlTelemetry &control_telemetry = m->control_telemetry;
  const TetherControlCommand &control_command = m->control_command;
  if (TetherIsNoUpdateCountValid(control_telemetry.no_update_count)) {
    controller_heard_ = true;
    last_flight_mode_ = static_cast<FlightMode>(control_telemetry.flight_mode);
    last_ground_station_mode_ =
        static_cast<GroundStationMode>(control_command.gs_mode_request);
    last_gs_unpause_transform_ = control_command.gs_unpause_transform;
    last_force_detwist_turn_once_ = control_telemetry.force_detwist_turn_once;
  }

  if (!controller_heard_) {
    printf("Haven't heard from controller yet. Not sending command.\n");
    return should_continue;
  }

  if (set_flight_mode_) {
    if (last_flight_mode_ == commanded_mode_) {
      printf("Requested flight mode reached; terminating.\n");
      return false;
    }
  } else if (set_ground_station_mode_) {
    if (last_ground_station_mode_ == ground_station_mode_) {
      printf("Requested ground station mode commanded; terminating.\n");
      return false;
    }
  } else if (gs_unpause_transform_) {
    if (last_gs_unpause_transform_ == gs_unpause_transform_) {
      printf("GS unpause sent; terminating.\n");
      return false;
    }
  } else if (force_detwist_turn_once_) {
    if (last_force_detwist_turn_once_ == force_detwist_turn_once_) {
      printf("Force detwist align sent; terminating.\n");
      return false;
    }
  }

  auto command = FlightCommandMessage();
  command.force_hover_accel = false;
  command.force_high_tension = false;
  command.force_reel = false;

  if (set_flight_mode_) {
    if (commanded_mode_ == kFlightModeHoverAccel) {
      command.force_hover_accel = true;
      command.safety_code = FLIGHT_COMMAND_SIGNAL;
    } else {
      LOG(FATAL) << "Bad value for commanded_mode_: "
                 << static_cast<int32_t>(commanded_mode_);
    }
  } else if (set_ground_station_mode_) {
    if (ground_station_mode_ == kGroundStationModeReel) {
      command.force_reel = true;
      command.safety_code = FLIGHT_COMMAND_SIGNAL;
    } else if (ground_station_mode_ == kGroundStationModeHighTension) {
      command.force_high_tension = true;
      command.safety_code = FLIGHT_COMMAND_SIGNAL;
    } else {
      LOG(FATAL) << "Bad value for ground_station_mode_: "
                 << static_cast<int32_t>(ground_station_mode_);
    }
  } else if (gs_unpause_transform_) {
    command.gs_unpause_transform = true;
    command.safety_code = FLIGHT_COMMAND_SIGNAL;
  } else if (force_detwist_turn_once_) {
    command.force_detwist_turn_once = true;
    command.safety_code = FLIGHT_COMMAND_SIGNAL;
  } else if (experiment_type_ > 0) {
    if (experiment_type_ >= kNumExperimentTypes) {
      LOG(FATAL) << "Bad value for experiment_type_: " << experiment_type_;
    } else {
      command.experiment_type = static_cast<ExperimentType>(experiment_type_);
      command.experiment_case_id = experiment_case_id_;
      command.safety_code = FLIGHT_COMMAND_SIGNAL;
    }
  } else {
    LOG(FATAL) << "No flight command options selected.";
  }

  AIO_SEND_PACKED(kMessageTypeFlightCommand, PackFlightCommandMessage,
                  PACK_FLIGHTCOMMANDMESSAGE_SIZE, &command);

  printf("Command sent.\n");

  return should_continue;
}

/*
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GS_FLIGHT_COMMAND_FLIGHT_COMMANDER_H_
#define GS_FLIGHT_COMMAND_FLIGHT_COMMANDER_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/tether_message_types.h"
#include "control/control_types.h"

// Issues FlightCommandMessages to the flight controller to force a
// transition to a specified flight mode, overriding any flight mode gates.
class FlightCommander {
 public:
  FlightCommander(bool set_flight_mode, FlightMode commanded_mode,
                  bool set_ground_station_mode,
                  GroundStationMode ground_station_mode,
                  bool gs_unpause_transform, bool force_detwist_turn_once,
                  int32_t num_attempts, bool support_sim,
                  uint8_t experiment_type, uint8_t experiment_case_id);

  // Sends a FlightCommandMessage if the controller is in the flight mode
  // immediately preceding the commanded flight mote. Returns true if this
  // function should be run again.
  //
  // Returns false to signal termination either when the maximum number of
  // attempts is reached or when the flight controller has reached the commanded
  // flight mode.
  bool RunOnce();

 private:
  const bool set_flight_mode_;
  const FlightMode commanded_mode_;
  const bool set_ground_station_mode_;
  const GroundStationMode ground_station_mode_;
  const bool gs_unpause_transform_;
  const bool force_detwist_turn_once_;
  int32_t attempts_left_;
  const uint8_t experiment_type_;
  const uint8_t experiment_case_id_;

  // If true, TetherDownMessages will be unpacked into the CVT from
  // SimTetherDownMessages sent by the simulator.
  const bool support_sim_;

  bool controller_heard_;
  FlightMode last_flight_mode_;
  GroundStationMode last_ground_station_mode_;
  bool last_gs_unpause_transform_;
  bool last_force_detwist_turn_once_;

  TetherDownMergeState tether_down_merge_state_;
};

#endif  // GS_FLIGHT_COMMAND_FLIGHT_COMMANDER_H_

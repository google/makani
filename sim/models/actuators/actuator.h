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

// The Actuator class is an abstract class that represents anything
// that is controlled by the controller (e.g. servos, tether release,
// motor drivers).  It provides the SetFromAvionicsPackets function,
// called in the main simulation loop, which updates the discrete
// state of model to reflect the signal from the controller.  All
// classes which derive from Actuator must implement a
// SetFromAvionicsPackets, which actually does the dirty work of
// updating state from the avionics packet.

#ifndef SIM_MODELS_ACTUATORS_ACTUATOR_H_
#define SIM_MODELS_ACTUATORS_ACTUATOR_H_

#include <stdint.h>

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "avionics/common/avionics_messages.h"
#include "common/macros.h"
#include "sim/models/model.h"
#include "sim/sim_messages.h"

struct AvionicsPackets {
  AvionicsPackets()
      : command_message(),
        ground_station_status(),
        ground_station_status_valid(false),
        motor_statuses(),
        loadcell_messages(),
        servo_statuses() {}
  ~AvionicsPackets() {}

  struct ServoLabelHasher {
    std::size_t operator()(ServoLabel label) const {
      return std::hash<int32_t>()(static_cast<int32_t>(label));
    }
  };

  ControllerCommandMessage command_message;
  GroundStationStatusMessage ground_station_status;
  bool ground_station_status_valid;
  std::vector<MotorStatusMessage> motor_statuses;
  std::vector<LoadcellMessage> loadcell_messages;
  std::unordered_map<ServoLabel, ServoDebugMessage, ServoLabelHasher>
      servo_statuses;
};

class Actuator : public Model {
 public:
  Actuator(const std::string &name__, double ts__) : Model(name__, ts__) {}
  explicit Actuator(const std::string &name__) : Actuator(name__, 0.0) {}
  ~Actuator() __attribute__((noinline)) {}

  virtual void SetFromAvionicsPackets(
      const AvionicsPackets &avionics_packets) = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(Actuator);
};

#endif  // SIM_MODELS_ACTUATORS_ACTUATOR_H_

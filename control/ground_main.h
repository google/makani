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

#ifndef CONTROL_GROUND_MAIN_H_
#define CONTROL_GROUND_MAIN_H_

#include <stdint.h>

#include <string>

#include "common/macros.h"
#include "control/control_types.h"
#include "control/system_types.h"
#include "sim/sim_messages.h"

namespace control {

// GroundStationEstimator is responsible for hooking up the ground station
// estimator to the simulator during on-computer simulations (i.e. not
// hardware-in-the-loop simulations).  It blocks on receiving faked
// sensor data and simulator commands from the simulator.
// It transmits the estimator output and telemetry information.
class GroundStationEstimator {
 public:
  GroundStationEstimator() : state_() {}
  ~GroundStationEstimator() {}

  GroundStationEstimator(const GroundStationEstimator &) = delete;
  GroundStationEstimator &operator=(const GroundStationEstimator &) = delete;

  void SetupComms();
  void WaitForSim() const;
  void CloseComms() const;

  int32_t Run();

 private:
  // Number of times that the controller will try a blocking receive
  // while waiting for the handshake packet.
  static constexpr int32_t kMaxConnectionAttempts = 50;

  void SendSmallControlTelemetry() const;
  void SendSimInput(const ControllerCommandMessage &command_message) const;
  void SendControllerSync(const ControllerSyncMessage &sync_message) const;
  void SendControlSlowTelemetry() const;
  void SendControlTelemetry() const;
  void SendControlDebug() const;
  void SendQ7SlowStatusMessage() const;
  bool ReceiveSimOutput(SimSensorMessage *sensor_message, uint16_t *sequence,
                        int64_t *timestamp);
  void SendControllerSync(
      ControllerSyncMessage sync_message[kNumControllers]) const;
  bool ReceiveSimCommand(SimCommandMessage *sim_command) const;
  void HandleSimCommand(const SimCommandMessage &sim_command);
  std::string GetSavedStateFilename() const;

  GroundEstimatorState state_;
};

}  // namespace control

#endif  // CONTROL_GROUND_MAIN_H_

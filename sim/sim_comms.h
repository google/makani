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

#ifndef SIM_SIM_COMMS_H_
#define SIM_SIM_COMMS_H_

#include <glog/logging.h>
#include <stdint.h>

#include <string>
#include <vector>

#include "avionics/common/controller_arbitration.h"
#include "avionics/linux/aio.h"
#include "avionics/network/aio_node.h"
#include "control/control_telemetry.h"
#include "control/system_types.h"
#include "sim/models/actuators/actuator.h"
#include "sim/sim_messages.h"
#include "sim/sim_telemetry.h"
#include "system/labels.h"

namespace sim {

struct SimTetherDownUpdateState {
  SmallControlTelemetryMessage small_control_telemetry;
  bool small_control_telemetry_updated;
};

// Base class for all simulator communication. Depending on the
// simulation condition the concrete implementations of this class may
// handle these connections synchronously or asynchronously.
class SimCommsInterface {
 public:
  SimCommsInterface() {}
  virtual ~SimCommsInterface() {}

  virtual void ReceiveAvionicsPackets(
      AvionicsPackets *avionics_packets, ControllerLabel *leader,
      SimTetherDownUpdateState *tether_down_update_state) = 0;
  virtual void ReceiveControllerSync(
      SimSensorMessage *sensor_message) const = 0;
  virtual void ReceiveControlTelemetry() = 0;

  bool ReceiveSimCommand(SimCommandMessage *command) const;
  void SendControllerInput(const SimSensorMessage &sensor_message) const;
  void SendSimTelemetry(const SimTelemetry &sim_telem) const;
  void SendSimCommand(const SimCommandMessage &command) const;
  void SendDynoCommand(const DynoCommandMessage &command) const;
  void SendSimTetherDownMessage(const SimTetherDownMessage &message) const;
};

// Class for synchronous simulator communications.  Blocks on
// receiving messages from the controllers, and also performs a
// handshake with the controllers before starting simulation.
class SimSyncComms : public SimCommsInterface {
 public:
  explicit SimSyncComms(int32_t num_controllers)
      : num_controllers_(num_controllers), controllers_started_(false) {}

  ~SimSyncComms() {}

  void WaitForControllers();
  void WaitForGroundStationEstimator();

  void ReceiveAvionicsPackets(
      AvionicsPackets *avionics_packets, ControllerLabel *leader,
      SimTetherDownUpdateState *tether_down_update_state) override;
  void ReceiveControllerSync(SimSensorMessage *sensor_message) const override;
  void ReceiveControlTelemetry() override;

 private:
  // Number of times that the simulator will try a blocking receive
  // while waiting for the handshake packet.
  static constexpr int32_t kMaxConnectionAttempts = 10;

  const int32_t num_controllers_;

  // The controllers run a state machine which waits for valid data to
  // become available before starting to run.  During this time,
  // telemetry is transmitted but no synchronization or command
  // messages are sent.  This flag is used to indicate that the state
  // machine has progressed to the point where all controllers are
  // running.  When multiple controllers are simulated it is assumed
  // that they will simultaneously arrive at the running state.
  bool controllers_started_;
};

// Class for handling asynchronous communication.  Uses the CVT to
// receive the most recent controller messages.
class SimAsyncComms : public SimCommsInterface {
 public:
  explicit SimAsyncComms(int32_t num_controllers__,
                         const HitlConfiguration &hitl_config);
  ~SimAsyncComms() {}

  int32_t num_controllers() const { return num_controllers_; }

  void ReceiveAvionicsPackets(
      AvionicsPackets *avionics_packets, ControllerLabel *leader,
      SimTetherDownUpdateState *tether_down_update_state) override;

  // During asynchronous communication, the controllers receive the
  // synchronization messages directly from one another.
  void ReceiveControllerSync(SimSensorMessage * /*message*/) const override {}
  void ReceiveControlTelemetry() override {}

  // Starts the update timer for incoming messages.
  void StartUpdateTimer();

  // Maximum time [s] between controller updates before the simulator exits.
  static constexpr double kCommsTimeout = 2.0;

 private:
  // Records the last update time [us] from a variety of sources. Update times
  // for components other than the controllers are only used during HITL.
  struct LastUpdateUsecs {
    explicit LastUpdateUsecs(int64_t start_usec)
        : any_controller(start_usec),
          gs02(start_usec),
          motors(kNumMotors, start_usec),
          loadcell_nodes(kNumLoadcellNodes, start_usec),
          servos(kNumServos, start_usec),
          winch(start_usec) {}
    ~LastUpdateUsecs() {}

    int64_t any_controller = 0;
    int64_t gs02;
    std::vector<int64_t> motors;
    std::vector<int64_t> loadcell_nodes;
    std::vector<int64_t> servos;
    int64_t winch;
  };

  const int32_t num_controllers_;
  const HitlConfiguration &hitl_config_;
  LastUpdateUsecs last_update_usecs_;
  ControllerArbitrationState controller_arbitration_state_;
};

}  // namespace sim

#endif  // SIM_SIM_COMMS_H_

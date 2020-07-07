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

#include "sim/sim_comms.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <stdint.h>
#include <sys/time.h>

#include <limits>
#include <string>
#include <vector>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/controller_arbitration.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/linux/aio.h"
#include "avionics/linux/clock.h"
#include "avionics/linux/cvt_util.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "control/common.h"
#include "control/control_telemetry.h"
#include "control/cvt_control_telemetry.h"
#include "control/system_types.h"
#include "sim/cvt_sim_messages.h"
#include "sim/pack_sim_messages.h"
#include "sim/pack_sim_telemetry.h"
#include "sim/sim_messages.h"
#include "sim/sim_telemetry.h"
#include "system/labels.h"

// This value is large due to timeout issues on GCE. See https://goo.gl/wkzH0d.
DEFINE_double(comms_timeout_sec, 5.0,
              "Timeout [s] for blocking receives from the controller during "
              "synchronous execution.");

namespace {

void CheckUpdateTime(int64_t last_update_usec, double timeout_sec,
                     int64_t now_usec, const std::string &name) {
  CHECK_GT(last_update_usec + static_cast<int64_t>(timeout_sec * 1000000),
           now_usec)
      << "Have not heard from " << name << " for too long. Exiting.";
}

// TODO: Validation here.
bool CommandValidator(const ControllerCommandMessage * /* msg */) {
  return true;
}

}  // namespace

namespace sim {

// Receive a SimCommandMessage over AIO.
//
// This is a non-blocking read.
//
// Args:
//   command: Message to read values into (if no packet has arrived, this
//       variable is left unchanged).
//
// Returns:
//   True if an updated packet was obtained.
bool SimCommsInterface::ReceiveSimCommand(SimCommandMessage *command) const {
  return CvtGetNextUpdate(kAioNodeSimulatedJoystick, CvtGetSimCommandMessage,
                          0.0, command);
}

// Send a SimSensorMessage to the destinations determined by Setup() (generally
// flight computers in HITL simulations and control processes otherwise).
//
// Args:
//   sensor_message: SimSensorMessage to be sent to a particular controller.
void SimCommsInterface::SendControllerInput(
    const SimSensorMessage &sensor_message) const {
  AIO_SEND_PACKED(kMessageTypeSimSensor, PackSimSensorMessage,
                  PACK_SIMSENSORMESSAGE_SIZE, &sensor_message);
}

void SimCommsInterface::SendDynoCommand(
    const DynoCommandMessage &command) const {
  AIO_SEND_PACKED(kMessageTypeDynoCommand, PackDynoCommandMessage,
                  PACK_DYNOCOMMANDMESSAGE_SIZE, &command);
}

// Sends a telemetry message to the location determined by Setup().
//
// Args:
//   sim_telem: Telemetry to transmit.
void SimCommsInterface::SendSimTelemetry(const SimTelemetry &sim_telem) const {
  AIO_SEND_PACKED(kMessageTypeSimTelemetry, PackSimTelemetry,
                  PACK_SIMTELEMETRY_SIZE, &sim_telem);
}

void SimCommsInterface::SendSimTetherDownMessage(
    const SimTetherDownMessage &message) const {
  AIO_SEND_PACKED(kMessageTypeSimTetherDown, PackSimTetherDownMessage,
                  PACK_SIMTETHERDOWNMESSAGE_SIZE, &message);
}

// Sends SimCommandMessage over AIO.
//
// Args:
//   command: Message to transmit.
void SimCommsInterface::SendSimCommand(const SimCommandMessage &command) const {
  AIO_SEND_PACKED(kMessageTypeSimCommand, PackSimCommandMessage,
                  PACK_SIMCOMMANDMESSAGE_SIZE, &command);
}

// Waits to receive a handshake packet from each controller that
// indicates that the controller is running.  Once a packet has been
// received from each controller, this sends an acknowledgment packet
// to each controller.
void SimSyncComms::WaitForControllers() {
  // Wait for controller packets.
  for (int32_t i = 0; i < num_controllers_; ++i) {
    LOG(INFO) << "Waiting for controller " << i << ".";

    SimCommandMessage control_handshake;
    memset(&control_handshake, 0, sizeof(control_handshake));

    int32_t num_retries = 0;

    const AioNode controller_node =
        ControllerLabelToControllerAioNode(static_cast<ControllerLabel>(i));
    while (!CvtGetNextUpdate(controller_node, CvtGetSimCommandMessage,
                             FLAGS_comms_timeout_sec, &control_handshake)) {
      CHECK_LT(num_retries, kMaxConnectionAttempts);
      num_retries++;
    }
    CHECK_EQ(kControllerHandshakeSignals[i], control_handshake.handshake)
        << "Received a non-handshake packet during initialization.";
  }

  // Send acknowledgment packets.
  SimCommandMessage sim_handshake = {kSimRecordStateCommandDont,
                                     kSimHandshakeSignal, false};
  AIO_SEND_PACKED(kMessageTypeSimCommand, PackSimCommandMessage,
                  PACK_SIMCOMMANDMESSAGE_SIZE, &sim_handshake);
}

// Waits to receive a handshake packet from the ground station estimator that
// indicates that the estimator is running.  Once a packet has been
// received from the estimator, this sends an acknowledgment packet.
void SimSyncComms::WaitForGroundStationEstimator() {
  // Wait for estimator packets.
  LOG(INFO) << "Waiting for estimator.";

  SimCommandMessage control_handshake;
  memset(&control_handshake, 0, sizeof(control_handshake));

  int32_t num_retries = 0;

  while (!CvtGetNextUpdate(kAioNodeGsEstimator, CvtGetSimCommandMessage,
                           FLAGS_comms_timeout_sec, &control_handshake)) {
    CHECK_LT(num_retries, kMaxConnectionAttempts);
    num_retries++;
  }
  CHECK_EQ(kGroundStationEstimatorHandshakeSignal, control_handshake.handshake)
      << "Received a non-handshake packet during initialization.";

  // Send acknowledgment packets.
  SimCommandMessage sim_handshake = {kSimRecordStateCommandDont,
                                     kSimHandshakeSignal, false};
  AIO_SEND_PACKED(kMessageTypeSimCommand, PackSimCommandMessage,
                  PACK_SIMCOMMANDMESSAGE_SIZE, &sim_handshake);
}

// Receives the avionics packets from the controller(s).
//
// Args:
//   avionics_packets: Packets received.
void SimSyncComms::ReceiveAvionicsPackets(
    AvionicsPackets *avionics_packets, ControllerLabel *leader,
    SimTetherDownUpdateState *tether_down_update_state) {
  // TODO(b/30442564): Support controller arbitration in synchronous sims.
  CHECK_EQ(num_controllers_, 1);
  *leader = kControllerA;

  auto &command_message = avionics_packets->command_message;

  double timeout = controllers_started_ ? FLAGS_comms_timeout_sec : 0.0;
  bool all_updated = true;

  for (int32_t i = 0; i < num_controllers_; ++i) {
    const AioNode controller_node =
        ControllerLabelToControllerAioNode(static_cast<ControllerLabel>(i));
    uint16_t sequence;
    if (CvtGetNextUpdate(controller_node, CvtGetControllerCommandMessage,
                         timeout, &command_message, &sequence, nullptr)) {
      sim_telem.comms.command_sequence[i] = sequence;
    } else {
      all_updated = false;
    }
  }
  if (controllers_started_) {
    CHECK(all_updated)
        << "In synchronous mode, controllers should always send a "
        << "command packet.";
  } else {
    CHECK(!all_updated)
        << "Controllers should not send command packets until they "
        << "have started.";
  }

  CHECK(CvtGetNextUpdate(ControllerLabelToControllerAioNode(*leader),
                         CvtGetSmallControlTelemetryMessage,
                         FLAGS_comms_timeout_sec,
                         &tether_down_update_state->small_control_telemetry))
      << "In synchronous mode, the sim should always receive a "
      << "SmallControlTelemetryMessage.";
  tether_down_update_state->small_control_telemetry_updated = true;
}

void SimSyncComms::ReceiveControllerSync(
    SimSensorMessage *sensor_message) const {
  double timeout = controllers_started_ ? FLAGS_comms_timeout_sec : 0.0;
  for (int32_t i = 0; i < num_controllers_; ++i) {
    const AioNode controller_node =
        ControllerLabelToControllerAioNode(static_cast<ControllerLabel>(i));
    bool is_updated = CvtGetNextUpdate(
        controller_node, CvtGetControllerSyncMessage, timeout,
        &sensor_message->control_input_messages.controller_sync[i]);
    if (controllers_started_) {
      CHECK(is_updated)
          << "In synchronous mode, controllers should always send a "
          << "sync packet.";
    } else {
      CHECK(!is_updated)
          << "Controllers should not send synchronization packets until they "
          << "have started.";
    }

    sensor_message->control_input_messages_updated.controller_sync[i] = true;
  }
}

void SimSyncComms::ReceiveControlTelemetry() {
  bool controllers_running = true;
  for (int32_t i = 0; i < num_controllers_; ++i) {
    ControlTelemetry message;
    const AioNode controller_node =
        ControllerLabelToControllerAioNode(static_cast<ControllerLabel>(i));
    bool is_updated =
        CvtGetNextUpdate(controller_node, CvtGetControlDebugMessage,
                         FLAGS_comms_timeout_sec, &message);
    CHECK(is_updated)
        << "In synchronous mode, controllers should always send a "
        << "telemetry packet.";
    InitializationState init_state =
        static_cast<InitializationState>(message.init_state);
    controllers_running =
        controllers_running && IsControlSystemRunning(init_state);
  }
  if (controllers_running) controllers_started_ = true;
}

SimAsyncComms::SimAsyncComms(int32_t num_controllers__,
                             const HitlConfiguration &hitl_config)
    : num_controllers_(num_controllers__),
      hitl_config_(hitl_config),
      last_update_usecs_(std::numeric_limits<int64_t>::min()),
      controller_arbitration_state_() {
  ControllerArbitrationInit(ClockGetUs(), &controller_arbitration_state_);
}

void SimAsyncComms::StartUpdateTimer() {
  last_update_usecs_ = LastUpdateUsecs(ClockGetUs());
}

// Receives the avionics packets from the controller(s) and/or wing.
//
// Args:
//   avionics_packets: Packets for which to poll AIO.
void SimAsyncComms::ReceiveAvionicsPackets(
    AvionicsPackets *avionics_packets, ControllerLabel *leader,
    SimTetherDownUpdateState *tether_down_update_state) {
  int64_t now_usec = ClockGetUs();

  ControllerArbitrationUpdateFromCvt(&controller_arbitration_state_);
  const ControllerCommandMessage *cvt_message = ControllerArbitrationGetCommand(
      now_usec, CommandValidator, &controller_arbitration_state_, leader);
  if (cvt_message) {
    last_update_usecs_.any_controller = now_usec;
    avionics_packets->command_message = *cvt_message;
  } else {
    CheckUpdateTime(last_update_usecs_.any_controller, kCommsTimeout, now_usec,
                    "any controller");
  }

  if (hitl_config_.gs02_level == kActuatorHitlLevelReal) {
    if (CvtGetGroundStationStatusMessage(
            kAioNodePlcGs02, &avionics_packets->ground_station_status, nullptr,
            nullptr)) {
      avionics_packets->ground_station_status_valid = true;
      last_update_usecs_.gs02 = now_usec;
    } else {
      CheckUpdateTime(last_update_usecs_.gs02, hitl_config_.gs02_timeout_sec,
                      now_usec, std::string("GS02"));
    }
  }

  // In motor HITL, process incoming motor status messages.
  std::vector<MotorStatusMessage> &motor_statuses =
      avionics_packets->motor_statuses;
  int32_t num_motor_statuses = static_cast<int32_t>(motor_statuses.size());
  CHECK(num_motor_statuses == 0 || num_motor_statuses == kNumMotors)
      << "Bad number of motor statuses (" << num_motor_statuses
      << ") found in AvionicsPackets.";

  for (int32_t i = 0; i < num_motor_statuses; ++i) {
    MotorLabel label = static_cast<MotorLabel>(i);
    AioNode node = MotorLabelToMotorAioNode(label);
    if (CvtGetMotorStatusMessage(node, &motor_statuses[i], nullptr, nullptr)) {
      last_update_usecs_.motors[i] = now_usec;
    } else {
      CheckUpdateTime(last_update_usecs_.motors[i],
                      hitl_config_.motor_timeout_sec, now_usec,
                      std::string("motor ") + MotorLabelToString(label));
    }
  }

  // In servo HITL, process incoming servo status messages.
  for (auto &servo_label_and_status : avionics_packets->servo_statuses) {
    ServoLabel label = servo_label_and_status.first;
    AioNode node = ServoLabelToServoAioNode(label);
    if (CvtGetServoDebugMessage(node, &servo_label_and_status.second, nullptr,
                                nullptr)) {
      last_update_usecs_.servos[label] = now_usec;
    } else {
      CheckUpdateTime(last_update_usecs_.servos[label],
                      hitl_config_.servo_timeout_sec, now_usec,
                      std::string("servo ") + ServoLabelToString(label));
    }
  }

  // In tether release HITL, process incoming loadcell messages.
  std::vector<LoadcellMessage> &loadcell_messages =
      avionics_packets->loadcell_messages;
  int32_t num_loadcell_messages =
      static_cast<int32_t>(loadcell_messages.size());
  CHECK(num_loadcell_messages == 0 ||
        num_loadcell_messages == kNumLoadcellNodes)
      << "Bad number of loadcell messages (" << num_loadcell_messages
      << ") found in AvionicsPackets.";

  for (int32_t i = 0; i < num_loadcell_messages; ++i) {
    auto label = static_cast<LoadcellNodeLabel>(i);
    AioNode node = LoadcellNodeLabelToLoadcellNodeAioNode(label);
    if (CvtGetLoadcellMessage(node, &loadcell_messages[i], nullptr, nullptr)) {
      last_update_usecs_.loadcell_nodes[i] = now_usec;
    } else {
      CheckUpdateTime(
          last_update_usecs_.loadcell_nodes[i],
          hitl_config_.tether_release_timeout_sec, now_usec,
          std::string("loadcell node ") + LoadcellNodeLabelToString(label));
    }
  }

  tether_down_update_state->small_control_telemetry_updated =
      CvtGetSmallControlTelemetryMessage(
          ControllerLabelToControllerAioNode(*leader),
          &tether_down_update_state->small_control_telemetry, nullptr, nullptr);
}

}  // namespace sim

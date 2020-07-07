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

#include "control/ground_main.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <signal.h>
#include <stdint.h>

#include <vector>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/linux/aio.h"
#include "avionics/linux/clock.h"
#include "avionics/linux/cvt_util.h"
#include "avionics/linux/q7_slow_status.h"
#include "avionics/linux/q7_slow_status_types.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "common/backtrace.h"
#include "control/avionics/avionics_sim.h"
#include "control/control_params.h"
#include "control/control_system.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_ground.h"
#include "control/pack_control_telemetry.h"
#include "control/pack_ground_telemetry.h"
#include "control/system_params.h"
#include "lib/json_load/load_params.h"
#include "sim/cvt_sim_messages.h"
#include "sim/pack_sim_messages.h"
#include "sim/sim_messages.h"
#include "system/labels.h"

// Time [s] for one of the blocking UDP receives to timeout.
//
// This used to be 1.0, but the simulator occasionally seems to stall on
// Compute Engine, necessitating a larger timeout. See https://goo.gl/wkzH0d
// for more discussion.
//
// The default value was determined empirically by running batch sims.
DEFINE_double(comms_timeout_sec, 7.5,
              "Timeout [s] for blocking receives from the sim during "
              "synchronous execution.");

#if !defined(MAKANI_TEST)

int main(int argc, char *argv[]) {
  InstallBacktraceHandler({SIGABRT, SIGSEGV});
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage("");
  google::ParseCommandLineFlags(&argc, &argv, true);

  json_load::LoadSystemParams(GetSystemParamsUnsafe());
  json_load::LoadControlParams(GetControlParamsUnsafe());
  control::GroundStationEstimator ground_station_estimator;
  ground_station_estimator.SetupComms();
  ground_station_estimator.WaitForSim();
  int32_t ret = ground_station_estimator.Run();
  ground_station_estimator.CloseComms();

  return ret;
}

#endif  // !defined (MAKANI_TEST)

namespace control {

// Creates the UDP sockets for the telemetry, output to the actuators,
// input from the sensors, and input from the simulator.
void GroundStationEstimator::SetupComms() {
  const std::vector<MessageType> kSubscribeTypes = {kMessageTypeSimCommand,
                                                    kMessageTypeSimSensor};
  AioSetup(kAioNodeGsEstimator, GetSystemParams()->comms.aio_port,
           kSubscribeTypes.data(),
           static_cast<int32_t>(kSubscribeTypes.size()));
}

// Sends handshake packets to the simulator to indicate that the
// controller is running and communications have been setup. Continues
// sending the packets until it receives a packet from the simulator.
void GroundStationEstimator::WaitForSim() const {
  SimCommandMessage control_handshake = {kSimRecordStateCommandDont,
                                         kGroundStationEstimatorHandshakeSignal,
                                         false};
  SimCommandMessage sim_handshake = {kSimRecordStateCommandDont, 0U, false};

  bool updated;
  int32_t num_retries = 0;
  do {
    CHECK_LT(num_retries, kMaxConnectionAttempts);
    num_retries++;

    LOG(INFO) << "Waiting for simulator.";
    AIO_SEND_PACKED(kMessageTypeSimCommand, PackSimCommandMessage,
                    PACK_SIMCOMMANDMESSAGE_SIZE, &control_handshake);

    updated = CvtGetNextUpdate(kAioNodeSimulator, CvtGetSimCommandMessage,
                               FLAGS_comms_timeout_sec, &sim_handshake);
  } while (!updated);

  CHECK_EQ(kSimHandshakeSignal, sim_handshake.handshake)
      << "Received a non-handshake packet during initialization.";
}

// Closes the UDP sockets.
void GroundStationEstimator::CloseComms() const { AioClose(); }

// Runs the estimator by looping over the following sequence of
// commands:
//
// 1) Listens for the ground sensor packet and the simulator command
//    packet.
// 2) Runs a single loop of the estimator.
// 3) Handles the simulator command (e.g. saves or loads estimator state).
// 4) Sends the ground estimator message.
int32_t GroundStationEstimator::Run() {
  // TODO: Remove / customize hitl_config for estimator.
  // Override the sync_sim field of the HITL config, copy the config to the
  // telemetry.
  HitlConfiguration &hitl_config = GetSystemParamsUnsafe()->hitl.config;
  hitl_config.sim_level = static_cast<int32_t>(kSimulatorHitlLevelSync);
  hitl_config.use_software_joystick = true;

  GetControlSlowTelemetry()->hitl_config = hitl_config;

  EstimatorGroundInit(&state_);

  // Run main loop.
  SimSensorMessage sensor_message;
  uint16_t sequence;
  int64_t timestamp;
  SimCommandMessage sim_command;
  int32_t num_iterations_completed = 0;
  while (ReceiveSimOutput(&sensor_message, &sequence, &timestamp) &&
         ReceiveSimCommand(&sim_command)) {
    UpdateGroundEstimatorCvtFromSimSensorMessage(&sensor_message, sequence,
                                                 timestamp);

    // Step the estimator forward and handle any commands from the
    // simulator.
    HandleSimCommand(sim_command);

    // TODO(b/66458887): Use a timer that measures process time, not
    // wall-clock time.
    int64_t start_us = ClockGetUs();

    EstimatorGroundStep(GetSystemParams(), GetControlParams(), &state_);

    assert(PACK_GROUNDESTIMATEMESSAGE_SIZE ==
           PACK_GROUNDESTIMATESIMMESSAGE_SIZE);
    AIO_SEND_PACKED(kMessageTypeGroundEstimateSim, PackGroundEstimateSimMessage,
                    PACK_GROUNDESTIMATESIMMESSAGE_SIZE,
                    &state_.ground_estimate);

    // TODO: Replace estimator telemetry.
    GetControlTelemetry()->loop_usec = ClockGetUs() - start_us;

    AIO_SEND_PACKED(kMessageTypeGroundTelemetry, PackGroundTelemetry,
                    PACK_GROUNDTELEMETRY_SIZE, GetGroundTelemetryMessage());

    ++num_iterations_completed;

    if (sim_command.stop) {
      return 0;
    }
  }

  LOG(FATAL) << "Timed out while waiting for simulator messages after "
             << num_iterations_completed << " complete iterations.";

  return 0;
}

// Receives estimator input from the simulator.  Returns true if it
// successfully received a packet.
bool GroundStationEstimator::ReceiveSimOutput(SimSensorMessage *sensor_message,
                                              uint16_t *sequence,
                                              int64_t *timestamp) {
  return CvtGetNextUpdate(kAioNodeSimulator, CvtGetSimSensorMessage,
                          FLAGS_comms_timeout_sec, sensor_message, sequence,
                          timestamp);
}

// Receives a command from the simulator.  Returns true if it
// successfully received a packet.
bool GroundStationEstimator::ReceiveSimCommand(
    SimCommandMessage *sim_command) const {
  return CvtGetNextUpdate(kAioNodeSimulator, CvtGetSimCommandMessage,
                          FLAGS_comms_timeout_sec, sim_command);
}

void GroundStationEstimator::HandleSimCommand(
    const SimCommandMessage &sim_command) {
  // TODO: Add state saving / loading.
  UNUSED(sim_command);
}

}  // namespace control

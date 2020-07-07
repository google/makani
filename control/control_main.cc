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

#include "control/control_main.h"

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
#include "control/pack_control_telemetry.h"
#include "control/system_params.h"
#include "lib/json_load/load_params.h"
#include "sim/cvt_sim_messages.h"
#include "sim/pack_sim_messages.h"
#include "sim/sim_messages.h"
#include "system/labels.h"

DEFINE_int32(controller_label, kControllerA, "Controller label.");

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

namespace {

// Checks that the controller label is within the correct range.
bool ValidateControllerLabel(const char * /*flag_name*/, int32_t value) {
  return 0 <= value && value < kNumControllers;
}
bool dummy __attribute__((unused)) = google::RegisterFlagValidator(
    &FLAGS_controller_label, &ValidateControllerLabel);

}  // namespace

#if !defined(MAKANI_TEST)

int main(int argc, char *argv[]) {
  InstallBacktraceHandler({SIGABRT, SIGSEGV});
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage("");
  google::ParseCommandLineFlags(&argc, &argv, true);

  json_load::LoadSystemParams(GetSystemParamsUnsafe());
  json_load::LoadControlParams(GetControlParamsUnsafe());

  control::ControlMain c(static_cast<ControllerLabel>(FLAGS_controller_label));
  c.SetupComms();
  c.WaitForSim();
  int32_t ret = c.Run();
  c.CloseComms();

  return ret;
}

#endif  // !defined (MAKANI_TEST)

namespace control {

// Creates the UDP sockets for the telemetry, output to the actuators,
// input from the sensors, and input from the simulator.
void ControlMain::SetupComms() {
  const std::vector<MessageType> kSubscribeTypes = {
      kMessageTypeFlightCommand, kMessageTypeSimCommand, kMessageTypeSimSensor,
      kMessageTypeGroundEstimateSim};
  AioSetup(ControllerLabelToControllerAioNode(controller_label_),
           GetSystemParams()->comms.aio_port, kSubscribeTypes.data(),
           static_cast<int32_t>(kSubscribeTypes.size()));
}

// Sends handshake packets to the simulator to indicate that the
// controller is running and communications have been setup. Continues
// sending the packets until it receives a packet from the simulator.
void ControlMain::WaitForSim() const {
  SimCommandMessage control_handshake = {
      kSimRecordStateCommandDont,
      kControllerHandshakeSignals[controller_label_], false};
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
void ControlMain::CloseComms() const { AioClose(); }

// Runs the controller by looping over the following sequence of
// commands:
//
// 1) Listens for the avionics sensor packet and the simulator command
//    packet.
// 2) Runs a single loop of the controller.
// 3) Handles the simulator command (e.g. saves or loads controller state).
// 4) Sends the avionics actuator packet and the telemetry packet.
int32_t ControlMain::Run() {
  // Override the sync_sim field of the HITL config, copy the config to the
  // telemetry.
  HitlConfiguration &hitl_config = GetSystemParamsUnsafe()->hitl.config;
  hitl_config.sim_level = static_cast<int32_t>(kSimulatorHitlLevelSync);
  hitl_config.use_software_joystick = true;

  GetControlSlowTelemetry()->hitl_config = hitl_config;

  // Initialize the control system.

  ControlSystemInit(GetControlParams(), &state_);

  // Initialize the slow status message.
  Q7SlowStatusContext q7_slow_status_context;
  Q7SlowStatusInit(&q7_slow_status_context);
  Q7SlowStatusInitMessage(&q7_slow_status_context, GetQ7SlowStatusMessage());

  // Run main loop.
  SimSensorMessage sensor_message;
  uint16_t sequence;
  int64_t timestamp;
  SimCommandMessage sim_command;
  int32_t num_iterations_completed = 0;
  while (ReceiveSimOutput(&sensor_message, &sequence, &timestamp) &&
         ReceiveSimCommand(&sim_command)) {
    if (sim_command.stop) return 0;

    if (!ReceiveAndPutGsEstimatorOutput()) {
      LOG(FATAL)
          << "Timed out while waiting for ground estimator messages after "
          << num_iterations_completed << " complete iterations.";
      return 0;
    }

    UpdateControllerCvtFromSimSensorMessage(&sensor_message, &hitl_config,
                                            sequence, timestamp);

    // We do not initialize the controller command message here as it allows
    // Valgrind to catch uninitialized command messages.
    //
    // TODO: Make a stand alone test that catches these errors.
    ControllerCommandMessage command_message;

    // Step the controller forward and handle any commands from the
    // simulator.
    HandleSimCommand(sim_command);
    ControllerSyncMessage sync_message;

    // TODO(b/66458887): Use a timer that measures process time, not
    // wall-clock time.
    int64_t start_us = ClockGetUs();
    bool send = ControlSystemStep(controller_label_, GetSystemParams(),
                                  GetControlParams(), &state_, &command_message,
                                  &sync_message);
    GetControlTelemetry()->loop_usec = ClockGetUs() - start_us;

    // Send controller output to the simulator and broadcast
    // telemetry.
    if (send) {
      SendSimInput(command_message);
      SendControllerSync(sync_message);
    }
    *GetControlDebugMessage() = *GetControlTelemetry();

    // NOTE(b/78533616): The simulator (when run on Jenkins!) appears to care
    // (sometimes) about the order in which these messages are sent.
    SendSmallControlTelemetry();
    SendControlDebug();

    if (num_iterations_completed % CONTROL_SLOW_TELEMETRY_DECIMATION == 0) {
      SendControlSlowTelemetry();
    }

    if (num_iterations_completed % Q7_SLOW_STATUS_DECIMATION == 0) {
      SendQ7SlowStatusMessage();
    }

    if (num_iterations_completed % CONTROL_TELEMETRY_DECIMATION == 0) {
      SendControlTelemetry();
    }

    ++num_iterations_completed;
  }

  LOG(FATAL) << "Timed out while waiting for simulator messages after "
             << num_iterations_completed << " complete iterations.";

  return 0;
}

// Sends the pilot telemetry.
void ControlMain::SendSmallControlTelemetry() const {
  AIO_SEND_PACKED(
      kMessageTypeSmallControlTelemetry, PackSmallControlTelemetryMessage,
      PACK_SMALLCONTROLTELEMETRYMESSAGE_SIZE, GetSmallControlTelemetry());
}

// Sends the output of the controller to the simulator.
void ControlMain::SendSimInput(
    const ControllerCommandMessage &command_message) const {
  AIO_SEND_PACKED(kMessageTypeControllerCommand, PackControllerCommandMessage,
                  PACK_CONTROLLERCOMMANDMESSAGE_SIZE, &command_message);
}

// Sends the control debug message.
void ControlMain::SendControlDebug() const {
  AIO_SEND_PACKED(kMessageTypeControlDebug, PackControlDebugMessage,
                  PACK_CONTROLDEBUGMESSAGE_SIZE, GetControlDebugMessage());
}

// Sends the slow telemetry.
void ControlMain::SendControlSlowTelemetry() const {
  AIO_SEND_PACKED(kMessageTypeControlSlowTelemetry, PackControlSlowTelemetry,
                  PACK_CONTROLSLOWTELEMETRY_SIZE, GetControlSlowTelemetry());
}

// Sends the output of the controller to the simulator.
void ControlMain::SendControllerSync(
    const ControllerSyncMessage &sync_message) const {
  AIO_SEND_PACKED(kMessageTypeControllerSync, PackControllerSyncMessage,
                  PACK_CONTROLLERSYNCMESSAGE_SIZE, &sync_message);
}

// Sends controller telemetry.
void ControlMain::SendControlTelemetry() const {
  AIO_SEND_PACKED(kMessageTypeControlTelemetry, PackControlTelemetry,
                  PACK_CONTROLTELEMETRY_SIZE, GetControlTelemetry());
}

// Sends the slow status message.
void ControlMain::SendQ7SlowStatusMessage() const {
  AIO_SEND_PACKED(kMessageTypeQ7SlowStatus, PackQ7SlowStatusMessage,
                  PACK_Q7SLOWSTATUSMESSAGE_SIZE, GetQ7SlowStatusMessage());
}

// Receives controller input from the simulator.  Returns true if it
// successfully received a packet.
bool ControlMain::ReceiveSimOutput(SimSensorMessage *sensor_message,
                                   uint16_t *sequence, int64_t *timestamp) {
  return CvtGetNextUpdate(kAioNodeSimulator, CvtGetSimSensorMessage,
                          FLAGS_comms_timeout_sec, sensor_message, sequence,
                          timestamp);
}

bool ControlMain::ReceiveAndPutGsEstimatorOutput() const {
  GroundEstimateMessage ground_estimate_message;
  uint16_t sequence;
  int64_t timestamp;

  bool received = CvtGetNextUpdate(
      kAioNodeGsEstimator, CvtGetGroundEstimateSimMessage,
      FLAGS_comms_timeout_sec, &ground_estimate_message, &sequence, &timestamp);

  if (!received) {
    return false;
  }

  CvtPutGroundEstimateMessage(kAioNodeGsEstimator, &ground_estimate_message,
                              sequence, timestamp);
  return true;
}
// Receives a command from the simulator.  Returns true if it
// successfully received a packet.
bool ControlMain::ReceiveSimCommand(SimCommandMessage *sim_command) const {
  return CvtGetNextUpdate(kAioNodeSimulator, CvtGetSimCommandMessage,
                          FLAGS_comms_timeout_sec, sim_command);
}

namespace {

void SaveStateToFile(const std::string filename, const ControlState &state) {
  FILE *f = fopen(filename.c_str(), "wb");
  CHECK_NOTNULL(f);
  size_t num_write = fwrite(&state, sizeof(state), 1U, f);
  CHECK_EQ(1, num_write) << "Saved state not written.";
  fclose(f);
}

void LoadStateFromFile(const std::string filename, ControlState *state) {
  FILE *f = fopen(filename.c_str(), "rb");
  CHECK_NOTNULL(f);
  size_t num_read = fread(state, sizeof(*state), 1U, f);
  CHECK_EQ(1, num_read) << "Saved state has too few bytes.";
  CHECK_EQ(fgetc(f), EOF) << "Saved state has too many bytes.";
  fclose(f);
}

}  // namespace

// Saves or loads the state of the controller.
void ControlMain::HandleSimCommand(const SimCommandMessage &sim_command) {
  switch (sim_command.record_mode) {
    case kSimRecordStateCommandOverwrite:
      SaveStateToFile(GetSavedStateFilename(), state_);
      break;

    case kSimRecordStateCommandLoad:
      LoadStateFromFile(GetSavedStateFilename(), &state_);
      break;

    default:  // No recording action.
      break;
  }
}

// Return the path to the file in to which that state should be saved.
//
// Returns:
//   A string for the hard-coded location that the controller state should be
//   saved.
std::string ControlMain::GetSavedStateFilename() const {
  std::string makani_home(getenv("MAKANI_HOME"));
  return makani_home + "/logs/controller_" +
         std::to_string(static_cast<int>(controller_label_)) +
         "_state_list.bin";
}

}  // namespace control

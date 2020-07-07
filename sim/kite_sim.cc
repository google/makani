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

#include "sim/kite_sim.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <signal.h>

#include <string>
#include <vector>

#include "avionics/common/pack_tether_message.h"
#include "avionics/common/tether_convert.h"
#include "avionics/common/tether_message.h"
#include "avionics/common/tether_message_types.h"
#include "common/backtrace.h"
#include "common/runfiles_dir.h"
#include "control/system_params.h"
#include "lib/json_load/load_params.h"
#include "sim/math/ode_solver.h"
#include "sim/math/ode_solver_gsl.h"
#include "sim/math/ode_solver_odeint.h"
#include "sim/math/util.h"
#include "sim/models/full_system.h"
#include "sim/models/sensors/sensor.h"
#include "sim/sim_save_states.h"

DEFINE_bool(async, false, "Run the simulation in asynchronous mode.");
DEFINE_double(time, NAN, "Run the simulation for a specified time.");
DEFINE_bool(load_state, false, "Load saved state.");
DEFINE_double(save_state_time, NAN, "Time at which to save the state.");
DEFINE_bool(quiet, false, "Avoid printing certain diagnostic information.");
DEFINE_bool(realtime, false,
            "Run the simulation in realtime (ignored when using AIO).");
DEFINE_int32(num_controllers, 1, "Number of controllers to expect.");
DEFINE_bool(gs_estimator, true,
            "Whether to run with a ground station estimator.");

namespace {

// Checks that the number of controllers is valid.
bool ValidateNumControllers(const char *flag_name, int32_t value) {
  if (value == 1 || value == 3) return true;
  LOG(ERROR) << "--" << flag_name << " must be 1 or 3.";
  return false;
}

bool dummy __attribute__((unused)) = google::RegisterFlagValidator(
    &FLAGS_num_controllers, &ValidateNumControllers);

}  // namespace

#if !defined(MAKANI_TEST)

int main(int argc, char *argv[]) {
  InstallBacktraceHandler({SIGABRT, SIGSEGV, SIGFPE});

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Log warnings to stderr.
  FLAGS_stderrthreshold = google::WARNING;

  // Set the source root to the runfiles directory to get access to database
  // files downloaded from GCE.
  SetRunfilesDirFromBinaryPath(argv[0]);

  // Load runtime parameters.
  json_load::LoadSystemParams(GetSystemParamsUnsafe());
  json_load::LoadSimParams(GetSimParamsUnsafe());

  // Handle command line flags.  Note that default values derived from
  // system and simulator parameters must come after the loading of
  // parameters above.
  double sim_time;
  if (isnan(FLAGS_time)) {
    sim_time = GetSimParams()->sim_time;
  } else {
    sim_time = FLAGS_time;
  }

  // Setup simulator communications.
  int32_t retval;
  if (FLAGS_async) {
    GetSystemParamsUnsafe()->hitl.config.sim_level =
        static_cast<int32_t>(kSimulatorHitlLevelAsync);

    // TODO: Evaluate the HITL simulator timing.
    sim::SimAsyncComms async_control_comms(FLAGS_num_controllers,
                                           GetSystemParams()->hitl.config);
    sim::KiteSim s(sim_time, *GetSystemParams(), *GetSimParams(),
                   &async_control_comms);
    async_control_comms.StartUpdateTimer();

    retval = s.RunAio();
  } else {
    // Override the sim_level and use_software_joystick, and check that we
    // aren't trying to put anything in the loop.
    HitlConfiguration &hitl_config = GetSystemParamsUnsafe()->hitl.config;
    hitl_config.sim_level = static_cast<int32_t>(kSimulatorHitlLevelSync);
    hitl_config.use_software_joystick = true;

    for (int32_t i = 0; i < kNumServos; ++i) {
      CHECK_EQ(kActuatorHitlLevelSimulated,
               static_cast<ActuatorHitlLevel>(hitl_config.servo_levels[i]))
          << "HITL servos may not be used during synchronous simulation.";
    }

    std::vector<MessageType> subscribe_types = {
        kMessageTypeControllerCommand,     kMessageTypeControllerSync,
        kMessageTypeControlDebug,          kMessageTypeJoystickStatus,
        kMessageTypeSmallControlTelemetry, kMessageTypeSimCommand};
    AioSetup(kAioNodeSimulator, GetSystemParams()->comms.aio_port,
             subscribe_types.data(),
             static_cast<int32_t>(subscribe_types.size()));

    // Setup the communications to the controller to behave synchronously.
    //
    // Perform handshakes after SimBase is constructed. Doing so has the
    // additional effect of clearing the AIO socket's buffer, which may have
    // filled up during construction of SimBase.
    sim::SimSyncComms sync_control_comms(FLAGS_num_controllers);
    sim::KiteSim s(sim_time, *GetSystemParams(), *GetSimParams(),
                   &sync_control_comms);
    sync_control_comms.WaitForControllers();
    if (FLAGS_gs_estimator) {
      sync_control_comms.WaitForGroundStationEstimator();
    }

    retval = s.Run();
    AioClose();
  }

  return retval;
}

#endif  // !defined(MAKANI_TEST)

namespace sim {

// Initialize variables before a simulation is run.
void KiteSim::InitRun() {
  SimBase::InitRun();

  sensor_message_ = SimSensorMessage();

  sensor_message_.hitl_config = system_params_.hitl.config;
  avionics_packets_.command_message = ControllerCommandMessage();
  // Initialize TetherUpMessages to invalidate no_update_counts.
  TetherUpInit(&tether_up_);
  for (int32_t i = 0; i < kNumTetherUpSources; ++i) {
    TetherUpInit(&sensor_message_.control_input_messages.tether_up_messages[i]);
  }
}

// Run the simulator for a single time step.
//
// Returns:
//   OdeSolverStatus::kSuccess if the step successfully integrates,
//   otherwise returns the error code from the integrator.
OdeSolverStatus KiteSim::RunStep() {
  if (!FLAGS_async && comms_) {
    HandleSimCommand(&sim_command_);
    // Pass command message on to simulated controller.
    comms_->SendSimCommand(sim_command_);
  }

  // Update the DiscreteState objects.
  system_model_->DiscreteUpdate(t());

  PublishMessages();

  // Integrate.
  int64_t integration_start_usec = ClockGetUs();
  OdeSolverStatus status =
      ode_solver_->Integrate(t(), t() + ts_, state_vec_, nullptr, &state_vec_);
  sim_telem.integration_usec = ClockGetUs() - integration_start_usec;
  if (status != OdeSolverStatus::kSuccess) return status;
  ++sim_time_step_;

  // Set model states and derived values.
  dynamic_system_->UpdateFromStateVector(t(), state_vec_);

  // Wait for real-time.
  if (FLAGS_realtime) {
    SleepForRealtime();
  }

  ReceiveMessages();

  return OdeSolverStatus::kSuccess;
}

void KiteSim::UpdateAndSendSimTetherDownMessage(
    const SimTetherDownUpdateState &update_state) {
  TetherDownMessage &tether_down =
      sim_tether_down_.messages[kTetherDownSourceCsA];
  tether_down.frame_index = static_cast<uint16_t>(
      (tether_down.frame_index + 1U) % TETHER_FRAME_INDEX_ROLLOVER);
  tether_down_sequence_ = static_cast<uint16_t>((tether_down_sequence_ + 1U) %
                                                TETHER_SEQUENCE_ROLLOVER);
  if (update_state.small_control_telemetry_updated) {
    tether_down.control_telemetry = update_state.small_control_telemetry;
    tether_down.control_telemetry.sequence = tether_down_sequence_;
    tether_down.control_telemetry.no_update_count = 0;
  }

  ControllerCommandMessageToTetherControlCommand(
      kControllerA, &avionics_packets_.command_message, tether_down_sequence_,
      &tether_down.control_command);

  // time_of_week is required by MergeTetherDown. Rather than use real ms since
  // Sunday morning, we start at 0 (see constructor) and increment by 10 ms per
  // cycle, based on a 100 Hz transmission rate, modulo ms-per-week.
  tether_down.gps_time.time_of_week =
      (tether_down.gps_time.time_of_week + 10) % (7 * 24 * 3600 * 1000);
  tether_down.gps_time.no_update_count = 0;

  for (int32_t i = 0; i < kNumMotors; ++i) {
    MotorStatusMessageUpdateTetherMotorStatus(
        &sensor_message_.control_input_messages.motor_statuses[i],
        &tether_down.node_status, &tether_down.motor_statuses[i]);
  }

  for (int32_t i = 0; i < kNumServos; ++i) {
    ServoStatusMessageUpdateTetherServoStatus(
        &sensor_message_.control_input_messages.servo_statuses[i],
        &tether_down.node_status, &tether_down.servo_statuses[i]);
  }

  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    NovAtelSolutionMessageToTetherGpsStatus(
        &sensor_message_.control_input_messages.wing_gps_novatel[i],
        tether_down_sequence_, &tether_down.gps_statuses[i]);
  }

  tether_down.batt_a.no_update_count = 0;
  tether_down.batt_b.no_update_count = 0;

  tether_down.batt_a.lv_b =
      sensor_message_.control_input_messages.servo_statuses[0]
          .servo_mon.analog_data[kServoAnalogVoltageLvB];  // Big/A Primary
  tether_down.batt_a.lv_a =
      sensor_message_.control_input_messages.servo_statuses[0]
          .servo_mon.analog_data[kServoAnalogVoltageLvA];  // Big/A Secondary
  tether_down.batt_b.lv_a =
      sensor_message_.control_input_messages.servo_statuses[0]
          .servo_mon.analog_data[kServoAnalogVoltageLvA];  // Small/B Primary
  tether_down.batt_b.lv_b =
      sensor_message_.control_input_messages.servo_statuses[0]
          .servo_mon.analog_data[kServoAnalogVoltageLvB];  // Small/B Secondary

  for (int32_t i = 0; i < kNumLoadcellNodes; ++i) {
    LoadcellMessageToTetherReleaseStatus(
        &sensor_message_.control_input_messages.loadcell_messages[i],
        &tether_down.node_status, &tether_down.release_statuses[i]);
  }

  sim_tether_down_.updated[kTetherDownSourceCsA] = true;
  sim_tether_down_.messages[kTetherDownSourceCsB] = tether_down;
  sim_tether_down_.updated[kTetherDownSourceCsB] = true;

  // A message is sent for CsGsA every TETHER_RADIO_DECIMATION cycles to mimic
  // the true send rate. We pack and then unpack it to quantize the contents.
  if (tether_down.frame_index % TETHER_RADIO_DECIMATION == 0) {
    TetherDownPackedMessage tether_down_packed;
    PackTetherDown(nullptr, &tether_down, &tether_down_packed);
    UnpackTetherDown(nullptr, &tether_down_packed,
                     &sim_tether_down_.messages[kTetherDownSourceCsGsA]);
    sim_tether_down_.updated[kTetherDownSourceCsGsA] = true;
  } else {
    sim_tether_down_.updated[kTetherDownSourceCsGsA] = false;
  }

  comms_->SendSimTetherDownMessage(sim_tether_down_);
}

bool KiteSim::UpdateTetherUpMessage(uint16_t sequence,
                                    TetherUpMessage *tether_up) const {
  const ControlInputMessages &messages = sensor_message_.control_input_messages;
  const ControlInputMessagesUpdated &updated =
      sensor_message_.control_input_messages_updated;

  tether_up->frame_index = static_cast<uint16_t>((tether_up->frame_index + 1) %
                                                 TETHER_FRAME_INDEX_ROLLOVER);

  if (updated.joystick) {
    JoystickStatusMessageToTetherJoystick(&messages.joystick, sequence,
                                          &tether_up->joystick);
  }
  return true;
}

bool KiteSim::UpdateRadioTetherUpMessage(TetherUpMessage *in,
                                         TetherUpMessage *out) const {
  // TetherUpMessage is sent every TETHER_RADIO_DECIMATION cycles to mimic CsA's
  // send rate. We pack and then unpack it to quantize the contents.
  if (in->frame_index % TETHER_RADIO_DECIMATION == 0) {
    TetherUpPackedMessage tether_up_packed;
    PackTetherUp(nullptr, in, &tether_up_packed);
    UnpackTetherUp(nullptr, &tether_up_packed, out);
    return true;
  }
  return false;
}

// Update the sensor message to be sent to controllers.
//
// Determines which sensors have been updated since the last simulation
// iteration and updates sensor_message_ accordingly.
void KiteSim::UpdateSensorMessage() {
  if (t() < sim_params_.sensor_blackout_duration) return;

  for (const std::unique_ptr<Sensor> &sensor : system_model_->sensors()) {
    if (sensor->UpdatedSince(t() - ts_ / 2.0)) {
      sensor->UpdateSensorOutputs(&sensor_message_, &tether_up_);
    }
  }

  // Populate TetherUpMessage. Ground core switches CsGsA and CsGsB compile
  // the TetherUpMessage using messages from the ground avionics nodes. They
  // send this message over the network at the full rate. In addition, CsGsA
  // sends a packed version of the same message to CsA at a reduced rate. CsA
  // then unpacks the message and transmits the unpacked version.
  ControlInputMessages &messages = sensor_message_.control_input_messages;
  ControlInputMessagesUpdated &updated =
      sensor_message_.control_input_messages_updated;
  tether_up_sequence_ = static_cast<uint16_t>((tether_up_sequence_ + 1) %
                                              TETHER_SEQUENCE_ROLLOVER);

  UpdateTetherUpMessage(tether_up_sequence_, &tether_up_);

  updated.tether_up_messages[kTetherUpSourceCsGsA] = true;
  messages.tether_up_messages[kTetherUpSourceCsGsA] = tether_up_;
  updated.tether_up_messages[kTetherUpSourceCsGsB] = true;
  messages.tether_up_messages[kTetherUpSourceCsGsB] = tether_up_;

  updated.tether_up_messages[kTetherUpSourceCsA] = UpdateRadioTetherUpMessage(
      &tether_up_, &messages.tether_up_messages[kTetherUpSourceCsA]);
}

// Receive a simulation command.
//
// Receives UDP packets from external interfaces giving instructions to the
// simulation.  This is currently only used for triggering the simulator
// to save its state.
//
// Args:
//   t: Current simulation time.
//   sim_command: Output packet that was received.
void KiteSim::HandleSimCommand(SimCommandMessage *sim_command) {
  // Reset the record_mode in case no packet is received.
  sim_command->record_mode = kSimRecordStateCommandDont;

  DCHECK(comms_);
  comms_->ReceiveSimCommand(sim_command);

  if (t() >= FLAGS_save_state_time && !saved_to_file_) {
    sim_command->record_mode = kSimRecordStateCommandOverwrite;
    saved_to_file_ = true;
  }

  if (sim_command->record_mode == kSimRecordStateCommandOverwrite) {
    CHECK_EQ(0, SaveStatesToFile(GetSavedStateFilename(), system_model_.get(),
                                 sim_time_step_));
  } else {
    sim_command->record_mode = kSimRecordStateCommandDont;
  }

  sim_command->stop = false;

  if (FLAGS_load_state && !loaded_from_file_) {
    CHECK_EQ(0, LoadStatesFromFile(GetSavedStateFilename(), system_model_.get(),
                                   &sim_time_step_));
    ResetStateVector();
    sim_command->record_mode = kSimRecordStateCommandLoad;
    loaded_from_file_ = true;
  }
}

// Occasionally sleeps to make the simulation time approximate realtime.
//
// Should be called once every simulation loop.  Every kNumSkipSleep calls,
// compares the wall-clock time elapsed to the simulator time elapsed and
// sleeps the difference.
//
// Args:
//   t_sim: Current simulator time.
void KiteSim::SleepForRealtime() {
  struct timeval curr_time;
  uint32_t sleep_time_usec, dt_sec, dt_usec;
  double dt_real, t_real;

  // Do every kNumSkipSleeps times.
  if (skip_sleep_counter_ == 0U) {
    gettimeofday(&curr_time, nullptr);

    dt_real = static_cast<double>(curr_time.tv_sec - last_time_.tv_sec) +
              static_cast<double>(curr_time.tv_usec - last_time_.tv_usec) / 1e6;
    t_real = static_cast<double>(curr_time.tv_sec - start_time_.tv_sec) +
             static_cast<double>(curr_time.tv_usec - start_time_.tv_usec) / 1e6;

    if (!FLAGS_quiet) {
      printf("\n t_real = %f, t_sim = %f, dt_real/dt_sim = %f", t_real, t(),
             dt_real / (kNumSkipSleeps * ts_));
    }

    dt_sec = static_cast<uint32_t>(curr_time.tv_sec - last_time_.tv_sec);
    dt_usec = static_cast<uint32_t>(curr_time.tv_usec - last_time_.tv_usec);
    sleep_time_usec = static_cast<uint32_t>(fmax(
        1000000.0 * kNumSkipSleeps * ts_ - (1000000 * dt_sec + dt_usec), 0.0));
    usleep(sleep_time_usec);
    gettimeofday(&last_time_, nullptr);
  }
  skip_sleep_counter_ = (skip_sleep_counter_ + 1U) % kNumSkipSleeps;
}

// Return the path to the file in to which that state should be saved.
//
// Returns:
//   A string for the hard-coded location that the simulator state should be
//   saved.
std::string KiteSim::GetSavedStateFilename() const {
  std::string makani_home(getenv("MAKANI_HOME"));
  return makani_home + "/logs/sim_state_list.json";
}

KiteSim::KiteSim(double sim_time, const SystemParams &system_params,
                 const SimParams &sim_params, SimCommsInterface *comms)
    : SimBase(sim_time, system_params, sim_params),
      comms_(CHECK_NOTNULL(comms)),
      sim_tether_down_(),
      avionics_packets_(),
      sensor_message_(),
      lead_controller_(kControllerA),
      sim_command_(),
      skip_sleep_counter_(0U),
      tether_up_(),
      tether_up_sequence_(0U),
      tether_down_sequence_(0U),
      saved_to_file_(false),
      loaded_from_file_(false) {
  system_model_.reset(new FullSystem(system_params_, sim_params_, &faults_,
                                     avionics_packets_, &sensor_message_));
  CHECK(faults_.AllFaultsAreClaimed())
      << "The faults schedule contains unsupported faults.";

  dynamic_system_.reset(new SimDynamicSystem(system_model_.get()));

  avionics_packets_.command_message = ControllerCommandMessage();

  if (system_params_.hitl.config.motor_level == kActuatorHitlLevelReal) {
    avionics_packets_.motor_statuses.assign(kNumMotors, MotorStatusMessage());
  }

  for (int32_t i = 0; i < kNumServos; ++i) {
    if (system_params_.hitl.config.servo_levels[i] ==
        static_cast<int32_t>(kActuatorHitlLevelReal)) {
      // This creates the map entry.
      avionics_packets_.servo_statuses[static_cast<ServoLabel>(i)] =
          ServoDebugMessage();
    }
  }

  if (system_params_.hitl.config.tether_release_level ==
      kActuatorHitlLevelReal) {
    avionics_packets_.loadcell_messages.assign(kNumLoadcellNodes,
                                               LoadcellMessage());
  }

  InitStateVector();
  InitOdeDriver();
  InitRealtime();
  InitRandom();

  for (int32_t i = 0; i < kNumTetherDownSources; ++i) {
    TetherDownInit(&sim_tether_down_.messages[i]);
    sim_tether_down_.messages[i].gps_time.time_of_week = 0;
    sim_tether_down_.updated[i] = false;
  }
}

void KiteSim::PublishMessages() {
  if (system_params_.hitl.config.send_dyno_commands) {
    comms_->SendDynoCommand(
        dynamic_cast<FullSystem *>(system_model_.get())->dyno_command_packet());
  }

  // Publish telemetry.  If the sample period is smaller than ts_,
  // publish every step.  The correction by half a time step guards
  // against rounding errors.
  if (t() - last_publish_time_ >
          sim_params_.telemetry_sample_period - ts_ / 2.0 ||
      ts_ > sim_params_.telemetry_sample_period) {
    system_model_->Publish();
    sim_telem.time = t();
    comms_->SendSimTelemetry(sim_telem);
    last_publish_time_ = t();
  }

  // Sim -> Sensors.
  UpdateSensorMessage();

  // Sensors -> Controller.
  comms_->SendControllerInput(sensor_message_);

  // Clear updated flags for the next iteration. This can't happen in
  // UpdateSensorMessage or it will clobber the update flags for
  // ControllerSyncMessages.
  memset(&sensor_message_.control_input_messages_updated, 0,
         sizeof(sensor_message_.control_input_messages_updated));
  memset(&sensor_message_.ground_input_messages_updated, 0,
         sizeof(sensor_message_.ground_input_messages_updated));
}

void KiteSim::ReceiveMessages() {
  // Receive incoming messages.
  comms_->ReceiveControlTelemetry();
  SimTetherDownUpdateState tether_down_update_state;
  comms_->ReceiveAvionicsPackets(&avionics_packets_, &lead_controller_,
                                 &tether_down_update_state);
  comms_->ReceiveControllerSync(&sensor_message_);

  UpdateAndSendSimTetherDownMessage(tether_down_update_state);
}

void KiteSim::FinishRun() {
  // The final sensor message will be ignored, but the controller must receive
  // it in order to process the stop command.
  comms_->SendControllerInput(sensor_message_);
  sim_command_.stop = true;
  comms_->SendSimCommand(sim_command_);
}

}  // namespace sim

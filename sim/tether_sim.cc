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

#include "sim/tether_sim.h"

#include <gflags/gflags.h>
#include <string>
#include <vector>

#include "common/runfiles_dir.h"
#include "lib/json_load/load_params.h"

DEFINE_string(output_file, "", "The output HDF5 log file.");
DEFINE_double(time, NAN, "Log time at which the replay stops.");

namespace sim {

TetherSim::TetherSim(FlightLogInterpolator *log_interpolator, double sim_time,
                     const SystemParams &system_params,
                     const SimParams &sim_params)
    : SimBase(sim_time, system_params, sim_params),
      message_log_(FLAGS_output_file) {
  system_model_.reset(new TetherSimulatorSystem(
      log_interpolator, system_params_, sim_params_, &faults_));
  dynamic_system_.reset(new SimDynamicSystem(system_model_.get()));

  sim_time_step_ =
      static_cast<uint64_t>(log_interpolator->EarliestTime() / ts_);

  InitStateVector();
  InitOdeDriver();
  InitRealtime();
  InitRandom();
}

void TetherSim::PublishMessages() {
  // TODO: Make it a utility function.
  // Publish telemetry.
  system_model_->Publish();
  sim_telem.time = t();
  // Pack SimTelemetry and write to HDF5 log.
  static uint8_t buffer[PACK_SIMTELEMETRY_SIZE];
  CHECK_EQ(PACK_SIMTELEMETRY_SIZE, PackSimTelemetry(&sim_telem, 1U, buffer));
  message_log_.WriteWithSpoofedHeaders(kAioNodeSimulator,
                                       kMessageTypeSimTelemetry, ClockGetUs(),
                                       buffer, PACK_SIMTELEMETRY_SIZE);
  last_publish_time_ = t();
}

// Run the simulator for a single time step.
//
// Returns:
//   OdeSolverStatus::kSuccess if the step successfully integrates,
//   otherwise returns the error code from the integrator.
OdeSolverStatus TetherSim::RunStep() {
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

  ReceiveMessages();

  return OdeSolverStatus::kSuccess;
}

}  // namespace sim

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc < 2 || FLAGS_output_file.empty()) {
    LOG(FATAL) << "Usage: " << argv[0] << " <log_file_1.h5> [log_file_2.h5]"
               << " [log_file_3.h5] ... --output_file <output_file.h5>";
  }

  std::vector<std::string> input_logs;
  for (int32_t i = 1; i < argc; ++i) {
    CHECK_NE(access(argv[i], F_OK), -1) << "Could not find input file "
                                        << argv[i] << ".";
    input_logs.push_back(argv[i]);
  }

  // Log warnings to stderr.
  FLAGS_stderrthreshold = google::WARNING;

  // Set the source root to the runfiles directory to get access to database
  // files downloaded from GCE.
  SetRunfilesDirFromBinaryPath(argv[0]);

  // Load runtime parameters.
  json_load::LoadSystemParams(GetSystemParamsUnsafe());
  json_load::LoadSimParams(GetSimParamsUnsafe());

  // Setup simulator.
  FlightLogInterpolator log_interpolator(input_logs);
  double sim_time = FLAGS_time;
  if (isnan(sim_time)) {
    sim_time = log_interpolator.LatestTime();
  }
  sim::TetherSim s(&log_interpolator, sim_time, *GetSystemParams(),
                   *GetSimParams());

  return s.Run();
}

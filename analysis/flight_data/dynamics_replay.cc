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

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <string>

#include "analysis/control/replay/estimator_replay.h"
#include "analysis/flight_data/dynamics_sim.h"
#include "avionics/linux/clock.h"
#include "common/runfiles_dir.h"
#include "control/common.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "lib/hdf5_to_pcap/h5log_reader.h"
#include "lib/pcap_to_hdf5/message_log.h"

DEFINE_string(output_file, "", "Path to the output .h5 file.");
DEFINE_bool(output_sim_telemetry, false,
            "Whether to include SimTelemetry in the output file. "
            "Note that telemetry for models unused by dynamics_replay "
            "will not be populated.");
DEFINE_bool(reprocess_estimator, false,
            "Reprocess the state estimate with the current estimator version");

namespace {

void ReadLogData(const char *filename,
                 std::vector<ControlTelemetry> *control_telemetry,
                 SystemParams *system_params) {
  h5log_reader::File file;
  h5log_reader::Dataset dataset;
  CHECK(file.Open(filename));

  CHECK(dataset.ReadData(
      file, "/messages/kAioNodeControllerA/kMessageTypeControlDebug"));
  *control_telemetry = dataset.Export<ControlDebugMessage>();

  CHECK(dataset.ReadData(file, "/parameters/system_params"));
  std::vector<SystemParams> params_vector = dataset.Export<SystemParams>();
  CHECK_EQ(params_vector.size(), 1U);
  *system_params = params_vector[0];
}

}  // namespace

// DynamicsReplaySystem is a system model used to run the simulator's dynamics
// models using input states drawn from a flight log.
//
// Sample usage:
//     ./dynamics_replay log_1.h5 log_2.h5 --output_file output.h5
// Then output.h5 will contain a SimTelemetry timeseries that is parallel
// to the ControlDebugMessage timeseries from the concatenation of the input
// files. The forces and moments in WingTelemetry are the primary outputs of
// interest.
//
// This system is used as a log processing utility. Roughly,
//   for t in sample_times:
//     inject ControlTelemetry at time t into system
//     run system connections
//     publish SimTelemetry
class DynamicsReplaySystem : public DynamicsSystem {
 public:
  DynamicsReplaySystem(const SystemParams &system_params,
                       const SimParams &sim_params, FaultSchedule *faults);
  virtual ~DynamicsReplaySystem() {}

  void set_control_telemetry(const ControlDebugMessage &in) {
    ControlDebugMessage out(in);
    if (FLAGS_reprocess_estimator) {
      EstimatorReplayIterate(&GetControlParams()->estimator, &in, &flight_mode_,
                             &estimator_state_, &out.state_est);
    }
    control_telemetry_.set_val(out);
  }

  const StateEstimate &state_est() {
    return control_telemetry_.val().state_est;
  }
  const ControlInput &control_input() {
    return control_telemetry_.val().control_input;
  }

 private:
  Vec3 Xg() override { return state_est().Xg; }
  Vec3 Vb() override { return state_est().Vb; }
  Vec3 pqr() override { return state_est().pqr_f; }
  Mat3 dcm_g2b() override { return state_est().dcm_g2b; }
  Vec3 wind_g() override {
    Vec3 wind_b, wind_g;
    Vec3 apparent_wind_f;
    ApparentWindSphToCart(&state_est().apparent_wind.sph_f, &apparent_wind_f);

    Vec3Add(&apparent_wind_f, &state_est().Vb, &wind_b);
    Mat3TransVec3Mult(&state_est().dcm_g2b, &wind_b, &wind_g);
    return wind_g;
  }
  Vec3 tether_force_b() override { return state_est().tether_force_b.vector; }
  Vec3 bridle_moment_b() override { return kVec3Zero; }
  Vec3 knot_pos_b() override {
    const WingParams &wing_params = system_params_.wing;
    const Vec3 &port_pos_b = wing_params.bridle_pos[kBridlePort];
    const Vec3 &star_pos_b = wing_params.bridle_pos[kBridleStar];

    // Define a temporary "bridle" frame as a child of the body frame. Its
    // origin is at the bridle pivot, and it is rotated about the body y-axis
    // by the tether pitch angle. The bridle knot's position in this frame
    // is {0.0, 0.0, bridle_rad}.
    Vec3 bridle_origin_pos_b = {(port_pos_b.x + star_pos_b.x) / 2.0,
                                wing_params.bridle_y_offset,
                                (port_pos_b.z + star_pos_b.z) / 2.0};
    Mat3 dcm_b2bridle;
    AngleToDcm(0.0, state_est().tether_force_b.sph.pitch, 0.0,
               kRotationOrderZyx, &dcm_b2bridle);

    Vec3 knot_pos_bridle = {0.0, 0.0, wing_params.bridle_rad};
    Vec3 knot_pos_b;
    InversePoseTransform(&dcm_b2bridle, &bridle_origin_pos_b, &knot_pos_bridle,
                         &knot_pos_b);
    return knot_pos_b;
  }
  const double *flaps() override { return control_input().flaps; }
  const double *rotors() override { return control_input().rotors; }

 private:
  EstimatorState estimator_state_;

  State<ControlDebugMessage> control_telemetry_;

  DISALLOW_COPY_AND_ASSIGN(DynamicsReplaySystem);
};

DynamicsReplaySystem::DynamicsReplaySystem(const SystemParams &system_params,
                                           const SimParams &sim_params,
                                           FaultSchedule *faults)
    : DynamicsSystem("DynamicsReplaySystem", system_params, sim_params, faults),
      estimator_state_(),
      control_telemetry_(new_derived_value(), "control_telemetry") {
  // Initialize the estimator.
  EstimatorReplayInit(&GetControlParams()->estimator, &flight_mode_,
                      &estimator_state_);
}

// Populates the ouptut DynamicReplayMessage.
DynamicsReplayMessage GenerateOutput(
    const SystemParams &system_params,
    const ControlDebugMessage &control_telemetry,
    const SimTelemetry &sim_telemetry, Vec3 *pqr_f_z1) {
  Vec3 omega_b_dot;
  {
    // Estimate b-frame angular acceleration.
    const Vec3 &omega_b = control_telemetry.state_est.pqr_f;
    Vec3 tmp;
    Vec3Sub(&omega_b, pqr_f_z1, &tmp);
    Vec3Scale(&tmp, 1.0 / system_params.ts, &omega_b_dot);
    *pqr_f_z1 = omega_b;
  }

  DynamicsReplayMessage out = GenerateDynamics(
      system_params, sim_telemetry, control_telemetry.state_est.Ab_f,
      control_telemetry.state_est.pqr_f, omega_b_dot);

  out.time = control_telemetry.time;
  out.flight_mode = control_telemetry.flight_mode;

  // Store WingParams, ControlInput, and StateEstimate for convenience.
  out.wing_params = system_params.wing;
  out.control_input = control_telemetry.control_input;
  out.state_est = control_telemetry.state_est;

  return out;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (argc < 2 || FLAGS_output_file.empty()) {
    LOG(FATAL) << "Usage: " << argv[0] << " <log_file_1.h5> [log_file_2.h5]"
               << " [log_file_3.h5] ... --output_file <output_file.h5>";
  }
  for (int32_t i = 1; i < argc; ++i) {
    CHECK_NE(access(argv[i], F_OK), -1) << "Could not find input file "
                                        << argv[i] << ".";
  }

  // Delete the output file if it already exists.
  if (access(FLAGS_output_file.c_str(), F_OK) != -1) {
    unlink(FLAGS_output_file.c_str());
  }

  // Set the source root to the runfiles directory to get access to database
  // files downloaded from Cloud Storage.
  SetRunfilesDirFromBinaryPath(argv[0]);

  FaultSchedule fault_schedule(nullptr);
  DynamicsReplaySystem system(*GetSystemParams(), *GetSimParams(),
                              &fault_schedule);

  ::lib::pcap_to_hdf5::MessageLog message_log(FLAGS_output_file);

  // pqr at the previous time step is required in order to estimate angular
  // acceleration.
  Vec3 pqr_f_z1;

  for (int32_t i = 1; i < argc; ++i) {
    const char *filename = argv[i];
    std::vector<ControlDebugMessage> messages;
    SystemParams system_params;
    ReadLogData(filename, &messages, &system_params);

    // Initialize pqr_f_z1 on the first dataset of the first file.
    if (i == 1) {
      pqr_f_z1 = messages[0].state_est.pqr_f;
    }

    // If the control system is not running, state estimates may contain
    // invalid data (e.g. inappropriate DCMs) that will crash our program.
    for (const ControlDebugMessage &control_telemetry : messages) {
      if (!IsControlSystemRunning(
              static_cast<InitializationState>(control_telemetry.init_state))) {
        continue;
      }

      system.ClearIntegrationState();

      const double t = control_telemetry.time;
      system.set_control_telemetry(control_telemetry);
      system.RunConnections(t);
      system.CalcDeriv(t);
      system.Publish();  // Populates global sim_telem.

      DynamicsReplayMessage output = GenerateOutput(
          system_params, control_telemetry, sim_telem, &pqr_f_z1);

      {  // Pack DynamicsReplayMessage and write to HDF5 log.
        static uint8_t buffer[PACK_DYNAMICSREPLAYMESSAGE_SIZE];
        CHECK_EQ(PACK_DYNAMICSREPLAYMESSAGE_SIZE,
                 PackDynamicsReplayMessage(&output, 1U, buffer));
        message_log.WriteWithSpoofedHeaders(
            kAioNodeSimulator, kMessageTypeDynamicsReplay, ClockGetUs(), buffer,
            PACK_DYNAMICSREPLAYMESSAGE_SIZE);
      }

      // Write SimTelemetry to log if requested.
      if (FLAGS_output_sim_telemetry) {
        sim_telem.time = control_telemetry.time;
        static uint8_t buffer[PACK_SIMTELEMETRY_SIZE];
        CHECK_EQ(PACK_SIMTELEMETRY_SIZE,
                 PackSimTelemetry(&sim_telem, 1U, buffer));
        message_log.WriteWithSpoofedHeaders(
            kAioNodeSimulator, kMessageTypeSimTelemetry, ClockGetUs(), buffer,
            PACK_SIMTELEMETRY_SIZE);
      }
    }
  }
}

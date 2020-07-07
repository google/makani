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
#include <unistd.h>

#include <string>

#include "analysis/control/replay/estimator_replay.h"
#include "avionics/linux/clock.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/hover/hover_angles.h"
#include "lib/hdf5_to_pcap/h5log_reader.h"
#include "lib/pcap_to_hdf5/message_log.h"
#include "sim/pack_sim_messages.h"
#include "sim/sim_messages.h"

DEFINE_string(output_file, "", "Path to the output .h5 file.");

std::vector<ControlDebugMessage> ReadLogData(const std::string &filename) {
  h5log_reader::File file;
  h5log_reader::Dataset dataset;
  CHECK(file.Open(filename));
  CHECK(dataset.ReadData(
      file, "/messages/kAioNodeControllerA/kMessageTypeControlDebug"));
  return dataset.Export<ControlDebugMessage>();
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

  ::lib::pcap_to_hdf5::MessageLog message_log(FLAGS_output_file);

  // Initialize.
  const EstimatorParams &params = GetControlParams()->estimator;
  FlightMode flight_mode;
  EstimatorState estimator_state;
  EstimatorReplayInit(&params, &flight_mode, &estimator_state);

  for (int32_t i = 1; i < argc; ++i) {
    const std::string filename = argv[i];
    const std::vector<ControlDebugMessage> messages = ReadLogData(filename);
    for (const ControlDebugMessage &input : messages) {
      EstimatorReplayMessage output;
      memset(&output, 0, sizeof(output));

      // Iterate.
      EstimatorReplayIterate(&params, &input, &flight_mode, &estimator_state,
                             &output.state_est);

      // Construct output.
      output.time = input.time;
      output.control_input = input.control_input;
      output.flight_mode = flight_mode;
      output.estimator_telemetry = GetControlTelemetry()->estimator;
      output.estimator_state = estimator_state;

      HoverAnglesGetAngles(&output.state_est.Xg, &output.state_est.vessel.pos_g,
                           &output.state_est.dcm_g2b, &output.hover_angles);

      // Pack EstimatorReplayMessage and write to HDF5 log.
      static uint8_t buffer[PACK_ESTIMATORREPLAYMESSAGE_SIZE];
      CHECK_EQ(PACK_ESTIMATORREPLAYMESSAGE_SIZE,
               PackEstimatorReplayMessage(&output, 1U, buffer));
      message_log.WriteWithSpoofedHeaders(
          kAioNodeSimulator, kMessageTypeEstimatorReplay, ClockGetUs(), buffer,
          PACK_ESTIMATORREPLAYMESSAGE_SIZE);
    }
  }
}

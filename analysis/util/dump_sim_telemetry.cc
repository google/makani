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

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "avionics/network/message_type.h"
#include "control/control_types.h"
#include "control/system_types.h"
#include "sim/pack_sim_telemetry.h"
#include "sim/sim_types.h"
#include "lib/datatools/load_pcap_hdf5_dataset.h"

DEFINE_int32(first_message, 0, "Index of first message to parse.");
DEFINE_int32(num_messages, -1,
             "Number of messages to parse (-1 indicating parse all).");
DEFINE_string(input_file, "", "Input log file.");
DEFINE_string(output_file, "", "Output file.");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Check the flags.
  CHECK(!FLAGS_input_file.empty()) << "Input filename required.";
  CHECK(!FLAGS_output_file.empty()) << "Output filename required.";

  // Print some diagnostic information.
  printf("Using input \"%s\".\n", FLAGS_input_file.c_str());
  printf("sizeof(SystemParams): %zu\n", sizeof(SystemParams));
  printf("sizeof(SimParams): %zu\n", sizeof(SimParams));
  printf("sizeof(ControlParams): %zu\n", sizeof(ControlParams));
  printf("sizeof(SimTelemetry): %zu\n", sizeof(SimTelemetry));

  // Read the three parameter structs.
  printf("Reading SystemParams.\n");
  SystemParams system_params;
  LoadHdf5Parameters(FLAGS_input_file, "system_params", &system_params);

  printf("Reading SimParams.\n");
  SimParams sim_params;
  LoadHdf5Parameters(FLAGS_input_file, "sim_params", &sim_params);

  printf("Reading ControlParams.\n");
  ControlParams control_params;
  LoadHdf5Parameters(FLAGS_input_file, "control_params", &control_params);

  // Open the HDF5 sim telemetry.
  printf("Reading SimTelemetry.\n");
  PcapToHdf5Dataset<SimTelemetry> recorded_messages(
      FLAGS_input_file, AioNodeToString(kAioNodeSimulator),
      MessageTypeToString(kMessageTypeSimTelemetry), FLAGS_first_message,
      FLAGS_first_message + FLAGS_num_messages, PACK_SIMTELEMETRY_SIZE,
      &UnpackSimTelemetry);

  // Open the output file.
  FILE *output_file = fopen(FLAGS_output_file.c_str(), "wb");
  CHECK(output_file != nullptr)
      << "Couldn't open the output file \"" << FLAGS_output_file << "\".";

  // First write the SystemParams, SimParams and ControlParams data.
  fwrite(&system_params, sizeof(system_params), 1, output_file);
  fwrite(&sim_params, sizeof(sim_params), 1, output_file);
  fwrite(&control_params, sizeof(control_params), 1, output_file);

  // Loop through sim telemetry messages and write raw bytes to the file.
  for (uint32_t i = 0U; i < recorded_messages.GetSize(); ++i) {
    SimTelemetry recorded_telemetry;
    recorded_messages.UnpackMessage(i, &recorded_telemetry);
    fwrite(reinterpret_cast<void *>(&recorded_telemetry),
           sizeof(recorded_telemetry), 1, output_file);
  }
  fclose(output_file);

  // Huzzah!
  printf("Wrote %zu messages to %s\n", recorded_messages.GetSize(),
         FLAGS_output_file.c_str());
  return EXIT_SUCCESS;
}

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

#include <gtest/gtest.h>

#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/pack_control_telemetry.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "lib/datatools/load_pcap_hdf5_dataset.h"
#include "lib/util/test_util.h"
#include "sim/sim_params.h"
#include "sim/sim_types.h"

TEST(PcapToHdf5Dataset, LoadEmpty) {
  const std::string empty_log_path(
      test_util::TestRunfilesDir() + "/lib/pcap_to_hdf5/empty_log.h5");
  PcapToHdf5Dataset<ControlTelemetry> recorded_messages(
      empty_log_path,
      AioNodeToString(kAioNodeControllerA),
      MessageTypeToString(kMessageTypeControlTelemetry),
      0, -1, PACK_CONTROLTELEMETRY_SIZE, &UnpackControlTelemetry);
  EXPECT_EQ(1U, recorded_messages.GetSize());
}

// Test hardcoded parameters against ones extracted from the log.
TEST(LoadHdf5Parameters, CompareAgainstHardcoded) {
  const std::string empty_log_path(
      test_util::TestRunfilesDir() + "/lib/pcap_to_hdf5/empty_log.h5");

  SystemParams system_params;
  LoadHdf5Parameters(empty_log_path, "system_params", &system_params);

  SimParams sim_params;
  LoadHdf5Parameters(empty_log_path, "sim_params", &sim_params);

  ControlParams control_params;
  LoadHdf5Parameters(empty_log_path, "control_params", &control_params);

  const uint8_t *log_bytes;
  const uint8_t *hardcoded_bytes;

  log_bytes = reinterpret_cast<uint8_t *>(&system_params);
  hardcoded_bytes = reinterpret_cast<const uint8_t *>(GetSystemParams());
  for (size_t k = 0; k < sizeof(system_params); k++) {
    EXPECT_EQ(hardcoded_bytes[k], log_bytes[k])
        << "Loaded SystemParams mismatch at index " << k << ".";
  }

  log_bytes = reinterpret_cast<uint8_t *>(&sim_params);
  hardcoded_bytes = reinterpret_cast<const uint8_t *>(GetSimParams());
  for (size_t k = 0; k < sizeof(sim_params); k++) {
    EXPECT_EQ(hardcoded_bytes[k], log_bytes[k])
        << "Loaded SimParams mismatch at index " << k << ".";
  }

  log_bytes = reinterpret_cast<uint8_t *>(&control_params);
  hardcoded_bytes = reinterpret_cast<const uint8_t *>(GetControlParams());
  for (size_t k = 0; k < sizeof(control_params); k++) {
    EXPECT_EQ(hardcoded_bytes[k], log_bytes[k])
        << "Loaded ControlParams mismatch at index " << k << ".";
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

// Attempt to zero all loadcells over AIO.
//
// Exits with return code EXIT_SUCCESS if all loadcells most recently
// reported being zeroed.  Otherwise, exits with EXIT_FAILURE.  See -help
// for usage.

#include <stdint.h>
#include <string.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/network_config.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/linux/aio.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"
#include "lib/util/operator_confirmation.h"
#include "system/labels.h"

DEFINE_int32(zero_skip_mask, 0,
             "Integer between 0 and 15 encoding a bit-mask of loadcells to "
             "skip loadcell zeroing.");

namespace {

// Bit-mask for all loadcells.
const uint16_t kAllLoadcellNodesMask = (1U << kNumLoadcellNodes) - 1U;

// Checks that zero_skip_mask has reasonable values.
bool ValidateZeroSkipMask(const char * /*flag_name*/, int32_t value) {
  return 0 <= value && value <= kAllLoadcellNodesMask;
}
bool dummy __attribute__((unused))
    = google::RegisterFlagValidator(&FLAGS_zero_skip_mask,
                                    &ValidateZeroSkipMask);

struct LoadcellCommandProgress {
  // Counter for the number of attempts we've made.
  int32_t num_attempts;

  // Bit-mask of loadcells we've heard from.
  uint16_t loadcells_heard;

  // Bit-mask of loadcells that are set.
  uint16_t loadcells_set;
};

// Number of attempts to make to zero all loadcells.
const int32_t kMaxNumAttempts = 10;

bool AioCallback(void *ptr) {
  LoadcellCommandProgress *progress =
      static_cast<LoadcellCommandProgress *>(ptr);

  for (int32_t i = 0; i < kNumLoadcellNodes; ++i) {
    LoadcellMessage status;
    if (CvtGetLoadcellMessage(
            LoadcellNodeLabelToLoadcellNodeAioNode(
                static_cast<LoadcellNodeLabel>(i)),
            &status, nullptr, nullptr)) {
      const uint16_t loadcell_bit = static_cast<uint16_t>(1 << i);
      progress->loadcells_heard |= loadcell_bit;

      if (status.command == kLoadcellCommandZeroCal) {
        progress->loadcells_set |= loadcell_bit;
      }
    }
  }

  // Terminate as all loadcells are set.
  if ((progress->loadcells_set | static_cast<uint16_t>(FLAGS_zero_skip_mask))
      == kAllLoadcellNodesMask) {
    return false;
  }

  LoadcellCommandMessage command_message;
  memset(&command_message, 0, sizeof(command_message));
  command_message.selected =
      ~progress->loadcells_set & ~FLAGS_zero_skip_mask & kAllLoadcellNodesMask;
  command_message.command = kLoadcellCommandZeroCal;

  AIO_SEND_PACKED(kMessageTypeLoadcellCommand, PackLoadcellCommandMessage,
                  PACK_LOADCELLCOMMANDMESSAGE_SIZE, &command_message);

  ++progress->num_attempts;
  return (progress->num_attempts < kMaxNumAttempts);
}

}  // namespace.

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage(
      "Send a command to all loadcells with retry.  Exactly one flag must be "
      "specified.");

  CHECK_EQ(4, kNumLoadcellNodes);
  LoadcellCommandProgress progress = {0, 0x0000, 0x0000};

  const MessageType kSubscribeMessages[] = {kMessageTypeLoadcell};
  const double ts = 0.1;  // Loop period [s] for AIO callback (10 Hz).
  AioLoopStart(kAioNodeControllerA, UDP_PORT_AIO,
               kSubscribeMessages, ARRAYSIZE(kSubscribeMessages),
               AioCallback, &progress, static_cast<int32_t>(1000000 * ts));
  AioClose();

  if (progress.loadcells_heard != progress.loadcells_set
      || (progress.loadcells_set != static_cast<uint16_t>(
          kAllLoadcellNodesMask & ~FLAGS_zero_skip_mask))) {
    LOG(ERROR) << "Failed: Please check monitors.";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

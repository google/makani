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

// Attempt to cover or uncover the pitot tube.

#include <stdint.h>
#include <string.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/safety_codes.h"
#include "avionics/linux/aio.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"
#include "lib/util/operator_confirmation.h"

DEFINE_bool(cover, false, "Cover pitot.");
DEFINE_bool(uncover, false, "Uncover pitot.");

namespace {

bool AioCallback(void */*ptr*/) {
  PitotSetStateMessage set_state_message;
  memset(&set_state_message, 0, sizeof(set_state_message));

  if (FLAGS_cover) {
    set_state_message.cover = true;
    set_state_message.safety_code = PITOT_COVER_SIGNAL;
  } else if (FLAGS_uncover) {
    set_state_message.cover = false;
    set_state_message.safety_code = PITOT_UNCOVER_SIGNAL;
  }

  AIO_SEND_PACKED(kMessageTypePitotSetState, PackPitotSetStateMessage,
                  PACK_PITOTSETSTATEMESSAGE_SIZE, &set_state_message);
  // TODO: Monitor status of pitot_cover_flag (in
  // FlightComputerSensorMessage) and report success when the move is complete.

  return false;
}

std::string GetActionString() {
  std::string action;
  if (FLAGS_cover) {
    action = "Cover pitot tube.";
  } else if (FLAGS_uncover) {
    action = "Uncover pitot tube.";
  } else {
    assert(false);
  }
  return action;
}

}  // namespace.

int main(int argc, char *argv[]) {
  FLAGS_stderrthreshold = 0;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage("Cover or uncover the pitot tube.");

  if (FLAGS_cover + FLAGS_uncover != 1) {
    LOG(ERROR) << "Must specify exactly one command (cover, uncover).";
    google::ShowUsageWithFlagsRestrict(argv[0], "pitot_command");
    return EXIT_FAILURE;
  }

  CHECK(OperatorConfirmationPrompt(GetActionString()))
      << "The string you entered does not match.";

  int placeholder;
  const double ts = 0.1;  // Loop period [s] for AIO callback (10 Hz).
  AioLoopStart(kAioNodeOperator, UDP_PORT_AIO, NULL, 0, AioCallback,
               &placeholder, static_cast<int32_t>(1000000 * ts));
  AioClose();

  return EXIT_SUCCESS;
}

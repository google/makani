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

// Attempt to enable or disable the FPV camera.

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

DEFINE_bool(enable, false, "Enable FPV.");
DEFINE_bool(disable, false, "Disable FPV.");

namespace {

bool AioCallback(void */*ptr*/) {
  FpvSetStateMessage set_state_message;
  memset(&set_state_message, 0, sizeof(set_state_message));

  if (FLAGS_enable) {
    set_state_message.enable = true;
    set_state_message.safety_code = FPV_ENABLE_SIGNAL;
  } else if (FLAGS_disable) {
    set_state_message.enable = false;
    set_state_message.safety_code = FPV_DISABLE_SIGNAL;
  }

  AIO_SEND_PACKED(kMessageTypeFpvSetState, PackFpvSetStateMessage,
                  PACK_FPVSETSTATEMESSAGE_SIZE, &set_state_message);

  return false;
}

std::string GetActionString() {
  std::string action;
  if (FLAGS_enable) {
    action = "Enable FPV camera";
  } else if (FLAGS_disable) {
    action = "Disable FPV camera";
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
  google::SetUsageMessage("Enable or disable the FPV camera.");

  if (FLAGS_enable + FLAGS_disable != 1) {
    LOG(ERROR) << "Must specify exactly one command (enable, disable).";
    google::ShowUsageWithFlagsRestrict(argv[0], "fpv_command");
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

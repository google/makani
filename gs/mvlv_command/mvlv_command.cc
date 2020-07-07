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

// Attempt to enable/disable and connect/disconnect MV-LV from the LV bus.

#include <stdint.h>
#include <string.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/mvlv_types.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/safety_codes.h"
#include "avionics/linux/aio.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"
#include "lib/util/operator_confirmation.h"

DEFINE_bool(clear_errors, false, "Clear errors for MV-LV node.");
DEFINE_bool(connect, false, "Connect MV-LV to the LV bus.");
DEFINE_bool(disconnect, false, "Disconnect MV-LV from the LV bus.");
DEFINE_bool(enable, false, "Enable MV-LV.");
DEFINE_bool(disable, false, "Disable MV-LV.");

namespace {
MvlvStateCommand state_command;

bool AioCallback(__attribute__((unused)) void *ptr) {
  MvlvCommandMessage set_state_message;
  memset(&set_state_message, 0, sizeof(set_state_message));
  set_state_message.state_command = state_command;

  switch (state_command) {
    case kMvlvStateCommandClearErrors:
      set_state_message.mvlv_signal = MVLV_CLEAR_ERRORS_SIGNAL;
      break;
    case kMvlvStateCommandConnect:
      set_state_message.mvlv_signal = MVLV_CONNECT_SIGNAL;
      break;
    case kMvlvStateCommandDisable:
      set_state_message.mvlv_signal = MVLV_DISABLE_SIGNAL;
      break;
    case kMvlvStateCommandDisconnect:
      set_state_message.mvlv_signal = MVLV_DISCONNECT_SIGNAL;
      break;
    case kMvlvStateCommandEnable:
      set_state_message.mvlv_signal = MVLV_ENABLE_SIGNAL;
      break;
    case kMvlvStateCommandNone:
    default:
      assert(false);
  }

  AIO_SEND_PACKED(kMessageTypeMvlvCommand, PackMvlvCommandMessage,
                  PACK_MVLVCOMMANDMESSAGE_SIZE, &set_state_message);
  return false;
}

std::string GetActionString() {
  std::string action;
  switch (state_command) {
    case kMvlvStateCommandClearErrors:
      action = "Clear errors";
      break;
    case kMvlvStateCommandConnect:
      action = "Connect";
      break;
    case kMvlvStateCommandDisconnect:
      action = "Disconnect";
      break;
    case kMvlvStateCommandEnable:
      action = "Enable";
      break;
    case kMvlvStateCommandDisable:
      action = "Disable";
      break;
    case kMvlvStateCommandNone:
    default:
      assert(false);
  }
  return action;
}

}  // namespace.

int main(int argc, char *argv[]) {
  FLAGS_stderrthreshold = 0;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage("Send a command to MV-LV with retry.  "
                          "Exactly one command flag must be specified.");

  if (FLAGS_clear_errors + FLAGS_connect + FLAGS_disconnect
      + FLAGS_enable + FLAGS_disable != 1) {
    LOG(ERROR) << "Must specify exactly one command (connect, "
               << "disconnect, enable, disable, or clear_errors).";
    google::ShowUsageWithFlagsRestrict(argv[0], "mvlv_command");
    return EXIT_FAILURE;
  }

  if (FLAGS_clear_errors) {
    state_command = kMvlvStateCommandClearErrors;
  } else if (FLAGS_connect) {
    state_command = kMvlvStateCommandConnect;
  } else if (FLAGS_disconnect) {
    state_command = kMvlvStateCommandDisconnect;
  } else if (FLAGS_enable) {
    state_command = kMvlvStateCommandEnable;
  } else if (FLAGS_disable) {
    state_command = kMvlvStateCommandDisable;
  } else {
    assert(false);
  }

  CHECK(OperatorConfirmationPrompt(GetActionString()))
      << "The string you entered does not match.";
  LOG(INFO) << GetActionString();

  const double ts = 0.1;  // Loop period [s] for AIO callback (10 Hz).
  AioLoopStart(kAioNodeOperator, UDP_PORT_AIO, nullptr, 0, AioCallback,
               nullptr, static_cast<int32_t>(1000000 * ts));
  AioClose();

  return EXIT_SUCCESS;
}

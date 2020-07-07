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

// Attempt to connect or disconnect the LV batteries from the LV bus.

#include <stdint.h>
#include <string.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/batt_types.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/safety_codes.h"
#include "avionics/linux/aio.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"
#include "lib/util/operator_confirmation.h"

DEFINE_bool(clear_errors, false, "Clear errors for LV batt node.");
DEFINE_bool(connect, false, "Connect batteries.");
DEFINE_bool(disconnect_a, false, "Disconnect LV battery box A.");
DEFINE_bool(disconnect_b, false, "Disconnect LV battery box B.");

BattStateCommand state_command;

bool AioCallback(void *ptr) {
  assert(ptr != NULL);

  BattCommandMessage set_state_message;
  memset(&set_state_message, 0, sizeof(set_state_message));
  set_state_message.state_command = state_command;

  if (state_command == kBattStateCommandClearErrors) {
    set_state_message.batt_signal = BATT_CLEAR_ERRORS_SIGNAL;
  } else if (state_command == kBattStateCommandConnect) {
    set_state_message.batt_signal = BATT_CONNECT_SIGNAL;
  } else if (state_command == kBattStateCommandDisconnectA) {
    set_state_message.batt_signal = BATT_DISCONNECT_A_SIGNAL;
  } else if (state_command == kBattStateCommandDisconnectB) {
    set_state_message.batt_signal = BATT_DISCONNECT_B_SIGNAL;
  }

  AIO_SEND_PACKED(kMessageTypeBattCommand, PackBattCommandMessage,
                  PACK_BATTCOMMANDMESSAGE_SIZE, &set_state_message);

  // TODO: Add battery responsiveness.
  // ++progress->num_attempts;
  // return (progress->num_attempts < kMaxNumAttempts);

  return false;
}

std::string GetActionString() {
  std::string action;
  switch (state_command) {
    case kBattStateCommandClearErrors:
      action = "Clear errors";
      break;
    case kBattStateCommandConnect:
      action = "Connect";
      break;
    case kBattStateCommandDisconnectA:
      action = "Disconnect A";
      break;
    case kBattStateCommandDisconnectB:
      action = "Disconnect B";
      break;
    case kBattStateCommandNone:
    default:
      assert(false);
  }
  return action;
}

int main(int argc, char *argv[]) {
  FLAGS_stderrthreshold = 0;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage("Send a command to LV batts with retry.  "
                          "Exactly one command flag must be specified.");

  if (FLAGS_clear_errors + FLAGS_connect + FLAGS_disconnect_a
      + FLAGS_disconnect_b != 1) {
    LOG(ERROR) << "Must specify exactly one command (connect, "
               << "disconnect_a, disconnect_b, or clear_errors).";
    google::ShowUsageWithFlagsRestrict(argv[0], "batt_command");
    return EXIT_FAILURE;
  }

  if (FLAGS_clear_errors) {
    state_command = kBattStateCommandClearErrors;
  } else if (FLAGS_connect) {
    state_command = kBattStateCommandConnect;
  } else if (FLAGS_disconnect_a) {
    state_command = kBattStateCommandDisconnectA;
  } else if (FLAGS_disconnect_b) {
    state_command = kBattStateCommandDisconnectB;
  } else {
    assert(false);
  }

  if (FLAGS_disconnect_a || FLAGS_disconnect_b) {
    CHECK(OperatorConfirmationPrompt(GetActionString()))
        << "The string you entered does not match.";
  }
  LOG(INFO) << GetActionString();

  // TODO: Add multiple send and battery responsiveness.
  int placeholder;
  const double ts = 0.1;  // Loop period [s] for AIO callback (10 Hz).
  AioLoopStart(kAioNodeOperator, UDP_PORT_AIO, NULL, 0, AioCallback,
               &placeholder, static_cast<int32_t>(1000000 * ts));
  AioClose();

  return EXIT_SUCCESS;
}

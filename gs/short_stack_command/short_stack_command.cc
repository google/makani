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

// Send control commands to the short stack.

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

DEFINE_bool(force_no_trips, false, "Prevent short_stack from tripping.");
DEFINE_bool(force_trip_b0, false, "Trip block 0 stack level.");
DEFINE_bool(force_trip_b1, false, "Trip block 1 stack level.");
DEFINE_bool(force_trip_b2, false, "Trip block 2 stack level.");
DEFINE_bool(force_trip_b3, false, "Trip block 3 stack level.");
DEFINE_bool(return_to_default, false,
            "Return to short_stack default operation.");

ShortStackCommandValue which_command;

bool AioCallback(void *ptr) {
  assert(ptr != NULL);

  ShortStackCommandMessage command_message;
  memset(&command_message, 0, sizeof(command_message));
  command_message.command_value = which_command;

  switch (which_command) {
    case kShortStackCommandValueForceNoTrips:
      command_message.command_signal = SHORT_STACK_FORCE_NO_TRIPS_SIGNAL;
      break;
    case kShortStackCommandValueForceTripB0:
      command_message.command_signal = SHORT_STACK_FORCE_TRIP_B0_SIGNAL;
      break;
    case kShortStackCommandValueForceTripB1:
      command_message.command_signal = SHORT_STACK_FORCE_TRIP_B1_SIGNAL;
      break;
    case kShortStackCommandValueForceTripB2:
      command_message.command_signal = SHORT_STACK_FORCE_TRIP_B2_SIGNAL;
      break;
    case kShortStackCommandValueForceTripB3:
      command_message.command_signal = SHORT_STACK_FORCE_TRIP_B3_SIGNAL;
      break;
    case kShortStackCommandValueReturnToDefault:
      command_message.command_signal = SHORT_STACK_RETURN_TO_DEFAULT_SIGNAL;
    case kShortStackCommandValueNone:
    default:
      break;
  }

  AIO_SEND_PACKED(kMessageTypeShortStackCommand, PackShortStackCommandMessage,
                  PACK_SHORTSTACKCOMMANDMESSAGE_SIZE, &command_message);
  return false;
}

std::string GetActionString() {
  std::string action;
  switch (which_command) {
    case kShortStackCommandValueForceNoTrips:
      action = "Force no trips.";
      break;
    case kShortStackCommandValueForceTripB0:
      action = "Force-trip block 0 stack level.";
      break;
    case kShortStackCommandValueForceTripB1:
      action = "Force-trip block 1 stack level.";
      break;
    case kShortStackCommandValueForceTripB2:
      action = "Force-trip block 2 stack level.";
      break;
    case kShortStackCommandValueForceTripB3:
      action = "Force-trip block 3 stack level.";
      break;
    case kShortStackCommandValueReturnToDefault:
      action = "Return to default operation.";
      break;
    case kShortStackCommandValueNone:
    default:
      assert(false);
  }
  return action;
}

int main(int argc, char *argv[]) {
  FLAGS_stderrthreshold = 0;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage("Send a command to short stack.  "
                          "Exactly one command flag must be specified.");

  if (FLAGS_force_no_trips + FLAGS_force_trip_b0 + FLAGS_force_trip_b1
      + FLAGS_force_trip_b2 + FLAGS_force_trip_b3 + FLAGS_return_to_default
      != 1) {
    LOG(ERROR) << "Must specify exactly one command (force_no_trips, "
               << "force_trip_b0, 1, 2, or 3, or return_to_default).";
    google::ShowUsageWithFlagsRestrict(argv[0], "short_stack_command");
    return EXIT_FAILURE;
  }

  if (FLAGS_force_no_trips) {
    which_command = kShortStackCommandValueForceNoTrips;
  } else if (FLAGS_force_trip_b0) {
    which_command = kShortStackCommandValueForceTripB0;
  } else if (FLAGS_force_trip_b1) {
    which_command = kShortStackCommandValueForceTripB1;
  } else if (FLAGS_force_trip_b2) {
    which_command = kShortStackCommandValueForceTripB2;
  } else if (FLAGS_force_trip_b3) {
    which_command = kShortStackCommandValueForceTripB3;
  } else if (FLAGS_return_to_default) {
    which_command = kShortStackCommandValueReturnToDefault;
  } else {
    assert(false);
  }

  CHECK(OperatorConfirmationPrompt(GetActionString()))
      << "The string you entered does not match.";
  LOG(INFO) << GetActionString();

  int placeholder;
  const double ts = 0.1;  // Loop period [s] for AIO callback (10 Hz).
  AioLoopStart(kAioNodeOperator, UDP_PORT_AIO, NULL, 0, AioCallback,
               &placeholder, static_cast<int32_t>(1000000 * ts));
  AioClose();

  return EXIT_SUCCESS;
}

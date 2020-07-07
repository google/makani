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

// Attempt to arm, disarm, or clear the errors of wing and ground actuators
// over AIO.
//
// Exits with return code EXIT_SUCCESS if all actuators most recently
// reported being armed.  Otherwise, exits with EXIT_FAILURE.  See -help
// for usage.

#include <stdint.h>
#include <string.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <algorithm>
#include <string>

#include "avionics/common/actuator_types.h"
#include "avionics/linux/aio.h"
#include "avionics/linux/clock.h"
#include "common/macros.h"
#include "gs/state_command/detwist_state.h"
#include "gs/state_command/gs02_state.h"
#include "gs/state_command/motor_state.h"
#include "gs/state_command/servo_state.h"
#include "gs/state_command/state_command.h"
#include "gs/state_command/tether_release_state.h"
#include "gs/state_command/winch_state.h"
#include "lib/util/operator_confirmation.h"

DEFINE_bool(arm, false, "Arm actuators.");
DEFINE_bool(disarm, false, "Disarm actuators.");
DEFINE_bool(clear_errors, false, "Clear errors.");

DEFINE_bool(detwist, false, "Command the detwist.");
DEFINE_bool(gs02, false, "Command the gs02.");
DEFINE_bool(motor, false, "Command the motors.");
DEFINE_bool(servo, false, "Command the servos.");
DEFINE_bool(tether_release, false, "Command the tether release.");
DEFINE_bool(winch, false, "Command the winch.");

DEFINE_string(skip, "", "Comma-separated list of actuators to skip (eg. A2,R1 "
              "or SBO,PTO).");

namespace {

// Number of attempts to make to set all actuators.
constexpr int32_t kMaxNumAttempts = 10;
constexpr int64_t kSendReplyTimeoutUs = 300 * 1000;

bool AioCallback(void *ptr) {
  StateCommandBase *state = static_cast<StateCommandBase *>(ptr);

  if (state->UpdateProgress()) {
    return false;  // Exit when complete.
  }
  if ((ClockGetUs() - state->GetSendTime()) > kSendReplyTimeoutUs) {
    state->SendCommand();
  }
  return state->GetNumAttempts() < kMaxNumAttempts;
}

std::string GetActionString(const StateCommandBase *state) {
  std::string action;
  std::string target;
  std::string skip = "";
  switch (state->GetCommand()) {
    case kActuatorStateCommandArm:
      action = "Arm";
      break;
    case kActuatorStateCommandDisarm:
      action = "Disarm";
      break;
    case kActuatorStateCommandClearErrors:
      action = "Clear errors on";
      break;
    case kActuatorStateCommandTest:
      action = "Test";
      break;
    case kActuatorStateCommandNone:
    default:
      assert(false);
  }

  target = state->GetName();
  transform(target.begin(), target.end(), target.begin(), ::tolower);
  std::string activity = action + " " + target + " targets:";
  activity += state->GetTargetsString(state->GetSelectedTargets());

  uint32_t skip_mask = state->GetSkipTargets();
  if (skip_mask != 0U) {
    activity += " (skipping" + state->GetTargetsString(skip_mask) + ")";
  }
  return activity;
}

}  // namespace.

int main(int argc, char *argv[]) {
  FLAGS_stderrthreshold = 0;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage("Send a command to all actuators with retry.  "
                          "Exactly one command flag and actuator type must be "
                          "specified.");

  if (FLAGS_disarm + FLAGS_arm + FLAGS_clear_errors != 1) {
    LOG(ERROR) << "Must specify exactly one command (disarm, arm, "
               << "clear_errors).";
    google::ShowUsageWithFlagsRestrict(argv[0], "state_command");
    return EXIT_FAILURE;
  }
  if (FLAGS_detwist + FLAGS_gs02 + FLAGS_motor + FLAGS_servo
      + FLAGS_tether_release + FLAGS_winch != 1) {
    LOG(ERROR) << "Must select exactly one actuator type "
               << "(detwist, motor, servo, tether_release, winch).";
    google::ShowUsageWithFlagsRestrict(argv[0], "state_command");
    return EXIT_FAILURE;
  }

  // Select actuator.
  StateCommandBase *state = nullptr;
  if (FLAGS_detwist) {
    state = new DetwistStateCommand();
  } else if (FLAGS_gs02) {
    state = new Gs02StateCommand();
  } else if (FLAGS_motor) {
    state = new MotorStateCommand();
  } else if (FLAGS_servo) {
    state = new ServoStateCommand();
  } else if (FLAGS_tether_release) {
    state = new TetherReleaseStateCommand();
  } else if (FLAGS_winch) {
    state = new WinchStateCommand();
  } else {
    CHECK(false);
  }

  // Select command.
  if (FLAGS_arm) {
    state->SetCommand(kActuatorStateCommandArm);
  } else if (FLAGS_disarm) {
    state->SetCommand(kActuatorStateCommandDisarm);
  } else if (FLAGS_clear_errors) {
    state->SetCommand(kActuatorStateCommandClearErrors);
  } else {
    CHECK(false);
  }

  // Select targets.
  uint32_t targets_mask = state->GetAllTargets();
  std::string token;
  std::stringstream input(FLAGS_skip);
  auto labels = state->GetLabels();
  while (getline(input, token, ',')) {
    token.erase(std::remove(token.begin(), token.end(), ' '), token.end());
    transform(token.begin(), token.end(), token.begin(), ::toupper);
    int match_count = 0;
    for (auto iter : labels) {
      std::string label(iter.second);
      transform(label.begin(), label.end(), label.begin(), ::toupper);
      if (token.compare(label) == 0) {
        match_count++;
        targets_mask &= ~(1U << iter.first);
      }
    }
    if (match_count != 1) {
      LOG(ERROR) << "Invalid actuator specification: " << token;
      return EXIT_FAILURE;
    }
  }
  state->SetTargets(targets_mask);

  if (FLAGS_arm || FLAGS_disarm) {
    CHECK(OperatorConfirmationPrompt(GetActionString(state)))
        << "The string you entered does not match.";
  }
  LOG(INFO) << GetActionString(state);

  // Run!
  const MessageType kSubscribeMessages[] = {
    kMessageTypeGroundStationPlcStatus,
    kMessageTypeGroundStationStatus,
    kMessageTypeGroundStationWinchStatus,
    kMessageTypeLoadcell,
    kMessageTypeMotorStatus,
    kMessageTypeServoStatus,
    kMessageTypeTetherDown,
  };
  const double ts = 0.1;  // Loop period [s] for AIO callback (10 Hz).
  AioLoopStart(kAioNodeOperator, UDP_PORT_AIO,
               kSubscribeMessages, ARRAYSIZE(kSubscribeMessages),
               AioCallback, state, static_cast<int32_t>(1000000 * ts));
  AioClose();

  // Display failures.
  if (state->GetSelectedTargets() != state->GetSetTargets()) {
    LOG(ERROR) << "Failed: Please check monitors. Some actuators may not be "
               << "set.";
    std::string name = state->GetName() + " targets";
    transform(name.begin(), name.end(), name.begin(), ::tolower);

    uint32_t targets_set = state->GetSetTargets();
    if (targets_set != 0U) {
      LOG(WARNING) << "Set " << name << ":"
                   << state->GetTargetsString(targets_set);
    }
    uint32_t targets_unset = state->GetUnsetTargets();
    if (targets_unset != 0U) {
      LOG(WARNING) << "Unset " << name << ":"
                   << state->GetTargetsString(targets_unset);
    }
    uint32_t targets_unresponsive = state->GetUnresponsiveTargets();
    if (targets_unresponsive != 0U) {
      LOG(WARNING) << "Unresponsive " << name << ":"
                   << state->GetTargetsString(targets_unresponsive);
    }
    return EXIT_FAILURE;
  }

  delete state;
  return EXIT_SUCCESS;
}

/*
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "avionics/loadcell/firmware/input.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/controller_arbitration.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/safety_codes.h"
#include "avionics/common/tether_message.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/firmware/cpu/clock.h"

static ControllerArbitrationState g_controller_arbitration_state;

const TetherReleaseSetStateMessage *
LoadcellInputQueryTetherReleaseSetState(void) {
  static TetherReleaseSetStateMessage set_state_msg;
  if (CvtGetTetherReleaseSetStateMessage(kAioNodeOperator, &set_state_msg,
                                         NULL, NULL)) {
    return &set_state_msg;
  }
  return NULL;
}

static bool ValidateControllerCommand(const ControllerCommandMessage *msg) {
  return msg->tether_release == 0
      || msg->tether_release_safety_code == TETHER_RELEASE_SAFETY_CODE;
}

bool LoadcellInputQueryTetherReleaseCommand(TetherReleaseCommand *cmd) {
  int64_t now = ClockGetUs();
  ControllerArbitrationUpdateFromCvt(&g_controller_arbitration_state);
  ControllerLabel source;
  const ControllerCommandMessage *msg =
      ControllerArbitrationGetCommand(now, ValidateControllerCommand,
                                      &g_controller_arbitration_state,
                                      &source);
  if (msg != NULL) {
    cmd->fire_tether_release = msg->tether_release != 0;
    cmd->safety_code = msg->tether_release_safety_code;
    return true;
  }
  return false;
}

const TetherJoystick *LoadcellInputQueryTetherJoystick(void) {
  static TetherUpMergeState merge_state;
  static bool initialized = false;

  if (!initialized) {
    initialized = true;
    TetherUpMergeStateInit(&merge_state);
  }
  const TetherUpMessage *out = TetherUpMergeCvtGet(&merge_state);

  if (TetherIsNoUpdateCountValid(out->joystick.no_update_count)) {
    return &out->joystick;
  }
  return NULL;
}

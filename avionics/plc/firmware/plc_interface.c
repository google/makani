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

#include "avionics/plc/firmware/plc_interface.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/fast_math/fast_math.h"
#include "avionics/common/network_config.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/pack_plc_messages.h"
#include "avionics/common/plc_messages.h"
#include "avionics/common/safety_codes.h"
#include "avionics/common/tether_convert.h"
#include "avionics/common/tether_message.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/params/param_util.h"
#include "avionics/plc/firmware/config_params.h"
#include "common/c_math/util.h"
#include "common/macros.h"

#define PLC_COMMAND_PERIOD_US 10e3
#define PLC_STATUS_TIMEOUT_US 1e6
#define COMMAND_TIMEOUT_US    5e6

static PlcInterfaceState g_state;

static bool IsPlcStatusMessage(uint16_t dest_port, int32_t length) {
  return dest_port == UDP_PORT_PLC && length == PACK_PLCSTATUSMESSAGE_SIZE;
}

static void LimitDetwistCommand(PlcCommandMessage *msg) {
  if (msg->detwist_cmd == kDetwistCommandMove) {
    // Do not move unless armed.
    if (g_state.detwist_state != kActuatorStateArmed) {
      msg->detwist_cmd = kDetwistCommandNone;
    }
  } else if (msg->detwist_cmd == kDetwistCommandReference) {
    // Do not reference unless in ready state.
    if (g_state.detwist_state != kActuatorStateReady) {
      msg->detwist_cmd = kDetwistCommandNone;
    }
  }
}

static void SendPlcCommandMessage(DetwistCommand cmd, double position) {
  PlcCommandMessage out = {
    .header = {
      .version = PLC_COMMS_VERSION,
      .message_type = kPlcMessageTypeCommand,
      .sequence = g_state.plc_command_sequence
    },
    .detwist_cmd = cmd,
    .detwist_position = position
  };
  ++g_state.plc_command_sequence;

  LimitDetwistCommand(&out);

  uint8_t data[PACK_PLCCOMMANDMESSAGE_SIZE];
  PackPlcCommandMessage(&out, 1, data);

  NetSendUdp(PLC_IP_ADDRESS, g_state.plc_mac,
             UDP_PORT_PLC, UDP_PORT_PLC,
             data, ARRAYSIZE(data), NULL);
}

static bool UnwrapTetherPositionCommand(float wrapped, int64_t now,
                                        double *unwrapped) {
  // Must compare to last reported position, so status cannot be stale.
  if (now - g_state.plc_status_timestamp < PLC_STATUS_TIMEOUT_US) {
    const float max = 2.0f * PI_F * TETHER_DETWIST_REVS;
    const float half = PI_F * TETHER_DETWIST_REVS;
    if (wrapped < 0.0f || wrapped >= max) {
      // TODO: Add warning flag for invalid command?
      return false;
    }
    float current_wrapped = fmodf(g_state.plc_status.detwist_position, max);
    if (current_wrapped < 0.0f) {
      current_wrapped += max;
    }
    float delta = wrapped - current_wrapped;
    if (delta > half) {
      delta -= max;
    } else if (delta < -half) {
      delta += max;
    }
    *unwrapped = g_state.plc_status.detwist_position + delta;
    return true;
  } else {
    return false;
  }
}

static void GeneratePlcCommandFromTether(const float tether_command_pos,
                                         int64_t now) {
  DetwistCommand cmd = kDetwistCommandNone;
  if (g_state.detwist_state == kActuatorStateArmed) {
    cmd = kDetwistCommandMove;
  }
  double unwrapped_pos;
  if (UnwrapTetherPositionCommand(tether_command_pos, now, &unwrapped_pos)) {
    SendPlcCommandMessage(cmd, unwrapped_pos);
  } else {
  }
}

static void QueryTetherDown(int64_t now, int64_t *state_timestamp,
                            TetherDownMergeState *state) {
  const TetherDownMessage *message = TetherDownGetMergeOutput(state);

  uint16_t control_command_sequence_z1 = message->control_command.sequence;

  TetherDownMergeCvtGet(state);
  if (control_command_sequence_z1
      != message->control_command.sequence) {
    *state_timestamp = now;
    GeneratePlcCommandFromTether(message->control_command.detwist_angle, now);
  }
}

static void HandlePlcOperatorMsg(const GroundStationPlcOperatorMessage *in) {
  const PlcCommandMessage *cmd = &in->command;
  SendPlcCommandMessage(cmd->detwist_cmd, cmd->detwist_position);
}

static void ForwardClearErrorCommand(void) {
  const DetwistCommand cmd =
      kDetwistCommandPopError
      | kDetwistCommandClearError
      | kDetwistCommandClearWarning;
  SendPlcCommandMessage(cmd, 0);
}

static void HandleSetStateMsg(const GroundStationDetwistSetStateMessage *in) {
  switch (in->state_command) {
    case kActuatorStateCommandDisarm:
      if (g_state.detwist_state == kActuatorStateArmed) {
        g_state.detwist_state = kActuatorStateReady;
      }
      break;
    case kActuatorStateCommandArm:
      if (g_state.detwist_state == kActuatorStateReady) {
        g_state.detwist_state = kActuatorStateArmed;
      }
      break;
    case kActuatorStateCommandClearErrors:
      if (g_state.detwist_state == kActuatorStateError) {
        g_state.detwist_state = kActuatorStateReady;
      }
      // Send command even if detwist_state is not in error, because it also
      // clears warnings and pops message from queue on the PLC.
      ForwardClearErrorCommand();
      break;
    default:
      break;
  }
}

static void UpdateActuatorStateForPlcStatus(const PlcStatusMessage *status) {
  // Transition from any state to error on PLC error.
  if (status->error_flags &&
      !(g_state.allow_arm_without_480v &&
        status->error_flags == kPlcErrorFlagDetwistServoABad)) {
    g_state.detwist_state = kActuatorStateError;
  }
  if (status->info_flags & kPlcInfoFlagDetwistReady
      || g_state.allow_arm_without_480v) {
    // Transition from init to ready if PLC is ready.
    if (g_state.detwist_state == kActuatorStateInit) {
      g_state.detwist_state = kActuatorStateReady;
    }
  } else {
    // Transition from ready or armed to init if PLC is not ready.
    if (g_state.detwist_state == kActuatorStateReady
        || g_state.detwist_state == kActuatorStateArmed) {
      if (!g_state.allow_arm_without_480v) {
        g_state.detwist_state = kActuatorStateInit;
      }
    }
  }
}

static void UpdateActuatorState(const PlcStatusMessage *new_status,
                                int64_t now) {
  if (new_status != NULL) {
    g_state.plc_status = *new_status;
    g_state.plc_status_timestamp = now;
  }
  if (now - g_state.plc_status_timestamp < PLC_STATUS_TIMEOUT_US) {
    UpdateActuatorStateForPlcStatus(new_status);
  } else {
    // Status message has timed out.
    if (g_state.detwist_state == kActuatorStateArmed
        || g_state.detwist_state == kActuatorStateReady) {
      g_state.detwist_state = kActuatorStateInit;
    }
  }
}

static void ForwardStatusMessage(const PlcStatusMessage *in, int64_t now) {
  if (in->header.version != PLC_COMMS_VERSION) {
    return;
  }
  UpdateActuatorState(in, now);

  // Output to AIO network.
  GroundStationPlcStatusMessage out = {
    .plc = *in,
    .detwist_state = g_state.detwist_state
  };
  NetSendSpoofedAio(
      kAioNodePlcTophat, kMessageTypeGroundStationPlcStatus,
      in->header.sequence,
      (PackAioMessageFunction)PackGroundStationPlcStatusMessage, &out);
}

static void QueryOperator(AioNode operator) {
  uint16_t sequence;
  GroundStationDetwistSetStateMessage state_message;
  if (CvtGetGroundStationDetwistSetStateMessage(operator, &state_message,
                                                &sequence, NULL)) {
    HandleSetStateMsg(&state_message);
  }
  GroundStationPlcOperatorMessage op_message;
  if (CvtGetGroundStationPlcOperatorMessage(operator, &op_message, &sequence,
                                            &g_state.operator_cmd_timestamp)) {
    HandlePlcOperatorMsg(&op_message);
  }
}

void PlcInterfaceInit(int64_t reference_time) {
  memset(&g_state, 0, sizeof(g_state));
  g_state.wakeup_plc_command = reference_time + PLC_COMMAND_PERIOD_US;
  ParamStringToEthernetAddress(kPlcConfigParams->plc_mac, &g_state.plc_mac);
  g_state.detwist_state = kActuatorStateInit;
  g_state.allow_arm_without_480v = false;
  g_state.operator_cmd_timestamp = 0;
  g_state.plc_status_timestamp = 0;
  g_state.tether_state_timestamp = 0;

  TetherDownMergeStateInit(&g_state.tether_state);
}

void PlcPollInput(int64_t now) {
  if (now >= g_state.wakeup_plc_command) {
    QueryTetherDown(now, &g_state.tether_state_timestamp,
                    &g_state.tether_state);
    g_state.wakeup_plc_command += PLC_COMMAND_PERIOD_US;
  }
  QueryOperator(kAioNodeOperator);
}

bool HandlePlcMessage(uint16_t dest_port, int32_t length,
                      const uint8_t *data, int64_t now) {
  if (IsPlcStatusMessage(dest_port, length)) {
    static PlcStatusMessage in;
    UnpackPlcStatusMessage(data, 1, &in);
    ForwardStatusMessage(&in, now);
    return true;
  }
  return false;
}

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

#include <assert.h>
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
#include "avionics/firmware/network/net.h"
#include "avionics/firmware/params/param_util.h"
#include "avionics/plc/firmware/config_params.h"
#include "avionics/plc/firmware/plc_interface.h"
#include "common/macros.h"

#define GS02_PLC_COMMS_VERSION 0
#define GS02_PLC_IP_ADDRESS    ((IpAddress) {192, 168, 1, 100})
#define GS02_UDP_PORT_PLC      2175

#define PLC_INPUT_PERIOD_US   10e3
#define PLC_STATUS_TIMEOUT_US 1e6
#define COMMAND_TIMEOUT_US    5e6

#define AZIMUTH_ENABLE_SIGNAL   0x63
#define DETWIST_ENABLE_SIGNAL   0x5A
#define LEVELWIND_ENABLE_SIGNAL 0xC7
#define WINCH_ENABLE_SIGNAL     0xe9

#define ALL_GS_AXES_MASK ((1 << kNumGroundStationActuators) - 1)

typedef struct {
  ActuatorState actuator_state[kNumGroundStationActuators];

  EthernetAddress plc_mac;
  int64_t plc_status_timestamp;
  PlcGs02StatusMessage plc_status;

  int64_t tether_state_timestamp;
  TetherDownMergeState tether_state;

  const TetherDownMessage *tether_down_message;
  uint16_t control_command_sequence_z1;  // Previous sequence value.

  int64_t weather_message_timestamp;
  GroundStationWeatherMessage weather_message;

  int64_t platform_sensors_message_timestamp[2];
  PlatformSensorsMessage platform_sensors_message[2];

  int64_t wakeup_plc_input;
  PlcGs02InputMessage plc_input;
  uint16_t plc_input_sequence;
} Gs02PlcInterfaceState;

static Gs02PlcInterfaceState g_plc_state;

static bool IsPlcGs02StatusMessage(uint16_t dest_port, int32_t length) {
  return (dest_port == GS02_UDP_PORT_PLC
          && length == PACK_PLCGS02STATUSMESSAGE_SIZE);
}

static bool IsPlcGs02ControlMessage(uint16_t dest_port, int32_t length) {
  return (dest_port == GS02_UDP_PORT_PLC
          && length == PACK_PLCGS02CONTROLMESSAGE_SIZE);
}

static bool IsSetStateValid(const GroundStationSetStateMessage *msg) {
  if (msg->state_command == kActuatorStateCommandArm) {
    return msg->arming_signal == GROUND_STATION_ARMING_SIGNAL;
  } else if (msg->state_command == kActuatorStateCommandDisarm) {
    return msg->arming_signal == GROUND_STATION_DISARM_SIGNAL;
  } else {
    return true;
  }
}

static bool UnwrapDetwistPosition(double detwist_position, float wrapped,
                                  double *unwrapped) {
  const float max = 2.0f * PI_F * TETHER_DETWIST_REVS;
  const float half = PI_F * TETHER_DETWIST_REVS;
  if (wrapped < 0.0f || wrapped >= max) {
    // TODO: Add warning flag for invalid command?
    return false;
  }
  float current_wrapped = fmodf(detwist_position, max);
  if (current_wrapped < 0.0f) {
    current_wrapped += max;
  }
  float delta = wrapped - current_wrapped;
  if (delta > half) {
    delta -= max;
  } else if (delta < -half) {
    delta += max;
  }
  *unwrapped = detwist_position + delta;

  return true;
}

static void ProcessGsSensorData(Gs02PlcInterfaceState *state) {
  // TODO: Inform GS if weather data are stale.
  state->plc_input.weather_temp = state->weather_message.weather.temperature;
  state->plc_input.weather_dewpoint = state->weather_message.weather.dewpoint;
  state->plc_input.perch_azi_angle[0] =
      state->platform_sensors_message[0].encoders.perch_azi;
  state->plc_input.perch_azi_angle[1] =
      state->platform_sensors_message[1].encoders.perch_azi;
}

static void SendPlcInputMessage(Gs02PlcInterfaceState *state) {
  state->plc_input.header.version = GS02_PLC_COMMS_VERSION;
  state->plc_input.header.message_type = kPlcMessageTypeGs02Input;
  state->plc_input.header.sequence = state->plc_input_sequence;
  ++state->plc_input_sequence;

  uint8_t data[PACK_PLCGS02INPUTMESSAGE_SIZE];
  PackPlcGs02InputMessage(&state->plc_input, 1, data);

  NetSendUdp(GS02_PLC_IP_ADDRESS, state->plc_mac,
             GS02_UDP_PORT_PLC, GS02_UDP_PORT_PLC,
             data, ARRAYSIZE(data), NULL);
}

static void QueryTetherDown(Gs02PlcInterfaceState *state) {
  state->tether_down_message =
      TetherDownGetMergeOutput(&state->tether_state);
  // Store last command sequence for later comparison.
  state->control_command_sequence_z1 =
      state->tether_down_message->control_command.sequence;
  // Updates state->tether_down_message->control_command.sequence.
  TetherDownMergeCvtGet(&state->tether_state);
}

static void ProcessTetherDown(int64_t now, Gs02PlcInterfaceState *state) {
  const TetherDownMessage *message = state->tether_down_message;
  if (state->control_command_sequence_z1 == message->control_command.sequence) {
    return;
  }
  state->tether_state_timestamp = now;

  // Ensure that we have fresh detwist position data for computing the
  // unwrapped position. The TetherDown position command is wrapped at
  // TETHER_DETWIST_REVS turns and must be unwrapped to the closest absolute
  // position to the current detwist position.
  if (now - state->plc_status_timestamp < PLC_STATUS_TIMEOUT_US) {
    UnwrapDetwistPosition(state->plc_status.status.detwist.position,
                          message->control_command.detwist_angle,
                          &state->plc_input.detwist_position_cmd);

    state->plc_input.winch_velocity_cmd =
        message->control_command.winch_velocity;
    state->plc_input.mode_request = message->control_command.gs_mode_request;

    state->plc_input.azi_cmd[0] =
        message->control_command.gs_azi_target;
    state->plc_input.azi_cmd[1] = 0.0f;
    state->plc_input.azi_cmd[2] = message->control_command.gs_azi_dead_zone;
    state->plc_input.unpause_transform =
        message->control_command.gs_unpause_transform;
  }
}

static void SetAxisEnableSignals(Gs02PlcInterfaceState *state) {
  state->plc_input.enable_azimuth =
      (state->actuator_state[kGroundStationActuatorAzimuth]
       == kActuatorStateArmed) ? AZIMUTH_ENABLE_SIGNAL : 0;
  state->plc_input.enable_detwist =
      (state->actuator_state[kGroundStationActuatorDetwist]
       == kActuatorStateArmed) ? DETWIST_ENABLE_SIGNAL : 0;
  state->plc_input.enable_levelwind =
      (state->actuator_state[kGroundStationActuatorLevelwind]
       == kActuatorStateArmed) ? LEVELWIND_ENABLE_SIGNAL : 0;
  state->plc_input.enable_winch =
      (state->actuator_state[kGroundStationActuatorWinch]
       == kActuatorStateArmed) ? WINCH_ENABLE_SIGNAL : 0;
}

static void ForwardClearErrorCommand(Gs02PlcInterfaceState *state) {
  Gs02Command cmd =
      kGs02CommandPopError
      | kGs02CommandClearErrors
      | kGs02CommandClearWarnings;
  state->plc_input.command = cmd;
  // Confirm no command bits lost in implicit type conversion.
  assert(state->plc_input.command == cmd);
}

static void HandleSetStateMsg(const ActuatorStateCommand state_command,
                              ActuatorState *actuator_state,
                              Gs02PlcInterfaceState *state) {
  switch (state_command) {
    case kActuatorStateCommandDisarm:
      if (*actuator_state == kActuatorStateArmed) {
        *actuator_state = kActuatorStateReady;
      }
      break;
    case kActuatorStateCommandArm:
      if (*actuator_state == kActuatorStateReady) {
        *actuator_state = kActuatorStateArmed;
      }
      break;
    case kActuatorStateCommandClearErrors:
      if (*actuator_state == kActuatorStateError) {
        *actuator_state = kActuatorStateReady;
      }
      // Send command even if detwist_state is not in error, because it also
      // clears warnings and pops message from queue on the PLC.
      ForwardClearErrorCommand(state);
      break;
    default:
      break;
  }
}

// Change axis state only if the transition is valid.
static void SetAxisState(ActuatorState new_state,
                         ActuatorState *actuator_state) {
  switch (new_state) {
    case kActuatorStateInit:
      *actuator_state = kActuatorStateInit;
      break;
    case kActuatorStateReady:
      if (*actuator_state == kActuatorStateInit
          || *actuator_state == kActuatorStateArmed) {
        *actuator_state = kActuatorStateReady;
      }
      break;
    case kActuatorStateArmed:
      if (*actuator_state == kActuatorStateReady) {
        *actuator_state = kActuatorStateArmed;
      }
      break;
    case kActuatorStateError:
      *actuator_state = kActuatorStateError;
      break;
    default:
      break;
  }
}

static void SetAxisStates(ActuatorState new_state, int32_t axis_mask,
                          Gs02PlcInterfaceState *state) {
  int axis;
  for (axis = 0; axis < kNumGroundStationActuators; axis++) {
    if (axis_mask & (1 << axis)) {
      SetAxisState(new_state, &state->actuator_state[axis]);
    }
  }
}

static void DisarmArmedAxes(Gs02PlcInterfaceState *state) {
  int axis;
  for (axis = 0; axis < kNumGroundStationActuators; axis++) {
    if (state->actuator_state[axis] == kActuatorStateArmed) {
      state->actuator_state[axis] = kActuatorStateReady;
    }
  }
}

static void ReadyInitedAxes(Gs02PlcInterfaceState *state) {
  int axis;
  for (axis = 0; axis < kNumGroundStationActuators; axis++) {
    if (state->actuator_state[axis] == kActuatorStateInit) {
      state->actuator_state[axis] = kActuatorStateReady;
    }
  }
}

static void UpdateActuatorState(int64_t now, Gs02PlcInterfaceState *state) {
  if (now - state->plc_status_timestamp >= PLC_STATUS_TIMEOUT_US) {
    // Status message has timed out.
    SetAxisStates(kActuatorStateInit, ALL_GS_AXES_MASK, state);
  }
  // Disarm if not receiving commands.
  if (now - state->tether_state_timestamp > COMMAND_TIMEOUT_US) {
    DisarmArmedAxes(state);
  }

  // Until PLC status reports useful axis status, move directly to ready
  // on startup.
  // TODO: Update states from last Gs02PlcInterfaceState
  // message.
  ReadyInitedAxes(state);
}

static void ForwardStatusMessage(const PlcGs02StatusMessage *in, int64_t now,
                                 Gs02PlcInterfaceState *state) {
  assert(in != NULL);
  if (in->header.version != GS02_PLC_COMMS_VERSION) {
    return;
  }

  state->plc_status = *in;
  state->plc_status_timestamp = now;

  UpdateActuatorState(now, state);

  // Output to AIO network.
  GroundStationStatusMessage out = {
    .status = in->status
  };
  memcpy(&out.actuator_state, &state->actuator_state,
         sizeof(state->actuator_state));

  NetSendSpoofedAio(
      kAioNodePlcGs02, kMessageTypeGroundStationStatus, in->header.sequence,
      (PackAioMessageFunction)PackGroundStationStatusMessage, &out);
}

static void ForwardControlMessage(const PlcGs02ControlMessage *in) {
  GroundStationControlMessage out = {
    .input = in->in,
    .output = in->out
  };
  NetSendSpoofedAio(
      kAioNodePlcGs02, kMessageTypeGroundStationControl, in->header.sequence,
      (PackAioMessageFunction)PackGroundStationControlMessage, &out);
}

static void QueryOperator(AioNode operator, Gs02PlcInterfaceState *state) {
  uint16_t sequence;
  GroundStationSetStateMessage state_message;
  if (CvtGetGroundStationSetStateMessage(operator, &state_message,
                                         &sequence, NULL)) {
    if (IsSetStateValid(&state_message)) {
      int axis;
      for (axis = 0; axis < kNumGroundStationActuators; axis++) {
        if (state_message.actuator_mask & (1 << axis)) {
          HandleSetStateMsg(state_message.state_command,
                            &state->actuator_state[axis],
                            state);
        }
      }
    }
  }
}

static void QueryGroundStation(Gs02PlcInterfaceState *state) {
  CvtGetGroundStationWeatherMessage(kAioNodePlatformSensorsA,
                                    &state->weather_message,
                                    NULL, &state->weather_message_timestamp);
  CvtGetPlatformSensorsMessage(kAioNodePlatformSensorsA,
                               &state->platform_sensors_message[0],
                               NULL,
                               &state->platform_sensors_message_timestamp[0]);
  CvtGetPlatformSensorsMessage(kAioNodePlatformSensorsB,
                               &state->platform_sensors_message[1],
                               NULL,
                               &state->platform_sensors_message_timestamp[1]);
}

void PlcInterfaceInit(int64_t reference_time) {
  memset(&g_plc_state, 0, sizeof(g_plc_state));
  g_plc_state.wakeup_plc_input = reference_time + PLC_INPUT_PERIOD_US;
  ParamStringToEthernetAddress(kPlcConfigParams->plc_mac, &g_plc_state.plc_mac);
  SetAxisStates(kActuatorStateInit, ALL_GS_AXES_MASK, &g_plc_state);
  TetherDownMergeStateInit(&g_plc_state.tether_state);
}

void PlcPollInput(int64_t now) {
  QueryGroundStation(&g_plc_state);
  QueryOperator(kAioNodeOperator, &g_plc_state);
  QueryTetherDown(&g_plc_state);

  ProcessGsSensorData(&g_plc_state);
  ProcessTetherDown(now, &g_plc_state);

  SetAxisEnableSignals(&g_plc_state);
  if (now >= g_plc_state.wakeup_plc_input) {
    g_plc_state.wakeup_plc_input += PLC_INPUT_PERIOD_US;
    SendPlcInputMessage(&g_plc_state);
  }
}

bool HandlePlcMessage(uint16_t dest_port, int32_t length,
                      const uint8_t *data, int64_t now) {
  if (IsPlcGs02StatusMessage(dest_port, length)) {
    static PlcGs02StatusMessage in;
    UnpackPlcGs02StatusMessage(data, 1, &in);
    ForwardStatusMessage(&in, now, &g_plc_state);
    return true;
  } else if (IsPlcGs02ControlMessage(dest_port, length)) {
    static PlcGs02ControlMessage in;
    UnpackPlcGs02ControlMessage(data, 1, &in);
    ForwardControlMessage(&in);
    return true;
  }
  UpdateActuatorState(now, &g_plc_state);
  return false;
}

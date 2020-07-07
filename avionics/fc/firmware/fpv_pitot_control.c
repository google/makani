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

#include "avionics/fc/firmware/fpv_pitot_control.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

#include "avionics/common/controller_arbitration.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pitot_cover_types.h"
#include "avionics/common/safety_codes.h"
#include "avionics/fc/firmware/config_params.h"
#include "avionics/fc/firmware/output.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/drivers/bcm53101.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/switch_config.h"

#define SWITCH_PORT 3
#define PITOT_REST_CYCLES CLOCK32_SEC_TO_CYCLES(2)
#define PITOT_UNCOVER_CYCLES CLOCK32_SEC_TO_CYCLES(4)
#define PITOT_COVER_CYCLES CLOCK32_SEC_TO_CYCLES(8)
#define PITOT_MOVE_CYCLES CLOCK32_SEC_TO_CYCLES(45)

static ControllerArbitrationState g_controller_arbitration_state;
static FlightComputerSensorMessage *g_sensor_message;

typedef enum {
  kPitotCommandUncover,
  kPitotCommandCover
} PitotCommand;

typedef enum {
  kPitotStateIdle,
  kPitotStateInit,
  kPitotStateCommand,
  kPitotStateWait,
  kPitotStateMove
} PitotState;

static struct {
  PitotCommand command;
  PitotState state;
  uint32_t wakeup_clock;
} g_pitot;

static void SetFpvPitotPower(bool enable) {
  // The switch optical port is used to inhibit the FPV or pitot power supply,
  // so the port should be disabled to enable the power to the device.
  SwitchOptions *options = SwitchConfigGetSwitchOptions();
  bool power_enabled =
      (options->port_disable_mask_current & (1 << SWITCH_PORT)) != 0;
  if (enable && !power_enabled) {
    options->port_disable_mask_current |= 1 << SWITCH_PORT;
    Bcm53101ReconfigureOptions();
  } else if (!enable && power_enabled) {
    options->port_disable_mask_current &= ~(1 << SWITCH_PORT);
    Bcm53101ReconfigureOptions();
  }
}

static void SetFpvEnable(bool enable) {
  SetFpvPitotPower(enable);
  FcOutputFpvState(enable, g_sensor_message);
}

void FpvPitotControlInit(FlightComputerSensorMessage *sensor_message) {
  g_sensor_message = sensor_message;
  FcOutputPitotState(kPitotCoverStatusUnknown, g_sensor_message);
  if (AppConfigGetAioNode() != kAioNodeFcC) {
    FcOutputFpvState(false, g_sensor_message);
    return;
  }
  ControllerArbitrationInit(ClockGetUs(), &g_controller_arbitration_state);
  SetFpvEnable(false);
}

static bool ValidateControllerCommand(const ControllerCommandMessage *msg) {
  (void)msg;
  return true;
}

static void CommandPitotCover(PitotCommand signal) {
  if ((g_pitot.state == kPitotStateIdle) || (g_pitot.command != signal)) {
    SetFpvPitotPower(false);
    g_pitot.command = signal;
    if (g_pitot.command == kPitotCommandCover) {
      FcOutputPitotState(kPitotCoverStatusClosing, g_sensor_message);
    } else if (g_pitot.command == kPitotCommandUncover) {
      FcOutputPitotState(kPitotCoverStatusOpening, g_sensor_message);
    } else {
      assert(false);
    }
    g_pitot.state = kPitotStateInit;
    g_pitot.wakeup_clock = Clock32GetCycles() + PITOT_REST_CYCLES;
  }
}

static void PitotCoverStatePoll(void) {
  switch (g_pitot.state) {
    case kPitotStateIdle:
      // Pitot cover power is off.
      // Refer to this doc for details on the pitot cover logic:
      // https://docs.google.com/document/d/1AI8B6Qv3N8hORgGIKrj1KYxhKiTe7pzyCaDpHYg_zWM/
      break;
    case kPitotStateInit:
      // Pitot cover power is off.
      if (CLOCK32_GT(Clock32GetCycles(), g_pitot.wakeup_clock)) {
        SetFpvPitotPower(true);
        g_pitot.state = kPitotStateCommand;
        if (g_pitot.command == kPitotCommandCover) {
          g_pitot.wakeup_clock += PITOT_COVER_CYCLES;
        } else if (g_pitot.command == kPitotCommandUncover) {
          g_pitot.wakeup_clock += PITOT_UNCOVER_CYCLES;
        } else {
          SetFpvPitotPower(false);
          g_pitot.state = kPitotStateIdle;
        }
      }
      break;
    case kPitotStateCommand:
      // Pitot cover power is on.
      if (CLOCK32_GT(Clock32GetCycles(), g_pitot.wakeup_clock)) {
        SetFpvPitotPower(false);
        g_pitot.state = kPitotStateWait;
        g_pitot.wakeup_clock += PITOT_REST_CYCLES;
      }
      break;
    case kPitotStateWait:
      // Pitot cover power is off.
      if (CLOCK32_GT(Clock32GetCycles(), g_pitot.wakeup_clock)) {
        SetFpvPitotPower(true);
        g_pitot.state = kPitotStateMove;
        g_pitot.wakeup_clock += PITOT_MOVE_CYCLES;
      }
      break;
    case kPitotStateMove:
      // Pitot cover power is on.
      if (CLOCK32_GT(Clock32GetCycles(), g_pitot.wakeup_clock)) {
        SetFpvPitotPower(false);
        g_pitot.state = kPitotStateIdle;
        if (g_pitot.command == kPitotCommandCover) {
          FcOutputPitotState(kPitotCoverStatusClosed, g_sensor_message);
        } else if (g_pitot.command == kPitotCommandUncover) {
          FcOutputPitotState(kPitotCoverStatusOpened, g_sensor_message);
        }
      }
      break;
    default:
      assert(false);
      break;
  }
}

void FpvPitotControlPoll(void) {
  if (AppConfigGetAioNode() != kAioNodeFcC) {
    return;
  }

  if (kFcConfigParams->fpv_pitot_config == kFpvPitotConfigFpvCamera) {
    FpvSetStateMessage fpv_msg;
    if (CvtGetFpvSetStateMessage(kAioNodeOperator, &fpv_msg, NULL, NULL)) {
      if (fpv_msg.enable && fpv_msg.safety_code == FPV_ENABLE_SIGNAL) {
        SetFpvEnable(true);
      } else if (!fpv_msg.enable && fpv_msg.safety_code == FPV_DISABLE_SIGNAL) {
        SetFpvEnable(false);
      }
    }

    ControllerArbitrationUpdateFromCvt(&g_controller_arbitration_state);
    ControllerLabel source;
    const ControllerCommandMessage *cmd_msg =
        ControllerArbitrationGetCommand(ClockGetUs(), ValidateControllerCommand,
                                        &g_controller_arbitration_state,
                                        &source);
    if (cmd_msg->tether_release &&
        cmd_msg->tether_release_safety_code == TETHER_RELEASE_SAFETY_CODE) {
      // This will remain on unless we send an updated FpvSetStateMessage.
      SetFpvEnable(true);
    }
  } else if (kFcConfigParams->fpv_pitot_config == kFpvPitotConfigPitotCover) {
    PitotSetStateMessage pitot_msg;
    if (CvtGetPitotSetStateMessage(kAioNodeOperator, &pitot_msg, NULL, NULL)) {
      if (pitot_msg.cover && pitot_msg.safety_code == PITOT_COVER_SIGNAL) {
        CommandPitotCover(kPitotCommandCover);
      } else if (!pitot_msg.cover &&
                 pitot_msg.safety_code == PITOT_UNCOVER_SIGNAL) {
        CommandPitotCover(kPitotCommandUncover);
      }
    }
    PitotCoverStatePoll();
  }
}

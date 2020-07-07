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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/faults.h"
#include "avionics/common/mvlv_types.h"
#include "avionics/common/safety_codes.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/drivers/log.h"
#include "avionics/firmware/monitors/mvlv_mcp342x_types.h"
#include "avionics/firmware/serial/carrier_serial.h"
#include "avionics/mvlv/firmware/output.h"
#include "avionics/mvlv/firmware/state.h"

#define MVLV_OUTPUT_EN0 kIoN2het1Pin16
#define MVLV_OUTPUT_EN1 kIoN2het1Pin18
#define MVLV_ENABLE     kIoN2het1Pin14

// TODO: Define pin logic (high enable or low enable) using
// hardware version.
#define MVLV_OUTPUT_EN_TRUE 1
#define MVLV_ENABLE_TRUE 0

#define RETRY_CYCLES CLOCK32_MSEC_TO_CYCLES(10)

typedef enum {
  kMvlvStateInit,
  kMvlvStateIdle,
  kMvlvStateRecvCommand,  // Receive command from operator.
  kMvlvStateCheckFault,   // Check system health and handle errors.
  kMvlvStateSwitchPins,   // Switch control pins.
} MvlvState;

static struct {
  MvlvState state;
  MvlvStateCommand cmd;
  bool connect;
  bool enable;
  bool fault_retry;
  uint32_t retry_timeout;
} g_mvlv;

static float GetLvBusVoltage(const MvlvMonitorData *mon) {
  return mon->analog_data[kMvlvAnalogVoltageVLv];
}

static float GetPriLvBusVoltage(const MvlvMonitorData *mon) {
  return mon->analog_data[kMvlvAnalogVoltageVLvPri];
}

static void ResetRetryTimeout(void) {
  g_mvlv.retry_timeout = Clock32GetCycles() + RETRY_CYCLES;
}

static void StartConnectRetry(MvlvMonitorData *mon) {
  g_mvlv.fault_retry = true;
  ResetRetryTimeout();
  SetStatus(kMvlvMonitorStatusConnected, false, &mon->flags);
  SetStatus(kMvlvMonitorStatusFaultRetry, true, &mon->flags);
  LOG_PRINTF("MV-LV enters fault-retry mode.\n");
}

static bool IsConnectReady(void) {
  return !g_mvlv.fault_retry ||
      CLOCK32_GE(Clock32GetCycles(), g_mvlv.retry_timeout);
}

// Close output FET switches such that mv-lv connects to LV bus.
static void ConnectMvlvToBus(MvlvMonitorData *mon) {
  // TODO: Check local LV bus to be sure it can power the bus.
  // TODO: Check primary LV bus to prevent backfeeding.
  if (GetLvBusVoltage(mon) < 70.0f) {
    LOG_PRINTF("MV-LV is not ready: voltage is too low to make connection.\n");
    StartConnectRetry(mon);
  } else if (GetPriLvBusVoltage(mon) > 73.0f) {
    LOG_PRINTF("MV-LV is not ready: primary bus voltage is too high.\n");
    StartConnectRetry(mon);
  } else if (IsConnectReady()) {
    g_mvlv.fault_retry = false;
    IoSetValue(MVLV_OUTPUT_EN0, MVLV_OUTPUT_EN_TRUE);
    IoSetValue(MVLV_OUTPUT_EN1, MVLV_OUTPUT_EN_TRUE);
    SetStatus(kMvlvMonitorStatusConnected, true, &mon->flags);
    SetStatus(kMvlvMonitorStatusFaultRetry, false, &mon->flags);
  }
}

// Open output FET switches such that mv-lv ceases to power LV bus.
static void DisconnectMvlvFromBus(MvlvMonitorData *mon) {
  IoSetValue(MVLV_OUTPUT_EN0, !MVLV_OUTPUT_EN_TRUE);
  IoSetValue(MVLV_OUTPUT_EN1, !MVLV_OUTPUT_EN_TRUE);
  SetStatus(kMvlvMonitorStatusConnected, false, &mon->flags);
}

// Set pin to disable mv-lv converter by stop HV side triggers.
static void DisableMvlv(MvlvMonitorData *mon) {
  // TODO: Check secondary LV to be sure MVLV can restart.
  IoSetValue(MVLV_ENABLE, !MVLV_ENABLE_TRUE);
  SetStatus(kMvlvMonitorStatusEnabled, false, &mon->flags);
}

// Set pin to enable mv-lv converter.
static void EnableMvlv(MvlvMonitorData *mon) {
  // TODO: Check HV side voltage once hardware is implemented.
  // TODO: Checking 12V power good and secondary LV bus voltage.
  IoSetValue(MVLV_ENABLE, MVLV_ENABLE_TRUE);
  SetStatus(kMvlvMonitorStatusEnabled, true, &mon->flags);
}

static bool GetMvlvConnectStatus(void) {
  return IoGetValue(MVLV_OUTPUT_EN0) == MVLV_OUTPUT_EN_TRUE &&
      IoGetValue(MVLV_OUTPUT_EN1) == MVLV_OUTPUT_EN_TRUE;
}

static bool GetMvlvEnableStatus(void) {
  return IoGetValue(MVLV_ENABLE) == MVLV_ENABLE_TRUE;
}

// Validate command received.
static bool CommandSignalValid(const MvlvCommandMessage *msg) {
  switch (msg->state_command) {
    case kMvlvStateCommandClearErrors:
      return msg->mvlv_signal == MVLV_CLEAR_ERRORS_SIGNAL;
    case kMvlvStateCommandConnect:
      return msg->mvlv_signal == MVLV_CONNECT_SIGNAL;
    case kMvlvStateCommandDisable:
      return msg->mvlv_signal == MVLV_DISABLE_SIGNAL;
    case kMvlvStateCommandDisconnect:
      return msg->mvlv_signal == MVLV_DISCONNECT_SIGNAL;
    case kMvlvStateCommandEnable:
      return msg->mvlv_signal == MVLV_ENABLE_SIGNAL;
    default:
      return false;
  }
}

static void MvlvProcessCmdClearErrors(MvlvMonitorData *mon) {
  assert(mon != NULL);
  ClearErrors(&mon->flags);
  g_mvlv.fault_retry = false;
  SetStatus(kMvlvMonitorStatusFaultRetry, false, &mon->flags);
  SetStatus(kMvlvMonitorStatusCmdProcessed, true, &mon->flags);
}

static void MvlvProcessCmdConnect(MvlvMonitorData *mon) {
  if (g_mvlv.enable) {
    g_mvlv.connect = true;
  }
  SetStatus(kMvlvMonitorStatusCmdProcessed, true, &mon->flags);
}

static void MvlvProcessCmdDisconnect(MvlvMonitorData *mon) {
  g_mvlv.connect = false;
  SetStatus(kMvlvMonitorStatusCmdProcessed, true, &mon->flags);
  g_mvlv.fault_retry = false;
  SetStatus(kMvlvMonitorStatusFaultRetry, false, &mon->flags);
}

static void MvlvProcessCmdDisable(MvlvMonitorData *mon) {
  g_mvlv.enable = false;
  g_mvlv.connect = false;
  SetStatus(kMvlvMonitorStatusCmdProcessed, true, &mon->flags);
}

static void MvlvProcessCmdEnable(MvlvMonitorData *mon) {
  g_mvlv.enable = true;
  SetStatus(kMvlvMonitorStatusCmdProcessed, true, &mon->flags);
}

// State Handling Functions.

static void MvlvHandleInit(MvlvState next) {
  IoInit();

  // Configure control pins for MV-LV enable and output switches.
  IoConfigureAsOutputPushPull(MVLV_ENABLE, !MVLV_ENABLE_TRUE);
  IoConfigureAsOutputPushPull(MVLV_OUTPUT_EN0, !MVLV_OUTPUT_EN_TRUE);
  IoConfigureAsOutputPushPull(MVLV_OUTPUT_EN1, !MVLV_OUTPUT_EN_TRUE);

  // MV-LV is disabled and output switch is disconnected by default.
  g_mvlv.enable = false;
  g_mvlv.connect = false;
  g_mvlv.fault_retry = false;

  g_mvlv.state = next;
}

static void MvlvHandleIdle(MvlvState next) {
  g_mvlv.state = next;
}

static void MvlvHandleRecvCommand(MvlvState next, MvlvMonitorData *mon) {
  MvlvCommandMessage msg;
  g_mvlv.cmd = kMvlvStateCommandNone;
  if (CvtGetMvlvCommandMessage(kAioNodeOperator, &msg, NULL, NULL)
      && CommandSignalValid(&msg)) {
    g_mvlv.cmd = msg.state_command;
    SetStatus(kMvlvMonitorStatusCmdReceived, true, &mon->flags);
  }
  switch (g_mvlv.cmd) {
    case kMvlvStateCommandClearErrors:
      LOG_PRINTF("Received MV-LV clear error signal.\n");
      MvlvProcessCmdClearErrors(mon);
      break;
    case kMvlvStateCommandConnect:
      LOG_PRINTF("Received MV-LV connect signal.\n");
      MvlvProcessCmdConnect(mon);
      break;
    case kMvlvStateCommandDisconnect:
      LOG_PRINTF("Received MV-LV disconnect signal.\n");
      MvlvProcessCmdDisconnect(mon);
      break;
    case kMvlvStateCommandEnable:
      LOG_PRINTF("Received MV-LV enable signal.\n");
      MvlvProcessCmdEnable(mon);
      break;
    case kMvlvStateCommandDisable:
      LOG_PRINTF("Received MV-LV disable signal.\n");
      MvlvProcessCmdDisable(mon);
      break;
    case kMvlvStateCommandNone:
    default:
      break;
  }
  g_mvlv.state = next;
}

static void MvlvHandleFault(MvlvState next, MvlvMonitorData *mon) {
  // TODO: Implement fault detection and handling.
  // Fault to be handled: Over temperature.

  // Handle over current fault.
  if (g_mvlv.connect && (GetLvBusVoltage(mon) < 45.0f)) {
    if (g_mvlv.fault_retry) {
      ResetRetryTimeout();
    } else {
      DisconnectMvlvFromBus(mon);
      LOG_PRINTF("MV-LV is over current: voltage drops below 45V.\n");
      StartConnectRetry(mon);
    }
  }
  g_mvlv.state = next;
}

static void MvlvHandleSwitchPins(MvlvState next, MvlvMonitorData *mon) {
  if (GetMvlvConnectStatus() != g_mvlv.connect) {
    if (g_mvlv.connect) {
      ConnectMvlvToBus(mon);
    } else {
      DisconnectMvlvFromBus(mon);
    }
  }
  if (GetMvlvEnableStatus() != g_mvlv.enable) {
    if (g_mvlv.enable) {
      EnableMvlv(mon);
    } else {
      DisableMvlv(mon);
    }
  }
  g_mvlv.state = next;
}

// Init.
void MvlvStateInit(void) {
  memset(&g_mvlv, 0, sizeof(g_mvlv));
  g_mvlv.state = kMvlvStateInit;
  g_mvlv.enable = false;
  g_mvlv.connect = false;
  g_mvlv.fault_retry = false;
}

void MvlvStatePoll(void) {
  MvlvMonitorData *mon = MvlvOutputGetMvlvMonitors();
  assert(mon != NULL);
  switch (g_mvlv.state) {
    case kMvlvStateInit:
      MvlvHandleInit(kMvlvStateIdle);
      break;
    case kMvlvStateIdle:
      MvlvHandleIdle(kMvlvStateRecvCommand);
      break;
    case kMvlvStateRecvCommand:
      MvlvHandleRecvCommand(kMvlvStateCheckFault, mon);
      break;
    case kMvlvStateCheckFault:
      MvlvHandleFault(kMvlvStateSwitchPins, mon);
      break;
    case kMvlvStateSwitchPins:
      MvlvHandleSwitchPins(kMvlvStateIdle, mon);
      break;
    default:
      g_mvlv.state = kMvlvStateIdle;
      break;
  }
}

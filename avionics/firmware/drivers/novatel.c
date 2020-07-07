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

#include "avionics/firmware/drivers/novatel.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/common/novatel_binary.h"
#include "avionics/common/novatel_serial.h"
#include "avionics/common/novatel_types.h"
#include "avionics/common/serial_parse.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/sci.h"
#include "avionics/firmware/drivers/gps_device.h"
#include "avionics/firmware/drivers/novatel_def.h"
#include "avionics/firmware/drivers/novatel_recv.h"
#include "common/macros.h"

typedef enum {
  kDevStateHardwareReset,
  kDevStatePause0,
  kDevStateSerialBreak1,
  kDevStatePause1,
  kDevStateSerialBreak2,
  kDevStatePause2,
  kDevStateSetCom1BaudRate,
  kDevStatePause3,
  kDevStateSetCom2BaudRate,
  kDevStateSendInitStrings,
  kDevStateIdle
} DevState;

static struct {
  DevState state;
  bool first_entry;
  int32_t index;
  uint32_t received_prompt;
  uint32_t timeout;
} g_init;

static uint8_t g_send[NOVATEL_WRITE_LENGTH];

static void InitDevice(const NovAtelDevice *dev) {
  GpsDeviceInit(&kGps);
  SciInit(dev->this_port, NOVATEL_DEFAULT_BAUD);
  SciInit(dev->other_port, NOVATEL_DEFAULT_BAUD);
  NovAtelReceiveInit();
}

// Perform hardware reset. Receiver requires approximately 8 seconds to resume
// operation.
static void DevStateHardwareReset(const NovAtelDevice *dev, DevState next) {
  uint32_t now = Clock32GetCycles();
  if (g_init.first_entry) {
    InitDevice(dev);
    g_init.timeout = now + NOVATEL_RESET_CYCLES;
    printf("NovAtel GPS reset.\n");
  } else if (CLOCK32_GE(now, g_init.timeout)) {
    GpsDeviceClearReset(&kGps);
    g_init.state = next;
  }
}

// Send serial break to reset the receiver's communication settings (see
// Section 2.5.14).
static void DevStateSerialBreak(const NovAtelDevice *dev, int32_t timeout,
                                DevState next) {
  uint32_t now = Clock32GetCycles();
  if (g_init.first_entry) {
    SciSetBreak(dev->this_port);
    SciSetBreak(dev->other_port);
    g_init.timeout = now + timeout;
  } else if (CLOCK32_GE(now, g_init.timeout)) {
    SciClearBreak(dev->this_port);
    SciClearBreak(dev->other_port);
    g_init.state = next;
  }
}

// Observe required no communication periods.
static void DevStatePause(int32_t timeout, DevState next) {
  uint32_t now = Clock32GetCycles();
  if (g_init.first_entry) {
    g_init.timeout = now + timeout;
  } else if (CLOCK32_GE(now, g_init.timeout)) {
    g_init.state = next;
  }
}

// Switch from default baud rate to operation baud rate.
static void DevStateSetBaudRate(const SciDevice *command_port,
                                const SciDevice *port, NovAtelPort port_id,
                                int32_t baud, DevState pass, DevState fail) {
  assert(command_port != port);  // Do not change port we're speaking on.
  uint32_t now = Clock32GetCycles();
  if (g_init.first_entry) {
    g_init.timeout = now + NOVATEL_REPLY_TIMEOUT_CYCLES;
    // TODO: Switch to SerialConfig after deprecating NovAtel OEM V2-L1.
    SciWrite(command_port,
             NovAtelBinaryWriteCom(port_id, baud, true, g_send), g_send);
  } else if (NovAtelReceivedResponse(kNovAtelMessageIdCom)) {
    SciInit(port, baud);
    g_init.state = pass;
  } else if (CLOCK32_GE(now, g_init.timeout)) {
    g_init.state = fail;
  }
}

static void SendAbbreviatedAscii(const NovAtelDevice *dev, const char *str) {
  g_init.received_prompt = NovAtelGetPromptIndex();
  printf("NovAtel command: %s\n", str);
  SciWriteString(dev->this_port, "\r\n");  // Clear receiver's input buffer.
  SciWriteString(dev->this_port, str);     // Send command.
  SciWriteString(dev->this_port, "\r\n");  // Terminate command.
}

static bool WaitForPrompt(void) {
  return g_init.received_prompt != NovAtelGetPromptIndex();
}

static void DevStateSendInitStrings(const NovAtelDevice *dev, DevState pass,
                                    DevState fail) {
  uint32_t now = Clock32GetCycles();
  if (g_init.first_entry) {
    g_init.index = 0;
  }
  if (g_init.first_entry || WaitForPrompt()) {
    if (g_init.index < dev->num_init_strings) {
      g_init.timeout = now + NOVATEL_REPLY_TIMEOUT_CYCLES;
      SendAbbreviatedAscii(dev, dev->init_string[g_init.index]);
      ++g_init.index;
    } else {
      g_init.state = pass;
    }
  } else if (CLOCK32_GE(now, g_init.timeout)) {
    g_init.state = fail;
  }
}

// Software watchdog to ensure GPS is still streaming data.
static void DevStateWatchdog(int32_t timeout, NovAtelMessageId message_id,
                             DevState next) {
  uint32_t now = Clock32GetCycles();
  if (g_init.first_entry || NovAtelReceivedMessage(message_id)) {
    g_init.timeout = now + timeout;
  } else if (CLOCK32_GE(now, g_init.timeout)) {
    g_init.state = next;
  }
}

// Monitor GPS state.
static void PollState(const NovAtelDevice *dev) {
  // See Section 2.5.14: COM COM port configuration control. This state
  // machine initializes the communication port to a known configuration
  // before issuing commands.
  DevState current = g_init.state;
  switch (current) {
    case kDevStateHardwareReset:
      DevStateHardwareReset(dev, kDevStatePause0);
      break;
    case kDevStatePause0:
      DevStatePause(CLOCK32_MSEC_TO_CYCLES(10000), kDevStateSerialBreak1);
      break;
    case kDevStateSerialBreak1:
      DevStateSerialBreak(dev, CLOCK32_MSEC_TO_CYCLES(250), kDevStatePause1);
      break;
    case kDevStatePause1:
      DevStatePause(CLOCK32_MSEC_TO_CYCLES(1500), kDevStateSerialBreak2);
      break;
    case kDevStateSerialBreak2:
      DevStateSerialBreak(dev, CLOCK32_MSEC_TO_CYCLES(250), kDevStatePause2);
      break;
    case kDevStatePause2:
      DevStatePause(CLOCK32_MSEC_TO_CYCLES(500), kDevStateSetCom1BaudRate);
      break;
    case kDevStateSetCom1BaudRate:
      DevStateSetBaudRate(dev->other_port, dev->this_port, kNovAtelPortCom1All,
                          NOVATEL_COM1_BAUD, kDevStatePause3,
                          kDevStateHardwareReset);
      break;
    case kDevStatePause3:
      DevStatePause(CLOCK32_MSEC_TO_CYCLES(500), kDevStateSetCom2BaudRate);
      break;
    case kDevStateSetCom2BaudRate:
      DevStateSetBaudRate(dev->this_port, dev->other_port, kNovAtelPortCom2All,
                          NOVATEL_COM2_BAUD, kDevStateSendInitStrings,
                          kDevStateHardwareReset);
      break;
    case kDevStateSendInitStrings:
      DevStateSendInitStrings(dev, kDevStateIdle, kDevStateHardwareReset);
      break;
    case kDevStateIdle:
      DevStateWatchdog(CLOCK32_MSEC_TO_CYCLES(10000), kNovAtelMessageIdBestXyz,
                       kDevStateHardwareReset);
      break;
    default:
      g_init.state = kDevStateIdle;
      assert(0);
      break;
  }
  g_init.first_entry = (current != g_init.state);
}

// Initialize to known state.
void NovAtelInit(void) {
  memset(&g_init, 0, sizeof(g_init));
  g_init.state = kDevStateHardwareReset;
  g_init.first_entry = true;
  NovAtelReceiveInit();
}

// Poll for new data.
bool NovAtelPoll(const NovAtelDevice *dev, NovAtelProto *proto) {
  PollState(dev);
  return NovAtelReceivePoll(proto);
}

void NovAtelInsertRtcm(const NovAtelDevice *dev, int32_t length,
                       const uint8_t *data) {
  if (g_init.state == kDevStateIdle) {
    SciWrite(dev->other_port, length, data);
  }
}

bool NovAtelIsReady(void) {
  return g_init.state == kDevStateIdle;
}

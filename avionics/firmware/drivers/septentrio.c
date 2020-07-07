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

#include "avionics/firmware/drivers/septentrio.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/common/septentrio_sbf.h"
#include "avionics/common/septentrio_serial.h"
#include "avionics/common/serial_parse.h"
#include "avionics/firmware/cpu/sci.h"
#include "avionics/firmware/drivers/gps_device.h"
#include "avionics/firmware/drivers/septentrio_def.h"
#include "common/macros.h"

typedef enum {
  kDevStateHardwareReset,
  kDevStateWaitForPrompt,
  kDevStateSetBaudRate,
  kDevStateSendInitStrings,
  kDevStateIdle
} DevState;

static struct {
  DevState state;
  bool first_entry;
  int64_t timeout;
  bool received_prompt;
  int64_t received_prompt_time;
  int32_t index;
  bool watchdog_reset;
} g_init;

// Store receive and parsing state for 'this_port' and 'other_port'.
// 'this_port': Primary serial communications port (used for commands and data).
// 'other_port': Secondary serial communications port (used for RTCM data).
static SerialReceiveBuffer g_this_recv, g_other_recv;
static SeptentrioReceive g_this_parse, g_other_parse;
static SeptentrioReceive *g_last_parsed = NULL;


static void InitDevice(const SeptentrioDevice *dev) {
  GpsDeviceInit(dev->hardware);
  SciInit(dev->this_port, SEPTENTRIO_DEFAULT_BAUD);
  SciInit(dev->other_port, SEPTENTRIO_DEFAULT_BAUD);
  SerialParseInit(&g_this_recv);
  SerialParseInit(&g_other_recv);
  memset(&g_this_parse, 0, sizeof(g_this_parse));
  memset(&g_other_parse, 0, sizeof(g_other_parse));
  g_last_parsed = NULL;
}

// Perform hardware reset.
static void DevStateHardwareReset(const SeptentrioDevice *dev, int64_t now,
                                  DevState next) {
  if (g_init.first_entry) {
    InitDevice(dev);
    g_init.timeout = now + SEPTENTRIO_RESET_US;
    printf("Septentrio GPS reset.\n");
  } else if (now >= g_init.timeout) {
    GpsDeviceClearReset(dev->hardware);
    g_init.state = next;
  }
}

// We should receive the following message after approximately 6 seconds.
// $TE Septentrio SSRC5 SN 3003160 is booting.
// COM1>
static void DevStateWaitForPrompt(int64_t now, DevState next) {
  if (g_init.first_entry) {
    g_init.received_prompt = false;
    g_init.timeout = now + SEPTENTRIO_RESET_RECOVERY_US;
  } else if (g_init.received_prompt) {
    g_init.state = next;
  } else if (now >= g_init.timeout) {
    g_init.state = kDevStateHardwareReset;
  }
}

// Set baud rates for all COM ports.
static void DevStateSetBaudRate(const SeptentrioDevice *dev, int64_t now,
                                DevState next) {
  if (g_init.first_entry) {
    SciWriteString(dev->this_port,
                   "setCOMSettings,all,baud" STR(SEPTENTRIO_DESIRED_BAUD) "\n");
    g_init.index = 0;
    g_init.received_prompt = false;
    g_init.timeout = now + SEPTENTRIO_REPLY_TIMEOUT_US;
  } else if (g_init.index == 0 && !SciIsWriteBusy(dev->this_port)) {
    SciInit(dev->this_port, SEPTENTRIO_DESIRED_BAUD);
    SciInit(dev->other_port, SEPTENTRIO_DESIRED_BAUD);
    g_init.index = 1;
  } else if (g_init.index == 1 && g_init.received_prompt) {
    g_init.state = next;
  } else if (now >= g_init.timeout) {
    g_init.state = kDevStateHardwareReset;
  }
}

// Send ASCII serial data and wait for reply.
static bool ReadyToSendCommand(const SeptentrioDevice *dev, int64_t now) {
  return !SciIsWriteBusy(dev->this_port) && g_init.received_prompt
      && now - g_init.received_prompt_time > SEPTENTRIO_REPLY_DELAY_US;
}

// Send all initialization strings.
static void DevStateSendInitStrings(const SeptentrioDevice *dev, int64_t now,
                                    DevState next) {
  if (g_init.first_entry) {
    g_init.index = 0;
  }
  if (g_init.first_entry || ReadyToSendCommand(dev, now)) {
    if (g_init.index < dev->num_init_strings) {
      g_init.received_prompt = false;
      g_init.timeout = now + SEPTENTRIO_REPLY_TIMEOUT_US;
      SciWriteString(dev->this_port, dev->init_string[g_init.index]);
      ++g_init.index;
    } else {
      g_init.state = next;
    }
  } else if (now >= g_init.timeout) {
    g_init.state = kDevStateHardwareReset;
  }
}

// Software watchdog to ensure the GPS is still working.
static void DevStateWatchdog(int64_t now, int64_t timeout, DevState next) {
  if (g_init.first_entry || g_init.watchdog_reset) {
    g_init.timeout = now + timeout;
    g_init.watchdog_reset = false;
  } else if (now >= g_init.timeout) {
    g_init.state = next;
  }
}

static void PollDevState(const SeptentrioDevice *dev, int64_t now) {
  do {
    DevState current = g_init.state;
    switch (current) {
      case kDevStateHardwareReset:
        DevStateHardwareReset(dev, now, kDevStateWaitForPrompt);
        break;
      case kDevStateWaitForPrompt:
        DevStateWaitForPrompt(now, kDevStateSetBaudRate);
        break;
      case kDevStateSetBaudRate:
        DevStateSetBaudRate(dev, now, kDevStateSendInitStrings);
        break;
      case kDevStateSendInitStrings:
        DevStateSendInitStrings(dev, now, kDevStateIdle);
        break;
      case kDevStateIdle:
        DevStateWatchdog(now, 1000000, kDevStateSendInitStrings);
        break;
      default:
        g_init.state = kDevStateIdle;
        assert(false);
        break;
    }
    g_init.first_entry = (current != g_init.state);
  } while (g_init.first_entry);
}

static void DebugAscii(const SeptentrioReceive *parsed) {
  static char msg[SERIAL_RECEIVE_SIZE];
  int32_t length = parsed->data_length;
  if (length >= SERIAL_RECEIVE_SIZE - 1) {
    length = SERIAL_RECEIVE_SIZE - 1;
  }
  memcpy(msg, parsed->data, length);
  msg[length] = '\0';
  printf("%s\n", msg);
}

static bool PollReceive(int64_t now, SeptentrioProto *proto) {
  bool output;

  if (g_last_parsed == &g_this_parse) {
    g_last_parsed = &g_other_parse;
    output = SeptentrioParse(&g_other_recv, &g_other_parse);
  } else {
    g_last_parsed = &g_this_parse;
    output = SeptentrioParse(&g_this_recv, &g_this_parse);
    if (output && g_this_parse.proto == kSeptentrioProtoAscii) {
      DebugAscii(&g_this_parse);
      g_init.received_prompt = SeptentrioGotPrompt(&g_this_parse);
      if (g_init.received_prompt) {
        g_init.received_prompt_time = now;
      }
    }
  }
  *proto = g_last_parsed->proto;
  g_init.watchdog_reset |= output;
  return output;
}

void SeptentrioInit(void) {
  memset(&g_init, 0, sizeof(g_init));
  g_init.state = kDevStateHardwareReset;
  g_init.first_entry = true;
}

bool SeptentrioPoll(const SeptentrioDevice *dev, int64_t now,
                    SeptentrioProto *proto) {
  PollDevState(dev, now);
  return PollReceive(now, proto);
}

bool SeptentrioGetSbf(const SeptentrioHeader **hdr, SeptentrioBlock *blk) {
  assert(hdr != NULL);
  assert(blk != NULL);

  if ((g_last_parsed == &g_this_parse || g_last_parsed == &g_other_parse)
      && g_last_parsed->proto == kSeptentrioProtoSbf) {
    *hdr = &g_last_parsed->sbf.header;
    return SeptentrioSbfDecode(*hdr, g_last_parsed->data, blk);
  }
  return false;
}

bool SeptentrioGetRtcm3(uint16_t *message_number, int32_t *length,
                        const uint8_t **data) {
  assert(message_number != NULL);
  assert(length != NULL);
  assert(data != NULL);

  if ((g_last_parsed == &g_this_parse || g_last_parsed == &g_other_parse)
      && g_last_parsed->proto == kSeptentrioProtoRtcm3) {
    *message_number = g_last_parsed->rtcm3.message_number;
    *length = g_last_parsed->data_length;
    *data = g_last_parsed->data;
    return true;
  }
  return false;
}

void SeptentrioInsertDataFromThisPort(int32_t length, const uint8_t *data) {
  SerialReadData(length, data, &g_this_recv);
}

void SeptentrioInsertDataFromOtherPort(int32_t length, const uint8_t *data) {
  SerialReadData(length, data, &g_other_recv);
}

void SeptentrioInsertRtcm(const SeptentrioDevice *dev, int32_t length,
                          const uint8_t *data) {
  if (g_init.state == kDevStateIdle) {
    SciWrite(dev->other_port, length, data);
  }
}

bool SeptentrioIsReady(void) {
  return g_init.state == kDevStateIdle;
}

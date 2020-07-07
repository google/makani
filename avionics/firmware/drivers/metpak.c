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

#include "avionics/firmware/drivers/metpak.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/gill_ascii.h"
#include "avionics/common/gill_serial.h"
#include "avionics/common/gill_types.h"
#include "avionics/common/serial_parse.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/sci.h"
#include "common/macros.h"

#define METPAK_BAUD 19200

// Manual specifies 9.5 seconds.
#define METPAK_POWER_UP_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(9500)
#define METPAK_REPLY_TIMEOUT_CYCLES    CLOCK32_MSEC_TO_CYCLES(100)
#define METPAK_WATCHDOG_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(5000)
#define METPAK_VERIFY_TRIES 3

static const char *kInitStrings[] = {
  "ASCTERM CRLF\n",
  "ECHO OFF\n",
  "REPORT FULL\n",
  "UNITS WIND MS\n",
  "UNITS PRESS HPA\n",
  "UNITS TEMP C\n",
  "UNITS DEWPOINT C\n",
  "UNITS RH %\n",
  "OUTFREQ 1HZ\n",
  "MSGMODE CONT\n",
};

typedef enum {
  kDevStatePowerUpDelay,
  kDevStateResetMode,
  kDevStateEnterSetupMode,
  kDevStateSendInitStrings,
  kDevStateVerifyReport,
  kDevStateVerifyUnits,
  kDevStateLeaveSetupMode,
  kDevStateWatchdog
} DevState;

static struct {
  DevState state;
  bool first_entry;
  uint32_t timeout;
  int32_t index;
  bool setup_mode;
  bool received_prompt;
  bool received_reply;
  bool verified_report;
  bool verified_units;
  bool watchdog_reset;
} g_init;

static SerialReceiveBuffer g_recv;
static GillReceive g_parse;

static void DevStatePowerUpDelay(const MetPakConfig *config, DevState next) {
  if (g_init.first_entry) {
    g_init.timeout = Clock32GetCycles() + METPAK_POWER_UP_TIMEOUT_CYCLES;
    g_init.watchdog_reset = false;
    SerialParseInit(&g_recv);
    SciInit(config->device, METPAK_BAUD);
  } else if (CLOCK32_GE(Clock32GetCycles(), g_init.timeout)) {
    g_init.state = next;
  }
}

// Make sure we're not already in SETUP MODE.
static void DevStateResetMode(const MetPakConfig *config, DevState next) {
  if (g_init.first_entry) {
    g_init.timeout = Clock32GetCycles() + METPAK_REPLY_TIMEOUT_CYCLES;
    SciWriteString(config->device, "EXIT\n");
  } else if (CLOCK32_GE(Clock32GetCycles(), g_init.timeout)) {
    g_init.state = next;
  }
}

static void DevStateEnterSetupMode(const MetPakConfig *config, DevState next) {
  if (g_init.first_entry) {
    g_init.timeout = Clock32GetCycles() + METPAK_REPLY_TIMEOUT_CYCLES;
    g_init.setup_mode = false;
    g_init.received_prompt = false;
    SciWriteString(config->device, "*Q\n");
  } else if (g_init.setup_mode && g_init.received_prompt) {
    g_init.state = next;
  } else if (CLOCK32_GE(Clock32GetCycles(), g_init.timeout)) {
    g_init.state = kDevStatePowerUpDelay;
  }
}

static void DevStateSendInitStrings(const MetPakConfig *config, DevState next) {
  if (g_init.first_entry) {
    g_init.index = 0;
  }
  if (g_init.first_entry || g_init.received_reply || g_init.received_prompt
      || CLOCK32_GE(Clock32GetCycles(), g_init.timeout)) {
    if (g_init.index < ARRAYSIZE(kInitStrings)) {
      g_init.received_reply = false;
      g_init.received_prompt = false;
      g_init.timeout = Clock32GetCycles() + METPAK_REPLY_TIMEOUT_CYCLES;
      SciWriteString(config->device, kInitStrings[g_init.index]);
      ++g_init.index;
    } else {
      g_init.state = next;
    }
  }
}

// Argument 'command' corresponds to one of MetPak's commands (i.e., REPORT or
// UNITS).
static void DevStateVerify(const MetPakConfig *config, const char *command,
                           DevState next, bool *response_verified) {
  assert(command != NULL);
  assert(response_verified != NULL);

  if (g_init.first_entry) {
    g_init.index = 0;
    g_init.received_reply = false;
    *response_verified = false;
  }
  if (*response_verified) {
    g_init.state = next;
  } else if (g_init.index > METPAK_VERIFY_TRIES) {
    // Device fails to respond.
    g_init.state = kDevStatePowerUpDelay;
  } else if (g_init.first_entry
             || CLOCK32_GE(Clock32GetCycles(), g_init.timeout)) {
    // Send command.
    g_init.timeout = Clock32GetCycles() + METPAK_REPLY_TIMEOUT_CYCLES;
    g_init.received_reply = false;
    SciWriteString(config->device, command);
    SciWriteString(config->device, "\n");
    ++g_init.index;  // Retry counter.
  } else if (g_init.received_reply) {
    // Device reports unexpected string.
    g_init.state = kDevStatePowerUpDelay;
  }
}

static void DevStateLeaveSetupMode(const MetPakConfig *config, DevState next) {
  SciWriteString(config->device, "EXIT\n");
  g_init.setup_mode = false;
  g_init.state = next;
}

static void DevStateWatchdog(uint32_t timeout, DevState next) {
  if (g_init.first_entry || g_init.watchdog_reset) {
    g_init.timeout = Clock32GetCycles() + timeout;
    g_init.watchdog_reset = false;
  } else if (CLOCK32_GE(Clock32GetCycles(), g_init.timeout)) {
    g_init.state = next;
  }
}

static void DevPollState(const MetPakConfig *config) {
  do {
    DevState current = g_init.state;
    switch (current) {
      case kDevStatePowerUpDelay:
        DevStatePowerUpDelay(config, kDevStateResetMode);
        break;
      case kDevStateResetMode:
        DevStateResetMode(config, kDevStateEnterSetupMode);
        break;
      case kDevStateEnterSetupMode:
        DevStateEnterSetupMode(config, kDevStateSendInitStrings);
        break;
      case kDevStateSendInitStrings:
        DevStateSendInitStrings(config, kDevStateVerifyReport);
        break;
      case kDevStateVerifyReport:
        DevStateVerify(config, "REPORT", kDevStateVerifyUnits,
                       &g_init.verified_report);
        break;
      case kDevStateVerifyUnits:
        DevStateVerify(config, "UNITS", kDevStateLeaveSetupMode,
                       &g_init.verified_units);
        break;
      case kDevStateLeaveSetupMode:
        DevStateLeaveSetupMode(config, kDevStateWatchdog);
        break;
      case kDevStateWatchdog:
        DevStateWatchdog(METPAK_WATCHDOG_TIMEOUT_CYCLES,
                         kDevStatePowerUpDelay);
        break;
      default:
        g_init.state = kDevStatePowerUpDelay;
        assert(false);
        break;
    }
    g_init.first_entry = (current != g_init.state);
  } while (g_init.first_entry);
}

static bool DecodeSetupMode(int32_t length, const uint8_t *data) {
  assert(data != NULL);
  assert(length >= 0);

  const char *token = "SETUP MODE";
  int32_t token_length = (int32_t)strlen(token);
  if (length == token_length && memcmp(data, token, token_length) == 0) {
    g_init.setup_mode = true;
    g_init.received_reply = true;
    return true;
  }
  return false;
}

static bool DecodeReport(int32_t length, const uint8_t *data) {
  assert(data != NULL);
  assert(length >= 0);

  const char *token = "REPORT = ";
  int32_t token_length = (int32_t)strlen(token);
  if (length >= token_length && memcmp(data, token, token_length) == 0) {
    const char *expect = "REPORT = NODE,DIR,SPEED,W-AXIS,PRESS,RH,TEMP,"
        "DEWPOINT,VOLT,STATUS,CHECK";
    int32_t expect_length = (int32_t)strlen(expect);
    g_init.verified_report =
        (length == expect_length && memcmp(data, expect, expect_length) == 0);
    g_init.received_reply = true;
    return true;
  }
  return false;
}

static bool DecodeUnits(int32_t length, const uint8_t *data) {
  assert(data != NULL);
  assert(length >= 0);

  const char *token = "UNITS = ";
  int32_t token_length = (int32_t)strlen(token);
  if (length >= token_length && memcmp(data, token, token_length) == 0) {
    const char *expect = "UNITS = -,DEG,MS,MS,HPA,%,C,C,V,-,-";
    int32_t expect_length = (int32_t)strlen(expect);
    g_init.verified_units =
        (length == expect_length && memcmp(data, expect, expect_length) == 0);
    g_init.received_reply = true;
    return true;
  }
  return false;
}

static void MetPakLineDecode(int32_t length, const uint8_t *data) {
  if (DecodeSetupMode(length, data)
      || DecodeReport(length, data)
      || DecodeUnits(length, data)) {
  }
}

static bool PollNewMessage(const MetPakConfig *config, GillData *out) {
  bool new_message = false;
  if (SerialPollSci(config->device, &g_recv) && GillParse(&g_recv, &g_parse)) {
    g_init.watchdog_reset = true;
    switch (g_parse.proto) {
      case kGillProtoAscii:
        new_message = GillAsciiDecodeMetPak(&g_parse.ascii, g_parse.data, out);
        break;
      case kGillProtoLine:
        MetPakLineDecode(g_parse.data_length, g_parse.data);
        break;
      case kGillProtoPrompt:
        g_init.received_prompt = true;
        break;
      case kGillProtoNmea:
        // Output not supported, use kGillProtoAscii.
        break;
      default:
        break;
    }
  }
  return new_message;
}

void MetPakInit(void) {
  memset(&g_init, 0, sizeof(g_init));
  g_init.state = kDevStatePowerUpDelay;
  g_init.first_entry = true;
}

bool MetPakPoll(const MetPakConfig *config, GillData *out) {
  DevPollState(config);
  return PollNewMessage(config, out);
}

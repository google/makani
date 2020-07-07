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

#include "avionics/firmware/drivers/windmaster.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/gill_ascii.h"
#include "avionics/common/gill_binary.h"
#include "avionics/common/gill_serial.h"
#include "avionics/common/gill_types.h"
#include "avionics/common/serial_parse.h"
#include "avionics/common/strings.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/sci.h"
#include "common/macros.h"

#define WIND_DEFAULT_BAUD 19200
#define WIND_INIT_TIMEOUT_CYCLES     CLOCK32_MSEC_TO_CYCLES(10000)
#define WIND_POST_TIMEOUT_CYCLES     CLOCK32_MSEC_TO_CYCLES(10000)
#define WIND_REPLY_TIMEOUT_CYCLES    CLOCK32_MSEC_TO_CYCLES(1000)
#define WIND_WATCHDOG_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(5000)

// Configuration. See Gill Windmaster documentation.
// Note: ** fields specify ASCII output format. See kGillWindmasterField.
// Baud rate (Bx) changes are not supported.
static const char *kInitStrings[] = {
  "A2",   // Select speed of sound output [m/s] (**).
  "C2",   // Select 360 degree wrap-around on analogue output.
  "E1",   // Select physical communications to auto.
  "G0",   // Select averaging disabled.
  "H1",   // Select power on message (required).
  "I1",   // Select analog input data off (**).
  "J2",   // Select high resolution.
  "K50",  // Select minimum wind direction velocity [mm/s].
  "L1",   // Select message terminator.
  "M1",   // Select message format.
  "O1",   // Select ASCII output format.
  "P8",   // Select output rate.
  "U1",   // Select units.
  "X1",   // Select axis alignment (see Figure 2 "U,V,W Axis Definition").
};

// Specified to correspond to Bx command, where x = index + 1.
static const int32_t kBaudRates[] = {
  2400, 4800, 9600, 19200, 38400, 57600
};

typedef enum {
  kDevStateInit,
  kDevStateTrySetConfigModeFromContinuous,
  kDevStateTryGetIdentifier,
  kDevStateTrySetConfigModeFromPolled,
  kDevStateTrySetMeasurementMode,
  kDevStateTryNextBaudRate,
  kDevStateGetConfig,
  kDevStateSetConfig,
  kDevStateSetMeasurementMode,
  kDevStateMeasurementMode
} DevState;

// Power-on self-test messages to expect when entering measurement mode.
typedef enum {
  kPostFieldWindMaster,   // WindMaster 20Hz Gill Instruments Ltd.
  kPostFieldChecksumRom,  // CHECKSUM ROM:9B50 9B50 *PASS*.
  kPostFieldChecksumFac,  // CHECKSUM FAC:07BC 07BC *PASS*.
  kPostFieldChecksumEng,  // CHECKSUM ENG:1E58 1E58 *PASS*.
  kPostFieldChecksumCal,  // CHECKSUM CAL:3891 3891 *PASS*.
  kNumPostFields
} PostField;

static struct {
  DevState state;          // Current state machine state.
  bool first_entry;        // True upon changing to a new state.
  int64_t timeout;         // State processing timeout.
  char identifier;         // Windmaster node identifier (default Q).
  const char *set_config;  // Next config string to send (may be NULL).
  int32_t baud_index;      // Baud rate index (Bx), where x = index + 1.
  int32_t baud_tries;      // Number of acquisition tries at current baud rate.
  uint32_t post_bitmask;   // Power-on self test results. See PostField.
} g_init;

static SerialReceiveBuffer g_recv;
static GillReceive g_parse;

// Parse response to "D3" command to validate each configuration option with
// those specified in kConfig. Return the desired configuration value for the
// first non-matching configuration option, or NULL if all configuration
// options match (or aren't specified). The expected response looks like:
// M2,U1,O1,L1,P1,B4,H1,NQ,E1,T1,S1,C2,A1,I1,J1,V1,X1,G0,K50.
static const char *GetNextConfig(int32_t length, const char *status) {
  int32_t start = 0;
  int32_t delim = 0;
  while (delim < length) {
    while (delim < length && status[delim] != ',') {
      ++delim;
    }
    for (int32_t i = 0; i < ARRAYSIZE(kInitStrings); ++i) {
      const char *str = kInitStrings[i];
      if (status[start] == str[0]
          && strncmp(str, &status[start], delim - start) != 0) {
        return str;  // Send this configuration string.
      }
    }
    ++delim;
    start = delim;
  }
  return NULL;  // All configuration strings match.
}

// Index corresponds to Bx command, where x = index + 1.
static int32_t LookupBaudRateIndex(int32_t baud) {
  for (int32_t i = 0; i < ARRAYSIZE(kBaudRates); ++i) {
    if (kBaudRates[i] == baud) {
      return i;
    }
  }
  return 0;
}

// This device implements a half duplex communication protocol, where if we
// transmit during a reply, it will stop transmitting data. The SCI driver
// handles the particulars of half duplex communication. Here, we clear
// the remainder of our command after we've received a properly formatted
// response to ensure correct operation.
static bool PollDevice(const WindmasterConfig *config, GillReceive *parse) {
  if (SciPoll(config->device)                    // Poll serial hardware device.
      && SerialPollSci(config->device, &g_recv)  // Copy receive data to parser.
      && GillParse(&g_recv, parse)) {            // Parse receive data.
    // Stop transmitting data once we've received a properly formatted message.
    if (parse->proto == kGillProtoLine) {
      SciClearWrite(config->device);
    }
    return true;
  }
  return false;
}

// Helper function for sending a command and expecting an ASCII response.
typedef void (* const SendFunction)(const SciDevice *device);
typedef bool (* const ReplyFunction)(int32_t length, const uint8_t *data);
static void CommandResponse(const WindmasterConfig *config,
                            SendFunction send_fn, ReplyFunction reply_fn,
                            DevState pass, DevState fail, uint32_t timeout) {
  if (g_init.first_entry) {
    send_fn(config->device);
    g_init.timeout = Clock32GetCycles() + timeout;
  }
  if (PollDevice(config, &g_parse) && g_parse.proto == kGillProtoLine
      && reply_fn(g_parse.data_length, g_parse.data)) {
    g_init.state = pass;
  } else if (CLOCK32_GE(Clock32GetCycles(), g_init.timeout)) {
    g_init.state = fail;
  }
}

// Used as parameter to CommandResponse. Device should respond with a single
// ASCII character, A-Z, indicating the node identifier.
static void SendGetIdentifier(const SciDevice *device) {
  SciWriteString(device, "\r\n&\r\n");
}

// Used as parameter to CommandResponse(SendGetIdentifier).
static bool ParseIdentifier(int32_t length, const uint8_t *data) {
  if (length == 1 && 'A' <= data[0] && data[0] <= 'Z') {
    g_init.identifier = (char)data[0];
    return true;
  }
  return false;
}

// Used as parameter to CommandResponse. Device should respond with
// "CONFIGURATION MODE" from polled mode.
static void SendSetConfigModeFromPolled(const SciDevice *device) {
  char str[] = {'\r', '\n', '*', g_init.identifier, '\r', '\n', 0};
  SciWriteString(device, str);
}

// Used as parameter to CommandResponse. Device should respond with
// "CONFIGURATION MODE" from continuous mode.
static void SendSetConfigModeFromContinuous(const SciDevice *device) {
  SciWriteString(device, "\r\n*\r\n");
}

// Used as parameter to CommandResponse(SendSetConfigModeFromPolled) or
// CommandResponse(SendSetConfigModeFromContinuous).
static bool ParseConfigMode(int32_t length, const uint8_t *data) {
  return length == 18
      && strncmp("CONFIGURATION MODE", (const char *)data, length) == 0;
}

// Used as parameter to CommandResponse. Device should respond with power-on
// self test (POST) results and begin transmitting data (in continuous mode).
// Make certain config Hx is set to enable the power-on message (H1).
// WindMaster 20Hz Gill Instruments Ltd
// 2329-601-03
// RS485 (AUTO)
// CHECKSUM ROM:9B50 9B50 *PASS*
// CHECKSUM FAC:07BC 07BC *PASS*
// CHECKSUM ENG:1E58 1E58 *PASS*
// CHECKSUM CAL:3891 3891 *PASS*
static void SendSetMeasurementMode(const SciDevice *device) {
  g_init.post_bitmask = 0x0;
  SciWriteString(device, "\r\nQ\r\n");
}

// Helper function to identify "*PASS*" in the power-on self test results.
static bool ParsePostForPass(int32_t post_len, const char *post,
                             const char *name) {
  int32_t name_len = strlen(name);
  return FindString(name_len, post, name_len, name) != NULL
      && FindString(post_len - name_len, &post[name_len], 6, "*PASS*") != NULL;
}

// Used as parameter to CommandResponse(SetMeasurementMode).
static bool ParseMeasurementMode(int32_t length, const uint8_t *data) {
  const char *str = (const char *)data;

  if (FindString(length, str, 10, "WindMaster") != NULL) {
    g_init.post_bitmask |= 1 << kPostFieldWindMaster;
  }
  if (ParsePostForPass(length, str, "CHECKSUM ROM:")) {
    g_init.post_bitmask |= 1 << kPostFieldChecksumRom;
  }
  if (ParsePostForPass(length, str, "CHECKSUM FAC:")) {
    g_init.post_bitmask |= 1 << kPostFieldChecksumFac;
  }
  if (ParsePostForPass(length, str, "CHECKSUM ENG:")) {
    g_init.post_bitmask |= 1 << kPostFieldChecksumEng;
  }
  if (ParsePostForPass(length, str, "CHECKSUM CAL:")) {
    g_init.post_bitmask |= 1 << kPostFieldChecksumCal;
  }
  return g_init.post_bitmask == (1 << kNumPostFields) - 1;
}

// Used as parameter to CommandResponse. Device responds with comma deliminated
// configuration string.
static void SendGetConfig(const SciDevice *device) {
  SciWriteString(device, "\r\nD3\r\n");
}

// Used as parameter to CommandResponse(SendGetConfig). Verify the device
// configuration matches that specified here. The expected response looks like:
// M2,U1,O1,L1,P1,B4,H1,NQ,E1,T1,S1,C2,A1,I1,J1,V1,X1,G0,K50.
static bool ParseGetConfig(int32_t length, const uint8_t *data) {
  if (FindString(length, (const char *)data, 1, ",") != NULL) {
    g_init.set_config = GetNextConfig(length, (const char *)data);
    return true;
  } else {
    g_init.set_config = NULL;
    return false;
  }
}

// Used as parameter to CommandResponse.
static void SendSetConfig(const SciDevice *device) {
  SciWriteString(device, "\r\n");
  SciWriteString(device, g_init.set_config);
  SciWriteString(device, "\r\n");
}

// Used as parameter to CommandResponse(SendSetConfig).
static bool ParseSetConfig(int32_t length, const uint8_t *data) {
  return (int32_t)strlen(g_init.set_config) == length
      && strncmp(g_init.set_config, (const char *)data, length);
}

static void DevStateInit(const WindmasterConfig *config, DevState next) {
  if (g_init.first_entry) {
    SciInit(config->device, WIND_DEFAULT_BAUD);
    g_init.baud_index = LookupBaudRateIndex(WIND_DEFAULT_BAUD);
    g_init.identifier = 'Q';  // Q = hardware default.
    g_init.timeout = Clock32GetCycles() + WIND_INIT_TIMEOUT_CYCLES;
  } else if (CLOCK32_GE(Clock32GetCycles(), g_init.timeout)) {
    g_init.state = next;
  }
}

static void DevStateTryNextBaudRate(const WindmasterConfig *config,
                                    DevState next) {
  ++g_init.baud_tries;
  if (g_init.baud_tries < 0 || 5 <= g_init.baud_tries) {
    g_init.baud_tries = 0;
    g_init.baud_index = (g_init.baud_index + 1) % ARRAYSIZE(kBaudRates);
    SciInit(config->device, kBaudRates[g_init.baud_index]);
  }
  g_init.state = next;
}

static bool DevStateMeasurementMode(const WindmasterConfig *config,
                                    GillData *out) {
  bool new_data = false;
  uint32_t now = Clock32GetCycles();
  if (g_init.first_entry) {
    g_init.timeout = now + WIND_WATCHDOG_TIMEOUT_CYCLES;
  } else if (PollDevice(config, &g_parse)) {
    if (g_parse.proto == kGillProtoAscii) {
      g_init.timeout = now + WIND_WATCHDOG_TIMEOUT_CYCLES;
      new_data = GillAsciiDecodeWindmasterUvw(&g_parse.ascii, g_parse.data,
                                              out);
    } else if (g_parse.proto == kGillProtoBinary) {
      g_init.timeout = now + WIND_WATCHDOG_TIMEOUT_CYCLES;
      new_data = GillBinaryDecodeWindmaster(&g_parse.binary, g_parse.data, out);
    }
  } else if (CLOCK32_GE(now, g_init.timeout)) {
    g_init.state = kDevStateInit;
  }
  return new_data;
}

void WindmasterInit(void) {
  memset(&g_init, 0, sizeof(g_init));
  g_init.state = kDevStateInit;
  g_init.first_entry = true;
  SerialParseInit(&g_recv);
}

bool WindmasterPoll(const WindmasterConfig *config, GillData *out) {
  bool new_data = false;
  DevState current = g_init.state;
  switch (current) {
    case kDevStateInit:
      DevStateInit(config, kDevStateTrySetMeasurementMode);
      break;
    case kDevStateTrySetMeasurementMode:
      CommandResponse(config, SendSetMeasurementMode, ParseMeasurementMode,
                      kDevStateTrySetConfigModeFromContinuous,
                      kDevStateTrySetConfigModeFromPolled,
                      WIND_POST_TIMEOUT_CYCLES);
      break;
    case kDevStateTrySetConfigModeFromContinuous:
      CommandResponse(config, SendSetConfigModeFromContinuous, ParseConfigMode,
                      kDevStateGetConfig, kDevStateTryGetIdentifier,
                      WIND_REPLY_TIMEOUT_CYCLES);
      break;
    case kDevStateTryGetIdentifier:
      CommandResponse(config, SendGetIdentifier, ParseIdentifier,
                      kDevStateTrySetConfigModeFromPolled,
                      kDevStateTryNextBaudRate, WIND_REPLY_TIMEOUT_CYCLES);
      break;
    case kDevStateTrySetConfigModeFromPolled:
      CommandResponse(config, SendSetConfigModeFromPolled, ParseConfigMode,
                      kDevStateGetConfig, kDevStateTryNextBaudRate,
                      WIND_REPLY_TIMEOUT_CYCLES);
      break;
    case kDevStateTryNextBaudRate:
      DevStateTryNextBaudRate(config, kDevStateTrySetMeasurementMode);
      break;
    case kDevStateGetConfig:
      CommandResponse(config, SendGetConfig, ParseGetConfig, kDevStateSetConfig,
                      kDevStateInit, WIND_REPLY_TIMEOUT_CYCLES);
      break;
    case kDevStateSetConfig:
      if (g_init.set_config != NULL) {
        CommandResponse(config, SendSetConfig, ParseSetConfig,
                        kDevStateGetConfig, kDevStateInit,
                        WIND_REPLY_TIMEOUT_CYCLES);
      } else {
        g_init.state = kDevStateSetMeasurementMode;
      }
      break;
    case kDevStateSetMeasurementMode:
      CommandResponse(config, SendSetMeasurementMode, ParseMeasurementMode,
                      kDevStateMeasurementMode, kDevStateInit,
                      WIND_POST_TIMEOUT_CYCLES);
      break;
    case kDevStateMeasurementMode:
      new_data = DevStateMeasurementMode(config, out);
      break;
    default:
      g_init.state = kDevStateInit;
      assert(false);
      break;
  }
  g_init.first_entry = (current != g_init.state);
  return new_data;
}

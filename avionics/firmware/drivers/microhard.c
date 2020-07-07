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

#include "avionics/firmware/drivers/microhard.h"

#include <assert.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avionics/common/strings.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/cpu/spi_pin.h"
#include "common/macros.h"

static int32_t kMicrohardTimeout = CLOCK32_MSEC_TO_CYCLES(1000);
static int32_t kMicrohardPollInterval = CLOCK32_MSEC_TO_CYCLES(1000);
static int32_t kMicrohardLoginTimeout = CLOCK32_MSEC_TO_CYCLES(5000);

// Datasheet can be obtained by registering at
// http://www.microhardcorp.com/pDDL.php.
static const char *kMicrohardConfigCommands[] = {
  "AT+MWRADIO",
  "AT+MWTXPOWER",
  "AT+MWDISTANCE",
  "AT+MWBAND",
  "AT+MWFREQ",
  "AT+MWRXDIV",
  "AT+MWVMODE",
  "AT+MWVRATE",
  "AT+MWEXTADDR",
  "AT+MWNETWORKID",
};

#define MICROHARD_DEBUG  0
// Helpful debug outputs.
static void DebugPrint(int32_t level, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
static void DebugPrint(int32_t level, const char *fmt, ...) {
  if (level > MICROHARD_DEBUG) {
    return;
  }
  printf("mh: ");
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}

static void DebugPrintCont(int32_t level, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
static void DebugPrintCont(int32_t level, const char *fmt, ...) {
  if (level > MICROHARD_DEBUG) {
    return;
  }
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}

static bool MicrohardSetInitValue(const Microhard *microhard,
                                  const char *command,
                                  ssize_t value_len, char *value) {
  const MicrohardParams *params = microhard->data->params;
  ssize_t print_len;

  if (!strcmp("AT+MWRADIO", command)) {
    print_len = snprintf(value, value_len, "%d", params->enabled);
  } else if (!strcmp("AT+MWTXPOWER", command)) {
    print_len = snprintf(value, value_len, "%d", params->tx_power);
  } else if (!strcmp("AT+MWDISTANCE", command)) {
    print_len = snprintf(value, value_len, "%d", params->distance);
  } else if (!strcmp("AT+MWBAND", command)) {
    print_len = snprintf(value, value_len, "%d", params->bandwidth);
  } else if (!strcmp("AT+MWFREQ", command)) {
    // Per the Web UI (which conflicts with the datasheet:
    // Ch 4 is 2405 MHz.  Each channel is an increment of 1 MHz.  Highest
    // channel/frequency is 79/2479
    assert(params->freq >= 2405);
    assert(params->freq <= 2479);
    print_len = snprintf(value, value_len, "%d", 4 + params->freq - 2405);
  } else if (!strcmp("AT+MWRXDIV", command)) {
    print_len = snprintf(value, value_len, "%d", params->rx_diversity);
  } else if (!strcmp("AT+MWVMODE", command)) {
    print_len = snprintf(value, value_len, "%d", params->mode);
  } else if (!strcmp("AT+MWVRATE", command)) {
    print_len = snprintf(value, value_len, "%d", params->tx_rate);
  } else if (!strcmp("AT+MWEXTADDR", command)) {
    print_len = snprintf(value, value_len, "%d", params->extended_addressing);
  } else if (!strcmp("AT+MWNETWORKID", command)) {
    print_len = snprintf(value, value_len, "%s", params->network_id);
  } else {
    return false;
  }

  if (print_len > value_len) {
    return false;  // Overflow.
  }

  return true;
}

static void MicrohardClearResponse(const Microhard *microhard) {
  microhard->data->response[0] = '\0';
  microhard->data->response_len = 0;
}

static bool MicrohardAppendResponse(const Microhard *microhard, char c) {
  if (microhard->data->response_len
      >= ARRAYSIZE(microhard->data->response) - 1) {
    return false;
  }
  if (microhard->data->response_len == 0) {
    DebugPrint(1, "< ");
  }
  if (c != '\r') {
    DebugPrintCont(1, "%c", c);
  }
  microhard->data->response[microhard->data->response_len] = c;
  ++microhard->data->response_len;
  microhard->data->response[microhard->data->response_len] = '\0';
  return true;
}

static bool MicrohardResponseContains(const Microhard *microhard,
                                      const char *token) {
  size_t token_len = strlen(token);
  size_t response_len = strlen(microhard->data->response);

  if (response_len < token_len) {
    return false;
  }

  return strstr(microhard->data->response, token) != NULL;
}

static bool MicrohardResponseEndsWith(const Microhard *microhard,
                                      const char *token) {
  size_t token_len = strlen(token);
  size_t response_len = strlen(microhard->data->response);

  if (response_len < token_len) {
    return false;
  }

  const char *response_offset =
      microhard->data->response + response_len - token_len;
  return !strcmp(response_offset, token);
}

static bool MicrohardIsMacAddr(const char *s) {
  // -Wall gives a warning if a signed char is passed to isxdigit.
  const unsigned char *str = (const unsigned char *)s;
  return isxdigit(str[0]) && isxdigit(str[1]) && str[2] == ':'
      && isxdigit(str[3]) && isxdigit(str[4]) && str[5] == ':'
      && isxdigit(str[6]) && isxdigit(str[7]) && str[8] == ':'
      && isxdigit(str[9]) && isxdigit(str[10]) && str[11] == ':'
      && isxdigit(str[12]) && isxdigit(str[13]) && str[14] == ':'
      && isxdigit(str[15]) && isxdigit(str[16]);
}

// Parses the response buffer (g_response) and returns a pointer to the string
// representing the useful part of the response.  Modifies g_response.
static const char *MicrohardParseResponse(const Microhard *microhard) {
  // Microhard's responses to AT queries are quite varied in format:
  //
  // (1)  joystick> AT+MWBAND=6
  //      ERROR: Invalid channel bandwidth [6]
  //      Available radio channel bandwidth for pDDL mode
  //       0 - 8 MHz
  //       1 - 4 MHz
  //       2 - 2 MHz
  //       3 - 1 MHz
  // (2)  joystick> AT+MWBAND=1
  //      OK
  // (3)  joystick> AT+MWNETWORKID
  //      +MWNETWORKID: Virtual Interface Network ID : konkers_test1
  //      OK
  // (4)  joystick> AT+MWTXPOWER
  //      +MWTXPOWER: 11 - 30 dbm
  //      OK
  // (5)  joystick> AT+MWVMODE
  //      +MWVMODE: Virtual Interface Mode: 0 - Master
  //      OK
  // (6)  joystick> AT+MWBAND
  //      +MWBAND:
  //       Channel-bandwidth : 0 - 8 MHz
  //      OK
  // (7)  joystick> AT+MWBAND
  //      +MWBAND:
  //       Channel-bandwidth : 2 - 2 MHz
  //       Symbol Rate : 1 - Fast
  //      OK
  // (8)  joystick> AT+MWRSSI
  //      ERROR: No device connected
  // (9)  joystick> AT+MWRSSI
  //        00:0F:92:FA:4F:4D  -67 dBm
  //      OK
  // (10) joystick> AT+MWRSSI
  //      -67
  //      OK

  // The radio terminates lines with \r\n.  To make parsing simpler, we
  // strip carriage returns.
  if (MicrohardResponseEndsWith(microhard, "\r")) {
    microhard->data->response_len--;
    microhard->data->response[microhard->data->response_len] = '\0';
    return NULL;
  }

  // Only try to parse when we have a complete line.
  if (!MicrohardResponseEndsWith(microhard, "\n")) {
    return NULL;
  }

  // Case (1)
  if (MicrohardResponseContains(microhard, "\nERROR")) {
    return "ERROR";
  }

  // Cases (2-7) end with OK
  if (!MicrohardResponseEndsWith(microhard, "OK\n")) {
    return NULL;
  }

  // Process one line at a time.
  char *newline_pos;
  char *line = microhard->data->response;
  while (line != NULL) {
    newline_pos = strchr(line, '\n');
    if (newline_pos) {
      *newline_pos = '\0';
      ++newline_pos;
    }

    char *value_str = line;

    // Case (9): Strip off preceding MAC Address.
    if (strlen(line) > 3 && line[0] == ' ' && line[1] == ' ' &&
        MicrohardIsMacAddr(line + 2)) {
      value_str += strlen("  XX:XX:XX:XX:XX:XX");
      while (*value_str && *value_str == ' ') {
        ++value_str;
      }
    }

    DebugPrint(3, "line: '%s'\n", value_str);
    // Strip off everything before the last ": ".
    const char * value_sep = ": ";
    char *sub_str;
    while ((sub_str = strstr(value_str, value_sep)) != NULL) {
      value_str = sub_str + strlen(value_sep);
    }

    // Case (10): Handle response to AT+MWRSSI in Microhard software build 1034.
    if (IsNumeric(value_str)) {
      return value_str;
    }

    // Cases (3)-(6)
    if (value_str != line) {
      // Values do not contain spaces.  Drop everything after a space.
      if ((sub_str = strchr(value_str, ' ')) != NULL) {
        sub_str[0] = '\0';
      }
      return value_str;
    }
    // Case (7) not supported.

    // OK if newline_pos is NULL since we wont iterate.
    line = newline_pos;
  }

  // Case (2): If we didn't find a valid value string, just return the OK
  // status.
  return "OK";
}

static void MicrohardSendLine(const Microhard *microhard, const char *str) {
  DebugPrint(1, "> %s\n", str);
  MicrohardClearResponse(microhard);
  SciWriteString(microhard->sci, str);
  SciWriteString(microhard->sci, "\n");
}

static void MicrohardSendConfig(const Microhard *microhard) {
  char command[64];
  const ssize_t command_size = ARRAYSIZE(command);

  assert(microhard->data->config_index < ARRAYSIZE(kMicrohardConfigCommands));
  const char *config_str =
      kMicrohardConfigCommands[microhard->data->config_index];

  ssize_t command_len = snprintf(command, command_size, "%s=", config_str);
  assert(command_len < command_size);

  bool ret = MicrohardSetInitValue(microhard, config_str,
                                   command_size - command_len,
                                   command + command_len);
  assert(ret);

  MicrohardSendLine(microhard, command);
}

static void MicrohardHandleStateInit(const Microhard *microhard,
                                     MicrohardState next) {
  (void)microhard;
  IoConfigureAsOutputPushPull(kIoSpi3PinScs1, true);
  SciInit(microhard->sci, microhard->baud_rate);
  microhard->data->next_state = next;
}

static void MicrohardHandleStateActivate(const Microhard *microhard,
                                         MicrohardState login,
                                         MicrohardState ready,
                                         MicrohardState failure) {
  static int32_t retries;
  if (microhard->data->first_entry) {
    MicrohardSendLine(microhard, "");
    TimerStart(kMicrohardTimeout, &microhard->data->timer);
    retries = 0;
  }
  if (MicrohardResponseEndsWith(microhard, "login: ")) {
    microhard->data->next_state = login;
  } else if (MicrohardResponseEndsWith(microhard, "> ")) {
    microhard->data->next_state = ready;
  } else if (TimerExpired(&microhard->data->timer)) {
    if (retries > 4) {
      microhard->data->next_state = failure;
    } else {
      MicrohardSendLine(microhard, "");
      TimerStart(kMicrohardTimeout, &microhard->data->timer);
      ++retries;
    }
  }
}

static void MicrohardHandleStateLogin(const Microhard *microhard,
                                      MicrohardState success,
                                      MicrohardState failure) {
  if (microhard->data->first_entry) {
    // TODO: Make configurable.
    MicrohardSendLine(microhard, "admin");
    TimerStart(kMicrohardTimeout, &microhard->data->timer);
  }
  if (MicrohardResponseEndsWith(microhard, "Password: ")) {
    microhard->data->next_state = success;
  } else if (TimerExpired(&microhard->data->timer)) {
    microhard->data->next_state = failure;
  }
}

static void MicrohardHandleStatePassword(const Microhard *microhard,
                                         MicrohardState success,
                                         MicrohardState failure) {
  if (microhard->data->first_entry) {
    // TODO: Make configurable.
    MicrohardSendLine(microhard, "makani");
    TimerStart(kMicrohardLoginTimeout, &microhard->data->timer);
  }
  if (MicrohardResponseEndsWith(microhard, "> ")) {
    microhard->data->next_state = success;
  } else if (TimerExpired(&microhard->data->timer)) {
    microhard->data->next_state = failure;
  }
}

static void MicrohardHandleStateReady(const Microhard *microhard,
                                      MicrohardState configure,
                                      MicrohardState poll_rssi) {
  if (microhard->data->first_entry) {
    if (!microhard->data->configured) {
      microhard->data->next_state = configure;
      return;
    }
    DebugPrint(1, "ready\n");
  }

  if (PollPeriodicCycles(kMicrohardPollInterval, &microhard->data->next_poll)) {
    microhard->data->next_state = poll_rssi;
  }
}

static void MicrohardHandleStateConfig(const Microhard *microhard,
                                       MicrohardState success,
                                       MicrohardState failure) {
  if (microhard->data->first_entry) {
    microhard->data->config_index = 0;
    MicrohardSendConfig(microhard);
    TimerStart(kMicrohardTimeout, &microhard->data->timer);
  }

  if (MicrohardResponseEndsWith(microhard, "OK")) {
    ++microhard->data->config_index;
    if (microhard->data->config_index >= ARRAYSIZE(kMicrohardConfigCommands)) {
      // Commit configuration.
      // TODO: Check the config and only write if values have changed.
      MicrohardSendLine(microhard, "AT&W");
      microhard->data->configured = true;
      microhard->data->next_state = success;
    } else {
      MicrohardSendConfig(microhard);
      TimerStart(kMicrohardTimeout, &microhard->data->timer);
    }
  } else if (MicrohardResponseEndsWith(microhard, "ERROR")
             || TimerExpired(&microhard->data->timer)) {
    microhard->data->next_state = failure;
  }
}

static void MicrohardHandleStatePollRssi(const Microhard *microhard,
                                         MicrohardState success,
                                         MicrohardState failure) {
  if (microhard->data->first_entry) {
    MicrohardSendLine(microhard, "AT+MWRSSI");
    TimerStart(kMicrohardTimeout, &microhard->data->timer);
  }

  const char *response = MicrohardParseResponse(microhard);
  MicrohardStatus status = {.connected = 0, .rssi = false};
  if (response != NULL) {
    DebugPrint(1, "response: %s\n", response);

    if (!strcmp(response, "ERROR")) {
      microhard->data->next_state = success;
    }

    char *endptr;
    status.rssi = strtol(response, &endptr, 10);
    if (endptr != response) {
      status.connected = true;
      microhard->data->next_state = success;
    } else {
      microhard->data->next_state = failure;
    }
  } else if (TimerExpired(&microhard->data->timer)) {
    microhard->data->next_state = failure;
  }

  if (microhard->data->next_state != kMicrohardStatePollRssi
      && microhard->set_status) {
    microhard->set_status(status);
  }
}

static void MicrohardHandleStateMachine(const Microhard *microhard) {
  MicrohardState current = microhard->data->next_state;
  switch (current) {
    case kMicrohardStateInit:
      MicrohardHandleStateInit(microhard, kMicrohardStateActivate);
      break;
    case kMicrohardStateActivate:
      MicrohardHandleStateActivate(microhard,
                                   kMicrohardStateLogin,
                                   kMicrohardStateReady,
                                   kMicrohardStateInit);
      break;
    case kMicrohardStateLogin:
      MicrohardHandleStateLogin(microhard,
                                kMicrohardStatePassword,
                                kMicrohardStateInit);
      break;
    case kMicrohardStatePassword:
      MicrohardHandleStatePassword(microhard, kMicrohardStateReady,
                                   kMicrohardStateInit);
      break;
    case kMicrohardStateReady:
      MicrohardHandleStateReady(microhard, kMicrohardStateConfig,
                                kMicrohardStatePollRssi);
      break;
    case kMicrohardStateConfig:
      MicrohardHandleStateConfig(microhard, kMicrohardStateActivate,
                                 kMicrohardStateInit);
      break;
    case kMicrohardStatePollRssi:
      MicrohardHandleStatePollRssi(microhard, kMicrohardStateReady,
                                   kMicrohardStateInit);
      break;
    default:
      break;
  }

  microhard->data->first_entry = current != microhard->data->next_state;

  if (microhard->data->first_entry) {
    DebugPrint(3, "%d -> %d.\n", current, microhard->data->next_state);
  }
}

void MicrohardInit(const Microhard *microhard, const MicrohardParams *params) {
  memset(microhard->data, 0x0, sizeof(*microhard->data));

  microhard->data->first_entry = true;
  microhard->data->next_state = kMicrohardStateInit;
  microhard->data->params = params;
}

void MicrohardPoll(const Microhard *microhard) {
  uint8_t c;

  // Receive at most one character per trip through the state machine.
  if (SciReadByte(microhard->sci, &c)) {
    MicrohardAppendResponse(microhard, c);
  }

  MicrohardHandleStateMachine(microhard);
}

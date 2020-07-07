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

#include "avionics/firmware/drivers/xlr.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avionics/common/serial_parse.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/sci.h"
#include "avionics/firmware/drivers/xlr_serial.h"
#include "common/c_math/util.h"
#include "common/macros.h"

#define XLR_GUARD_CYCLES CLOCK32_MSEC_TO_CYCLES(1500)
#define XLR_RESPONSE_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(1000)
#define XLR_WATCHDOG_CYCLES CLOCK32_MSEC_TO_CYCLES(1000)

// Specify the length of AT[command].
#define AT_COMMAND_LENGTH 4

typedef enum {
  kXlrStateFlagSetNetworkId        = 1 << 0,
  kXlrStateFlagSetAesEncryptionKey = 1 << 1,
} XlrStateFlag;

// Specify baud rates to support. Baud rate index parameters correspond to the
// ATBD[index] command. Set to zero to disable.
static const int32_t kIndexToBaudRate[10] = {
  [1] = 2400,
  [2] = 4800,
  [3] = 9600,
  [4] = 19200,
  [5] = 38400,
  [6] = 57600,
  [7] = 115200,
  [8] = 230400,
  [9] = 460800  // Not supported by Ground I/O board's RS232 transceiver.
};

// Helpful debug outputs.
static void DebugPrint(const char *prefix, int32_t length,
                       const uint8_t *data) {
#ifdef XLR_DEBUG
  printf("%s: ", prefix);
  for (int32_t i = 0; i < length; ++i) {
    printf("%c", data[i]);
  }
  printf("\n");
#else
  (void)prefix;
  (void)length;
  (void)data;
#endif
}

// Handle timeouts.

static void SetTimeout(uint32_t cycles, XlrStateMachine *state) {
  state->timeout = Clock32GetCycles() + cycles;
}

static bool WaitForTimeout(const XlrStateMachine *state) {
  return CLOCK32_GE(Clock32GetCycles(), state->timeout);
}

// Handle interface baud rate.

// Prevent user from specifying an invalid (or disabled) baud rate parameter.
static int32_t SanitizeBaudRateIndex(int32_t index) {
  if (index < 0) {
    index = 0;
  }
  while (index < ARRAYSIZE(kIndexToBaudRate) && kIndexToBaudRate[index] <= 0) {
    ++index;
  }
  if (index >= ARRAYSIZE(kIndexToBaudRate)) {
    index = ARRAYSIZE(kIndexToBaudRate) - 1;
  }
  while (index > 0 && kIndexToBaudRate[index] <= 0) {
    --index;
  }
  assert(kIndexToBaudRate[index] > 0);
  return index;
}

// Check for ATBD[index] command. We use this function to interpret the user
// initialization strings, and then set the corresponding interface baud rate.
static bool IsAtBaudRateCommand(int32_t command_length, const char *command,
                                int32_t *baud_index) {
  if (command_length > 4 && memcmp(command, "ATBD", 4) == 0) {
    *baud_index = SanitizeBaudRateIndex(atoi(&command[4]));
    return true;
  }
  return false;
}

static int32_t GetBaudRate(const XlrStateMachine *state) {
  return kIndexToBaudRate[state->baud_index];
}

static void SetDefaultBaudRate(const XlrConfig *conf, XlrStateMachine *state) {
  // Set baud rate to manufacture's default at 9600 bps (ATBD3).
  state->baud_index = 3;

  // Check for user specified default.
  for (int32_t i = 0; i < conf->num_at_strings; ++i) {
    const char *command = conf->at_string[i];
    IsAtBaudRateCommand((int32_t)strlen(command), command, &state->baud_index);
  }

  SciInit(conf->device, kIndexToBaudRate[state->baud_index]);
}

static void TryNextBaudRate(const XlrConfig *conf, XlrStateMachine *state) {
  do {
    ++state->baud_index;
    state->baud_index %= ARRAYSIZE(kIndexToBaudRate);
  } while (kIndexToBaudRate[state->baud_index] <= 0);

  SciInit(conf->device, kIndexToBaudRate[state->baud_index]);
}

// Handle initialization strings. We use an array of strings to specify the
// initialization strings. Initialization strings should comply with format
// AT[command][parameter], where [command] represents the two character
// command and [parameter] represents the command's parameter value. AT
// commands should not contain a trailing linefeed or carriage return.

static void SetFirstAtInitString(XlrStateMachine *state) {
  state->init_index = 0;
}

static const char *GetAtInitString(const XlrConfig *conf,
                                   const XlrStateMachine *state) {
  return conf->at_string[state->init_index % conf->num_at_strings];
}

static bool TryNextAtInitString(const XlrConfig *conf, XlrStateMachine *state) {
  ++state->init_index;
  return 0 <= state->init_index && state->init_index < conf->num_at_strings;
}

// Handle response data. The parsing route places response data into a ring
// buffer for processing by the state machine.

static bool WaitForAtResponse(XlrResponseBuffer *buf) {
  // Drop previous responses (Api or None).
  for ( ; buf->tail != buf->head; ++buf->tail) {
    const XlrResponseData *response =
        &buf->data[buf->tail % ARRAYSIZE(buf->data)];
    if (response->response_type == kXlrResponseTypeAtCommand) {
      return true;
    }
  }
  return false;
}

static XlrResponseData *InsertResponse(XlrResponseType type,
                                       XlrResponseBuffer *buf) {
  // Drop old responses in favor the new responses.
  while (buf->head - buf->tail >= ARRAYSIZE(buf->data)) {
    ++buf->tail;
  }

  XlrResponseData *out = &buf->data[buf->head % ARRAYSIZE(buf->data)];
  out->response_type = type;
  ++buf->head;

  return out;
}

static void InsertAtResponse(int32_t length, const char *data,
                             XlrResponseBuffer *buf) {
  XlrResponseData *out = InsertResponse(kXlrResponseTypeAtCommand, buf);
  XlrAtCommandResponse *at = &out->u.at_command;
  at->length = MinInt32(length, ARRAYSIZE(at->data));
  memcpy(at->data, data, at->length);
}

static void InsertApiResponse(const XlrApiCommandResponse *response,
                              XlrResponseBuffer *buf) {
  XlrResponseData *out = InsertResponse(kXlrResponseTypeApiCommand, buf);
  out->u.api_command = *response;
  // Prevent driver from accessing data. Since data points to the parsing
  // buffer, it is not safe to reference data from the response queue.
  out->u.api_command.length = 0;
  out->u.api_command.data = NULL;
}

// Clear response buffer before sending a new command.
static void ClearAtResponse(XlrResponseBuffer *buf) {
  for (uint32_t i = buf->tail; i != buf->head; ++i) {
    XlrResponseData *response = &buf->data[i % ARRAYSIZE(buf->data)];
    if (response->response_type == kXlrResponseTypeAtCommand) {
      response->response_type = kXlrResponseTypeNone;
    }
  }
}

static bool CheckAtResponseForString(const XlrResponseBuffer *buf,
                                     const char *str) {
  int32_t length = strlen(str);
  for (uint32_t i = buf->tail; i != buf->head; ++i) {
    const XlrResponseData *response = &buf->data[i % ARRAYSIZE(buf->data)];
    if (response->response_type == kXlrResponseTypeAtCommand
        && response->u.at_command.length == length
        && memcmp(response->u.at_command.data, str, length) == 0) {
      return true;
    }
  }
  return false;
}

static bool CheckAtResponseForOk(const XlrResponseBuffer *buf) {
  return CheckAtResponseForString(buf, "OK");
}

static bool CheckAtResponseForError(const XlrResponseBuffer *buf) {
  return CheckAtResponseForString(buf, "ERROR");
}

static bool CheckAtResponseForParam(const XlrResponseBuffer *buf,
                                    const char *command) {
  int32_t length = strlen(command);
  return length <= AT_COMMAND_LENGTH
      || CheckAtResponseForString(buf, &command[AT_COMMAND_LENGTH]);
}

static const XlrApiCommandResponse *CheckApiResponseForFrameId(
    uint8_t frame_id, const XlrResponseBuffer *buf) {
  assert(frame_id != 0);

  for (uint32_t i = buf->tail; i != buf->head; ++i) {
    const XlrResponseData *response = &buf->data[i % ARRAYSIZE(buf->data)];
    if (response->response_type == kXlrResponseTypeApiCommand
        && response->u.api_command.frame_id == frame_id) {
      return &response->u.api_command;
    }
  }
  return NULL;
}

// Handle command data. The command interface uses a temporary buffer to
// render the packet to transmit to the modem (with frame and escapes). We
// then transfer the packet to the serial interface layer. If the serial
// interface layer does not have sufficient buffer capacity for the packet,
// we transfer as much as possible each iteration. We do not process the
// state machine until this transfer completes.

static bool IsTransmitBufferReady(const XlrCommandBuffer *buf) {
  return buf->index == buf->length;
}

static bool TransmitCommandData(const XlrConfig *conf, XlrCommandBuffer *out) {
  if (out->index < out->length) {
    out->index += SciWrite(conf->device, out->length - out->index,
                           &out->data[out->index]);
  }
  return IsTransmitBufferReady(out);
}

static void WriteEnterAtCommandMode(XlrCommandBuffer *out) {
  assert(IsTransmitBufferReady(out));
  out->data[0] = '+';
  out->data[1] = '+';
  out->data[2] = '+';
  out->length = 3;
  out->index = 0;
}

static void WriteAtBaudRate(int32_t index, XlrCommandBuffer *out) {
  assert(IsTransmitBufferReady(out));
  out->length = snprintf((char *)out->data, ARRAYSIZE(out->data), "ATBD%ld\r",
                         index);
  out->index = 0;
}

static void WriteAtCommand(int32_t command_length, const char *command,
                           XlrCommandBuffer *out) {
  assert(IsTransmitBufferReady(out));
  assert(command_length < ARRAYSIZE(out->data));
  memcpy(out->data, command, command_length);
  out->data[command_length] = '\r';
  out->length = command_length + 1;
  out->index = 0;
}

// This function obtains the next frame_id given pointer *frame_id from the
// application. If the application specifies NULL, then we assume that the
// application does not require a response. We then return 0 to instruct the
// modem to not return a response. If the application specifies a non-NULL
// pointer, then we compute the next frame_id != 0 and return it to the
// application.
static uint8_t GetNextFrameId(const XlrCommandBuffer *buf, uint8_t *frame_id) {
  if (frame_id != 0U) {
    *frame_id = 1U + (buf->frame_id % 254U);
    return *frame_id;
  }
  return 0U;
}

static bool WriteApiCommandBuffer(uint8_t next_id, int32_t length,
                                  XlrCommandBuffer *out) {
  assert(IsTransmitBufferReady(out));
  out->length = length;
  out->index = 0;

  // Only increment frame_id for valid commands that require a response.
  if (length > 0 && next_id > 0U) {
    out->frame_id = next_id;
  }
  return length > 0;
}

static bool WriteApiAtCommand(XlrAtCommand command, int32_t param_length,
                              const void *param, uint8_t *frame_id,
                              XlrCommandBuffer *out) {
  uint8_t next_id = GetNextFrameId(out, frame_id);
  return WriteApiCommandBuffer(next_id, XlrApiWriteAtCommand(
      kXlrApiFrameTypeCommand, next_id, command,
      param_length, param, ARRAYSIZE(out->data), out->data), out);
}

static bool WriteApiTxRequest(uint64_t dest_addr, uint8_t hops, uint8_t options,
                              int32_t data_length, const uint8_t *data,
                              uint8_t *frame_id, XlrCommandBuffer *out) {
  uint8_t next_id = GetNextFrameId(out, frame_id);
  return WriteApiCommandBuffer(next_id, XlrApiWriteTxRequest(
      next_id, dest_addr, hops, options, data_length, data,
      ARRAYSIZE(out->data), out->data), out);
}

// Handle combine send commands. These functions simplify the state machine.

static void SendEnterAtCommandMode(const XlrConfig *conf, XlrDevice *x) {
  ClearAtResponse(&x->response);
  WriteEnterAtCommandMode(&x->command);
  DebugPrint("XLR send", x->command.length, x->command.data);
  TransmitCommandData(conf, &x->command);
  SetTimeout(XLR_GUARD_CYCLES, &x->state);
}

static void SendAtCommand(const XlrConfig *conf, int32_t command_length,
                          const char *command, XlrDevice *x) {
  assert((int32_t)strlen(command) >= command_length);
  ClearAtResponse(&x->response);
  if (IsAtBaudRateCommand(command_length, command, &x->state.baud_index)) {
    WriteAtBaudRate(x->state.baud_index, &x->command);
  } else {
    WriteAtCommand(command_length, command, &x->command);
  }
  DebugPrint("XLR send", x->command.length, x->command.data);
  TransmitCommandData(conf, &x->command);
}

static void SendAtQuery(const XlrConfig *conf, const char *command,
                        XlrDevice *x) {
  // Send AT[command], where [command] represents the two character command
  // to query the associated parameter.
  SendAtCommand(conf, AT_COMMAND_LENGTH, command, x);
}

static bool SendApiAtCommand(const XlrConfig *conf, XlrAtCommand command,
                             int32_t param_length, const void *param,
                             uint8_t *frame_id, XlrDevice *x) {
  if (x->state.api_mode
      && x->command.index == x->command.length
      && WriteApiAtCommand(command, param_length, param, frame_id,
                           &x->command)) {
    TransmitCommandData(conf, &x->command);
    return true;
  }
  return false;
}

static bool SendApiTxRequest(const XlrConfig *conf, uint64_t dest_addr,
                             uint8_t hops, uint8_t options, int32_t data_length,
                             const uint8_t *data, uint8_t *frame_id,
                             XlrDevice *x) {
  if (x->state.api_mode
      && x->command.index == x->command.length
      && WriteApiTxRequest(dest_addr, hops, options, data_length, data,
                           frame_id, &x->command)) {
    TransmitCommandData(conf, &x->command);
    return true;
  }
  return false;
}

// Handle state machine.

static void StateInit(const XlrConfig *conf, XlrState next, XlrDevice *x) {
  if (x->state.first_entry) {
    x->state.flags |= kXlrStateFlagSetAesEncryptionKey;
    x->state.flags |= kXlrStateFlagSetNetworkId;
    x->state.api_mode = false;
    SetDefaultBaudRate(conf, &x->state);
    // Specify initial delay for EnterAtCommandMode escape.
    SetTimeout(XLR_GUARD_CYCLES, &x->state);
  }
  if (WaitForTimeout(&x->state)) {
    x->state.next = next;
  }
}

// To prevent accidentally entering command mode, the modem requires a guard
// time before and after the command mode sequence, where the host remains
// quiet. The default guard time is one second (specified by the GT parameter)
// and the default command sequence is +++ (specified by the CC parameter).
// This state assumes the previous state observed the guard time before
// transitioning.
static void StateEnterAtCommandMode(const XlrConfig *conf, XlrState next,
                                    XlrDevice *x) {
  if (x->state.first_entry) {
    SendEnterAtCommandMode(conf, x);
  }
  if (CheckAtResponseForOk(&x->response)) {
    x->state.next = next;
  } else if (WaitForTimeout(&x->state)) {
    TryNextBaudRate(conf, &x->state);
    SendEnterAtCommandMode(conf, x);
  }
}

static void StateSendAtCommand(const XlrConfig *conf, const char *command,
                               XlrState success, XlrState failure,
                               XlrDevice *x) {
  if (x->state.first_entry) {
    SendAtCommand(conf, (int32_t)strlen(command), command, x);
    SetTimeout(XLR_RESPONSE_TIMEOUT_CYCLES, &x->state);
  }
  if (CheckAtResponseForOk(&x->response)) {
    x->state.next = success;
  } else if (CheckAtResponseForError(&x->response)
             || WaitForTimeout(&x->state)) {
    x->state.next = failure;
  }
}

static const XlrApiCommandResponse *StateSendApiAtCommand(
    const XlrConfig *conf, XlrAtCommand command, int32_t param_length,
    const void *param, XlrState success, XlrState failure, XlrDevice *x) {
  if (x->state.first_entry) {
    SendApiAtCommand(conf, command, param_length, param, &x->state.frame_id, x);
    SetTimeout(XLR_RESPONSE_TIMEOUT_CYCLES, &x->state);
  }

  const XlrApiCommandResponse *response =
      CheckApiResponseForFrameId(x->state.frame_id, &x->response);
  if (response == NULL) {
    if (WaitForTimeout(&x->state)) {
      x->state.next = failure;
    }
  } else if (response->status == kXlrApiResponseStatusOk) {
    x->state.next = success;
  } else {
    x->state.next = failure;
  }
  return response;
}

static void StateSendAtInit(XlrState next, XlrDevice *x) {
  SetFirstAtInitString(&x->state);
  x->state.save_config = false;
  x->state.next = next;
}

static void StateGetAtConfig(const XlrConfig *conf, XlrState done,
                             XlrState mismatch, XlrState failure,
                             XlrDevice *x) {
  if (x->state.first_entry) {
    SendAtQuery(conf, GetAtInitString(conf, &x->state), x);
  }
  if (WaitForAtResponse(&x->response)) {
    if (!CheckAtResponseForParam(&x->response,
                                 GetAtInitString(conf, &x->state))) {
      x->state.next = mismatch;
      x->state.save_config = true;
    } else if (TryNextAtInitString(conf, &x->state)) {
      SendAtQuery(conf, GetAtInitString(conf, &x->state), x);
    } else {
      x->state.next = done;
    }
  } else if (WaitForTimeout(&x->state)) {
    x->state.next = failure;
  }
}

static void StateSetAtConfig(const XlrConfig *conf, XlrState done,
                             XlrState next, XlrState failure, XlrDevice *x) {
  StateSendAtCommand(conf, GetAtInitString(conf, &x->state), done, failure, x);
  if (x->state.next == done && TryNextAtInitString(conf, &x->state)) {
    x->state.next = next;
  }
}

static void StateSaveAtConfig(const XlrConfig *conf, XlrState success,
                              XlrState failure, XlrDevice *x) {
  // As a safety mechanism, we only save the configuration if we detect a
  // discrepancy, and at most, once per power cycle.
  if (x->state.first_entry
      && (!x->state.save_config || x->state.saved_config)) {
    x->state.next = success;
  } else {
    x->state.save_config = false;
    x->state.saved_config = true;
    StateSendAtCommand(conf, "ATWR", success, failure, x);
  }
}

static void StateLeaveAtCommandMode(const XlrConfig *conf, XlrState success,
                                    XlrState failure, XlrDevice *x) {
  StateSendAtCommand(conf, "ATCN", success, failure, x);
  if (x->state.next == success) {
    // Baud rate change takes affect after leaving command mode.
    SciInit(conf->device, GetBaudRate(&x->state));
    x->state.api_mode = true;
  }
}

static void StateReady(XlrDevice *x) {
  if (x->state.flags & kXlrStateFlagSetAesEncryptionKey) {
    x->state.flags &= ~kXlrStateFlagSetAesEncryptionKey;
    x->state.next = kXlrStateSetAesEncryptionKey;
  } else if (x->state.flags & kXlrStateFlagSetNetworkId) {
    x->state.flags &= ~kXlrStateFlagSetNetworkId;
    x->state.next = kXlrStateSetNetworkId;
  } else if (
      Clock32GetCycles() - x->response.last_api_cycles > XLR_WATCHDOG_CYCLES) {
    x->state.next = kXlrStateWatchdog;
  }
}

static void ProcessStateMachine(const XlrConfig *conf, XlrDevice *x) {
  XlrState current = x->state.next;
  switch (current) {
    case kXlrStateInit:
      StateInit(conf, kXlrStateEnterAtCommandMode, x);
      break;
    case kXlrStateEnterAtCommandMode:
      StateEnterAtCommandMode(conf, kXlrStateSendAtInit, x);
      break;
    case kXlrStateSendAtInit:
      StateSendAtInit(kXlrStateGetAtConfig, x);
      break;
    case kXlrStateGetAtConfig:
      StateGetAtConfig(conf, kXlrStateSaveAtConfig, kXlrStateSetAtConfig,
                       kXlrStateInit, x);
      break;
    case kXlrStateSetAtConfig:
      StateSetAtConfig(conf, kXlrStateSaveAtConfig, kXlrStateGetAtConfig,
                       kXlrStateInit, x);
      break;
    case kXlrStateSaveAtConfig:
      StateSaveAtConfig(conf, kXlrStateLeaveAtCommandMode, kXlrStateInit, x);
      break;
    case kXlrStateLeaveAtCommandMode:
      StateLeaveAtCommandMode(conf, kXlrStateReady, kXlrStateInit, x);
      break;
    case kXlrStateReady:
      StateReady(x);
      break;
    case kXlrStateSetAesEncryptionKey:
      StateSendApiAtCommand(conf, kXlrAtCommandAesEncryptionKey,
                            sizeof(x->state.aes_key), x->state.aes_key,
                            kXlrStateReady, kXlrStateInit, x);
      break;
    case kXlrStateSetNetworkId:
      StateSendApiAtCommand(conf, kXlrAtCommandNetworkId,
                            sizeof(x->state.network_id), &x->state.network_id,
                            kXlrStateReady, kXlrStateInit, x);
      break;
    case kXlrStateWatchdog:
      StateSendApiAtCommand(conf, kXlrAtCommandFirmwareVersion, 0, NULL,
                            kXlrStateReady, kXlrStateInit, x);
      break;
    default:
      x->state.next = kXlrStateInit;
      assert(false);
      break;
  }
  x->state.first_entry = current != x->state.next;
}

static void HandleReceiveAscii(int32_t length, const uint8_t *data,
                               XlrResponseBuffer *buf) {
  DebugPrint("XLR recv", length, data);
  InsertAtResponse(length, (const char *)data, buf);
}

static bool HandleReceiveApi(int32_t length, const uint8_t *data,
                             XlrResponseBuffer *buf, XlrApiData *out) {
  if (XlrApiParseData(length, data, out)) {
    buf->last_api_cycles = Clock32GetCycles();
    if (out->frame_type == kXlrApiFrameTypeCommandResponse) {
      InsertApiResponse(&out->u.command_response, buf);
    }
    return true;
  }
  return false;
}

static bool HandleReceive(const XlrConfig *conf, XlrDevice *x,
                          XlrApiData *out) {
  bool out_valid = false;
  if (SerialPollSci(conf->device, &x->receive)
      && XlrParse(&x->receive, &x->parse)) {
    switch (x->parse.proto) {
      case kXlrProtoAscii:
        HandleReceiveAscii(x->parse.data_length, x->parse.data, &x->response);
        break;
      case kXlrProtoApiNonEscaped:
      case kXlrProtoApiEscaped:
        out_valid = HandleReceiveApi(x->parse.data_length, x->parse.data,
                                     &x->response, out);
        break;
      default:
        break;
    }
  }
  return out_valid;
}

void XlrInit(XlrDevice *x) {
  memset(x, 0, sizeof(*x));
  SerialParseInit(&x->receive);
  x->state.first_entry = true;
  x->state.next = kXlrStateInit;
}

void XlrSetAesEncryptionKey(const uint32_t aes_key[4], XlrDevice *x) {
  memcpy(x->state.aes_key, aes_key, sizeof(x->state.aes_key));
  x->state.flags |= kXlrStateFlagSetAesEncryptionKey;
}

void XlrSetNetworkId(uint32_t network_id, XlrDevice *x) {
  x->state.network_id = network_id;
  x->state.flags |= kXlrStateFlagSetNetworkId;
}

bool XlrPoll(const XlrConfig *conf, XlrDevice *x, XlrApiData *out) {
  bool out_valid = HandleReceive(conf, x, out);

  // The state machine requires exclusive access to the command buffer. Wait
  // for data transfer from command buffer to serial interface layer before
  // iterating the state machine.
  if (TransmitCommandData(conf, &x->command)) {
    ProcessStateMachine(conf, x);
  }
  return out_valid;
}

bool XlrSendUnicast(const XlrConfig *conf, uint64_t dest_addr, uint8_t hops,
                    uint8_t options, int32_t data_length, const void *data,
                    uint8_t *frame_id, XlrDevice *x) {
  return SendApiTxRequest(conf, dest_addr, hops, options, data_length,
                          (const uint8_t *)data, frame_id, x);
}

bool XlrSendBroadcast(const XlrConfig *conf, uint8_t hops, uint8_t options,
                      int32_t data_length, const void *data, uint8_t *frame_id,
                      XlrDevice *x) {
  return SendApiTxRequest(conf, 0xFFFF, hops, options, data_length,
                          (const uint8_t *)data, frame_id, x);
}

bool XlrSendGetTemperature(const XlrConfig *conf, XlrDevice *x) {
  uint8_t frame_id;
  return SendApiAtCommand(conf, kXlrAtCommandTemperature, 0, NULL, &frame_id,
                          x);
}

bool XlrSendGetReceivedSignalStrength(const XlrConfig *conf, XlrDevice *x) {
  uint8_t frame_id;
  return SendApiAtCommand(conf, kXlrAtCommandReceivedSignalStrength, 0, NULL,
                          &frame_id, x);
}

bool XlrSendGetReceivedErrorCount(const XlrConfig *conf, XlrDevice *x) {
  uint8_t frame_id;
  return SendApiAtCommand(conf, kXlrAtCommandReceivedErrorCount, 0, NULL,
                          &frame_id, x);
}

bool XlrSendClearReceivedErrorCount(const XlrConfig *conf, XlrDevice *x) {
  uint16_t zero = 0U;
  return SendApiAtCommand(conf, kXlrAtCommandReceivedErrorCount, sizeof(zero),
                          &zero, NULL, x);
}

bool XlrSendGetGoodPacketsReceived(const XlrConfig *conf, XlrDevice *x) {
  uint8_t frame_id;
  return SendApiAtCommand(conf, kXlrAtCommandGoodPacketsReceived, 0, NULL,
                          &frame_id, x);
}

bool XlrSendClearGoodPacketsReceived(const XlrConfig *conf, XlrDevice *x) {
  uint16_t zero = 0U;
  return SendApiAtCommand(conf, kXlrAtCommandGoodPacketsReceived, sizeof(zero),
                          &zero, NULL, x);
}

bool XlrSendGetTransmissionFailureCount(const XlrConfig *conf, XlrDevice *x) {
  uint8_t frame_id;
  return SendApiAtCommand(conf, kXlrAtCommandTransmissionFailureCount, 0, NULL,
                          &frame_id, x);
}

bool XlrSendClearTransmissionFailureCount(const XlrConfig *conf, XlrDevice *x) {
  uint16_t zero = 0U;
  return SendApiAtCommand(conf, kXlrAtCommandTransmissionFailureCount,
                          sizeof(zero), &zero, NULL, x);
}

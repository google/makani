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

#include "avionics/firmware/drivers/novatel_recv.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/common/novatel_binary.h"
#include "avionics/common/novatel_serial.h"
#include "avionics/common/novatel_types.h"
#include "avionics/common/serial_parse.h"
#include "avionics/firmware/drivers/novatel.h"
#include "common/macros.h"

#define MESSAGE_BUFFER_SIZE 16

typedef struct {
  NovAtelMessageId data[MESSAGE_BUFFER_SIZE];
  uint32_t head;
  uint32_t tail;
} MessageIdBuffer;

static uint32_t g_this_abbr = 0;
static uint32_t g_this_prompt = 0;
static SerialReceiveBuffer g_this_recv;
static SerialReceiveBuffer g_other_recv;
static NovAtelReceive g_this_parse;
static NovAtelReceive g_other_parse;
static NovAtelReceive *g_last_parsed = NULL;
static MessageIdBuffer g_resp;
static MessageIdBuffer g_message;

static void PrintResponse(int32_t length, const uint8_t *data) {
  char msg[128];
  if ((size_t)length >= sizeof(msg)) {
    memcpy(msg, data, sizeof(msg));
    msg[ARRAYSIZE(msg) - 1] = '\0';
  } else {
    memcpy(msg, data, (size_t)length);
    msg[length] = '\0';
  }
  printf("NovAtel: %s\n", msg);
}

static void AppendMessage(const NovAtelHeader *hdr, MessageIdBuffer *buf) {
  buf->data[buf->head] = hdr->message_id;
  buf->head = (buf->head + 1) % MESSAGE_BUFFER_SIZE;
  if (buf->head == buf->tail) {
    buf->tail = (buf->tail + 1) % MESSAGE_BUFFER_SIZE;
  }
}

// Wait for asynchronous message.
static bool ReceivedMessage(NovAtelMessageId message_id, MessageIdBuffer *buf) {
  bool found = false;
  while (buf->tail != buf->head && !found) {
    found = (buf->data[buf->tail] == message_id);
    buf->tail = (buf->tail + 1) % MESSAGE_BUFFER_SIZE;
  }
  return found;
}

static bool ParsePort(SerialReceiveBuffer *buffer, NovAtelReceive *parse) {
  if (NovAtelParse(buffer, parse)) {
    if (parse->proto == kNovAtelProtoBinary) {
      if (parse->binary.header.response == kNovAtelResponseNone) {
        AppendMessage(&parse->binary.header, &g_message);
      } else if (parse->binary.header.sequence == 0) {
        AppendMessage(&parse->binary.header, &g_resp);
      }
    }
    return true;
  }
  return false;
}

void NovAtelReceiveInit(void) {
  SerialParseInit(&g_this_recv);
  SerialParseInit(&g_other_recv);
  memset(&g_this_parse, 0, sizeof(g_this_parse));
  memset(&g_other_parse, 0, sizeof(g_other_parse));
  memset(&g_resp, 0, sizeof(g_resp));
  memset(&g_message, 0, sizeof(g_message));
}

bool NovAtelReceivePoll(NovAtelProto *proto) {
  bool output;
  if (g_last_parsed == &g_this_parse) {
    g_last_parsed = &g_other_parse;
    output = ParsePort(&g_other_recv, &g_other_parse);
  } else {
    g_last_parsed = &g_this_parse;
    output = ParsePort(&g_this_recv, &g_this_parse);
    if (output) {
      // We only send commands on 'this_port'.
      if (g_last_parsed->proto == kNovAtelProtoAbbreviated) {
        PrintResponse(g_last_parsed->data_length, g_last_parsed->data);
        ++g_this_abbr;
      } else if (g_last_parsed->proto == kNovAtelProtoPrompt) {
        ++g_this_prompt;
      }
    }
  }
  *proto = g_last_parsed->proto;
  return output;
}

void NovAtelInsertDataFromThisPort(int32_t length, const uint8_t *data) {
  SerialReadData(length, data, &g_this_recv);
}

void NovAtelInsertDataFromOtherPort(int32_t length, const uint8_t *data) {
  SerialReadData(length, data, &g_other_recv);
}

// Wait for asynchronous response message.
bool NovAtelReceivedResponse(NovAtelMessageId message_id) {
  return ReceivedMessage(message_id, &g_resp);
}

// Wait for asynchronous non-response message.
bool NovAtelReceivedMessage(NovAtelMessageId message_id) {
  return ReceivedMessage(message_id, &g_message);
}

uint32_t NovAtelGetAbbreviatedIndex(void) {
  return g_this_abbr;
}

uint32_t NovAtelGetPromptIndex(void) {
  return g_this_prompt;
}

bool NovAtelGetBinary(const NovAtelHeader **hdr, NovAtelLog *log) {
  assert(hdr != NULL);
  assert(log != NULL);

  if ((g_last_parsed == &g_this_parse || g_last_parsed == &g_other_parse)
      && g_last_parsed->proto == kNovAtelProtoBinary) {
    *hdr = &g_last_parsed->binary.header;
    return NovAtelBinaryDecode(*hdr, g_last_parsed->data, log);
  }
  return false;
}

bool NovAtelGetRtcm3(uint16_t *message_number, int32_t *length,
                     const uint8_t **data) {
  assert(message_number != NULL);
  assert(length != NULL);
  assert(data != NULL);

  if ((g_last_parsed == &g_this_parse || g_last_parsed == &g_other_parse)
      && g_last_parsed->proto == kNovAtelProtoRtcm3) {
    *message_number = g_last_parsed->rtcm3.message_number;
    *length = g_last_parsed->data_length;
    *data = g_last_parsed->data;
    return true;
  }
  return false;
}

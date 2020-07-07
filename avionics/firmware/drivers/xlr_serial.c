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

#include "avionics/firmware/drivers/xlr_serial.h"

#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/serial_parse.h"
#include "avionics/firmware/drivers/xlr_api.h"
#include "common/macros.h"

#define XLR_ASCII_RECEIVE_LENGTH 64

static bool ParseApiNonEscaped(int32_t length, uint8_t ch, XlrApiParse *state,
                               int32_t *parsed) {
  // Default.
  *parsed = 0;

  // Count actual number of bytes in packet.
  ++state->non_escaped_bytes;

  // Initialize parser.
  bool sync = true;
  if (length <= 1) {
    sync = (ch == XLR_FRAME_DELIM);
    state->checksum = XLR_CHKSUM_INIT;
    state->non_escaped_bytes = 1;
    state->packet_length = 0;
  } else {
    int32_t escaped_bytes = length - state->non_escaped_bytes;
    sync = (state->packet_length + escaped_bytes < XLR_RECEIVE_LENGTH);
  }

  // Wait for packet.
  if (state->non_escaped_bytes <= 1) {
    // Do nothing.
  } else if (state->non_escaped_bytes == 2) {
    state->packet_length = ch;
  } else if (state->non_escaped_bytes == 3) {
    state->packet_length <<= 8;
    state->packet_length |= ch;
    state->packet_length += 4;  // Add frame delimiter, length, and checksum.
    sync = (state->packet_length <= XLR_RECEIVE_LENGTH);
  } else if (state->non_escaped_bytes == state->packet_length) {
    sync = (ch == state->checksum);
    *parsed = length;
  } else {
    // Checksum characters between packet length and checksum fields.
    state->checksum = (uint8_t)(state->checksum - ch);
  }
  return sync;
}

static bool ParseApiEscaped(int32_t length, uint8_t ch,
                            XlrApiParseEscaped *state, int32_t *parsed) {
  // Default.
  *parsed = 0;

  if (length <= 1) {
    state->length = 0;
    state->escaped = false;
  } else if (state->escaped) {
    state->escaped = false;
    ch ^= XLR_ESCAPE_XOR;
  } else if (ch == XLR_ESCAPE) {
    state->escaped = true;
  }
  if (!state->escaped && state->length < ARRAYSIZE(state->data)) {
    state->data[state->length] = ch;
    ++state->length;
  }
  return state->escaped || ParseApiNonEscaped(length, ch, &state->api, parsed);
}

static bool ParseAscii(int32_t length, const uint8_t *data, int32_t *parsed) {
  assert(length > 0);
  assert(data != NULL);
  assert(parsed != NULL);

  // Default.
  *parsed = 0;

  // Parse.
  bool sync;
  uint8_t ch = data[length - 1];
  if (length > XLR_ASCII_RECEIVE_LENGTH) {
    sync = false;
  } else if (ch == '\r' || ch == '\n') {
    sync = (length > 1);
    *parsed = length - 1;
  } else {
    sync = isprint(ch);
  }
  return sync;
}

static bool ParseApiNonEscapedWrapper(uint32_t sync_flags, int32_t length,
                                      const uint8_t *data, void *context,
                                      int32_t *parsed) {
  assert(length > 0);
  assert(data != NULL);
  assert(parsed != NULL);

  (void)sync_flags;
  uint8_t ch = data[length - 1];
  return ParseApiNonEscaped(length, ch, (XlrApiParse *)context, parsed);
}

static bool ParseApiEscapedWrapper(uint32_t sync_flags, int32_t length,
                                   const uint8_t *data, void *context,
                                   int32_t *parsed) {
  assert(length > 0);
  assert(data != NULL);
  assert(parsed != NULL);

  (void)sync_flags;
  uint8_t ch = data[length - 1];
  return ParseApiEscaped(length, ch, (XlrApiParseEscaped *)context, parsed);
}

static bool ParseAsciiWrapper(uint32_t sync_flags, int32_t length,
                              const uint8_t *data, void *context,
                              int32_t *parsed) {
  (void)context;
  return (sync_flags & (kSerialSyncPacket | kSerialSyncEol))
      && ParseAscii(length, data, parsed);
}

bool XlrParse(SerialReceiveBuffer *buffer, XlrReceive *rx) {
  const SerialParser kSerialParsers[] = {
    [kXlrProtoApiNonEscaped] = {ParseApiNonEscapedWrapper, &rx->api},
    [kXlrProtoApiEscaped] = {ParseApiEscapedWrapper, &rx->api_escaped},
    [kXlrProtoAscii] = {ParseAsciiWrapper, NULL},
  };
  int32_t protocol = 0;

  rx->data = SerialParse(ARRAYSIZE(kSerialParsers), kSerialParsers, buffer,
                         &protocol, &rx->data_length);
  rx->proto = (XlrProto)protocol;

  if (rx->data != NULL && rx->proto == kXlrProtoApiEscaped) {
    rx->data_length = rx->api_escaped.length;
    rx->data = rx->api_escaped.data;
  }
  return rx->data != NULL;
}

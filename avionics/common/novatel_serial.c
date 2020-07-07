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

#include "avionics/common/novatel_serial.h"

#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/crc.h"
#include "avionics/common/endian.h"
#include "avionics/common/novatel_binary.h"
#include "avionics/common/novatel_types.h"
#include "avionics/common/rtcm3.h"
#include "avionics/common/serial_parse.h"
#include "avionics/common/strings.h"
#include "common/macros.h"

#define MAX_PROMPT_LENGTH 8
#define MAX_ABBREVIATED_LENGTH 128

static bool ParseBinary(int32_t length, const uint8_t *data,
                        NovAtelBinary *bin, int32_t *parsed) {
  assert(bin != NULL);
  assert(length > 0);
  assert(data != NULL);
  assert(parsed != NULL);

  // Default.
  *parsed = 0;
  bool sync = true;

  // Compute CRC32 of entire message.
  if (length <= 1) {
    bin->crc32 = NOVATEL_CRC_INIT;
    bin->packet_length = SERIAL_RECEIVE_SIZE;
  }
  if (length <= bin->packet_length) {
    bin->crc32 = Crc32(bin->crc32, 1, &data[length - 1]);
  }

  // Acquire packet sync.
  uint16_t u16;
  switch (length) {
    case 1:
      sync = (data[0] == NOVATEL_SYNC0);
      break;
    case 2:
      sync = (data[1] == NOVATEL_SYNC1);
      break;
    case 3:
      sync = (data[2] == NOVATEL_SYNC2);
      break;
    case 4:
      bin->header.header_length = data[3];
      sync = (bin->header.header_length >= NOVATEL_HEADER_LENGTH);
      break;
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
    case 10:
      ReadUint16Le(&data[8], &u16);
      bin->header.message_length = u16;
      bin->packet_length = (bin->header.header_length
                            + bin->header.message_length);
      bin->total_length = bin->packet_length + NOVATEL_CRC_LENGTH;
      sync = (bin->total_length <= SERIAL_RECEIVE_SIZE);
      break;
    default:
      if (length == bin->total_length) {
        uint32_t crc32;
        ReadUint32Le(&data[bin->packet_length], &crc32);
        sync = (~bin->crc32 == crc32
                && NovAtelBinaryDecodeHeader(bin->total_length, data,
                                             &bin->header));
        if (sync) {
          *parsed = bin->total_length;
        }
      }
      break;
  }
  return sync;
}

// Handle [COM1] style prompts.
static bool ParsePrompt(int32_t length, const uint8_t *data, int32_t *parsed) {
  assert(length > 0);
  assert(data != NULL);
  assert(parsed != NULL);

  // Default.
  *parsed = 0;

  // Parse.
  bool sync;
  uint8_t ch = data[length - 1];
  if (length == 1) {
    sync = (ch == '[');
  } else if (ch == ']') {
    sync = true;
    *parsed = length;
  } else {
    sync = (length < MAX_PROMPT_LENGTH && isalnum(ch));
  }
  return sync;
}

// Handle user query replies (lines of text).
static bool ParseAbbreviated(int32_t length, const uint8_t *data,
                             int32_t *parsed) {
  assert(length > 0);
  assert(data != NULL);
  assert(parsed != NULL);

  // Default.
  *parsed = 0;

  // Parse.
  bool sync;
  uint8_t ch = data[length - 1];
  if (length == 1) {
    sync = (data[0] == '<');
  } else if (ch == '\n' || ch == '\r') {
    sync = true;
    *parsed = length - 1;
  } else {
    sync = (length < MAX_ABBREVIATED_LENGTH && isprint(ch));
  }
  return sync;
}

static bool ParseAscii(int32_t length, const uint8_t *data, NovAtelAscii *asc,
                       int32_t *parsed) {
  assert(asc != NULL);
  assert(length > 0);
  assert(data != NULL);
  assert(parsed != NULL);

  // Test message synchronization.
  if (data[0] != '#') {
    return false;
  }

  // Reset on first character.
  uint8_t ch = data[length - 1];
  if (length <= 1) {
    asc->fields = 0;
    asc->header_fields = 0;
    asc->crc = NOVATEL_CRC_INIT;
    asc->crc_index = SERIAL_RECEIVE_SIZE;
    asc->in_quotes = false;
  }

  // Default.
  *parsed = 0;
  bool sync = true;

  // Parse.
  if (asc->in_quotes) {
    // Limit characters allowed within quotes.
    sync = isprint(ch);

  } else if (ch == '*') {
    // Mark crc position.
    sync = (length < asc->crc_index);
    asc->crc_index = length;

  } else if (ch == '\r' || ch == '\n') {
    // Complete message.
    uint32_t crc;
    sync = (asc->crc_index + 9 == length
            && ReadHexUint32((const char *)&data[asc->crc_index], 8, &crc) == 8
            && ~asc->crc == crc);
    *parsed = length;

  } else if (asc->crc_index + 8 < length) {
    // Expected <CR> and/or <LF> sequence.
    sync = false;

  } else {
    // Limit characters allowed.
    sync = isprint(ch);
  }

  // Compute CRC32 of all characters between # and *.
  if (1 < length && length < asc->crc_index) {
    asc->crc = Crc32(asc->crc, 1, &ch);
  }

  // Parse fields as data arrives.
  if (ch == '"') {
    asc->in_quotes = !asc->in_quotes;
  } else if (!asc->in_quotes && (ch == ',' || ch == ';' || ch == '*')) {
    if (asc->fields < NOVATEL_ASCII_FIELDS_MAX) {
      asc->field_delim[asc->fields] = length - 1;
      ++asc->fields;
    }
    if (ch == ';') {
      asc->header_fields = asc->fields;
    }
  }
  return sync;
}

static bool ParseBinaryWrapper(uint32_t sync_flags, int32_t length,
                               const uint8_t *data, void *context,
                               int32_t *parsed) {
  (void)sync_flags;
  return ParseBinary(length, data, (NovAtelBinary *)context, parsed);
}

static bool ParsePromptWrapper(uint32_t sync_flags, int32_t length,
                               const uint8_t *data, void *context,
                               int32_t *parsed) {
  (void)context;
  return (sync_flags & (kSerialSyncPacket | kSerialSyncCrLf)) != 0
      && ParsePrompt(length, data, parsed);
}

static bool ParseAbbreviatedWrapper(uint32_t sync_flags, int32_t length,
                                    const uint8_t *data, void *context,
                                    int32_t *parsed) {
  (void)context;
  return (sync_flags & (kSerialSyncPacket | kSerialSyncCrLf)) != 0
      && ParseAbbreviated(length, data, parsed);
}

static bool ParseAsciiWrapper(uint32_t sync_flags, int32_t length,
                              const uint8_t *data, void *context,
                              int32_t *parsed) {
  (void)sync_flags;
  return ParseAscii(length, data, (NovAtelAscii *)context, parsed);
}

static bool ParseRtcm3Wrapper(uint32_t sync_flags, int32_t length,
                              const uint8_t *data, void *context,
                              int32_t *parsed) {
  (void)sync_flags;
  return Rtcm3Parse(length, data, (Rtcm3Receive *)context, parsed);
}

bool NovAtelParse(SerialReceiveBuffer *buffer, NovAtelReceive *rx) {
  const SerialParser kSerialParsers[] = {
    [kNovAtelProtoAbbreviated] = {ParseAbbreviatedWrapper, NULL},
    [kNovAtelProtoAscii] = {ParseAsciiWrapper, &rx->ascii},
    [kNovAtelProtoBinary] = {ParseBinaryWrapper, &rx->binary},
    [kNovAtelProtoPrompt] = {ParsePromptWrapper, NULL},
    [kNovAtelProtoRtcm3] = {ParseRtcm3Wrapper, &rx->rtcm3}
  };
  int32_t protocol = 0;

  rx->data = SerialParse(ARRAYSIZE(kSerialParsers), kSerialParsers, buffer,
                         &protocol, &rx->data_length);
  rx->proto = (NovAtelProto)protocol;

  return rx->data != NULL;
}

void NovAtelAsciiGetField(const NovAtelReceive *rx, int32_t field,
                          int32_t *field_length, int32_t *field_start) {
  assert(rx != NULL);
  assert(field >= 0);
  assert(field_length != NULL);
  assert(field_start != NULL);

  if (field >= rx->ascii.fields) {
    *field_start = 0;
    *field_length = 0;
  } else {
    if (field == 0) {
      *field_start = 0;
    } else {
      *field_start = rx->ascii.field_delim[field - 1] + 1;
    }
    *field_length = rx->ascii.field_delim[field] - *field_start;
  }
}

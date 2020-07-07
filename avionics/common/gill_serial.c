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

#include "avionics/common/gill_serial.h"

#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/common/gill_binary.h"
#include "avionics/common/nmea.h"
#include "avionics/common/serial_parse.h"
#include "avionics/common/strings.h"
#include "common/macros.h"

#define MAX_LINE_LENGTH 128
#define ASCII_STX 0x02
#define ASCII_ETX 0x03

static bool ParseAscii(int32_t length, const uint8_t *data, GillAscii *asc,
                       int32_t *parsed) {
  assert(length > 0);
  assert(data != NULL);
  assert(asc != NULL);
  assert(parsed != NULL);

  // Test message synchronization.
  if (data[0] != ASCII_STX) {
    return false;
  }

  // Reset on first character.
  if (length <= 1) {
    asc->fields = 0;
    asc->checksum = 0;
    asc->checksum_index = SERIAL_RECEIVE_SIZE;
  }

  // Default.
  *parsed = 0;
  bool sync = true;

  // Parse.
  uint8_t ch = data[length - 1];
  if (ch == ASCII_ETX) {
    // Mark checksum position.
    sync = (length < asc->checksum_index);
    asc->checksum_index = length;

  } else if (ch == '\r' || ch == '\n') {
    // Complete message.
    uint32_t checksum;
    sync = (asc->checksum_index + 3 == length
            && ReadHexUint32((const char *)&data[asc->checksum_index], 2,
                             &checksum) == 2 && asc->checksum == checksum);
    *parsed = length;
  } else if (asc->checksum_index + 3 < length) {
    // Expected <CR> and/or <LF> sequence.
    sync = false;

  } else if (length > 1) {
    // Limit characters allowed.
    sync = isprint(ch);
  }

  // Compute XOR of all characters between <STX> and <ETX>.
  if (1 < length && length < asc->checksum_index) {
    asc->checksum ^= ch;
  }

  // Parse fields as data arrives.
  if (ch == ',' || ch == ASCII_ETX) {
    if (asc->fields < GILL_FIELDS_MAX) {
      asc->field_delim[asc->fields] = length - 1;
      ++asc->fields;
    }
  }
  return sync;
}

static bool ParseBinary(int32_t length, const uint8_t *data, GillBinary *bin,
                        int32_t *parsed) {
  assert(length > 0);
  assert(data != NULL);
  assert(bin != NULL);
  assert(parsed != NULL);

  // Default.
  *parsed = 0;
  bool sync = true;

  // Compute checksum: exclusive OR of bytes between start of message
  // identifier and checksum byte.
  if (length < 2) {
    bin->checksum = 0x0;
  } else {
    bin->checksum ^= data[length - 2];
  }

  // Acquire packet sync.
  switch (length) {
    case 1:
      bin->id = (GillBinaryId)data[0];
      bin->length = GillBinaryGetLength(bin->id);
      sync = (bin->length > 0);
      break;
    case 2:
      sync = (data[1] == bin->id);
      break;
    default:
      if (length == bin->length) {
        sync = (data[length - 1] == bin->checksum);
        if (sync) {
          *parsed = length;
        }
      }
      break;
  }
  return sync;
}

// Handle user query replies (lines of text).
static bool ParseLine(int32_t length, const uint8_t *data, int32_t *parsed) {
  assert(length > 0);
  assert(data != NULL);
  assert(parsed != NULL);

  // Default.
  *parsed = 0;

  // Parse.
  bool sync;
  uint8_t ch = data[length - 1];
  if (data[0] == '$' || length > MAX_LINE_LENGTH) {
    sync = false;  // Do not parse NMEA.
  } else if (ch == '\r' || ch == '\n') {
    sync = (length > 1);
    *parsed = length - 1;
  } else {
    sync = isprint(ch);
  }
  return sync;
}

static bool ParseString(const char *str, int32_t length, const uint8_t *data,
                        int32_t *parsed) {
  *parsed = (str[length] == '\0') ? length : 0;
  return str[length - 1] == data[length - 1];
}

static bool ParseAsciiWrapper(uint32_t sync_flags, int32_t length,
                              const uint8_t *data, void *context,
                              int32_t *parsed) {
  (void)sync_flags;
  return ParseAscii(length, data, (GillAscii *)context, parsed);
}

static bool ParseBinaryWrapper(uint32_t sync_flags, int32_t length,
                               const uint8_t *data, void *context,
                               int32_t *parsed) {
  (void)sync_flags;
  return ParseBinary(length, data, (GillBinary *)context, parsed);
}

static bool ParseLineWrapper(uint32_t sync_flags, int32_t length,
                             const uint8_t *data, void *context,
                             int32_t *parsed) {
  (void)context;
  return (sync_flags & (kSerialSyncPacket | kSerialSyncCrLf)) != 0
      && ParseLine(length, data, parsed);
}

static bool ParsePromptWrapper(uint32_t sync_flags, int32_t length,
                               const uint8_t *data, void *context,
                               int32_t *parsed) {
  (void)context;
  return (sync_flags & (kSerialSyncPacket | kSerialSyncCrLf)) != 0
      && ParseString("> ", length, data, parsed);
}

static bool ParseNmeaWrapper(uint32_t sync_flags, int32_t length,
                             const uint8_t *data, void *context,
                             int32_t *parsed) {
  (void)sync_flags;
  return NmeaParse(true, length, data, (NmeaReceive *)context, parsed);
}

bool GillParse(SerialReceiveBuffer *buffer, GillReceive *rx) {
  const SerialParser kSerialParsers[] = {
    [kGillProtoAscii] = {ParseAsciiWrapper, &rx->ascii},
    [kGillProtoBinary] = {ParseBinaryWrapper, &rx->binary},
    [kGillProtoLine] = {ParseLineWrapper, NULL},
    [kGillProtoPrompt] = {ParsePromptWrapper, NULL},
    [kGillProtoNmea] = {ParseNmeaWrapper, &rx->nmea}
  };
  int32_t protocol = 0;

  rx->data = SerialParse(ARRAYSIZE(kSerialParsers), kSerialParsers, buffer,
                         &protocol, &rx->data_length);
  rx->proto = (GillProto)protocol;

  return rx->data != NULL;
}

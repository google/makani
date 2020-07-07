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

#include "avionics/common/septentrio_serial.h"

#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/crc.h"
#include "avionics/common/endian.h"
#include "avionics/common/rtcm3.h"
#include "avionics/common/septentrio_types.h"
#include "avionics/common/serial_parse.h"
#include "common/macros.h"

static bool IsPrompt(int32_t length, const uint8_t *data) {
  // Prompt marks end of transmission (and atomic sequence).
  if (length == 5 && memcmp(data, "COM1>", (size_t)length) == 0) {
    return true;
  }
  if (length == 5 && memcmp(data, "COM2>", (size_t)length) == 0) {
    return true;
  }
  return false;
}

static bool IsStop(int32_t length, const uint8_t *data) {
  // Stop marks end of receiver operation (and atomic sequence).
  return length == 5 && memcmp(data, "STOP>", (size_t)length) == 0;
}

static bool IsBlockEnd(int32_t length, const uint8_t *data) {
  // Block end marks end of atomic sequence (not end of transmission).
  return length == 5 && memcmp(data, "---->", (size_t)length) == 0;
}

static bool ParseAscii(int32_t length, const uint8_t *data,
                       SeptentrioAscii *ascii, int32_t *parsed) {
  assert(ascii != NULL);
  assert(length > 0);
  assert(data != NULL);
  assert(parsed != NULL);

  // Default.
  *parsed = 0;
  bool sync = true;

  // Acquire packet sync.
  char c = (char)data[length - 1];
  switch (length) {
    case 1:
      ascii->line = 0;
      ascii->complete = false;
      ascii->stop = false;
      sync = (c == '$');
      break;
    case 2:
      sync = (c == 'R' || c == 'T' || c == '-');
      break;
    case 3:
      if (data[1] == '-') {
        // Sync on "$--".
        sync = (c == '-');
      } else if (data[1] == 'R') {
        // Sync on "$R:", "$R;", and "$R?".
        sync = (c == ':' || c == ';' || c == '?');
      }
      break;
    default:
      // Only parse atomic sequences (we can decode these sequences elsewhere).
      if (c == '\r' || c == '\n') {
        ascii->line = length;
      } else if (!isprint((int32_t)c)) {
        sync = false;
      } else if (IsBlockEnd(length - ascii->line, &data[ascii->line])) {
        *parsed = length;
      } else if (IsStop(length - ascii->line, &data[ascii->line])) {
        ascii->complete = true;
        ascii->stop = true;
        *parsed = length;
      } else if (IsPrompt(length - ascii->line, &data[ascii->line])) {
        ascii->complete = true;
        *parsed = length;
      }
      break;
  }
  return sync;
}

static bool ParseSbf(int32_t length, const uint8_t *data, SeptentrioSbf *sbf,
                     int32_t *parsed) {
  assert(sbf != NULL);
  assert(length > 0);
  assert(data != NULL);
  assert(parsed != NULL);

  // Default.
  *parsed = 0;
  bool sync = true;

  // Compute CRC16-CCITT of all bytes after CRC header field.
  if (length <= 4) {
    sbf->crc16_sum = 0;
  } else {
    sbf->crc16_sum = Crc16Ccitt(sbf->crc16_sum, 1, &data[length - 1]);
  }

  // Acquire packet sync.
  uint16_t u16;
  switch (length) {
    case 1:
      sync = (data[0] == '$');
      break;
    case 2:
      sync = (data[1] == '@');
      break;
    case 3:
      break;
    case 4:
      ReadUint16Le(&data[2], &sbf->crc16_check);
      break;
    case 5:
      break;
    case 6:
      ReadUint16Le(&data[4], &u16);
      sbf->header.block_id = (SeptentrioId)(u16 & 0x1FFF);
      sbf->header.block_rev = (u16 >> 13);
      break;
    case 7:
      break;
    case 8:
      ReadUint16Le(&data[6], &u16);
      sbf->header.header_length = 8;
      sbf->header.data_length = (int32_t)u16 - sbf->header.header_length;
      sbf->total_length = sbf->header.header_length + sbf->header.data_length;
      sync = (8 <= u16 && u16 <= SERIAL_RECEIVE_SIZE && u16 % 4 == 0);
      // Fall through.
    default:
      if (sbf->total_length == length && sync) {
        sync = (sbf->crc16_check == sbf->crc16_sum);
        if (sync) {
          *parsed = length;
        }
      }
      break;
  }
  return sync;
}

static bool ParseSnmp(int32_t length, const uint8_t *data, SeptentrioSnmp *snmp,
                      int32_t *parsed) {
  assert(snmp != NULL);
  assert(length > 0);
  assert(data != NULL);
  assert(parsed != NULL);

  // Default.
  *parsed = 0;
  bool sync = true;

  // Compute XOR of PDU.
  if (length < 8) {
    snmp->xor_sum = 0;
  } else {
    snmp->xor_sum ^= data[length - 1];
  }

  // Acquire packet sync.
  uint16_t u16;
  switch (length) {
    case 1:
      sync = (data[0] == '$');
      break;
    case 2:
      sync = (data[1] == '&');
      break;
    case 3:
      snmp->version = data[2];
      break;
    case 4:
      snmp->xor_check = data[3];
      break;
    case 5:
      break;
    case 6:
      ReadUint16Le(&data[4], &u16);
      snmp->header_length = 8;
      snmp->data_length = (int32_t)u16;
      sync = (4 <= u16 && u16 <= SERIAL_RECEIVE_SIZE - 8);
      break;
    case 7:
      break;
    case 8:
      break;
    default:
      if (snmp->header_length + snmp->data_length == length) {
        sync = (snmp->xor_check == snmp->xor_sum);
        if (sync) {
          *parsed = length;
        }
      }
      break;
  }
  return sync;
}

static bool ParseAsciiWrapper(uint32_t sync_flags, int32_t length,
                              const uint8_t *data, void *context,
                              int32_t *parsed) {
  (void)sync_flags;
  return ParseAscii(length, data, (SeptentrioAscii *)context, parsed);
}

static bool ParseSbfWrapper(uint32_t sync_flags, int32_t length,
                            const uint8_t *data, void *context,
                            int32_t *parsed) {
  (void)sync_flags;
  return ParseSbf(length, data, (SeptentrioSbf *)context, parsed);
}

static bool ParseSnmpWrapper(uint32_t sync_flags, int32_t length,
                             const uint8_t *data, void *context,
                             int32_t *parsed) {
  (void)sync_flags;
  return ParseSnmp(length, data, (SeptentrioSnmp *)context, parsed);
}

static bool ParseRtcm3Wrapper(uint32_t sync_flags, int32_t length,
                              const uint8_t *data, void *context,
                              int32_t *parsed) {
  (void)sync_flags;
  return Rtcm3Parse(length, data, (Rtcm3Receive *)context, parsed);
}

bool SeptentrioParse(SerialReceiveBuffer *buffer, SeptentrioReceive *rx) {
  const SerialParser kSerialParsers[] = {
    [kSeptentrioProtoAscii] = {ParseAsciiWrapper, &rx->ascii},
    [kSeptentrioProtoSbf] = {ParseSbfWrapper, &rx->sbf},
    [kSeptentrioProtoSnmp] = {ParseSnmpWrapper, &rx->snmp},
    [kSeptentrioProtoRtcm3] = {ParseRtcm3Wrapper, &rx->rtcm3}
  };
  int32_t protocol = 0;

  rx->data = SerialParse(ARRAYSIZE(kSerialParsers), kSerialParsers, buffer,
                         &protocol, &rx->data_length);
  rx->proto = (SeptentrioProto)protocol;

  return rx->data != NULL;
}

bool SeptentrioGotPrompt(const SeptentrioReceive *rx) {
  assert(rx != NULL && rx->proto == kSeptentrioProtoAscii);
  return rx->ascii.complete;
}

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

#include "avionics/common/serial_parse.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef __linux__
#include "avionics/firmware/cpu/sci.h"
#endif  // !__linux__

#ifdef __linux__
#include <unistd.h>  // For read().
#endif  // __linux__

#include "common/macros.h"

// Constant parsing offset in SerialReceiveBuffer::data to store a buffer of
// previously parsed bytes. This offset facilitates newline parsing. See
// GetSyncFlags().
#define SERIAL_PARSE_OFFSET 2

COMPILE_ASSERT(SERIAL_PARSE_OFFSET >= 2,
               SERIAL_PARSE_OFFSET_must_allow_for_newline_detection);

static uint32_t GetSyncFlags(const SerialReceiveBuffer *rx,
                             const uint8_t *data) {
  uint32_t sync_flags = 0x0;

  if (rx->synchronized) {
    sync_flags |= kSerialSyncPacket;
  }
  if (data[-2] == '\r' && data[-1] == '\n') {
    sync_flags |= kSerialSyncCrLf;
  }
  if (data[-1] == '\r') {
    sync_flags |= kSerialSyncCr;
  }
  if (data[-1] == '\n') {
    sync_flags |= kSerialSyncLf;
  }
  return sync_flags;
}

static bool ParseProtocols(int32_t protocols, const SerialParser parsers[],
                           int32_t length, const uint8_t *data,
                           SerialReceiveBuffer *rx, int32_t *parsed,
                           int32_t *protocol) {
  assert(length > 0);

  // Default.
  *parsed = 0;

  // Reset sync bit flag for all parsers on first byte.
  if (length <= 1) {
    rx->sync_bits = (1U << protocols) - 1;
  }

  // Newline detection and packet synchronization.
  uint32_t sync_flags = GetSyncFlags(rx, data);

  // Iterate through all parsers until one succeeds.
  for (int32_t i = 0; i < protocols && *parsed == 0; ++i) {
    assert(parsers[i].func != NULL);
    uint32_t mask = 1U << i;
    if ((rx->sync_bits & mask) != 0 && parsers[i].func(
            sync_flags, length, data, parsers[i].context, parsed)) {
      *protocol = i;
    } else {
      *parsed = 0;
      rx->sync_bits &= ~mask;
    }
  }
  return rx->sync_bits != 0;
}

void SerialParseInit(SerialReceiveBuffer *rx) {
  memset(rx, 0, sizeof(*rx));
  rx->synchronized = true;
  rx->offset = SERIAL_PARSE_OFFSET;
  rx->length = SERIAL_PARSE_OFFSET;
}

const uint8_t *SerialParse(int32_t protocols, const SerialParser parsers[],
                           SerialReceiveBuffer *rx, int32_t *protocol,
                           int32_t *length) {
  assert(parsers != NULL);
  assert(rx != NULL);
  assert(protocol != NULL);
  assert(length != NULL);
  assert(SERIAL_PARSE_OFFSET <= rx->offset);
  assert(rx->offset <= rx->length);
  assert(0 <= rx->parsed && rx->parsed <= rx->offset - SERIAL_PARSE_OFFSET);

  int32_t shift = SERIAL_PARSE_OFFSET + rx->parsed;
  if (rx->parsed > 0) {
    rx->parsed = 0;
    rx->offset = shift;
  }
  while (rx->offset < rx->length && rx->parsed == 0) {
    ++rx->offset;
    if (!ParseProtocols(protocols, parsers, rx->offset - shift,
                        &rx->data[shift], rx, &rx->parsed, protocol)) {
      ++shift;             // Sync failed, drop garbage byte.
      rx->offset = shift;  // Reset parsing length to zero.
      rx->parsed = 0;      // Parser failed, set parsed length to zero.
      rx->synchronized = false;
    } else if (rx->parsed == 0 && shift == SERIAL_PARSE_OFFSET
               && rx->offset == SERIAL_RECEIVE_SIZE) {
      ++shift;             // Buffer overrun, drop byte.
      rx->offset = shift;  // Reset parsing length to zero.
      rx->synchronized = false;
    }
  }
  shift -= SERIAL_PARSE_OFFSET;
  if (shift > 0) {
    memmove(&rx->data[0], &rx->data[shift], (size_t)(rx->length - shift));
    rx->length -= shift;
    rx->offset -= shift;
  }
  rx->synchronized |= (rx->parsed > 0);

  *length = rx->parsed;
  return (rx->parsed > 0) ? &rx->data[SERIAL_PARSE_OFFSET] : NULL;
}

int32_t SerialReadGetAvailableBytes(const SerialReceiveBuffer *rx) {
  return SERIAL_RECEIVE_SIZE - rx->length;
}

int32_t SerialReadData(int32_t length, const uint8_t *data,
                       SerialReceiveBuffer *rx) {
  int32_t count = length;
  if (rx->length + count > SERIAL_RECEIVE_SIZE) {
    count = SERIAL_RECEIVE_SIZE - rx->length;
  }
  memcpy(&rx->data[rx->length], data, (size_t)count);
  rx->length += count;
  return count;
}

#ifndef __linux__
bool SerialPollSci(const SciDevice *dev, SerialReceiveBuffer *rx) {
  int32_t available = SERIAL_RECEIVE_SIZE - rx->length;
  int32_t count = SciRead(dev, available, &rx->data[rx->length]);
  if (count > 0) {
    rx->length += count;
  }
  return count > 0 || rx->parsed > 0;
}
#endif  // !__linux__

#ifdef __linux__
bool SerialPollDevice(int32_t device_fd, SerialReceiveBuffer *rx) {
  size_t available = (size_t)(SERIAL_RECEIVE_SIZE - rx->length);
  int32_t count = (int32_t)read(device_fd, &rx->data[rx->length], available);
  if (count > 0) {
    rx->length += count;
  }
  return count > 0 || rx->parsed > 0;
}
#endif  // __linux__

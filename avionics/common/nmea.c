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

#include "avionics/common/nmea.h"

#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/common/serial_parse.h"
#include "avionics/common/strings.h"

bool NmeaParse(bool require_checksum, int32_t length, const uint8_t *data,
               NmeaReceive *nmea, int32_t *parsed) {
  assert(length > 0);
  assert(data != NULL);
  assert(nmea != NULL);
  assert(parsed != NULL);

  // Test message synchronization.
  if (data[0] != '$') {
    return false;
  }

  // Reset on first character.
  if (length <= 1) {
    nmea->fields = 0;
    nmea->checksum = 0;
    nmea->checksum_index = SERIAL_RECEIVE_SIZE;
  }

  // Default.
  *parsed = 0;
  bool sync = true;

  // Parse.
  uint8_t ch = data[length - 1];
  if (ch == '*') {
    // Mark crc position.
    sync = (length < nmea->checksum_index);
    nmea->checksum_index = length;

  } else if (ch == '\r' || ch == '\n') {
    // Found complete message.
    if (require_checksum || nmea->checksum_index < length) {
      uint32_t checksum;
      sync = (nmea->checksum_index + 3 == length
              && ReadHexUint32((const char *)&data[nmea->checksum_index], 2,
                               &checksum) == 2 && nmea->checksum == checksum);
    } else if (nmea->fields < NMEA_FIELDS_MAX) {
      nmea->field_delim[nmea->fields] = length - 1;
      ++nmea->fields;
    }
    *parsed = length;

  } else if (nmea->checksum_index + 2 < length) {
    // Expected <CR> and/or <LF> sequence.
    sync = false;

  } else {
    // Limit characters allowed.
    sync = isprint(ch);
  }

  // Compute XOR of all characters between $ and *.
  if (1 < length && length < nmea->checksum_index) {
    nmea->checksum ^= ch;
  }

  // Parse fields as data arrives.
  if (ch == ',' || ch == '*') {
    if (nmea->fields < NMEA_FIELDS_MAX) {
      nmea->field_delim[nmea->fields] = length - 1;
      ++nmea->fields;
    }
  }
  return sync;
}

void NmeaGetField(const NmeaReceive *nmea, int32_t field, int32_t *field_length,
                  int32_t *field_start) {
  assert(nmea != NULL);
  assert(field >= 0);
  assert(field_length != NULL);
  assert(field_start != NULL);

  if (field >= nmea->fields) {
    *field_start = 0;
    *field_length = 0;
  } else if (field == 0) {
    *field_start = 0;
    *field_length = nmea->field_delim[0];
  } else {
    *field_start = nmea->field_delim[field - 1] + 1;
    *field_length = nmea->field_delim[field] - *field_start;
  }
}

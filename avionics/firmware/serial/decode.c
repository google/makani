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

#include "avionics/firmware/serial/decode.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avionics/firmware/serial/serial_params.h"

static const uint32_t kLegacySerialParamsV1VersionNumber = 0x02142937;
static const uint32_t kLegacySerialParamsV2VersionNumber = 0x7c8945bf;

static bool ConvertSerialParamsV1(const SerialParamsV1 *in, SerialParams *out) {
  snprintf(out->serial_number, sizeof(out->serial_number),
           "%ld", in->serial_number);

  strlcpy(out->part_name, in->part_name, sizeof(out->part_name));

  if (in->google_part_number[0] != '\0') {
    strlcpy(out->part_number, in->google_part_number, sizeof(out->part_number));
  } else if (in->makani_part_number[0] != '\0') {
    strlcpy(out->part_number, in->makani_part_number, sizeof(out->part_number));
  } else {
    return false;
  }

  out->hardware_revision = in->hardware_revision;
  out->date_of_manufacture = in->date_of_manufacture;

  return true;
}

static bool ConvertSerialParamsV2(const SerialParamsV2 *in, SerialParams *out) {
  snprintf(out->serial_number, sizeof(out->serial_number),
           "%ld", in->serial_number);

  strlcpy(out->part_name, "Carrier", sizeof(out->part_name));

  const char *rev_str;
  rev_str = strchr(in->part_number, '.');
  if (rev_str == NULL) {
    return false;
  }

  size_t part_num_len = rev_str - in->part_number;
  if (part_num_len >= sizeof(out->part_number)) {
    return false;
  }

  strncpy(out->part_number, in->part_number, part_num_len);
  out->part_number[part_num_len] = '\0';

  ++rev_str;
  // We assume single digit revisions.
  if (*rev_str < '0' || *rev_str > '9') {
    return false;
  }

  out->hardware_revision = *rev_str - '0';
  out->date_of_manufacture = in->date_of_manufacture;

  return true;
}

static bool DecodeSerialParams(
    const void *(* const get_func)(uint32_t *version_number),
    SerialParams *out) {
  const void *data;
  uint32_t version_number;

  assert(get_func != NULL);
  data = get_func(&version_number);
  if (data == NULL) {
    return false;
  }

  if (version_number == kSerialParamsCrc) {
    memcpy(out, data, sizeof(*out));
    return true;
  } else if (version_number == kSerialParamsV2Crc
             || version_number == kLegacySerialParamsV2VersionNumber) {
    return ConvertSerialParamsV2(data, out);
  } else if (version_number == kLegacySerialParamsV1VersionNumber) {
    return ConvertSerialParamsV1(data, out);
  }

  return false;
}

const SerialParams *ReadSerialParams(
    const void *(* const get_func)(uint32_t *version_number),
    bool *valid,
    SerialParams *out) {
  *valid = *valid || DecodeSerialParams(get_func, out);
  if (*valid) {
    return out;
  }
  return NULL;
}

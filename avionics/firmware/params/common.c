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

#include "avionics/firmware/params/common.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/common/crc.h"
#include "avionics/firmware/params/param_header.h"

static const ParamHeader *GetParamHeader(const void *section_data) {
  return (const ParamHeader *)section_data;
}

static const uint8_t *GetParamData(const void *section_data) {
  return (const uint8_t*)section_data + sizeof(ParamHeader);
}

static int32_t GetParamDataMaxSize(int32_t section_size) {
  return section_size - sizeof(ParamHeader);
}

static bool VerifyParamHeader(const ParamHeader *header, int32_t max_size,
                              const uint8_t *data) {
  if (header->param_format_version != kParamHeaderVersionCurrent)
    return false;
  if (header->data_length > max_size)
    return false;
  if (header->data_crc != Crc32(0U, header->data_length, data))
    return false;
  return true;
}

const void *GetParams(int32_t section_size, const void *section_data,
                      uint32_t *version_number) {
  const ParamHeader *header = GetParamHeader(section_data);
  const uint8_t *data = GetParamData(section_data);
  int32_t max_size = GetParamDataMaxSize(section_size);

  if (!VerifyParamHeader(header, max_size, data)) {
    return NULL;
  }
  *version_number = header->version_number;
  return data;
}

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

#include "avionics/firmware/params/param_server.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/firmware/cpu/memcpy.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/firmware/params/flash_internal.h"
#include "avionics/firmware/params/i2c_internal.h"
#include "avionics/firmware/params/param_section.h"

static ParamRequestMessage g_param_request;
static ParamResponseMessage g_param_response;

static const void *GetParamSectionData(ParamSection section) {
  const ParamSectionInfo *info = GetParamSectionInfo(section);

  assert(info->location == kParamLocationFlash
         || info->location == kParamLocationI2c);

  switch (info->location) {
    case kParamLocationFlash:
      return GetFlashParamSectionData(info);
    case kParamLocationI2c:
      return GetI2cParamSectionData(info);
    default:
      return NULL;
  }
}

static int32_t GetParamSectionSize(ParamSection section) {
  const ParamSectionInfo *info = GetParamSectionInfo(section);

  assert(info->location == kParamLocationFlash
         || info->location == kParamLocationI2c);

  switch (info->location) {
    case kParamLocationFlash:
      return GetFlashParamSectionSize(info);
    case kParamLocationI2c:
      return GetI2cParamSectionSize(info);
    default:
      return 0;
  }
}

static bool ParamServerIsValidSection(ParamSection section, uint32_t flags) {
  switch (section) {
    case kParamSectionConfig:
      if (flags & kParamServerConfig) {
        return true;
      }
      break;
    case kParamSectionCalib:
      if (flags & kParamServerCalib) {
        return true;
      }
      break;
    case kParamSectionSerial:
      if (flags & kParamServerSerial) {
        return true;
      }
      break;
    case kParamSectionCarrierSerial:
      if (flags & kParamServerCarrierSerial) {
        return true;
      }
      break;
    default:
      break;
  }

  return false;
}

// TODO: Should we print an error on failures here?
void ParamServerPoll(uint32_t flags) {
  if (CvtGetParamRequestMessage(kAioNodeOperator, &g_param_request,
                                NULL, NULL) == false) {
    return;
  }

  if (g_param_request.node_id != AppConfigGetAioNode()) {
    return;
  }

  ParamSection section = g_param_request.section;
  uint32_t offset = g_param_request.offset;

  if (!ParamServerIsValidSection(section, flags)) {
    return;
  }

  size_t section_size = GetParamSectionSize(section);
  const void *data;
  if ((data = GetParamSectionData(section)) == NULL) {
    return;
  }

  uint32_t length = sizeof(g_param_response.data);
  g_param_response.section = section;
  g_param_response.offset = offset;

  if (offset >= section_size) {
    length = 0;
  }
  if (length > sizeof(g_param_response.data)) {
    length = sizeof(g_param_response.data);
  }
  if (length > 0 && offset + length > section_size) {
    length = section_size - offset;
  }

  g_param_response.length = length;
  if (length > 0) {
    FastCopy(length, data + offset, g_param_response.data);
  }

  NetSendAioParamResponseMessage(&g_param_response);
}

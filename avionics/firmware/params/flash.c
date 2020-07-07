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

#include "avionics/firmware/params/flash.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/firmware/params/common.h"
#include "avionics/firmware/params/flash_internal.h"
#include "avionics/firmware/params/param_header.h"
#include "avionics/firmware/params/param_section.h"
#include "common/macros.h"

#define FLASH_SEGMENT_SIZE  (16 * 1024)
#define MAX_PARAM_DATA_SIZE (FLASH_SEGMENT_SIZE - sizeof(ParamHeader))

COMPILE_ASSERT(sizeof(ParamHeader) <= FLASH_SEGMENT_SIZE,
               ParamHeader_must_fit_within_one_flash_bank);
COMPILE_ASSERT(sizeof(ParamHeader) % 4 == 0,
               ParamHeader_must_be_a_multiple_of_4_bytes);

static const void *GetFlashParams(const ParamSectionInfo *info,
                                  uint32_t *version) {
  assert(info->location == kParamLocationFlash);
  return GetParams(GetFlashParamSectionSize(info),
                   GetFlashParamSectionData(info),
                   version);
}

const void *GetFlashParamSectionData(const ParamSectionInfo *info) {
  assert(info->location == kParamLocationFlash);
  return info->flash_info.begin;
}

int32_t GetFlashParamSectionSize(const ParamSectionInfo *info) {
  assert(info->location == kParamLocationFlash);
  return info->flash_info.end - info->flash_info.begin;
}

const void *GetConfigParamsRaw(uint32_t *version_number) {
  return GetFlashParams(GetParamSectionInfo(kParamSectionConfig),
                        version_number);
}

const void *GetCalibParamsRaw(uint32_t *version_number) {
  return GetFlashParams(GetParamSectionInfo(kParamSectionCalib),
                        version_number);
}

const void *GetSerialParamsRaw(uint32_t *version_number) {
  return GetFlashParams(GetParamSectionInfo(kParamSectionSerial),
                        version_number);
}

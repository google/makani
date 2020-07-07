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

#ifndef AVIONICS_FIRMWARE_PARAMS_PARAM_SECTION_H_
#define AVIONICS_FIRMWARE_PARAMS_PARAM_SECTION_H_

#include <stdint.h>

#include "avionics/firmware/params/param_types.h"

typedef enum {
  kParamLocationForceSigned = -1,
  kParamLocationInvalid = 0,
  kParamLocationFlash = 1,
  kParamLocationI2c = 2,
} ParamLocation;

typedef struct {
  ParamLocation location;
  union {
    struct {
      const void *begin;
      const void *end;
    } flash_info;
    struct {
      uint16_t offset;
    } i2c_info;
  };
} ParamSectionInfo;

const ParamSectionInfo *GetParamSectionInfo(ParamSection section);

#endif  // AVIONICS_FIRMWARE_PARAMS_PARAM_SECTION_H_

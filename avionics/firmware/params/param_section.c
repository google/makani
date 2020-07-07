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

#include "avionics/firmware/params/param_section.h"

#include <assert.h>
#include <stddef.h>

#include "avionics/firmware/startup/ldscript.h"
#include "common/macros.h"

static const ParamSectionInfo kParamSections[] = {
  [kParamSectionSerial] = {
    .location = kParamLocationFlash,
    .flash_info = {
      .begin = &ldscript_serial_param_begin,
      .end = &ldscript_serial_param_end,
    }
  },
  [kParamSectionConfig] = {
    .location = kParamLocationFlash,
    .flash_info = {
      .begin = &ldscript_config_param_begin,
      .end = &ldscript_config_param_end,
    }
  },
  [kParamSectionCalib] = {
    .location = kParamLocationFlash,
    .flash_info = {
      .begin = &ldscript_calib_param_begin,
      .end = &ldscript_calib_param_end,
    }
  },
  [kParamSectionCarrierSerial] = {
    .location = kParamLocationI2c,
    .i2c_info = {
      .offset = 0x0,
    }
  },
};

const ParamSectionInfo *GetParamSectionInfo(ParamSection section) {
  assert(section >= 0);
  assert(section < ARRAYSIZE(kParamSections));

  if (kParamSections[section].location == kParamLocationInvalid) {
    return NULL;
  }
  return &kParamSections[section];
}

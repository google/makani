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

#include "avionics/firmware/params/i2c.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/eeprom24.h"
#include "avionics/firmware/params/common.h"
#include "avionics/firmware/params/i2c_internal.h"
#include "avionics/firmware/params/param_header.h"
#include "avionics/firmware/params/param_section.h"
#include "avionics/firmware/serial/serial_params.h"
#include "common/macros.h"

#define I2C_PARAM_BUFFER_SIZE 128

COMPILE_ASSERT(sizeof(ParamHeader) + sizeof(SerialParamsV1)
               <= I2C_PARAM_BUFFER_SIZE,
               ParamHeader_and_SerialParamsV1_must_fit_in_i2c_buffer);
COMPILE_ASSERT(sizeof(ParamHeader) + sizeof(SerialParamsV2)
               <= I2C_PARAM_BUFFER_SIZE,
               ParamHeader_and_SerialParamsV2_must_fit_in_i2c_buffer);
COMPILE_ASSERT(sizeof(ParamHeader) + sizeof(SerialParams)
               <= I2C_PARAM_BUFFER_SIZE,
               ParamHeader_and_SerialParams_must_fit_in_i2c_buffer);
COMPILE_ASSERT(I2C_PARAM_BUFFER_SIZE % 4 == 0,
               I2C_PARAM_BUFFER_SIZE_must_be_a_multiple_of_4_bytes);

// Declare as uint32_t to ensure 32bit alignment.
static uint32_t g_i2c_param_buffer_storage[I2C_PARAM_BUFFER_SIZE / 4];
static void * const g_i2c_param_buffer = g_i2c_param_buffer_storage;
static int32_t g_i2c_param_buffer_offset = -1;

static bool I2cParamReadBuffer(uint16_t offset) {
  if (g_i2c_param_buffer_offset == offset) {
    return true;
  }

  if (!Eeprom24ReadSync(offset, I2C_PARAM_BUFFER_SIZE, g_i2c_param_buffer)) {
    g_i2c_param_buffer_offset = -1;
    return false;
  }

  g_i2c_param_buffer_offset = offset;
  return true;
}

static const void *GetI2cParams(const ParamSectionInfo *info,
                                uint32_t *version) {
  assert(info->location == kParamLocationI2c);
  return GetParams(GetI2cParamSectionSize(info),
                   GetI2cParamSectionData(info),
                   version);
}

const void *GetI2cParamSectionData(const ParamSectionInfo *info) {
  assert(info->location == kParamLocationI2c);
  for (int try = 0; try < 2; ++try) {
    if (I2cParamReadBuffer(info->i2c_info.offset)) {
      return g_i2c_param_buffer;
    }
  }
  return NULL;
}

int32_t GetI2cParamSectionSize(const ParamSectionInfo *info) {
  assert(info->location == kParamLocationI2c);
  (void)info;
  return I2C_PARAM_BUFFER_SIZE;
}

const void *GetCarrierSerialParamsRaw(uint32_t *version_number) {
  return GetI2cParams(GetParamSectionInfo(kParamSectionCarrierSerial),
                      version_number);
}

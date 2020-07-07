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

#ifndef AVIONICS_FIRMWARE_PARAMS_FLASH_PARAMS_HEADER_H_
#define AVIONICS_FIRMWARE_PARAMS_FLASH_PARAMS_HEADER_H_

#include <stdint.h>

typedef struct {
  uint32_t param_format_version;
  int32_t unused;  // Previously AppType/HardwareType. Now set to -1 by default.
  int32_t data_length;
  uint32_t checksum;
  uint32_t version_number;
} FlashParamHeader;

#define PARAM_FORMAT_VERSION 0UL

#endif  // AVIONICS_FIRMWARE_PARAMS_FLASH_PARAMS_HEADER_H_

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

#include "avionics/firmware/drivers/mcp342x_types.h"

#include <assert.h>
#include <stdint.h>


uint8_t Mcp342xBuildConfig(const Mcp342xConfig *config) {
  assert((config->channel & 0x03) == config->channel);
  assert((config->mode & 0x01) == config->mode);
  assert((config->sps & 0x03) == config->sps);
  assert((config->gain & 0x03) == config->gain);

  return (uint8_t)((config->channel & 0x03) << 5
                   | (config->mode & 0x01) << 4
                   | (config->sps & 0x03) << 2
                   | (config->gain & 0x03)
                   | 0x80);
}

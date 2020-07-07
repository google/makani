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

#ifndef AVIONICS_FIRMWARE_PARAMS_PARAM_SERVER_H_
#define AVIONICS_FIRMWARE_PARAMS_PARAM_SERVER_H_

#include <stdint.h>

typedef enum {
  kParamServerConfig = (1 << 0),
  kParamServerCalib  = (1 << 1),
  kParamServerSerial = (1 << 2),
  kParamServerCarrierSerial = (1 << 2),
} ParamServerFlag;

void ParamServerPoll(uint32_t flags);

#endif  // AVIONICS_FIRMWARE_PARAMS_PARAM_SERVER_H_

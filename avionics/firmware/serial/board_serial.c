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

#include "avionics/firmware/serial/board_serial.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/firmware/params/flash.h"
#include "avionics/firmware/serial/decode.h"
#include "avionics/firmware/serial/serial_params.h"

static SerialParams g_board_serial_params;
static bool g_board_serial_params_valid = false;

static const SerialParams *ReadBoardSerialParams(void) {
  return ReadSerialParams(GetSerialParamsRaw,
                          &g_board_serial_params_valid,
                          &g_board_serial_params);
}

void BoardSerialParamsInit(void) {
  ReadBoardSerialParams();
}

const SerialParams *GetBoardSerialParams(void) {
  if (g_board_serial_params_valid) {
    return &g_board_serial_params;
  } else {
    return ReadBoardSerialParams();
  }
}

int32_t GetBoardHardwareRevision(void) {
  const SerialParams *params = GetBoardSerialParams();
  if (params != NULL) {
    return params->hardware_revision;
  } else {
    return -1;
  }
}

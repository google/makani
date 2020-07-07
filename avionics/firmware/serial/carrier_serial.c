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

#include "avionics/firmware/serial/carrier_serial.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/firmware/params/params.h"
#include "avionics/firmware/params/i2c.h"
#include "avionics/firmware/serial/decode.h"
#include "avionics/firmware/serial/serial_params.h"

static SerialParams g_carrier_serial_params;
static bool g_carrier_serial_params_valid = false;

static const SerialParams *ReadCarrierSerialParams(void) {
  return ReadSerialParams(GetCarrierSerialParamsRaw,
                          &g_carrier_serial_params_valid,
                          &g_carrier_serial_params);
}

void CarrierSerialParamsInit(void) {
  ReadCarrierSerialParams();
}

const SerialParams *GetCarrierSerialParams(void) {
  if (g_carrier_serial_params_valid) {
    return &g_carrier_serial_params;
  } else {
    return ReadCarrierSerialParams();
  }
}

int32_t GetCarrierHardwareRevision(void) {
  const SerialParams *params = GetCarrierSerialParams();
  if (params != NULL) {
    return params->hardware_revision;
  } else {
    return -1;
  }
}

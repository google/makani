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

#ifndef AVIONICS_FIRMWARE_DRIVERS_XLR_SERIAL_H_
#define AVIONICS_FIRMWARE_DRIVERS_XLR_SERIAL_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/serial_parse.h"

#define XLR_RECEIVE_LENGTH SERIAL_RECEIVE_SIZE

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  kXlrProtoApiNonEscaped = 0,  // Prefer over kXlrProtoApiEscaped.
  kXlrProtoApiEscaped    = 1,
  kXlrProtoAscii         = 2,
} XlrProto;

typedef struct {
  int32_t non_escaped_bytes;
  int32_t packet_length;
  uint8_t checksum;
} XlrApiParse;

typedef struct {
  int32_t length;
  uint8_t data[XLR_RECEIVE_LENGTH];
  bool escaped;
  XlrApiParse api;
} XlrApiParseEscaped;

typedef struct {
  int32_t data_length;
  const uint8_t *data;
  SerialReceiveBuffer buffer;
  XlrProto proto;
  XlrApiParse api;
  XlrApiParseEscaped api_escaped;
} XlrReceive;

bool XlrParse(SerialReceiveBuffer *buffer, XlrReceive *rx);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_FIRMWARE_DRIVERS_XLR_SERIAL_H_

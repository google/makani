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

#ifndef AVIONICS_COMMON_NOVATEL_SERIAL_H_
#define AVIONICS_COMMON_NOVATEL_SERIAL_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/novatel_types.h"
#include "avionics/common/rtcm3.h"
#include "avionics/common/serial_parse.h"

typedef enum {
  kNovAtelProtoAbbreviated,
  kNovAtelProtoAscii,
  kNovAtelProtoBinary,
  kNovAtelProtoPrompt,
  kNovAtelProtoRtcm3
} NovAtelProto;

typedef struct {
  uint32_t crc32;
  int32_t packet_length;
  int32_t total_length;
  NovAtelHeader header;
} NovAtelBinary;

#define NOVATEL_ASCII_FIELDS_MAX 64

typedef struct {
  uint32_t crc;
  int32_t crc_index;
  int32_t fields;
  int32_t header_fields;
  int32_t field_delim[NOVATEL_ASCII_FIELDS_MAX];
  bool in_quotes;
} NovAtelAscii;

typedef struct {
  int32_t data_length;
  const uint8_t *data;
  SerialReceiveBuffer buffer;
  NovAtelProto proto;
  NovAtelBinary binary;
  NovAtelAscii ascii;
  Rtcm3Receive rtcm3;
} NovAtelReceive;

bool NovAtelParse(SerialReceiveBuffer *buffer, NovAtelReceive *rx);
void NovAtelAsciiGetField(const NovAtelReceive *rx, int32_t field,
                          int32_t *field_length, int32_t *field_start);

#endif  // AVIONICS_COMMON_NOVATEL_SERIAL_H_

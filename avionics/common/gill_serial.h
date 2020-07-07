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

#ifndef AVIONICS_COMMON_GILL_SERIAL_H_
#define AVIONICS_COMMON_GILL_SERIAL_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/gill_binary.h"
#include "avionics/common/nmea.h"
#include "avionics/common/serial_parse.h"

#define GILL_FIELDS_MAX 16

typedef enum {
  kGillProtoAscii,
  kGillProtoBinary,
  kGillProtoLine,
  kGillProtoPrompt,
  kGillProtoNmea,
} GillProto;

typedef struct {
  uint8_t checksum;
  int32_t checksum_index;
  int32_t fields;
  int32_t field_delim[GILL_FIELDS_MAX];
} GillAscii;

typedef struct {
  GillAscii ascii;
  GillBinary binary;
  NmeaReceive nmea;
  GillProto proto;
  const uint8_t *data;
  int32_t data_length;
} GillReceive;

bool GillParse(SerialReceiveBuffer *buffer, GillReceive *rx);

#endif  // AVIONICS_COMMON_GILL_SERIAL_H_

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

#ifndef AVIONICS_COMMON_SEPTENTRIO_SERIAL_H_
#define AVIONICS_COMMON_SEPTENTRIO_SERIAL_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/rtcm3.h"
#include "avionics/common/septentrio_types.h"
#include "avionics/common/serial_parse.h"

typedef struct {
  int32_t line;
  bool complete;
  bool stop;
} SeptentrioAscii;

typedef struct {
  uint16_t crc16_sum;
  uint16_t crc16_check;
  int32_t total_length;
  SeptentrioHeader header;
} SeptentrioSbf;

typedef struct {
  uint8_t xor_sum;
  uint8_t xor_check;
  uint8_t version;
  int32_t header_length;
  int32_t data_length;
} SeptentrioSnmp;

typedef struct {
  int32_t data_length;
  const uint8_t *data;
  SeptentrioProto proto;
  SeptentrioAscii ascii;
  SeptentrioSbf sbf;
  SeptentrioSnmp snmp;
  Rtcm3Receive rtcm3;
} SeptentrioReceive;

bool SeptentrioParse(SerialReceiveBuffer *buffer, SeptentrioReceive *rx);
bool SeptentrioGotPrompt(const SeptentrioReceive *rx);

#endif  // AVIONICS_COMMON_SEPTENTRIO_SERIAL_H_

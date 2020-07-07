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

#ifndef AVIONICS_COMMON_NMEA_H_
#define AVIONICS_COMMON_NMEA_H_

#include <stdbool.h>
#include <stdint.h>

#define NMEA_FIELDS_MAX 64

typedef struct {
  uint8_t checksum;
  int32_t checksum_index;
  int32_t fields;
  int32_t field_delim[NMEA_FIELDS_MAX];
} NmeaReceive;

bool NmeaParse(bool require_checksum, int32_t length, const uint8_t *data,
               NmeaReceive *nmea, int32_t *parsed);
void NmeaGetField(const NmeaReceive *nmea, int32_t field, int32_t *field_length,
                  int32_t *field_start);

#endif  // AVIONICS_COMMON_NMEA_H_

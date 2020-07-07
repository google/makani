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

#ifndef AVIONICS_COMMON_GPS_PARSE_H_
#define AVIONICS_COMMON_GPS_PARSE_H_

#include <stdint.h>

#include "avionics/common/gps_types.h"

void GpsParseSubframe1(const uint32_t *sf1, GpsEphemeris *eph);
void GpsParseSubframe2(const uint32_t *sf2, GpsEphemeris *eph);
void GpsParseSubframe3(const uint32_t *sf3, GpsEphemeris *eph);

#endif  // AVIONICS_COMMON_GPS_PARSE_H_

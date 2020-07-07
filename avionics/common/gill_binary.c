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

#include "avionics/common/gill_binary.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/endian.h"
#include "avionics/common/fast_math/fast_math.h"
#include "avionics/common/gill_types.h"

// Returns length of binary message and -1 for unknown messages.
int32_t GillBinaryGetLength(GillBinaryId id) {
  switch (id) {
    case kGillBinaryIdWindmasterMode7:
      return 13;
    case kGillBinaryIdWindmasterMode8:
      return 13;
    case kGillBinaryIdWindmasterMode9:
      return 23;
    case kGillBinaryIdWindmasterMode10:
      return 23;
    default:
      return -1;
  }
}

static bool WindmasterMode7(const uint8_t *data, GillData *gill) {
  int32_t o = 2;
  int16_t i16;

  GillDataWindmasterPolar *out = &gill->u.windmaster_polar;
  gill->id = kGillDataIdWindmasterPolar;

  o += ReadInt16Be(&data[o], &i16);
  out->status = (uint8_t)i16;
  if ((out->status & 0xFF00) == 0x0) {
    o += ReadInt16Be(&data[o], &i16);  // [deg]
    out->wind_direction = i16 * PI_F / 180.0f;  // [rad]

    o += ReadInt16Be(&data[o], &i16);
    out->wind_speed = i16 * 0.01f;

    o += ReadInt16Be(&data[o], &i16);
    out->w_axis = i16 * 0.01f;

    o += ReadInt16Be(&data[o], &i16);
    out->speed_of_sound = i16 * 0.01f;

    return true;
  }
  return false;
}

static bool WindmasterMode8(const uint8_t *data, GillData *gill) {
  int32_t o = 2;
  int16_t i16;

  GillDataWindmasterUvw *out = &gill->u.windmaster_uvw;
  gill->id = kGillDataIdWindmasterUvw;

  o += ReadInt16Be(&data[o], &i16);
  out->status = (uint8_t)i16;
  if ((out->status & 0xFF00) == 0x0) {
    o += ReadInt16Be(&data[o], &i16);
    out->wind_velocity[0] = i16 * 0.01f;

    o += ReadInt16Be(&data[o], &i16);
    out->wind_velocity[1] = i16 * 0.01f;

    o += ReadInt16Be(&data[o], &i16);
    out->wind_velocity[2] = i16 * 0.01f;

    o += ReadInt16Be(&data[o], &i16);
    out->speed_of_sound = i16 * 0.01f;

    return true;
  }
  return false;
}

static bool WindmasterMode9(const uint8_t *data, GillData *gill) {
  // Additional Mode9 fields not supported.
  return WindmasterMode7(data, gill);
}

static bool WindmasterMode10(const uint8_t *data, GillData *gill) {
  // Additional Mode10 fields not supported.
  return WindmasterMode8(data, gill);
}

bool GillBinaryDecodeWindmaster(const GillBinary *bin, const uint8_t *data,
                                GillData *out) {
  switch (bin->id) {
    case kGillBinaryIdWindmasterMode7:
      return WindmasterMode7(data, out);
    case kGillBinaryIdWindmasterMode8:
      return WindmasterMode8(data, out);
    case kGillBinaryIdWindmasterMode9:
      return WindmasterMode9(data, out);
    case kGillBinaryIdWindmasterMode10:
      return WindmasterMode10(data, out);
    default:
      return false;
  }
}

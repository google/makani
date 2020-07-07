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

#include "avionics/common/gill_ascii.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "avionics/common/gill_serial.h"
#include "avionics/common/gill_types.h"
#include "avionics/common/strings.h"

static bool DecodeNode(int32_t length, const uint8_t *data, char *node) {
  if (length != 1) {
    return false;
  }

  *node = (char)data[0];
  return true;
}

static bool DecodeFloatField(int32_t length, const uint8_t *data, float *f) {
  return ReadDecFloat((const char *)data, length, f) == length;
}

static bool DecodeStatus(int32_t length, const uint8_t *data,
                         uint8_t *status) {
  return ReadHexUint8((const char *)data, length, status) == length;
}

bool GillAsciiDecodeMetPak(const GillAscii *asc, const uint8_t *data,
                           GillData *output) {
  int32_t length, start;

  // Data union.
  output->id = kGillDataIdMetPakFull;
  GillDataMetPakFull *out = &output->u.metpak_full;

  // REPORT = NODE,DIR,SPEED,W-AXIS,PRESS,RH,TEMP,DEWPOINT,VOLT,STATUS,CHECK

  // Validate number of fields.
  if (asc->fields != 11) {
    return false;
  }

  // Node.
  GillAsciiGetField(asc, kGillMetPakFieldNode, &length, &start);
  if (!DecodeNode(length, &data[start], &out->node)) {
    return false;
  }

  // Wind direction.
  GillAsciiGetField(asc, kGillMetPakFieldDirection, &length, &start);
  if (!DecodeFloatField(length, &data[start], &out->wind_direction)) {
    return false;
  }

  // Wind speed.
  GillAsciiGetField(asc, kGillMetPakFieldSpeed, &length, &start);
  if (!DecodeFloatField(length, &data[start], &out->wind_speed)) {
    return false;
  }

  // Vertical wind speed.
  GillAsciiGetField(asc, kGillMetPakFieldWAxis, &length, &start);
  if (!DecodeFloatField(length, &data[start], &out->w_axis)) {
    return false;
  }

  // Pressure.
  GillAsciiGetField(asc, kGillMetPakFieldPressure, &length, &start);
  if (!DecodeFloatField(length, &data[start], &out->pressure)) {
    return false;
  }

  // Humidity.
  GillAsciiGetField(asc, kGillMetPakFieldHumidity, &length, &start);
  if (!DecodeFloatField(length, &data[start], &out->humidity)) {
    return false;
  }

  // Temperature.
  GillAsciiGetField(asc, kGillMetPakFieldTemperature, &length, &start);
  if (!DecodeFloatField(length, &data[start], &out->temperature)) {
    return false;
  }

  // Dewpoint.
  GillAsciiGetField(asc, kGillMetPakFieldDewpoint, &length, &start);
  if (!DecodeFloatField(length, &data[start], &out->dewpoint)) {
    return false;
  }

  // Supply voltage.
  GillAsciiGetField(asc, kGillMetPakFieldVoltage, &length, &start);
  if (!DecodeFloatField(length, &data[start], &out->voltage)) {
    return false;
  }

  // Status.
  GillAsciiGetField(asc, kGillMetPakFieldStatus, &length, &start);
  if (!DecodeStatus(length, &data[start], &out->status)) {
    return false;
  }
  return true;
}

bool GillAsciiDecodeWindmasterUvw(const GillAscii *asc, const uint8_t *data,
                                  GillData *output) {
  int32_t length, start;

  // Data union.
  output->id = kGillDataIdWindmasterUvw;
  GillDataWindmasterUvw *out = &output->u.windmaster_uvw;

  // Some fields are [optional]! See configuration.
  // <STX><ID>,±UUU.UUU,±VVV.VVV,±WWW.WWW,U,±CCC.CC,±TTT.TT,
  // SS,[±1.1111,±2.2222,±3.3333,±4.4444],±PP.PPC,<ETX>CC<CR><LF>

  // Validate number of fields.
  if (asc->fields < kNumGillWindmasterFields) {
    return false;
  }

  // Node. Skip.

  // U-velocity.
  GillAsciiGetField(asc, kGillWindmasterFieldUVelocity, &length, &start);
  if (!DecodeFloatField(length, &data[start], &out->wind_velocity[0])) {
    return false;
  }

  // V-velocity.
  GillAsciiGetField(asc, kGillWindmasterFieldVVelocity, &length, &start);
  if (!DecodeFloatField(length, &data[start], &out->wind_velocity[1])) {
    return false;
  }

  // W-velocity.
  GillAsciiGetField(asc, kGillWindmasterFieldWVelocity, &length, &start);
  if (!DecodeFloatField(length, &data[start], &out->wind_velocity[2])) {
    return false;
  }

  // Units. Skip.

  // Speed of sound.
  GillAsciiGetField(asc, kGillWindmasterFieldSpeedOfSound, &length, &start);
  if (!DecodeFloatField(length, &data[start], &out->speed_of_sound)) {
    return false;
  }

  // Status.
  GillAsciiGetField(asc, kGillWindmasterFieldStatus, &length, &start);
  if (!DecodeStatus(length, &data[start], &out->status)) {
    return false;
  }
  return true;
}

void GillAsciiGetField(const GillAscii *asc, int32_t field,
                       int32_t *field_length, int32_t *field_start) {
  assert(asc != NULL);
  assert(field >= 0);
  assert(field_length != NULL);
  assert(field_start != NULL);

  if (field >= asc->fields) {
    *field_start = 0;
    *field_length = 0;
  } else {
    if (field == 0) {
      *field_start = 1;  // Strip <STX> character.
    } else {
      *field_start = asc->field_delim[field - 1] + 1;
    }
    *field_length = asc->field_delim[field] - *field_start;
  }
}

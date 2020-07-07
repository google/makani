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

#include "avionics/firmware/drivers/encoder.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/common/fast_math/fast_math.h"
#include "avionics/firmware/drivers/encoder_params.h"
#include "avionics/firmware/drivers/ssi_encoder.h"
#include "common/macros.h"

// Converts an encoder measurement in counts to a calibrated value in
// radians in the range [-pi, pi). If the number of counts lies outside of
// the expected range, this function returns false, indicating a possible
// failure. It returns true when the number of counts lies within the
// expected range.
bool CalibrateAngleEncoder(const EncoderCalib *cal, int32_t in, float *out) {
  if (in < cal->min_encoder_value || in > cal->max_encoder_value) {
    *out = 0.0f;
    return false;
  } else {
    *out = WrapAngle(((float)in * cal->scale_factor) - cal->zero_position);
    return true;
  }
}

static bool GetEncoderCounts(const EncoderCalib *cal,
                             const SsiEncoderOutput *in,
                             int32_t *counts, uint8_t *flags) {
  bool success = false;
  if (0 <= cal->channel && cal->channel < ARRAYSIZE(in->raw)) {
    int16_t counts16;
    switch (cal->type) {
      case kEncoderTypeAmo4306Single:
        success = SsiGetAmo4306Single(in, cal->channel, counts, flags);
        break;
      case kEncoderTypeIha608Multi:
        success = SsiGetIha608Multi(in, cal->channel, counts);
        break;
      case kEncoderTypeMagA300:
        success = SsiGetMagA300(in, cal->channel, &counts16);
        *counts = counts16;
        break;
      case kEncoderTypeMagA550:
        success = SsiGetMagA550(in, cal->channel, counts);
        break;
      case kEncoderTypeRha507Single:
        success = SsiGetRha507Single(in, cal->channel, &counts16);
        *counts = counts16;
        break;
      default:
        break;
    }
  }
  return success;
}

EncoderStatus EncoderGet(const EncoderCalib *cal, const SsiEncoderOutput *in,
                         float *value, uint8_t *flags) {
  EncoderStatus status = 0x0;
  int32_t counts;

  *value = 0.0f;
  if (flags != NULL) {
    *flags = 0x0;
  }

  if (!GetEncoderCounts(cal, in, &counts, flags)) {
    status |= kEncoderStatusError;
  }
  if (!CalibrateAngleEncoder(cal, counts, value)) {
    status |= kEncoderStatusWarning;
  }
  return status;
}

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

#include "avionics/common/fast_math/fast_math.h"

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/common/fast_math/trig_lookup_tables.h"
#include "common/macros.h"

#define NUM_SIN_INDEX ((uint32_t)ARRAYSIZE(kSinQuadrantTable) - 1U)

COMPILE_ASSERT(NUM_SIN_INDEX != 0U
               && (NUM_SIN_INDEX & (~NUM_SIN_INDEX + 1U)) == NUM_SIN_INDEX,
               NUM_SIN_INDEX_should_be_a_power_of_two);

float Atan2Lookup(float y, float x) {
  int32_t sign = 1;
  float offset = 0.0f;
  float y_positive = y, x_positive = x;

  // Protect against 0.0f / 0.0f.
  if (x == 0.0f && y == 0.0f) {
    return 0.0f;
  }

  // Reflect into right hand plane.
  if (x < 0.0f) {
    sign *= -1;
    x_positive = -x;
    offset = PI_F;
  }

  // Reflect into first quadrant.
  if (y < 0.0f) {
    sign *= -1;
    y_positive = -y;
    offset = -offset;
  }

  // Reflect into first octant and calculate the atan lookup argument.
  float arg;
  if (y_positive < x_positive) {
    arg = y_positive / x_positive;
  } else {
    arg = x_positive / y_positive;
    offset += (float)sign * (PI_F / 2.0f);
    sign *= -1;
  }

  return offset + (float)sign * UniformInterp1(kAtanOctantTable,
                                               ARRAYSIZE(kAtanOctantTable),
                                               arg * kAtanOctantIndexScale);
}

// NOTE: The use of unsigned integers in this function
// caused a noticeable speed-up (~20%) on the TMS570.
float SinLookup(float angle) {
  // Convert the angle to an index (not necessarily within the range
  // of the lookup table).
  float scaled_angle;
  uint32_t quadrant;
  if (angle >= 0.0f) {
    scaled_angle = angle * kSinQuadrantIndexScale;
    quadrant = 0U;
  } else {
    scaled_angle = angle * -kSinQuadrantIndexScale;
    quadrant = 2U;
  }
  assert(scaled_angle < (float)UINT32_MAX);
  uint32_t index = (uint32_t)scaled_angle;
  float frac = scaled_angle - (float)index;

  // Determine which quadrant the angle is in.  Then reduce the index
  // to be within the first quadrant and lookup and interpolate the
  // value.
  quadrant = (index / NUM_SIN_INDEX + quadrant) % 4U;
  switch (quadrant) {
    default:
      assert(false);
      // Intentionally fall through to next case by default.
    case 0U:
      index = index % NUM_SIN_INDEX;
      return kSinQuadrantTable[index]
          + frac * (kSinQuadrantTable[index + 1U] - kSinQuadrantTable[index]);
    case 1U:
      index = NUM_SIN_INDEX - index % NUM_SIN_INDEX;
      return kSinQuadrantTable[index]
          + frac * (kSinQuadrantTable[index - 1U] - kSinQuadrantTable[index]);
    case 2U:
      index = index % NUM_SIN_INDEX;
      return -kSinQuadrantTable[index]
          + frac * (kSinQuadrantTable[index] - kSinQuadrantTable[index + 1U]);
    case 3U:
      index = NUM_SIN_INDEX - index % NUM_SIN_INDEX;
      return -kSinQuadrantTable[index]
          + frac * (kSinQuadrantTable[index] - kSinQuadrantTable[index - 1U]);
  }
}

// NOTE: The use of unsigned integers here caused a
// noticeable speed-up (~20%) on the TMS570.
void SinCosLookup(float angle, float *sin_val, float *cos_val) {
  assert(sin_val != NULL && cos_val != NULL);

  // Convert the angle to an index (not necessarily within the range
  // of the lookup table).
  float scaled_angle;
  bool sin_sign_flip;
  if (angle >= 0.0f) {
    scaled_angle = angle * kSinQuadrantIndexScale;
    sin_sign_flip = false;
  } else {
    scaled_angle = angle * -kSinQuadrantIndexScale;
    sin_sign_flip = true;
  }
  assert(scaled_angle < (float)UINT32_MAX);
  uint32_t index = (uint32_t)scaled_angle;
  float frac = scaled_angle - (float)index;

  // Calculate the index within the first quadrant and determine which
  // quadrant the angle is in.
  uint32_t lookup_index = index % NUM_SIN_INDEX;
  uint32_t mirror_index = NUM_SIN_INDEX - lookup_index;
  uint32_t quadrant = (index / NUM_SIN_INDEX) % 4U;
  switch (quadrant) {
    default:
      assert(false);
      // Intentionally fall through to next case by default.
    case 0U:
      *sin_val = kSinQuadrantTable[lookup_index]
          + frac * (kSinQuadrantTable[lookup_index + 1U]
                    - kSinQuadrantTable[lookup_index]);
      *cos_val = kSinQuadrantTable[mirror_index]
          + frac * (kSinQuadrantTable[mirror_index - 1U]
                    - kSinQuadrantTable[mirror_index]);
      break;
    case 1U:
      *sin_val = kSinQuadrantTable[mirror_index]
          + frac * (kSinQuadrantTable[mirror_index - 1U]
                    - kSinQuadrantTable[mirror_index]);
      *cos_val = -kSinQuadrantTable[lookup_index]
          + frac * (kSinQuadrantTable[lookup_index]
                    - kSinQuadrantTable[lookup_index + 1U]);
      break;
    case 2U:
      *sin_val = -kSinQuadrantTable[lookup_index]
          + frac * (kSinQuadrantTable[lookup_index]
                    - kSinQuadrantTable[lookup_index + 1U]);
      *cos_val = -kSinQuadrantTable[mirror_index]
          + frac * (kSinQuadrantTable[mirror_index]
                    - kSinQuadrantTable[mirror_index - 1U]);
      break;
    case 3U:
      *sin_val = -kSinQuadrantTable[mirror_index]
          + frac * (kSinQuadrantTable[mirror_index]
                    - kSinQuadrantTable[mirror_index - 1U]);
      *cos_val = kSinQuadrantTable[lookup_index]
          + frac * (kSinQuadrantTable[lookup_index + 1U]
                    - kSinQuadrantTable[lookup_index]);
      break;
  }
  if (sin_sign_flip) *sin_val *= -1.0f;
}

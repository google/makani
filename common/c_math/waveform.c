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

#include "common/c_math/waveform.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "common/c_math/util.h"

double PulseTrain(double t, double t_start, double duration, double period) {
  double t_offset = t - t_start;
  if (t_offset > 0.0 && fmod(t_offset, period) < duration) {
    return 1.0;
  } else {
    return 0.0;
  }
}

double TriangleWave(double t, double period) {
  assert(period > DBL_EPSILON);

  if (!(period > DBL_EPSILON)) return 0.0;

  double x = fmod(t, period);
  if (x < 0.0) {
    x += period;
  }
  x /= period;

  double y;
  if (x < 0.25) {
    y = 4.0 * x;
  } else if (x < 0.75) {
    y = 2.0 - 4.0 * x;
  } else {
    y = 4.0 * x - 4.0;
  }

  return y;
}

double Fourier(double t, double T, double a0, const double *a, const double *b,
               int32_t n) {
  assert(T > 0.0);
  assert(a != NULL && b != NULL);
  assert(n >= 0);

  double y = a0 / 2.0;
  for (int32_t i = 0; i < n; ++i) {
    y += a[i] * cos(2.0 * PI * (i + 1) * t / T) +
         b[i] * sin(2.0 * PI * (i + 1) * t / T);
  }
  return y;
}

double LinearChirp(double t, double t_start, double freq_start, double t_end,
                   double freq_end) {
  assert(t_start < t_end);
  assert(freq_start > 0.0 && freq_end > 0.0);

  if (t_start < t && t < t_end) {
    double freq_slope =
        (freq_end - freq_start) / fmax(t_end - t_start, DBL_EPSILON);
    double delta_t = t - t_start;
    return sin(2.0 * PI *
               (freq_start * delta_t + freq_slope * delta_t * delta_t / 2.0));
  } else {
    return 0.0;
  }
}

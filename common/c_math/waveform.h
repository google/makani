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

#ifndef COMMON_C_MATH_WAVEFORM_H_
#define COMMON_C_MATH_WAVEFORM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Defines a pulse train that first goes "high" at t_start, with
// pulses of set magnitude, duration, and period.
double PulseTrain(double t, double t_start, double duration, double period);

// Triangle wave.
//
// Args:
//   t: Current time.
//   period: Period of the triangle wave.
//
// Returns:
//   The piecewise linear function:
//
//         { 4*x         0 <= x < 1/4
//     y = { 2 - 4*x   1/4 <= x < 3/4
//         { 4*x - 4   3/4 <= x < 1
//
//   for x = t/period in the range [0, 1).  Repeats outside this range.
double TriangleWave(double t, double period);

// Fourier series.
double Fourier(double t, double T, double a0, const double *a, const double *b,
               int32_t n);

// Linearly ramped chirp.  Creates a sinusoidal signal with a linearly
// varying instantaneous frequency.
//
// Args:
//   t: Current time.
//   t_start: Time to start chirp.
//   freq_start: Instantaneous frequency at start of chirp.
//   t_end: Time to stop chirp.
//   freq_end: Instantaneous frequency at end of chirp.
//
// Returns:
//   The linearly ramped chirp signal between t_start and t_end, 0.0
//   otherwise.
double LinearChirp(double t, double t_start, double freq_start, double t_end,
                   double freq_end);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_WAVEFORM_H_

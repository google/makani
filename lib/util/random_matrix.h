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

#ifndef LIB_UTIL_RANDOM_MATRIX_H_
#define LIB_UTIL_RANDOM_MATRIX_H_

#include <gsl/gsl_rng.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void MatArrRandomGaussian(int32_t nr, int32_t nc, const gsl_rng *rng,
                          double *d);
void MatArrRandomOrthogonal(int32_t n, const gsl_rng *rng, double *d);
void MatArrRandomWithRank(int32_t nr, int32_t nc, int32_t r,
                          const gsl_rng *rng, double *d);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIB_UTIL_RANDOM_MATRIX_H_

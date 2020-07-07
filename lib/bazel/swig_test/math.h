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

#ifndef LIB_BAZEL_SWIG_TEST_MATH_H_
#define LIB_BAZEL_SWIG_TEST_MATH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

double Percent(uint16_t a, uint16_t b);

double MissRate(uint16_t a, uint16_t b);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIB_BAZEL_SWIG_TEST_MATH_H_

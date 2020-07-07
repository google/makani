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

#ifndef COMMON_C_MATH_UTIL_LINALG_IO_H_
#define COMMON_C_MATH_UTIL_LINALG_IO_H_

#include <stdio.h>

#include "common/c_math/linalg.h"

#ifdef __cplusplus
extern "C" {
#endif

void VecDisp(const Vec *v);
void VecScan(FILE *f, Vec *v);

void MatDisp(const Mat *m);
void MatScan(FILE *f, Mat *m);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_UTIL_LINALG_IO_H_

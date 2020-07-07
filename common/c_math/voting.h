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

#ifndef COMMON_C_MATH_VOTING_H_
#define COMMON_C_MATH_VOTING_H_

#include "common/c_math/quaternion.h"
#include "common/c_math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

double Median3(double a, double b, double c);
const Vec3 *Median3Vec3(const Vec3 *a, const Vec3 *b, const Vec3 *c, Vec3 *m);
const Quat *Median3Quat(const Quat *a, const Quat *b, const Quat *c, Quat *m);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_VOTING_H_

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

#ifndef VIS_VIS_WING_H_
#define VIS_VIS_WING_H_

#include "common/c_math/vec3.h"
#include "vis/vis_main.h"

#ifdef __cplusplus
extern "C" {
#endif

void InitWing(void);
void DrawWing(const Vec3 *Xg, const Vec3 *eulers);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // VIS_VIS_WING_H_

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

#ifndef VIS_VIS_GS02_H_
#define VIS_VIS_GS02_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include "common/c_math/util.h"

void InitGs02(void);
void DrawGs02(double platform_azi, double drum_angle, double levelwind_ele,
              double detwist_angle, double gsg_yoke, double gsg_termination);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // VIS_VIS_GS02_H_

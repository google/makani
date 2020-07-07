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

#ifndef SIM_PHYSICS_AERO_FRAME_H_
#define SIM_PHYSICS_AERO_FRAME_H_

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"

// Conversion between wind and body coordinates.

const Mat3 *CalcDcmWToB(double alpha, double beta, Mat3 *dcm_b2w);
const Vec3 *RotBToW(const Vec3 *Vb, double alpha, double beta, Vec3 *Vw);
const Vec3 *RotWToB(const Vec3 *Vw, double alpha, double beta, Vec3 *Vb);

// Conversion between stability and body coordinates.

const Mat3 *CalcDcmSToB(double alpha, Mat3 *dcm_s2b);
const Vec3 *RotBToS(const Vec3 *Vb, double alpha, Vec3 *Vs);
const Vec3 *RotSToB(const Vec3 *Vs, double alpha, Vec3 *Vb);

#endif  // SIM_PHYSICS_AERO_FRAME_H_

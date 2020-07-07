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

#ifndef CONTROL_PERCH_FRAME_H_
#define CONTROL_PERCH_FRAME_H_

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

// Conversion between Ground-station and Perch.

const Mat3 *CalcDcmGToP(double perch_azi, Mat3 *dcm_g2p);
const Vec3 *GToP(const Vec3 *Xg, double perch_azi, Vec3 *Xp);
const Vec3 *RotGToP(const Vec3 *Vg, double perch_azi, Vec3 *Vp);
const Vec3 *PToG(const Vec3 *Xp, double perch_azi, Vec3 *Xg);
const Vec3 *RotPToG(const Vec3 *Vp, double perch_azi, Vec3 *Vg);

// Conversion between Perch and Levelwind.

const Mat3 *CalcDcmPToLw(double levelwind_ele, Mat3 *dcm_p2lw);
const Vec3 *CalcXpLevelwindOrigin(double drum_angle, Vec3 *Xp_lw_frame_origin);
const Vec3 *PToLw(const Vec3 *Xp, double levelwind_ele, double drum_angle,
                  Vec3 *X_lw);
const Vec3 *RotPToLw(const Vec3 *Xp, double levelwind_ele, Vec3 *X_lw);
const Vec3 *LwToP(const Vec3 *Xp_lw, double levelwind_ele, double drum_angle,
                  Vec3 *Xp);
const Vec3 *RotLwToP(const Vec3 *X_lw, double levelwind_ele, Vec3 *Xp);

// Conversion between Perch and Winch drum.

const Mat3 *CalcDcmPToWd(double drum_angle, Mat3 *dcm_p2wd);
const Vec3 *PToWd(const Vec3 *Xp, double drum_angle, Vec3 *X_wd);
const Vec3 *RotPToWd(const Vec3 *Vp, double drum_angle, Vec3 *V_wd);
const Vec3 *WdToP(const Vec3 *X_wd, double drum_angle, Vec3 *Xp);
const Vec3 *RotWdToP(const Vec3 *V_wd, double drum_angle, Vec3 *Vp);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_PERCH_FRAME_H_

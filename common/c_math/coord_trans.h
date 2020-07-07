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

#ifndef COMMON_C_MATH_COORD_TRANS_H_
#define COMMON_C_MATH_COORD_TRANS_H_

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

// Earth axes and eccentricities.
extern const double kEarthA, kEarthB, kEarthE, kEarthE2;

const Mat3 *RotateCov(const Mat3 *cov_a, const Mat3 *dcm_a2b, Mat3 *cov_b);

// Conversions between NED and HTV.

const Vec3 *NedToHtv(const Vec3 *V_ned, Vec3 *V_htv);
const Vec3 *HtvToNed(const Vec3 *V_htv, Vec3 *V_ned);

// Conversions between NED and ECEF.

const Mat3 *CalcDcmNedToEcef(const Vec3 *X_ecef, Mat3 *dcm_ned2ecef);
const Vec3 *NedToEcef(const Vec3 *X_ned, const Vec3 *X_ecef_0, Vec3 *X_ecef);
const Vec3 *RotNedToEcef(const Vec3 *X_ned, const Vec3 *X_ecef_0, Vec3 *X_ecef);
const Mat3 *CalcDcmEcefToNed(const Vec3 *X_ecef, Mat3 *dcm_ecef2ned);
const Vec3 *EcefToNed(const Vec3 *X_ecef, const Vec3 *X_ecef_0, Vec3 *X_ned);
const Vec3 *RotEcefToNed(const Vec3 *X_ecef, const Vec3 *X_ecef_0, Vec3 *X_ned);

// Conversions between LLH and ECEF.

const Vec3 *LlhToEcef(const Vec3 *X_llh, Vec3 *X_ecef);
const Vec3 *EcefToLlh(const Vec3 *X_ecef, Vec3 *X_llh);

// Conversions between NED and LLH.

const Vec3 *NedToLlh(const Vec3 *X_ned, const Vec3 *X_llh_0, Vec3 *X_llh);
const Vec3 *LlhToNed(const Vec3 *X_llh, const Vec3 *X_llh_0, Vec3 *X_ned);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_COORD_TRANS_H_

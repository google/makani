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

#include "control/ground_frame.h"

#include <math.h>

#include "common/c_math/coord_trans.h"
#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/system_params.h"
#include "control/system_types.h"

// Conversions between North-East-Down and Ground frame.

const Mat3 *CalcDcmNedToG(double g_heading, Mat3 *dcm_ned2g) {
  return AngleToDcm(g_heading, 0.0, 0.0, kRotationOrderZyx, dcm_ned2g);
}

const Vec3 *NedToG(const Vec3 *X_ned, double g_heading, Vec3 *Xg) {
  Mat3 dcm_ned2g;
  CalcDcmNedToG(g_heading, &dcm_ned2g);
  return PoseTransform(&dcm_ned2g, &kVec3Zero, X_ned, Xg);
}

const Vec3 *RotNedToG(const Vec3 *V_ned, double g_heading, Vec3 *Vg) {
  Mat3 dcm_ned2g;
  CalcDcmNedToG(g_heading, &dcm_ned2g);
  return Mat3Vec3Mult(&dcm_ned2g, V_ned, Vg);
}

const Vec3 *GToNed(const Vec3 *Xg, double g_heading, Vec3 *X_ned) {
  Mat3 dcm_ned2g;
  CalcDcmNedToG(g_heading, &dcm_ned2g);
  return InversePoseTransform(&dcm_ned2g, &kVec3Zero, Xg, X_ned);
}

const Vec3 *RotGToNed(const Vec3 *Vg, double g_heading, Vec3 *V_ned) {
  Mat3 dcm_ned2g;
  CalcDcmNedToG(g_heading, &dcm_ned2g);
  return Mat3TransVec3Mult(&dcm_ned2g, Vg, V_ned);
}

// Conversions between ECEF and Ground frame.

const Mat3 *CalcDcmEcefToG(const Vec3 *gs_pos, double g_heading,
                           Mat3 *dcm_ecef2g) {
  Mat3 dcm_ned2g, dcm_ecef2ned;
  return Mat3Mat3Mult(CalcDcmNedToG(g_heading, &dcm_ned2g),
                      CalcDcmEcefToNed(gs_pos, &dcm_ecef2ned), dcm_ecef2g);
}

const Vec3 *EcefToG(const Vec3 *X_ecef, const Vec3 *gs_pos, double g_heading,
                    Vec3 *Xg) {
  Vec3 X_ned;
  return NedToG(EcefToNed(X_ecef, gs_pos, &X_ned), g_heading, Xg);
}

const Vec3 *RotEcefToG(const Vec3 *V_ecef, const Vec3 *gs_pos, double g_heading,
                       Vec3 *Vg) {
  Vec3 V_ned;
  return RotNedToG(RotEcefToNed(V_ecef, gs_pos, &V_ned), g_heading, Vg);
}

const Vec3 *GToEcef(const Vec3 *Xg, const Vec3 *gs_pos, double g_heading,
                    Vec3 *X_ecef) {
  Vec3 X_ned;
  return NedToEcef(GToNed(Xg, g_heading, &X_ned), gs_pos, X_ecef);
}

const Vec3 *RotGToEcef(const Vec3 *Vg, const Vec3 *gs_pos, double g_heading,
                       Vec3 *V_ecef) {
  Vec3 V_ned;
  return RotNedToEcef(RotGToNed(Vg, g_heading, &V_ned), gs_pos, V_ecef);
}

double VecGToAzimuth(const Vec3 *pos_g) {
  return Wrap(atan2(pos_g->y, pos_g->x) +
                  GetSystemParams()->ground_station.azi_ref_offset,
              -PI, PI);
}

double VecGToElevation(const Vec3 *pos_g) {
  return atan2(-pos_g->z, Vec3XyNorm(pos_g));
}

void CylToVecG(double azi, double r, double z, Vec3 *pos_g) {
  CylToCart(azi - GetSystemParams()->ground_station.azi_ref_offset, r, z,
            pos_g);
}

void VecGToCyl(const Vec3 *pos_g, double *azi, double *r, double *z) {
  CartToCyl(pos_g, azi, r, z);
  *azi = Wrap(*azi + GetSystemParams()->ground_station.azi_ref_offset, -PI, PI);
}

void SphToVecG(double azi, double ele, double r, Vec3 *pos_g) {
  // Negate the elevation because positive elevation corresponds to the
  // -z-direction.
  SphToCart(azi - GetSystemParams()->ground_station.azi_ref_offset, -ele, r,
            pos_g);
}

void VecGToSph(const Vec3 *pos_g, double *azi, double *ele, double *r) {
  CartToSph(pos_g, azi, ele, r);
  *azi = Wrap(*azi + GetSystemParams()->ground_station.azi_ref_offset, -PI, PI);
  // Negate the elevation because positive elevation corresponds to the
  // -z-direction.
  *ele = -*ele;
}

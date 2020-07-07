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

#include "common/c_math/coord_trans.h"

#include <math.h>
#include <stdint.h>

#include "common/c_math/linalg_common.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"

// Semi-major axis
const double kEarthA = 6378137.0;  // [m]

// Eccentricity  sqrt(1 - B^2/A^2)
const double kEarthE = 0.081819190842621;

// Semi-minor axis
const double kEarthB = 6356752.314245;  // [m]

// Second eccentricity  sqrt(A^2/B^2 - 1)
const double kEarthE2 = 0.082094437949696;

// Change basis of a covariance matrix
//
// cov_b = cov(Xb, Xb) = E[(Xb - Xb_mean)*(Xb - Xb_mean)']
//       = E[dcm_a2b*(Xa - Xa_mean)*(Xa - Xa_mean)'*dcm_a2b']
//       = dcm_a2b * cov(Xa, Xa) * dcm_a2b'
//       = dcm_a2b * cov_a * dcm_a2b'
const Mat3 *RotateCov(const Mat3 *cov_a, const Mat3 *dcm_a2b, Mat3 *cov_b) {
  Mat3Mult(cov_a, kNoTrans, dcm_a2b, kTrans, cov_b);
  return Mat3Mat3Mult(dcm_a2b, cov_b, cov_b);
}

// Conversions between HTV and NED.

// NedToHtv   Converts a NED velocity to a horizontal speed, bearing,
// and vertical velocity.  (Bearing is in radians.)
const Vec3 *NedToHtv(const Vec3 *V_ned_in, Vec3 *V_htv) {
  Vec3 V_ned = *V_ned_in;
  V_htv->x = sqrt(V_ned.x * V_ned.x + V_ned.y * V_ned.y);
  V_htv->y = atan2(V_ned.y, V_ned.x);
  V_htv->z = -V_ned.z;
  return V_htv;
}

// HtvToNed   Converts horizontal speed, bearing, vertical velocity to an
// NED velocity.  (Bearing is in radians.)
const Vec3 *HtvToNed(const Vec3 *V_htv_in, Vec3 *V_ned) {
  Vec3 V_htv = *V_htv_in;
  V_ned->x = V_htv.x * cos(V_htv.y);
  V_ned->y = V_htv.x * sin(V_htv.y);
  V_ned->z = -V_htv.z;
  return V_ned;
}

// Conversions between NED and ECEF.

// Calculates a DCM for the rotation from NED coordinates to ECEF
// coordinates, given a reference position vector in ECEF coordinates.
const Mat3 *CalcDcmNedToEcef(const Vec3 *X_ecef, Mat3 *dcm_ned2ecef) {
  Vec3 X_llh;
  EcefToLlh(X_ecef, &X_llh);
  double ele = X_llh.x * PI / 180.0;
  double azi = X_llh.y * PI / 180.0;

  dcm_ned2ecef->d[0][0] = -cos(azi) * sin(ele);
  dcm_ned2ecef->d[0][1] = -sin(azi);
  dcm_ned2ecef->d[0][2] = -cos(azi) * cos(ele);
  dcm_ned2ecef->d[1][0] = -sin(azi) * sin(ele);
  dcm_ned2ecef->d[1][1] = cos(azi);
  dcm_ned2ecef->d[1][2] = -sin(azi) * cos(ele);
  dcm_ned2ecef->d[2][0] = cos(ele);
  dcm_ned2ecef->d[2][1] = 0.0;
  dcm_ned2ecef->d[2][2] = -sin(ele);

  return dcm_ned2ecef;
}

// Converts a vector NED coordinates to a vector in ECEF coordinates,
// given a reference position vector in ECEF coordinates.
const Vec3 *NedToEcef(const Vec3 *X_ned, const Vec3 *X_ecef_0, Vec3 *X_ecef) {
  Mat3 dcm_ned2ecef;
  Vec3 X_ecef_tmp;  // Makes it safe to reuse X_ned, X_ecef_0 in output
  CalcDcmNedToEcef(X_ecef_0, &dcm_ned2ecef);
  Mat3Vec3Mult(&dcm_ned2ecef, X_ned, &X_ecef_tmp);
  return Vec3Add(&X_ecef_tmp, X_ecef_0, X_ecef);
}

// Rotates a vector NED coordinates to a vector in ECEF coordinates,
// given a reference position vector in ECEF coordinates.
const Vec3 *RotNedToEcef(const Vec3 *X_ned, const Vec3 *X_ecef_0,
                         Vec3 *X_ecef) {
  Mat3 dcm_ned2ecef;
  CalcDcmNedToEcef(X_ecef_0, &dcm_ned2ecef);
  return Mat3Vec3Mult(&dcm_ned2ecef, X_ned, X_ecef);
}

// Calculates a DCM for the rotation from ECEF coordinates to NED
// coordinates, given a reference position vector in ECEF coordinates.
const Mat3 *CalcDcmEcefToNed(const Vec3 *X_ecef, Mat3 *dcm_ecef2ned) {
  return Mat3Trans(CalcDcmNedToEcef(X_ecef, dcm_ecef2ned), dcm_ecef2ned);
}

// Converts a vector in ECEF coordinates to one in NED coordinates,
// given a reference position vector in ECEF coordinates.
const Vec3 *EcefToNed(const Vec3 *X_ecef, const Vec3 *X_ecef_0, Vec3 *X_ned) {
  Mat3 dcm_ecef2ned;
  Vec3 X_ecef_tmp;  // Makes it safe to reuse X_ned, X_ecef_0 in output
  CalcDcmEcefToNed(X_ecef_0, &dcm_ecef2ned);
  Mat3Vec3Mult(&dcm_ecef2ned, Vec3Sub(X_ecef, X_ecef_0, &X_ecef_tmp), X_ned);
  return X_ned;
}

// Rotates a vector in ECEF coordinates to one in NED coordinates,
// given a reference position vector in ECEF coordinates.
const Vec3 *RotEcefToNed(const Vec3 *X_ecef, const Vec3 *X_ecef_0,
                         Vec3 *X_ned) {
  Mat3 dcm_ecef2ned;
  CalcDcmEcefToNed(X_ecef_0, &dcm_ecef2ned);
  return Mat3Vec3Mult(&dcm_ecef2ned, X_ecef, X_ned);
}

// Conversions between ECEF and LLH.

// Converts WGS84 lat, long, altitude coordinates to WGS84
// ECEF (Earth-Centered, Earth-Fixed) coordinates.
//
// From Wikipedia:
// The distance from the surface to the z-axis along the ellipsoid
// normal is denoted N(lat):
// N(lat) = a/sqrt(1 - e^2*sin^2(lat))
// where a is the major-axis radius and e is the eccentricity.
// X = (N(lat) + h) cos(lat) cos(lon)
// Y = (N(lat) + h) cos(lat) sin(lon)
// Z = (N(lat)(1 - e^2) + h) sin(lat)
const Vec3 *LlhToEcef(const Vec3 *X_llh_in, Vec3 *X_ecef) {
  Vec3 X_llh = *X_llh_in;  // Makes it safe to reuse X_llh_in in output
  double esinlat = kEarthE * sin(PI / 180.0 * X_llh.x);
  double N_lat = kEarthA / sqrt(1.0 - esinlat * esinlat);
  double d_xy = (N_lat + X_llh.z) * cos(PI / 180.0 * X_llh.x);

  X_ecef->x = d_xy * cos(PI / 180.0 * X_llh.y);
  X_ecef->y = d_xy * sin(PI / 180.0 * X_llh.y);
  X_ecef->z =
      ((1.0 - kEarthE * kEarthE) * N_lat + X_llh.z) * sin(PI / 180.0 * X_llh.x);

  return X_ecef;
}

// Converts WGS84 ECEF coordinates to LLH
//
// We use Bowring's method as described in:
// G. Gerdan and R. Deakin, "Transforming Cartesian coordinates X,Y,Z
// to Geographical coordinates phi, lambda, h" Australian Surveyor 44 (1)
//
// tan(phi) = (Z + b*e'^2*sin^3(psi))/(p - a*e^2*cos^3(psi))
//
// tan(psi_0) = a*Z/(b*p)
// tan(psi_i) = b/a * tan(phi_{i-1})
//
// a = semi-major axis
// b = semi-minor axis
// p = sqrt(X^2 + Y^2)
// e = first eccentricity (e^2 = 1 - b^2/a^2)
// e' = second eccentricity (e'^2 = a^2/b^2 - 1)
//
// The equation for height is based on combining the following equations to
// eliminate singularities:
// h = sqrt(X^2+Y^2)/cos(phi) - N
// h = Z/sin(phi) - N*(1-e^2)
// where N = a/sqrt(1 - e^2*sin^2(phi))
const Vec3 *EcefToLlh(const Vec3 *X_ecef, Vec3 *X_llh) {
  // Calculating these first makes it safe to reuse X_ecef in output
  double d_xy = sqrt(X_ecef->x * X_ecef->x + X_ecef->y * X_ecef->y);
  double lambda = atan2(X_ecef->y, X_ecef->x);
  double phi, psi = atan2(kEarthA / kEarthB * X_ecef->z, d_xy);
  int32_t num_iter = 0;
  while (num_iter < 6) {
    phi =
        atan2(X_ecef->z + kEarthE2 * kEarthE2 * kEarthB * ThirdPower(sin(psi)),
              d_xy - kEarthE * kEarthE * kEarthA * ThirdPower(cos(psi)));
    psi = atan2(kEarthB / kEarthA * sin(phi), cos(phi));
    num_iter++;
  }

  X_llh->x = 180.0 / PI * phi;
  X_llh->y = 180.0 / PI * lambda;
  X_llh->z = d_xy * cos(phi) + X_ecef->z * sin(phi) -
             kEarthA * sqrt(1.0 - kEarthE * kEarthE * sin(phi) * sin(phi));

  return X_llh;
}

// Conversions between NED and LLH.

// Convert between North-East-Down local vertical coordinate system and the
// Latitude-Longitude-Height coordinate system.
const Vec3 *NedToLlh(const Vec3 *X_ned, const Vec3 *X_llh_0, Vec3 *X_llh) {
  Vec3 X_ecef;
  NedToEcef(X_ned, LlhToEcef(X_llh_0, &X_ecef), &X_ecef);
  return EcefToLlh(&X_ecef, X_llh);
}

// Convert between Latitude-Longitude-Height coordinate system and the
// North-East-Down local vertical coordinate system.
const Vec3 *LlhToNed(const Vec3 *X_llh, const Vec3 *X_llh_0, Vec3 *X_ned) {
  Vec3 X_ecef, X_ecef_0;
  LlhToEcef(X_llh, &X_ecef);
  LlhToEcef(X_llh_0, &X_ecef_0);
  return EcefToNed(&X_ecef, &X_ecef_0, X_ned);
}

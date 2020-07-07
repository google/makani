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

#ifndef AVIONICS_COMMON_GPS_TYPES_H_
#define AVIONICS_COMMON_GPS_TYPES_H_

#include <stdint.h>

// GPS PI specified according to ICD-GPS-200. Use this value to convert
// semi-circles to radians with double precision math.
#define GPS_PI 3.1415926535898  // Do not change!

typedef struct {
  // Satellite and timestamp.
  uint32_t tow;  // [ms].
  uint16_t wnc;  // [week].
  uint8_t prn;   // 0 = Invalid.

  // Subframe 1.
  uint16_t wn;  // [week].
  uint8_t l2_ca_or_p;
  uint8_t ura;
  uint8_t health;
  uint16_t iodc;
  uint8_t l2pdata;
  float t_gd;     // [s].
  uint32_t t_oc;  // [s].
  float a_f2;     // [s/s/s].
  float a_f1;     // [s/s].
  float a_f0;     // [s].

  // Subframe 2.
  uint8_t iode2;
  float c_rs;     // [m].
  float delta_n;  // [semi-circle/s].
  double m_0;     // [semi-circle].
  float c_uc;     // [rad].
  double ecc;     // [#].
  float c_us;     // [rad].
  double sqrt_a;  // [m^(1/2)]
  uint32_t t_oe;  // [s].
  uint8_t fit_interval_flag;

  // Subframe 3.
  float c_ic;       // [rad].
  double omega_0;   // [semi-circle].
  float c_is;       // [rad].
  double i_0;       // [semi-circle].
  float c_rc;       // [m].
  double omega;     // [semi-circle].
  float omega_dot;  // [semi-circle/s]
  uint8_t iode3;
  float i_dot;      // [semi-circle/s].
} GpsEphemeris;

typedef struct {
  double alpha0;  // [s].
  double alpha1;  // [s/semi-circle].
  double alpha2;  // [s/semi-circle^2].
  double alpha3;  // [s/semi-circle^3].
  double beta0;   // [s].
  double beta1;   // [s/semi-circle].
  double beta2;   // [s/semi-circle^2].
  double beta3;   // [s/semi-circle^3].
} GpsIonosphere;

typedef struct {
  double a0;        // [s].
  double a1;        // [s/s].
  uint32_t tot;     // [s].
  uint16_t wnt;     // [week].
  uint16_t wn_lsf;  // [week].
  uint16_t dn;      // [day].
  int16_t dt_ls;    // [s].
  int16_t dt_lsf;   // [s].
} GpsUtc;

#endif  // AVIONICS_COMMON_GPS_TYPES_H_

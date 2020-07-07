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

#ifndef SIM_PHYSICS_AERO_TYPES_H_
#define SIM_PHYSICS_AERO_TYPES_H_

#include "common/c_math/force_moment.h"
#include "common/c_math/vec3.h"
#include "system/labels.h"

typedef struct {
  double CX, CY, CZ, CL, CM, CN;
  double CXp, CYp, CZp, CLp, CMp, CNp;
  double CXq, CYq, CZq, CLq, CMq, CNq;
  double CXr, CYr, CZr, CLr, CMr, CNr;
  double CXd[kNumFlaps];
  double CYd[kNumFlaps];
  double CZd[kNumFlaps];
  double CLd[kNumFlaps];
  double CMd[kNumFlaps];
  double CNd[kNumFlaps];
} AvlAeroCoeffs;

typedef struct {
  ForceMoment cfm;
  ForceMoment dcfm_dp;
  ForceMoment dcfm_dq;
  ForceMoment dcfm_dr;
  ForceMoment flap_cfms[kNumFlaps];
  ForceMoment flap_dcfm_dps[kNumFlaps];
  ForceMoment flap_dcfm_dqs[kNumFlaps];
  ForceMoment flap_dcfm_drs[kNumFlaps];
} DvlAeroCoeffs;

typedef struct {
  // Total coefficient.
  double dvl_weight;
  ForceMoment force_moment_coeff;

  // Raw AVL coefficients.
  double lower_reynolds_small_deflections_weight;
  AvlAeroCoeffs lower_reynolds_small_deflections_aero_coeffs;
  AvlAeroCoeffs upper_reynolds_small_deflections_aero_coeffs;
  ForceMoment small_deflections_force_moment_coeff;

  // Raw DVL coefficients.
  double lower_reynolds_dvl_weight;
  DvlAeroCoeffs lower_reynolds_dvl_aero_coeffs;
  DvlAeroCoeffs upper_reynolds_dvl_aero_coeffs;
  ForceMoment dvl_force_moment_coeff;
} RawAeroCoeffs;

typedef struct {
  Vec3 p;
  Vec3 q;
  Vec3 r;
} AeroRateDerivatives;

// Structure containing force or moment coefficients, their
// derivatives with respect to the body rates, and flap deflections.
typedef struct {
  Vec3 coeff;
  AeroRateDerivatives rate_derivatives;
  Vec3 flap_derivatives[kNumFlaps];
} AeroCoeffs;

// Structure containing additional offsets and slope offsets to apply
// to important coefficients.
typedef struct {
  double CD, CC, CL;
  double Cl, Cm, Cn;
  double dCldbeta, dCmdalpha, dCndbeta;
} AeroCoeffOffsets;

#endif  // SIM_PHYSICS_AERO_TYPES_H_

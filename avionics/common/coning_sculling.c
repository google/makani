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

#include "avionics/common/coning_sculling.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/vec3f.h"

#define CONING_SCULLING_LSTEP_COUNT 2

// See Strapdown Analytics, 2nd Ed. ~ Paul Savage.

// Integral of time(m-1) to time(l).
typedef struct {
  // Time increment.
  int32_t count;
  float dt;

  // Attitude increment.
  Vec3f dalpha_l;
  Vec3f alpha;
  Vec3f beta;

  // Velocity increment.
  Vec3f dnu_l;
  Vec3f nu;
  Vec3f dvscul;
} ConingScullingLStep;

typedef struct {
  Vec3f omega_ib_b;
  Vec3f a_sf_b;
  int32_t l;
  ConingScullingLStep lstep[CONING_SCULLING_LSTEP_COUNT];
} ConingScullingLCycle;

static ConingScullingLCycle g_lcycle;

// Compute trigonometric ratios using Taylor series expansion to ensure
// numerical stability near zero.  This function guarantees stability for
// 0 <= x2 < 1.
// Compute (1 - sin(a)/a)/a^2 when x2==a^2 and f0==3.
// Compute (1 - cos(a))/a^2 when x2==a^2 and f0==2.
static float DcmTaylorSeriesTrig(float x2, int32_t f0) {
  // First order term.
  int32_t f = 1;
  int32_t fact = 1;
  while (f < f0) {
    ++f;
    fact *= f;
  }
  float delta = 1.0f / (float)fact;
  float ans = delta;

  // Higher order terms.
  while (fabsf(delta) > FLT_EPSILON && f < 64) {
    ++f;
    fact = f;
    ++f;
    fact *= -f;
    delta *= x2 / (float)fact;
    ans += delta;
  }
  return ans;
}

// Increment l-step computation cycle attitude increment.
//
// Args:
//   lstep_1: l-step computation cycle at time(l-1).
//   dalpha_l: Angular rate integral over time(l-1) to time(l).
//   alpha: Integrated B frame angular rate increment over time(m-1) to time(l).
//   beta: Integrated B frame coning increment over time(m-1) to time(l).
static void LStepAlphaBeta(const ConingScullingLStep *lstep_1,
                           const Vec3f *dalpha_l, Vec3f *alpha, Vec3f *beta) {
  // Equation (7.1.1.1.1-17).
  // alpha = alpha_1 + Dalpha_l.
  Vec3fAdd(&lstep_1->alpha, dalpha_l, alpha);

  // Equation (7.1.1.1.1-18).
  // Dbeta = 0.5*cross(alpha_1 + 1/6*Dalpha_l_1), Dalpha_l).
  // beta = beta_1 + Dbeta.
  Vec3f a, dbeta;
  Vec3fLinComb(0.5f, &lstep_1->alpha, 0.5f / 6.0f, &lstep_1->dalpha_l, &a);
  Vec3fCross(&a, dalpha_l, &dbeta);
  Vec3fAdd(&lstep_1->beta, &dbeta, beta);
}

// Increment l-step computation cycle velocity increment.
//
// Args:
//   lstep_1: l-step computation cycle at time(l-1).
//   dalpha_l: Angular rate integral over time(l-1) to time(l).
//   dnu_l: Specific force integral over time(l-1) to time(l).
//   alpha: Integrated B frame angular rate increment over time(m-1) to time(l).
//   nu: Integrated B frame acceleration increment over time(m-1) to time(l).
//   dvscul: Integrated B frame sculling increment over time(m-1) to time(l).
static void LStepNuDvScul(const ConingScullingLStep *lstep_1,
                          const Vec3f *dalpha_l, const Vec3f *dnu_l,
                          const Vec3f *alpha, Vec3f *nu, Vec3f *dvscul) {
  // Equation (7.2.2.2.2-14).
  // nu = nu_1 + Dnu.
  Vec3fAdd(&lstep_1->nu, dnu_l, nu);

  // Equation (7.2.2.2.2-15).
  // dvScul = 0.5*(cross((alpha_l + 1/6*Dalpha_l_1), Dnu_l)
  //               + cross((nu_l + 1/6*Dnu_l_1), Dalpha_l)).
  // DvScul = DvScul_1 + dvScul.
  Vec3f a, b;
  Vec3fLinComb(0.5f, alpha, 0.5f / 6.0f, &lstep_1->dalpha_l, &a);
  Vec3fCross(&a, dnu_l, dvscul);
  Vec3fLinComb(0.5f, nu, 0.5f / 6.0f, &lstep_1->dnu_l, &a);
  Vec3fCross(&a, dalpha_l, &b);
  Vec3fAdd(dvscul, &b, dvscul);
}

// Increment l-step computation cycle.
//
// Args:
//   lstep_1: l-step computation cycle at time(l-1).
//   dt_l: Time integral over time(l-1) to time(l).
//   dalpha_l: Angular rate integral over time(l-1) to time(l).
//   dnu_l: Specific force integral over time(l-1) to time(l).
//   lstep: l-step computation cycle at time(l) output.
static int32_t LStepInc(const ConingScullingLStep *lstep_1, float dt_l,
                        const Vec3f *dalpha_l, const Vec3f *dnu_l,
                        ConingScullingLStep *lstep) {
  // Time increment.
  lstep->dt = lstep_1->dt + dt_l;
  lstep->count = lstep_1->count + 1;

  // Attitude increment.
  LStepAlphaBeta(lstep_1, dalpha_l, &lstep->alpha, &lstep->beta);
  lstep->dalpha_l = *dalpha_l;

  // Velocity increment.
  LStepNuDvScul(lstep_1, dalpha_l, dnu_l, &lstep->alpha, &lstep->nu,
                &lstep->dvscul);
  lstep->dnu_l = *dnu_l;

  return lstep->count;
}

static void LStepPhim(const ConingScullingLStep *lstep, Vec3f *phi_m) {
  // Equation (7.1.1.1-12).
  // phi_m = alpha_m + beta_m.
  Vec3fAdd(&lstep->alpha, &lstep->beta, phi_m);
}

static void LStepDvRotm(const ConingScullingLStep *lstep, Vec3f *dvrot_m) {
  // Equation (7.2.2.2.1-7).
  // Equation (7.2.2.2.1-8).
  // DvRot_m = (1 - cos(a))/a^2 * cross(alpha, nu)
  //           + 1/a^2 * (1 - sin(a)/a) * cross(alpha, cross(alpha, nu))
  float alpha2 = Vec3fDot(&lstep->alpha, &lstep->alpha);
  Vec3f alpha_x_nu, alpha_x_alpha_x_nu;
  Vec3fCross(&lstep->alpha, &lstep->nu, &alpha_x_nu);
  Vec3fCross(&lstep->alpha, &alpha_x_nu, &alpha_x_alpha_x_nu);
  Vec3fScale(&alpha_x_nu, DcmTaylorSeriesTrig(alpha2, 2), dvrot_m);
  Vec3fAxpy(DcmTaylorSeriesTrig(alpha2, 3), &alpha_x_alpha_x_nu, dvrot_m);
}

static void LStepDvSFm(const ConingScullingLStep *lstep, Vec3f *dvsf_m) {
  // Equation (7.2.2.2-23).
  // DvSFm_BIm_1 = nu_m + DvRot_m + DvScul_m.
  LStepDvRotm(lstep, dvsf_m);
  Vec3fAdd(dvsf_m, &lstep->nu, dvsf_m);
  Vec3fAdd(dvsf_m, &lstep->dvscul, dvsf_m);
}

static void LStepSample(const ConingScullingLStep *lstep, float *dt_m,
                        Vec3f *phi_m, Vec3f *dvsf_m, Vec3f *alpha_m,
                        Vec3f *nu_m) {
  // Sample time increment.
  *dt_m = lstep->dt;

  // Sample attitude increment.
  LStepPhim(lstep, phi_m);

  // Sample velocity increment.
  LStepDvSFm(lstep, dvsf_m);

  // Output integrals for bias compensation.
  *alpha_m = lstep->alpha;
  *nu_m = lstep->nu;
}

static void LStepReset(ConingScullingLStep *lstep) {
  // Reset time increment.
  lstep->dt = 0;
  lstep->count = 0;

  // Equation (7.1.1.1.1-17).
  lstep->alpha = kVec3fZero;

  // Equation (7.1.1.1.1-18).
  lstep->beta = kVec3fZero;

  // Equation (7.2.2.2.2-14).
  lstep->nu = kVec3fZero;

  // Equation (7.2.2.2.2-15).
  lstep->dvscul = kVec3fZero;
}

void ConingScullingInit(void) {
  memset(&g_lcycle, 0, sizeof(g_lcycle));
}

bool ConingScullingUpdateInc(int32_t dec, float dt_l, const Vec3f *dalpha_l,
                             const Vec3f *dnu_l) {
  assert(dalpha_l != NULL);
  assert(dnu_l != NULL);

  // Determine l-step indices for time(l-1) and time(l).
  int32_t l_1 = g_lcycle.l;
  ++g_lcycle.l;
  g_lcycle.l %= CONING_SCULLING_LSTEP_COUNT;

  // Increment.
  return LStepInc(&g_lcycle.lstep[l_1], dt_l, dalpha_l, dnu_l,
                  &g_lcycle.lstep[g_lcycle.l]) >= dec;
}

bool ConingScullingUpdateRaw(int32_t dec, float dt_l, const Vec3f *omega_ib_b,
                             const Vec3f *a_sf_b) {
  assert(omega_ib_b != NULL);
  assert(a_sf_b != NULL);

  // Trapezoidal integration of omega_ib_b.
  float half_dt = 0.5f * dt_l;
  Vec3f dalpha_l;
  Vec3fLinComb(half_dt, &g_lcycle.omega_ib_b, half_dt, omega_ib_b, &dalpha_l);
  g_lcycle.omega_ib_b = *omega_ib_b;

  // Trapezoidal integration of a_sf_b.
  Vec3f dnu_l;
  Vec3fLinComb(half_dt, &g_lcycle.a_sf_b, half_dt, a_sf_b, &dnu_l);
  g_lcycle.a_sf_b = *a_sf_b;

  return ConingScullingUpdateInc(dec, dt_l, &dalpha_l, &dnu_l);
}

void ConingScullingSample(float *dt_m, Vec3f *phi_m, Vec3f *dvsf_m,
                          Vec3f *alpha_m, Vec3f *nu_m) {
  LStepSample(&g_lcycle.lstep[g_lcycle.l], dt_m, phi_m, dvsf_m, alpha_m, nu_m);
  LStepReset(&g_lcycle.lstep[g_lcycle.l]);
}

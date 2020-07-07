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

#include "control/simple_aero.h"

#include <assert.h>
#include <float.h>
#include <math.h>

#include "common/c_math/optim.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"

double CalcLocalAirspeed(double airspeed, double local_pressure_coeff,
                         const Vec3 *pos, const Vec3 *pqr) {
  assert(local_pressure_coeff <= 1.0);
  assert(pos != NULL && pqr != NULL);

  Vec3 rotational_vel;
  Vec3Cross(pqr, pos, &rotational_vel);
  return (airspeed + rotational_vel.x) * Sqrt(1.0 - local_pressure_coeff);
}

double AlphaToCD(double alpha, const SimpleAeroModelParams *params) {
  return params->dCD_dalpha * alpha + params->CD_0;
}

double AlphaToCL(double alpha, const SimpleAeroModelParams *params) {
  return params->dCL_dalpha * alpha + params->CL_0;
}

double CLToAlpha(double CL, const SimpleAeroModelParams *params) {
  assert(params->dCL_dalpha > 0.0);  // Prevents divide-by-zero.
  return (CL - params->CL_0) / params->dCL_dalpha;
}

double CLAndFlapsToAlpha(double CL, const double flaps[],
                         const SimpleAeroModelParams *params) {
  double alpha = CL - params->CL_0;
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    alpha -= params->dCL_dflap[i] * (flaps[i] - params->base_flaps[i]);
  }
  alpha /= params->dCL_dalpha;
  return alpha;
}

double BetaToCY(double beta, const SimpleAeroModelParams *params) {
  return params->dCY_dbeta * beta + params->CY_0;
}

double CYToBeta(double CY, const SimpleAeroModelParams *params) {
  assert(params->dCY_dbeta < 0.0);  // Prevents divide-by-zero.
  return (CY - params->CY_0) / params->dCY_dbeta;
}

double JToOmega(double J, double v_freestream, double D) {
  assert(J > 0.0 && D > 0.0);
  return 2.0 * PI * v_freestream / fmax(J * D, 1e-6);
}

double OmegaToJ(double omega, double v_freestream, double D) {
  assert(omega >= 0.0);
  return 2.0 * PI * v_freestream / fmax(omega * D, 1.0);
}

// The following functions calculate the thrust and torque
// coefficients, which are both defined by:
//
//   T = air_density * (omega / 2.0 / PI)^2 * D^4 * k(J)
//
// where T is thrust or torque and where the thrust or torque
// coefficient is modeled as:
//
//   k(J) = a1*(J - J_neutral) + a2*(J - J_neutral)^2 + a3*(J - J_neutral)^3
//
// where J is the advance ratio and J_neutral is the advance ratio at
// zero thrust or torque (approximately the same).
static double CalcThrustCoeff(double J, const SimpleRotorModelParams *params) {
  double dJ = J - params->J_neutral;
  return PolyVal(params->thrust_coeffs, dJ, NUM_SIMPLE_ROTOR_MODEL_COEFFS) * dJ;
}

double OmegaToThrust(double omega, double v_freestream, double air_density,
                     const SimpleRotorModelParams *params) {
  assert(omega >= 0.0 && air_density > 0.0);
  double J = OmegaToJ(omega, v_freestream, params->D);
  double angular_rate_hz = omega / (2.0 * PI);

  return air_density * angular_rate_hz * angular_rate_hz * params->D4 *
         CalcThrustCoeff(J, params);
}

// Structure used as the context for functions passed to the Newton's
// method solver.
typedef struct {
  double value;
  double v_freestream;
  double air_density;
  const SimpleRotorModelParams *model;
} NewtonFuncParams;

// The following two functions are passed to the Newton's method
// solver to invert the OmegaToThrust function.
//
//   f(omega) = T - scale * k_T(J) * omega^2
//
//   f'(omega) = scale * (-k_T'(J) * omega^2 - 2*k_T(J) * omega)
//             = scale * ((3*a3*(J - J_neutral)^2 + 2*a2*(J - J_neutral) + a1)*J
//                        - 2*k_T(J)) * omega
//
// where scale = air_density * D^4 / (4.0 * PI^2).
static double NewtonFunc(double omega, const void *context) {
  const NewtonFuncParams *params = context;
  double value = OmegaToThrust(omega, params->v_freestream, params->air_density,
                               params->model);
  return params->value - value;
}

// Due to a saturation in the OmegaToThrust function, this is not an
// exact derivative when omega is very small.  This should not affect
// the function in the normal operating regime.
static double NewtonFuncDerivative(double omega, const void *context) {
  const NewtonFuncParams *params = context;
  // The simple rotor model polynomials are defined as having no
  // constant term.  The PolyDer call below correctly computes:
  //     derive_coeffs[i] = (n - i) * params->coeffs[i]^(n - i - 1).
  double deriv_coeffs[NUM_SIMPLE_ROTOR_MODEL_COEFFS];
  PolyDer(params->model->thrust_coeffs, ARRAYSIZE(deriv_coeffs), deriv_coeffs);

  double J = OmegaToJ(omega, params->v_freestream, params->model->D);
  double dJ = J - params->model->J_neutral;
  double thrust_coeff = CalcThrustCoeff(J, params->model);

  return (params->air_density * params->model->D4 / (4.0 * PI * PI)) *
         (PolyVal(deriv_coeffs, dJ, ARRAYSIZE(deriv_coeffs)) * J -
          2.0 * thrust_coeff) *
         omega;
}

double ThrustToOmega(double thrust, double v_freestream, double air_density,
                     const SimpleRotorModelParams *params) {
  assert(params->D > 0.0 && air_density > 0.0);
  assert(0.0 < params->J_neutral && params->J_neutral < params->J_max);

  double omega;
  if (fabs(v_freestream) < DBL_EPSILON) {
    // For v_freestream == 0.0, assume the static thrust special case where
    // you may simply invert the equation because the thrust
    // coefficient does not depend on omega.
    double coeff = (air_density * params->D4 / (4.0 * PI * PI)) *
                   CalcThrustCoeff(0.0, params);
    omega = sqrt(fmax(thrust / coeff, 0.0));
  } else {
    // Assume stall point defines a minimum omega and minimum thrust,
    // i.e. maximum drag.
    double omega_min =
        JToOmega(params->J_max, fmax(v_freestream, 1.0), params->D);
    double thrust_min =
        OmegaToThrust(omega_min, v_freestream, air_density, params);

    // Start with neutral thrust omega as guess.  This must be greater
    // than omega_min if J_neutral < J_max.
    double omega0 =
        JToOmega(params->J_neutral, fmax(v_freestream, 10.0), params->D);
    NewtonFuncParams newton_func_params = {.value = fmax(thrust, thrust_min),
                                           .v_freestream = v_freestream,
                                           .air_density = air_density,
                                           .model = params};
    omega = Newton(&NewtonFunc, &NewtonFuncDerivative, &newton_func_params,
                   omega0, omega_min, INFINITY, 0.1, 10);
  }
  return omega;
}

double CalcMinThrust(double v_app_local, double omega_idle, double air_density,
                     const SimpleRotorModelParams *params) {
  double omega_J_max =
      JToOmega(params->J_max, fmax(0.0, v_app_local), params->D);
  return OmegaToThrust(fmax(omega_idle, omega_J_max), v_app_local, air_density,
                       params);
}

void CalcAeroForce(const Vec3 *apparent_wind_b, double density,
                   double reference_area, const SimpleAeroModelParams *params,
                   Vec3 *aero_force_b) {
  // We would like to use 'ApparentWindCartToSph' but having
  // simple_aero depend on control_util introduces a cyclic
  // dependency.
  const double alpha = atan2(-apparent_wind_b->z, -apparent_wind_b->x);
  const double beta = atan2(-apparent_wind_b->y, Vec3XzNorm(apparent_wind_b));

  Vec3 apparent_wind_b_hat;
  Vec3Normalize(apparent_wind_b, &apparent_wind_b_hat);

  const double q = 0.5 * density * Vec3NormSquared(apparent_wind_b);

  double CL = AlphaToCL(alpha, params);
  double CY = BetaToCY(beta, params);
  double CD = AlphaToCD(alpha, params);

  Vec3 lift, drag, side;
  Vec3Scale(&apparent_wind_b_hat, q * reference_area * CD, &drag);
  Vec3Scale(&apparent_wind_b_hat, q * reference_area * CL, &lift);
  Vec3Cross(&lift, &kVec3Y, &lift);
  Vec3Scale(&kVec3Y, q * reference_area * CY, &side);

  Vec3Add3(&drag, &lift, &side, aero_force_b);
}

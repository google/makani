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

#ifndef CONTROL_SIMPLE_AERO_H_
#define CONTROL_SIMPLE_AERO_H_

#include "common/c_math/vec3.h"
#include "control/simple_aero_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Returns an approximate local airspeed accounting for angular rate
// and pressure coefficient variations.  The pressure coefficient,
// C_P, is related to local airspeed through the equation:
//
//   C_P = 1 - (v / v_freestream)^2
//
double CalcLocalAirspeed(double airspeed, double local_pressure_coeff,
                         const Vec3 *pos, const Vec3 *pqr);

// Converts an angle-of-attack (alpha) in radians to a drag
// coefficient (CD) using a simplified linear model of the CD
// vs. alpha curve.
double AlphaToCD(double alpha, const SimpleAeroModelParams *params);

// Converts an angle-of-attack (alpha) in radians to a lift
// coefficient (CL) using a simplified linear model of the CL
// vs. alpha curve.
double AlphaToCL(double alpha, const SimpleAeroModelParams *params);

// Converts a lift coefficient (CL) to an angle-of-attack (alpha)
// using a simplified linear model of the CL vs. alpha curve.
double CLToAlpha(double CL, const SimpleAeroModelParams *params);

// Converts a lift coefficient (CL) and array of flap deflections (flaps)
// to an angle-of-attack (alpha) using a model of CL that is linear
// in alpha and the flap offsets.
double CLAndFlapsToAlpha(double CL, const double flaps[],
                         const SimpleAeroModelParams *params);

// Converts a sideslip (beta) in radians to a sideways lift
// coefficient (CY) using a simplified linear model of the CY
// vs. beta curve.
double BetaToCY(double beta, const SimpleAeroModelParams *params);

// Converts a sideways lift coefficient (CY) to a sideslip (beta) in
// radians using a simplified linear model of the CY vs. beta curve.
double CYToBeta(double CY, const SimpleAeroModelParams *params);

// Converts a rotor angular rate omega to an equivalent thrust at a
// given freestream velocity.
double OmegaToThrust(double omega, double v_freestream, double air_density,
                     const SimpleRotorModelParams *params);

// Calculates angular rate omega from advance ratio J and freestream
// velocity v_freestream.
double JToOmega(double J, double v_freestream, double D);

// Calculates advance ratio J from angular rate omega and freestream
// velocity v_freestream.
double OmegaToJ(double omega, double v_freestream, double D);

// Converts a thrust request to an equivalent angular rate omega by
// using the Newton-Raphson method to find inverse of OmegaToThrust.
// We include a special case to handle v_freestream == 0.0 both for
// computational speed and also for a smoother function.
double ThrustToOmega(double thrust, double v_freestream, double air_density,
                     const SimpleRotorModelParams *params);

// Calculate the minimum thrust a rotor can apply without slowing
// below omega_idle or exceeding its advance ratio limits.
double CalcMinThrust(double v_app_local, double omega_idle, double air_density,
                     const SimpleRotorModelParams *params);

// Calculate the total aerodynamic force on the wing, given the
// apparent wind and a simple aero model.
void CalcAeroForce(const Vec3 *apparent_wind_b, double density,
                   double reference_area, const SimpleAeroModelParams *params,
                   Vec3 *aero_force_b);
#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_SIMPLE_AERO_H_

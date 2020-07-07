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

#ifndef SIM_PHYSICS_AVL_AERO_DATABASE_H_
#define SIM_PHYSICS_AVL_AERO_DATABASE_H_

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include <stdint.h>

#include <string>

#include "common/c_math/linalg.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "sim/sim_params.h"
#include "sim/sim_types.h"
#include "system/labels.h"

class AvlAeroDatabase {
  friend class AeroTest;

 public:
  AvlAeroDatabase(const std::string &filename,
                  const AeroSimParams &aero_sim_params);
  ~AvlAeroDatabase();

  // Applies coefficient fudge factors.
  //
  // Args:
  //   coeff_offsets: Offsets to apply.
  //   force_coeff_w_scale_factors: Scale factors [#] to apply to the
  //       force coefficients.  These scale factors should be represented in
  //       the wind axes.
  //   moment_coeff_b_scale_factors: Scale factors [#] to apply to the
  //       moment coefficients.  These scale factors should be represented in
  //       the body axes.
  void ApplyCoeffAdjustments(const AeroCoeffOffsets &coeff_offsets,
                             const AeroCoeffs &force_coeff_w_scale_factors,
                             const AeroCoeffs &moment_coeff_b_scale_factors);

  // Returns the total force-moment coefficient at a specific (alpha,
  // beta, omega_hat, flap deflection) operating point.
  void CalcForceMomentCoeff(double alpha, double beta, const Vec3 &omega_hat,
                            const Vec &flaps, ForceMoment *cfm,
                            double thrust_coeff) const {
    CalcForceMomentCoeff(alpha, beta, omega_hat, flaps, cfm, thrust_coeff,
                         nullptr);
  }

  // See function above.  Optionally returns the breakdown of the
  // force-moment coefficient into control and stability derivatives
  // if aero_coeffs is not null.
  void CalcForceMomentCoeff(double alpha, double beta, const Vec3 &omega_hat,
                            const Vec &flaps, ForceMoment *cfm,
                            double thrust_coeff,
                            AvlAeroCoeffs *aero_coeffs) const;

  // Returns the nominal elevator deflection for a given angle-of-attack
  // and sideslip angle.
  double GetNominalElevatorDeflection(double alpha, double beta) const;

  double reynolds_number() const { return reynolds_number_; }

  int32_t num_flaps() const { return num_flaps_; }
  const Vec3 &omega_hat_0() const { return omega_hat_0_; }

 private:
  // Returns true if the database passes basic sign checks.
  bool IsValid() const;

  // Reduces flap effectiveness at large deflections.
  void AdjustNonlinearFlaps(const Vec &flaps, Vec *flaps_nonlin) const;

  // Returns the nominal force-moment coefficient and the control and
  // stability derivatives.
  void CalcAvlAeroCoeffs(double alpha, double beta,
                         AvlAeroCoeffs *coeffs) const;

  const AeroSimParams &aero_sim_params_;

  double reynolds_number_;
  int32_t num_flaps_;
  gsl_vector *alphads_, *betads_;
  gsl_matrix *eleds_;

  Vec3 omega_hat_0_;
  gsl_matrix *CX_, *CY_, *CZ_, *CL_, *CM_, *CN_;
  gsl_matrix *CXp_, *CYp_, *CZp_, *CLp_, *CMp_, *CNp_;
  gsl_matrix *CXq_, *CYq_, *CZq_, *CLq_, *CMq_, *CNq_;
  gsl_matrix *CXr_, *CYr_, *CZr_, *CLr_, *CMr_, *CNr_;
  gsl_matrix *CXd_[kNumFlaps];
  gsl_matrix *CYd_[kNumFlaps];
  gsl_matrix *CZd_[kNumFlaps];
  gsl_matrix *CLd_[kNumFlaps];
  gsl_matrix *CMd_[kNumFlaps];
  gsl_matrix *CNd_[kNumFlaps];

  DISALLOW_COPY_AND_ASSIGN(AvlAeroDatabase);
};

#endif  // SIM_PHYSICS_AVL_AERO_DATABASE_H_

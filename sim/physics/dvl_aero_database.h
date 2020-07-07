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

#ifndef SIM_PHYSICS_DVL_AERO_DATABASE_H_
#define SIM_PHYSICS_DVL_AERO_DATABASE_H_

#include <gsl/gsl_vector.h>
#include <stddef.h>
#include <stdint.h>

#include <string>
#include <vector>

#include "common/c_math/force_moment.h"
#include "common/c_math/linalg.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "sim/physics/aero_types.h"
#include "system/labels.h"

class DvlAeroDatabase {
 public:
  explicit DvlAeroDatabase(const std::string &filename);
  ~DvlAeroDatabase();

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
                            DvlAeroCoeffs *aero_coeffs) const;

  int32_t num_flaps() const { return num_flaps_; }
  double reynolds_number() const { return reynolds_number_; }

 private:
  // Returns true if the database passes basic sign checks.
  bool IsValid() const;

  // Returns the total force-moment coefficient at a specific (alpha,
  // beta, omega_hat, flap deflection) operating point and the
  // breakdown of this value into control and stability derivatives.
  void CalcDvlAeroCoeffs(double alpha, double beta, const Vec &flaps,
                         DvlAeroCoeffs *coeffs) const;

  void LookupForceMomentCoeff2D(const gsl_vector &cfm_data, size_t size1,
                                size_t size2, int32_t s0, int32_t t0, double ds,
                                double dt, ForceMoment *cfm) const;
  void LookupForceMomentCoeff3D(const gsl_vector &cfm_data, int32_t indices[],
                                double du, double dv, double dw,
                                ForceMoment *cfm) const;

  // Number of flaps used in database.
  int32_t num_flaps_;

  // Reynolds number [#] at which the database was calculated.
  double reynolds_number_;

  // List of angles-of-attack [rad] used in database look-up table.
  gsl_vector *alphas_;

  // List of sideslip angles [rad] used in database look-up table.
  gsl_vector *betas_;

  // Vector of lists of flap deflections [rad], one list for each
  // flap, used in database look-up table.
  std::vector<gsl_vector *> deltas_;

  // Tensor grid of force-moment coefficients at zero flap deflections
  // over angle-of-attack and sideslip, stored as a vector.
  gsl_vector *cfm_;

  // Tensor grid of force-moment stability derivatives at zero flap
  // deflections over angle-of-attack and sideslip, stored as a
  // vector.
  gsl_vector *dcfm_dp_;
  gsl_vector *dcfm_dq_;
  gsl_vector *dcfm_dr_;

  // Vector of tensor grids of increments to the force-moment
  // coefficients, due to flap deflections, for each flap.  The tensor
  // grid is over angle-of-attack, sideslip, and flap deflection, and
  // is stored as a vector.
  std::vector<gsl_vector *> flap_cfms_;

  // Vector of tensor grids of increments to the force-moment
  // stability derivatives, due to flap deflections, for each flap.
  // The tensor grid is over angle-of-attack, sideslip, and flap
  // deflection, and is stored as a vector.
  std::vector<gsl_vector *> flap_dcfm_dps_;
  std::vector<gsl_vector *> flap_dcfm_dqs_;
  std::vector<gsl_vector *> flap_dcfm_drs_;

  DISALLOW_COPY_AND_ASSIGN(DvlAeroDatabase);
};

#endif  // SIM_PHYSICS_DVL_AERO_DATABASE_H_

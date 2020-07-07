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

#ifndef SIM_PHYSICS_DELTA_COEFF_AERO_DATABASE_H_
#define SIM_PHYSICS_DELTA_COEFF_AERO_DATABASE_H_

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

class DeltaCoeffAeroDatabase {
  friend class AeroTest;

 public:
  explicit DeltaCoeffAeroDatabase(const std::string &filename);
  ~DeltaCoeffAeroDatabase();

  // Returns the nominal force-moment coefficient after adjusting for
  // delta coefficients.
  void AdjustForceMomentCoeff(double alpha, const Vec &flaps,
                              ForceMoment *force_moment_coeff) const;

 private:
  // Returns true if the database passes basic sign checks.
  bool IsValid() const;

  bool applicable_flaps_[kNumFlaps];

  double reynolds_number_;
  gsl_vector *alphads_, *deltads_;

  gsl_matrix *CX_[kNumFlaps];
  gsl_matrix *CY_[kNumFlaps];
  gsl_matrix *CZ_[kNumFlaps];
  gsl_matrix *CL_[kNumFlaps];
  gsl_matrix *CM_[kNumFlaps];
  gsl_matrix *CN_[kNumFlaps];

  DISALLOW_COPY_AND_ASSIGN(DeltaCoeffAeroDatabase);
};

#endif  // SIM_PHYSICS_DELTA_COEFF_AERO_DATABASE_H_

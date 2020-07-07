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

#ifndef SIM_PHYSICS_AERO_H_
#define SIM_PHYSICS_AERO_H_

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <stdint.h>

#include <memory>
#include <string>
#include <vector>

#include "common/c_math/force_moment.h"
#include "common/c_math/linalg.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/physics/aero_types.h"
#include "sim/physics/avl_aero_database.h"
#include "sim/physics/delta_coeff_aero_database.h"
#include "sim/physics/dvl_aero_database.h"
#include "sim/sim_types.h"

class Aero {
  friend class AeroTest;

 public:
  explicit Aero(const AeroSimParams &aero_sim_params);
  ~Aero() {}

  // Calculates the force-moment coefficient for a given (alpha, beta,
  // omega_hat, Reynolds number, flap deflection) operating point,
  // taking into account information from multiple databases.
  void CalcForceMomentCoeff(double alpha, double beta, const Vec3 &omega_hat,
                            const Vec &flaps, double reynolds_number,
                            ForceMoment *force_moment_coeff,
                            double thrust_coeff) const {
    CalcForceMomentCoeff(alpha, beta, omega_hat, flaps, reynolds_number,
                         force_moment_coeff, thrust_coeff, nullptr);
  }

  // See function above.  Optionally returns the breakdown of the
  // force-moment coefficient into stability and control derivatives
  // if raw_aero_coeffs is not null.
  void CalcForceMomentCoeff(double alpha, double beta, const Vec3 &omega_hat,
                            const Vec &flaps, double reynolds_number,
                            ForceMoment *force_moment_coeff,
                            double thrust_coeff,
                            RawAeroCoeffs *raw_aero_coeffs) const;

  void CalcLowIncidenceCoeffs(double alpha, double beta, const Vec3 &omega_hat,
                              const Vec &flaps, double reynolds_number,
                              ForceMoment *force_moment_coeff,
                              double thrust_coeff) const;

  void CalcHighIncidenceCoeffs(double alpha, double beta, const Vec3 &omega_hat,
                               const Vec &flaps, double reynolds_number,
                               ForceMoment *force_moment_coeff,
                               double thrust_coeff) const;

  double CalcHingeCM(FlapLabel label, double alpha, double delta) const;

  double CalcEmpiricalPitchingMoment(double alpha) const;

 private:
  // Returns true if the parameters to the aerodynamic model pass
  // basic consistency checks.
  bool ValidateParams(const AeroSimParams &params) const;

  // Linearly interpolates the force-moment coefficients between two
  // aerodynamics databases based on Reynolds number.  Returns the
  // weight used to mix the databases of the lower Reynolds database.
  template <typename Database, typename DatabaseCoeffs>
  double InterpolateAeroDatabases(
      double alpha, double beta, const Vec3 &omega_hat, const Vec &flaps,
      double reynolds_number,
      const std::vector<std::unique_ptr<Database>> &databases,
      DatabaseCoeffs *lower_coeffs, DatabaseCoeffs *upper_coeffs,
      ForceMoment *cfm, double thrust_coeff) const;
  const AeroSimParams &aero_sim_params_;

  // Set of databases to use when the flaps are deflected in the
  // linear range.  These databases are typically used during
  // crosswind flight.
  std::vector<std::unique_ptr<AvlAeroDatabase>> small_deflection_databases_;

  // Set of databases to use when the flaps may be deflected a large
  // amount, outside the linear range.  These databases are typically
  // used during hover and transition when the elevator may be
  // feathered.
  std::vector<std::unique_ptr<DvlAeroDatabase>> large_deflection_databases_;

  // Set of databases to use when the spoilers are deflected to
  // max range.  These databases are typically used during
  // crosswind flight, during high speed events.
  std::unique_ptr<DeltaCoeffAeroDatabase> spoiler_offset_database_;

  DISALLOW_COPY_AND_ASSIGN(Aero);
};

#endif  // SIM_PHYSICS_AERO_H_

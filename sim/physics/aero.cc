// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sim/physics/aero.h"

#include <jansson.h>

#include <glog/logging.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <algorithm>
#include <string>
#include <vector>

#include "common/c_math/force_moment.h"
#include "common/c_math/linalg.h"
#include "common/c_math/vec3.h"
#include "common/runfiles_dir.h"
#include "control/system_types.h"
#include "lib/json_load/json_array_loader.h"
#include "lib/json_load/json_load_or_die.h"
#include "sim/math/util.h"
#include "sim/physics/aero_frame.h"
#include "sim/physics/aero_types.h"
#include "sim/physics/avl_aero_database.h"
#include "sim/physics/delta_coeff_aero_database.h"
#include "sim/physics/dvl_aero_database.h"
#include "sim/sim_params.h"
#include "sim/sim_types.h"

Aero::Aero(const AeroSimParams &aero_sim_params)
    : aero_sim_params_(aero_sim_params),
      small_deflection_databases_(),
      large_deflection_databases_(),
      spoiler_offset_database_() {
  spoiler_offset_database_.reset(new DeltaCoeffAeroDatabase(
      RunfilesDir() + "/database/" +
      aero_sim_params_.spoiler_offset_database.name));

  for (const DatabaseName &dbname :
       aero_sim_params_.large_deflection_databases) {
    const char *name = dbname.name;
    if (strlen(name) != 0) {
      large_deflection_databases_.emplace_back(
          new DvlAeroDatabase(RunfilesDir() + "/database/" + name));
    }
  }
  std::sort(large_deflection_databases_.begin(),
            large_deflection_databases_.end(),
            [](const std::unique_ptr<DvlAeroDatabase> &a,
               const std::unique_ptr<DvlAeroDatabase> &b) -> bool {
              return a->reynolds_number() < b->reynolds_number();
            });
  for (const DatabaseName &dbname :
       aero_sim_params_.small_deflection_databases) {
    const char *name = dbname.name;
    if (strlen(name) != 0) {
      AvlAeroDatabase *db = new AvlAeroDatabase(
          RunfilesDir() + "/database/" + name, aero_sim_params_);
      small_deflection_databases_.emplace_back(db);
    }
  }

  for (std::unique_ptr<AvlAeroDatabase> &db : small_deflection_databases_) {
    db->ApplyCoeffAdjustments(aero_sim_params_.coeff_offsets,
                              aero_sim_params_.force_coeff_w_scale_factors,
                              aero_sim_params_.moment_coeff_b_scale_factors);
  }

  std::sort(small_deflection_databases_.begin(),
            small_deflection_databases_.end(),
            [](const std::unique_ptr<AvlAeroDatabase> &a,
               const std::unique_ptr<AvlAeroDatabase> &b) -> bool {
              return a->reynolds_number() < b->reynolds_number();
            });

  CHECK(ValidateParams(aero_sim_params))
      << "aero_sim_params does not pass consistency checks.";
}

bool Aero::ValidateParams(const AeroSimParams &params) const {
  // The angle over which the difference databases are blended must be
  // positive.
  if (params.linear_to_stalled_blending_angle <= 0.0) {
    return false;
  }

  // The low and high stall angles-of-attack must be further apart
  // than the angle used to blend the databases.
  if ((params.high_alpha_stall_angle - params.low_alpha_stall_angle) <
      params.linear_to_stalled_blending_angle) {
    return false;
  }

  // The low and high stall sideslip angles must be further apart than
  // the angle used to blend the databases.
  if ((params.high_beta_stall_angle - params.low_beta_stall_angle) <
      params.linear_to_stalled_blending_angle) {
    return false;
  }

  return true;
}

template <typename Database, typename DatabaseCoeffs>
double Aero::InterpolateAeroDatabases(
    double alpha, double beta, const Vec3 &omega_hat, const Vec &flaps,
    double reynolds_number,
    const std::vector<std::unique_ptr<Database>> &databases,
    DatabaseCoeffs *lower_coeffs, DatabaseCoeffs *upper_coeffs,
    ForceMoment *force_moment_coeff, double thrust_coeff) const {
  // If the Reynolds number lies outside of the range spanned by the DVL
  // databases, then use the nearest database.
  if (reynolds_number <= databases[0]->reynolds_number()) {
    databases[0]->CalcForceMomentCoeff(alpha, beta, omega_hat, flaps,
                                       force_moment_coeff, thrust_coeff,
                                       lower_coeffs);
    *upper_coeffs = *lower_coeffs;
    return 1.0;
  }
  if (reynolds_number >= (*databases.rbegin())->reynolds_number()) {
    (*databases.rbegin())
        ->CalcForceMomentCoeff(alpha, beta, omega_hat, flaps,
                               force_moment_coeff, thrust_coeff, upper_coeffs);
    *lower_coeffs = *upper_coeffs;
    return 0.0;
  }

  // Find the DVL databases whose Reynolds numbers we lie between, and linearly
  // interpolate the ForceMoment from them.
  int32_t i_upper = 0;
  double lower_reynolds_number = databases[0]->reynolds_number();
  double upper_reynolds_number = databases[0]->reynolds_number();
  while (reynolds_number > databases[i_upper]->reynolds_number()) {
    ++i_upper;
    lower_reynolds_number = databases[i_upper - 1]->reynolds_number();
    upper_reynolds_number = databases[i_upper]->reynolds_number();
  }

  ForceMoment cfm_lower;
  databases[i_upper - 1]->CalcForceMomentCoeff(
      alpha, beta, omega_hat, flaps, &cfm_lower, thrust_coeff, lower_coeffs);
  ForceMoment cfm_upper;
  databases[i_upper]->CalcForceMomentCoeff(
      alpha, beta, omega_hat, flaps, &cfm_upper, thrust_coeff, upper_coeffs);

  double c = (upper_reynolds_number - reynolds_number) /
             (upper_reynolds_number - lower_reynolds_number);

  ForceMomentLinComb(c, &cfm_lower, 1.0 - c, &cfm_upper, force_moment_coeff);

  return c;
}

void Aero::CalcLowIncidenceCoeffs(double alpha, double beta,
                                  const Vec3 &omega_hat, const Vec &flaps,
                                  double reynolds_number,
                                  ForceMoment *force_moment_coeff,
                                  double thrust_coeff) const {
  RawAeroCoeffs aero = RawAeroCoeffs();
  InterpolateAeroDatabases(alpha, beta, omega_hat, flaps, reynolds_number,
                           small_deflection_databases_,
                           &aero.lower_reynolds_small_deflections_aero_coeffs,
                           &aero.upper_reynolds_small_deflections_aero_coeffs,
                           &aero.small_deflections_force_moment_coeff,
                           thrust_coeff);

  *force_moment_coeff = aero.small_deflections_force_moment_coeff;
}

void Aero::CalcHighIncidenceCoeffs(double alpha, double beta,
                                   const Vec3 &omega_hat, const Vec &flaps,
                                   double reynolds_number,
                                   ForceMoment *force_moment_coeff,
                                   double thrust_coeff) const {
  RawAeroCoeffs aero = RawAeroCoeffs();
  InterpolateAeroDatabases(alpha, beta, omega_hat, flaps, reynolds_number,
                           large_deflection_databases_,
                           &aero.lower_reynolds_dvl_aero_coeffs,
                           &aero.upper_reynolds_dvl_aero_coeffs,
                           &aero.dvl_force_moment_coeff, thrust_coeff);

  *force_moment_coeff = aero.dvl_force_moment_coeff;
}

void Aero::CalcForceMomentCoeff(double alpha, double beta,
                                const Vec3 &omega_hat, const Vec &flaps,
                                double reynolds_number,
                                ForceMoment *force_moment_coeff,
                                double thrust_coeff,
                                RawAeroCoeffs *raw_aero_coeffs) const {
  RawAeroCoeffs aero = RawAeroCoeffs();

  // Offset the flaps by a set amount.  This is usually zero, but it
  // can be used to test for sensitivity to trim.
  VEC_CLONE(kNumFlaps, offset_flaps, aero_sim_params_.flap_offsets);
  VecAdd(&offset_flaps, &flaps, &offset_flaps);

  // Determine the mixing coefficient between the different aero
  // databases.
  if (aero_sim_params_.merge_databases) {
    double c_alpha =
        0.5 +
        fmax(Saturate((alpha - aero_sim_params_.high_alpha_stall_angle) /
                          aero_sim_params_.linear_to_stalled_blending_angle,
                      -0.5, 0.5),
             Saturate((aero_sim_params_.low_alpha_stall_angle - alpha) /
                          aero_sim_params_.linear_to_stalled_blending_angle,
                      -0.5, 0.5));
    double c_beta =
        0.5 +
        fmax(Saturate((beta - aero_sim_params_.high_beta_stall_angle) /
                          aero_sim_params_.linear_to_stalled_blending_angle,
                      -0.5, 0.5),
             Saturate((aero_sim_params_.low_beta_stall_angle - beta) /
                          aero_sim_params_.linear_to_stalled_blending_angle,
                      -0.5, 0.5));
    aero.dvl_weight = 1.0 - (1.0 - c_alpha) * (1.0 - c_beta);
  } else {
    aero.dvl_weight = 0.0;
  }

  if ((1e-6 < aero.dvl_weight && aero.dvl_weight < 1.0) ||
      aero_sim_params_.force_use_both_databases) {
    // Mix databases as function of angle-of-attack and sideslip.
    aero.lower_reynolds_small_deflections_weight = InterpolateAeroDatabases(
        alpha, beta, omega_hat, offset_flaps, reynolds_number,
        small_deflection_databases_,
        &aero.lower_reynolds_small_deflections_aero_coeffs,
        &aero.upper_reynolds_small_deflections_aero_coeffs,
        &aero.small_deflections_force_moment_coeff, thrust_coeff);

    aero.lower_reynolds_dvl_weight = InterpolateAeroDatabases(
        alpha, beta, omega_hat, offset_flaps, reynolds_number,
        large_deflection_databases_, &aero.lower_reynolds_dvl_aero_coeffs,
        &aero.upper_reynolds_dvl_aero_coeffs, &aero.dvl_force_moment_coeff,
        thrust_coeff);

    ForceMomentLinComb(
        aero.dvl_weight, &aero.dvl_force_moment_coeff, 1.0 - aero.dvl_weight,
        &aero.small_deflections_force_moment_coeff, force_moment_coeff);
  } else if (aero.dvl_weight <= 1e-6) {
    // Only use small deflections database.
    aero.lower_reynolds_small_deflections_weight = InterpolateAeroDatabases(
        alpha, beta, omega_hat, offset_flaps, reynolds_number,
        small_deflection_databases_,
        &aero.lower_reynolds_small_deflections_aero_coeffs,
        &aero.upper_reynolds_small_deflections_aero_coeffs,
        &aero.small_deflections_force_moment_coeff, thrust_coeff);

    *force_moment_coeff = aero.small_deflections_force_moment_coeff;

    if (aero_sim_params_.use_spoilers) {
      // Apply spoiler effects.
      spoiler_offset_database_->AdjustForceMomentCoeff(alpha, flaps,
                                                       force_moment_coeff);
    }
  } else {
    // Only use the DVL database.
    aero.lower_reynolds_dvl_weight = InterpolateAeroDatabases(
        alpha, beta, omega_hat, offset_flaps, reynolds_number,
        large_deflection_databases_, &aero.lower_reynolds_dvl_aero_coeffs,
        &aero.upper_reynolds_dvl_aero_coeffs, &aero.dvl_force_moment_coeff,
        thrust_coeff);

    *force_moment_coeff = aero.dvl_force_moment_coeff;
  }

  if (raw_aero_coeffs != nullptr) {
    *raw_aero_coeffs = aero;
    raw_aero_coeffs->force_moment_coeff = *force_moment_coeff;
  }
}

double Aero::CalcHingeCM(FlapLabel label, double alpha, double delta) const {
  DCHECK(-PI <= alpha && alpha <= PI) << "Angle-of-attack is out of range.";
  DCHECK(-PI <= delta && delta <= PI) << "Flap deflection is out of range.";

  // Saturate alpha and delta so we do not use this fit well outside
  // its intended range.
  double alphad = 180.0 / M_PI * Saturate(alpha, -0.35, 0.35);
  double deltad = 180.0 / M_PI * Saturate(delta, -0.35, 0.35);
  HingeMomentCoeffs hcoeff = aero_sim_params_.hinge_moment_coeffs[label];

  return hcoeff.c + hcoeff.c_deltad * deltad + hcoeff.c_alphad * alphad +
         hcoeff.c_alphad_deltad * alphad * deltad;
}

double Aero::CalcEmpiricalPitchingMoment(double alpha) const {
  double alpha_scale_factor = fmax(sin(alpha), 0.0);
  return aero_sim_params_.empirical_pitching_moment_correction *
         alpha_scale_factor;
}

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

#include "sim/physics/rotor_database.h"

#include <glog/logging.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <jansson.h>
#include <math.h>
#include <stdint.h>

#include <string>

#include "common/macros.h"
#include "lib/json_load/json_array_loader.h"
#include "lib/json_load/json_load_or_die.h"
#include "sim/math/util.h"

RotorDatabase::RotorDatabase(const std::string &filename)
    : diameter_(),
      angular_rates_(),
      freestream_vels_(),
      thrust_coeffs_(),
      power_coeffs_() {
  json_t *rotor_json = json_load::LoadFileOrDie(filename.c_str());

  int32_t num_angular_rates =
      json_load::LoadInt32OrDie(rotor_json, "num_omegas");
  int32_t num_freestream_vels =
      json_load::LoadInt32OrDie(rotor_json, "num_v_freestreams");

  diameter_ = json_load::LoadDoubleOrDie(rotor_json, "diameter");

  json_load::JsonArrayLoader loader(num_angular_rates, num_freestream_vels,
                                    rotor_json);

  angular_rates_ = gsl_vector_alloc(num_angular_rates);
  freestream_vels_ = gsl_vector_alloc(num_freestream_vels);

  loader.LoadVector("omegas", angular_rates_);
  loader.LoadVector("v_freestreams", freestream_vels_);

  power_coeffs_ = gsl_matrix_alloc(num_angular_rates, num_freestream_vels);
  thrust_coeffs_ = gsl_matrix_alloc(num_angular_rates, num_freestream_vels);

  loader.LoadMatrix("thrust_coeffs", thrust_coeffs_);
  loader.LoadMatrix("power_coeffs", power_coeffs_);

  json_decref(rotor_json);

  CHECK(IsValid()) << "Invalid rotor database (" << filename << ").";
}

RotorDatabase::~RotorDatabase() {
  gsl_vector_free(angular_rates_);
  gsl_vector_free(freestream_vels_);
  gsl_matrix_free(thrust_coeffs_);
  gsl_matrix_free(power_coeffs_);
}

double RotorDatabase::CalcThrust(double angular_rate, double freestream_vel,
                                 double air_density) const {
  double angular_rate_hz = angular_rate / (2.0 * M_PI);
  // See Eq. 9.18, Houghton & Carpenter, 4th ed.
  return air_density * angular_rate_hz * angular_rate_hz * pow(diameter_, 4.0) *
         LookupThrustCoeff(angular_rate, freestream_vel);
}

double RotorDatabase::CalcTorque(double angular_rate, double freestream_vel,
                                 double air_density) const {
  double angular_rate_hz = angular_rate / (2.0 * M_PI);
  // See Eq. 9.19, Houghton & Carpenter, 4th ed.
  return air_density * angular_rate_hz * angular_rate_hz * pow(diameter_, 5.0) *
         LookupTorqueCoeff(angular_rate, freestream_vel);
}

double RotorDatabase::CalcPower(double angular_rate, double freestream_vel,
                                double air_density) const {
  // See Eq. 9.22, Houghton & Carpenter, 4th ed.
  return air_density * pow(angular_rate / (2.0 * M_PI), 3.0) *
         pow(diameter_, 5.0) * LookupPowerCoeff(angular_rate, freestream_vel);
}

// kInterpOptionSaturate clamps the range of the input values for
// rotor angular rate and freestream velocity to values defined in the
// database.
//
// Negative freestream velocities are considered if using the 3D rotor
// aerodynamic database. Corrections are applied in
// sim/models/actuators/rotor.cc.
double RotorDatabase::LookupThrustCoeff(double angular_rate,
                                        double freestream_vel) const {
  return gsl_interp2(angular_rates_, freestream_vels_, thrust_coeffs_,
                     angular_rate, freestream_vel, kInterpOptionSaturate);
}

double RotorDatabase::LookupTorqueCoeff(double angular_rate,
                                        double freestream_vel) const {
  // See Eq. 9.22b, Houghton & Carpenter, 4th ed.
  return LookupPowerCoeff(angular_rate, freestream_vel) / (2.0 * M_PI);
}

double RotorDatabase::LookupPowerCoeff(double angular_rate,
                                       double freestream_vel) const {
  return gsl_interp2(angular_rates_, freestream_vels_, power_coeffs_,
                     angular_rate, freestream_vel, kInterpOptionSaturate);
}

bool RotorDatabase::IsValid() const {
  for (uint32_t i = 1U; i < angular_rates_->size; ++i) {
    // Check that the thrust coefficient is positive near zero
    // freestream velocity.
    if (gsl_matrix_get(thrust_coeffs_, i, 0U) < 0.0) return false;

    // Check that the power coefficient is negative near zero
    // freestream velocity.
    if (gsl_matrix_get(power_coeffs_, i, 0U) > 0.0) return false;
  }

  return true;
}

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

#include "sim/physics/rotor_database_3d.h"

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

RotorDatabase3d::RotorDatabase3d(const std::string &filename)
    : radius_(),
      radius_pow4_(),
      radius_pow5_(),
      spin_(),
      angular_rates_(),
      axial_vels_(),
      edgewise_vels_(),
      z_prime_force_coeffs_(),
      y_prime_force_coeffs_(),
      z_prime_mom_coeffs_(),
      y_prime_mom_coeffs_(),
      thrust_coeffs_(),
      torque_coeffs_() {
  json_t *rotor_3d_json = json_load::LoadFileOrDie(filename.c_str());

  int32_t num_angular_rates =
      json_load::LoadInt32OrDie(rotor_3d_json, "num_omegas");
  int32_t num_axial_vels =
      json_load::LoadInt32OrDie(rotor_3d_json, "num_v_axials");
  int32_t num_edgewise_vels =
      json_load::LoadInt32OrDie(rotor_3d_json, "num_v_edges");

  radius_ = json_load::LoadDoubleOrDie(rotor_3d_json, "radius");
  radius_pow4_ = pow(radius_, 4.0);
  radius_pow5_ = pow(radius_, 5.0);

  // Spin direction (about thrust axis) the tables were created for.
  // +1 = right-handed, -1 = left-handed.
  spin_ = json_load::LoadInt32OrDie(rotor_3d_json, "spin");

  json_load::JsonArrayLoader loader(num_angular_rates, num_axial_vels,
                                    rotor_3d_json);

  angular_rates_ = gsl_vector_alloc(num_angular_rates);
  axial_vels_ = gsl_vector_alloc(num_axial_vels);
  edgewise_vels_ = gsl_vector_alloc(num_edgewise_vels);

  loader.LoadVector("omegas", angular_rates_);
  loader.LoadVector("v_axials", axial_vels_);
  loader.LoadVector("v_edges", edgewise_vels_);

  z_prime_force_coeffs_ =
      gsl_vector_alloc(num_angular_rates * num_axial_vels * num_edgewise_vels);
  y_prime_force_coeffs_ =
      gsl_vector_alloc(num_angular_rates * num_axial_vels * num_edgewise_vels);
  z_prime_mom_coeffs_ =
      gsl_vector_alloc(num_angular_rates * num_axial_vels * num_edgewise_vels);
  y_prime_mom_coeffs_ =
      gsl_vector_alloc(num_angular_rates * num_axial_vels * num_edgewise_vels);
  thrust_coeffs_ =
      gsl_vector_alloc(num_angular_rates * num_axial_vels * num_edgewise_vels);
  torque_coeffs_ =
      gsl_vector_alloc(num_angular_rates * num_axial_vels * num_edgewise_vels);

  loader.LoadVector("cfx", thrust_coeffs_);
  loader.LoadVector("cfy", y_prime_force_coeffs_);
  loader.LoadVector("cfz", z_prime_force_coeffs_);
  loader.LoadVector("cmx", torque_coeffs_);
  loader.LoadVector("cmy", y_prime_mom_coeffs_);
  loader.LoadVector("cmz", z_prime_mom_coeffs_);

  json_decref(rotor_3d_json);
}

RotorDatabase3d::~RotorDatabase3d() {
  gsl_vector_free(angular_rates_);
  gsl_vector_free(axial_vels_);
  gsl_vector_free(edgewise_vels_);
  gsl_vector_free(z_prime_force_coeffs_);
  gsl_vector_free(y_prime_force_coeffs_);
  gsl_vector_free(z_prime_mom_coeffs_);
  gsl_vector_free(y_prime_mom_coeffs_);
  gsl_vector_free(thrust_coeffs_);
  gsl_vector_free(torque_coeffs_);
}

double RotorDatabase3d::CalcForceZPrime(double angular_rate, double axial_vel,
                                        double edgewise_vel,
                                        double air_density) const {
  return air_density * M_PI * Square(angular_rate) * radius_pow4_ *
         LookupZForceCoeff(angular_rate, axial_vel, edgewise_vel);
}

double RotorDatabase3d::CalcForceYPrime(double angular_rate, double axial_vel,
                                        double edgewise_vel, double air_density,
                                        int dir) const {
  return air_density * M_PI * Square(angular_rate) * radius_pow4_ *
         LookupYForceCoeff(angular_rate, axial_vel, edgewise_vel) *
         (spin_ * dir);
}

double RotorDatabase3d::CalcMomZPrime(double angular_rate, double axial_vel,
                                      double edgewise_vel, double air_density,
                                      int dir) const {
  // The sign is flipped if rotor direction does not match direction the
  // the database was created assuming.
  return air_density * M_PI * Square(angular_rate) * radius_pow5_ *
         LookupZMomCoeff(angular_rate, axial_vel, edgewise_vel) * (spin_ * dir);
}

double RotorDatabase3d::CalcMomYPrime(double angular_rate, double axial_vel,
                                      double edgewise_vel,
                                      double air_density) const {
  return air_density * M_PI * Square(angular_rate) * radius_pow5_ *
         LookupYMomCoeff(angular_rate, axial_vel, edgewise_vel);
}

double RotorDatabase3d::CalcThrust(double angular_rate, double axial_vel,
                                   double edgewise_vel,
                                   double air_density) const {
  return air_density * M_PI * Square(angular_rate) * radius_pow4_ *
         LookupThrustCoeff(angular_rate, axial_vel, edgewise_vel);
}

double RotorDatabase3d::CalcTorque(double angular_rate, double axial_vel,
                                   double edgewise_vel, double air_density,
                                   int dir) const {
  // The sign is flipped if rotor direction does not match direction the
  // the database was created assuming.
  return air_density * M_PI * Square(angular_rate) * radius_pow5_ *
         LookupTorqueCoeff(angular_rate, axial_vel, edgewise_vel) *
         (spin_ * dir);
}

// kInterpOptionSaturate clamps the range of the input values for
// rotor angular rate and freestream velocity to values defined in the
// database.
double RotorDatabase3d::LookupZForceCoeff(double angular_rate, double axial_vel,
                                          double edgewise_vel) const {
  return gsl_interp3(angular_rates_, axial_vels_, edgewise_vels_,
                     z_prime_force_coeffs_, angular_rate, axial_vel,
                     edgewise_vel, kInterpOptionSaturate);
}

double RotorDatabase3d::LookupYForceCoeff(double angular_rate, double axial_vel,
                                          double edgewise_vel) const {
  return gsl_interp3(angular_rates_, axial_vels_, edgewise_vels_,
                     y_prime_force_coeffs_, angular_rate, axial_vel,
                     edgewise_vel, kInterpOptionSaturate);
}

double RotorDatabase3d::LookupZMomCoeff(double angular_rate, double axial_vel,
                                        double edgewise_vel) const {
  return gsl_interp3(angular_rates_, axial_vels_, edgewise_vels_,
                     z_prime_mom_coeffs_, angular_rate, axial_vel, edgewise_vel,
                     kInterpOptionSaturate);
}

double RotorDatabase3d::LookupYMomCoeff(double angular_rate, double axial_vel,
                                        double edgewise_vel) const {
  return gsl_interp3(angular_rates_, axial_vels_, edgewise_vels_,
                     y_prime_mom_coeffs_, angular_rate, axial_vel, edgewise_vel,
                     kInterpOptionSaturate);
}

double RotorDatabase3d::LookupThrustCoeff(double angular_rate, double axial_vel,
                                          double edgewise_vel) const {
  return gsl_interp3(angular_rates_, axial_vels_, edgewise_vels_,
                     thrust_coeffs_, angular_rate, axial_vel, edgewise_vel,
                     kInterpOptionSaturate);
}

double RotorDatabase3d::LookupTorqueCoeff(double angular_rate, double axial_vel,
                                          double edgewise_vel) const {
  return gsl_interp3(angular_rates_, axial_vels_, edgewise_vels_,
                     torque_coeffs_, angular_rate, axial_vel, edgewise_vel,
                     kInterpOptionSaturate);
}

double RotorDatabase3d::CalcThrustScaleFactor(double angular_rate,
                                              double axial_vel,
                                              double edgewise_vel) const {
  double thrust_coeff_zero_edgewise =
      // Saturate the min axial velocity to hover so that the scale factor
      // is also useful for pure descent thrust scaling from the 2D tables.
      LookupThrustCoeff(angular_rate, fmax(axial_vel, 0.1), 0.0);
  double scale_factor;
  // This logic prevents large scale factors when the denominator is small.
  if (fabs(thrust_coeff_zero_edgewise) < 0.0025) {
    scale_factor = 1.0;
  } else {
    scale_factor = (LookupThrustCoeff(angular_rate, axial_vel, edgewise_vel) /
                    thrust_coeff_zero_edgewise);
  }
  return scale_factor;
}

double RotorDatabase3d::CalcTorqueScaleFactor(double angular_rate,
                                              double axial_vel,
                                              double edgewise_vel) const {
  double torque_coeff_zero_edgewise =
      LookupTorqueCoeff(angular_rate, fmax(axial_vel, 0.1), 0.0);
  double scale_factor;
  // This logic prevents large scale factors when the denominator is small.
  if (fabs(torque_coeff_zero_edgewise) < 0.0005) {
    scale_factor = 1.0;
  } else {
    scale_factor = (LookupTorqueCoeff(angular_rate, axial_vel, edgewise_vel) /
                    torque_coeff_zero_edgewise);
  }
  return scale_factor;
}

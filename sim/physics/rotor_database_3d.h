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

#ifndef SIM_PHYSICS_ROTOR_DATABASE_3D_H_
#define SIM_PHYSICS_ROTOR_DATABASE_3D_H_

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include <string>

#include "common/macros.h"

class RotorDatabase3d {
 public:
  explicit RotorDatabase3d(const std::string &filename);
  virtual ~RotorDatabase3d();

  // The rotor force prime frame is attached to an isolated rotor, and is
  // defined as follows:
  // X_prime: The rotor shaft axis, positive in the thrusting direction.
  // Y_prime: The in-plane axis in the Z_prime cross X_prime direction.
  // Z_prime: The in-plane axis aligned with the apparent edgewise wind
  //          direction. Positive is into the wind.

  // Z_prime force [N] is a rotor in-plane force [N].
  double CalcForceZPrime(double angular_rate, double axial_vel,
                         double edgewise_vel, double air_density) const;

  // Y_prime force [N] is the rotor in-plane side force [N].
  // The sign is flipped if rotor direction does not match direction the
  // the database was created assuming.
  double CalcForceYPrime(double angular_rate, double axial_vel,
                         double edgewise_vel, double air_density,
                         int dir) const;

  // Roll moment [N-m] is the right-handed moment about the Z_prime force axis.
  double CalcMomZPrime(double angular_rate, double axial_vel,
                       double edgewise_vel, double air_density, int dir) const;

  // Pitch moment [N-m] is the right-handed moment about the Y_prime axis:
  // positive is pitch up/back.
  double CalcMomYPrime(double angular_rate, double axial_vel,
                       double edgewise_vel, double air_density) const;

  // Thrust [N] is positive for propulsive/motoring, and is equivalent
  // to the rotor X or X_prime axis force.
  double CalcThrust(double angular_rate, double axial_vel, double edgewise_vel,
                    double air_density) const;

  // Torque [N-m] is aerodynamic moment about thrust axis (opposes rotation
  // direction for positive thrusts).
  double CalcTorque(double angular_rate, double axial_vel, double edgewise_vel,
                    double air_density, int dir) const;

  double LookupZForceCoeff(double angular_rate, double axial_vel,
                           double edgewise_vel) const;
  double LookupYForceCoeff(double angular_rate, double axial_vel,
                           double edgewise_vel) const;
  double LookupZMomCoeff(double angular_rate, double axial_vel,
                         double edgewise_vel) const;
  double LookupYMomCoeff(double angular_rate, double axial_vel,
                         double edgewise_vel) const;
  double LookupThrustCoeff(double angular_rate, double axial_vel,
                           double edgewise_vel) const;
  double LookupTorqueCoeff(double angular_rate, double axial_vel,
                           double edgewise_vel) const;
  double CalcThrustScaleFactor(double angular_rate, double axial_vel,
                               double edgewise_vel) const;
  double CalcTorqueScaleFactor(double angular_rate, double axial_vel,
                               double edgewise_vel) const;

 private:
  double radius_;
  double radius_pow4_;
  double radius_pow5_;
  int32_t spin_;

  gsl_vector *angular_rates_;
  gsl_vector *axial_vels_;
  gsl_vector *edgewise_vels_;

  gsl_vector *z_prime_force_coeffs_;
  gsl_vector *y_prime_force_coeffs_;
  gsl_vector *z_prime_mom_coeffs_;
  gsl_vector *y_prime_mom_coeffs_;
  gsl_vector *thrust_coeffs_;
  gsl_vector *torque_coeffs_;

  DISALLOW_COPY_AND_ASSIGN(RotorDatabase3d);
};

#endif  // SIM_PHYSICS_ROTOR_DATABASE_3D_H_

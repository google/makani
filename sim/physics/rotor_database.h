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

#ifndef SIM_PHYSICS_ROTOR_DATABASE_H_
#define SIM_PHYSICS_ROTOR_DATABASE_H_

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include <string>

#include "common/macros.h"

class RotorDatabase {
 public:
  explicit RotorDatabase(const std::string &filename);
  virtual ~RotorDatabase();

  // Calculates the rotor's thrust [N], applied torque [N-m], or
  // generated power [W] based on the angular rate, freestream
  // velocity, and air density.  Torque and power are defined to be
  // negative during thrusting.
  double CalcThrust(double angular_rate, double freestream_vel,
                    double air_density) const;
  double CalcTorque(double angular_rate, double freestream_vel,
                    double air_density) const;
  double CalcPower(double angular_rate, double freestream_vel,
                   double air_density) const;

  // Looks-up the value of the rotor's thrust, torque, or power
  // coefficients [#] at a given angular rate and freestream velocity
  // in a 2-D look-up table.  Values are interpolated linearly between
  // points and the inputs are saturated to the limits of the table.
  double LookupThrustCoeff(double angular_rate, double freestream_vel) const;
  double LookupTorqueCoeff(double angular_rate, double freestream_vel) const;
  double LookupPowerCoeff(double angular_rate, double freestream_vel) const;

 private:
  // Returns true if the rotor database follows our sign conventions:
  //
  // - Power coefficient decreases with increasing angular rate for
  //   the low freestream velocity case, i.e. power is positive in
  //   generation.
  //
  // - Thrust coefficient increases with increasing angular rate for
  //   the low freestream velocity case.
  bool IsValid() const;

  // Diameter of the propeller [m].
  double diameter_;

  // Vector of angular rates [rad/s], which define the rows of the
  // look-up table.
  gsl_vector *angular_rates_;

  // Vector of freestream velocities [m/s], which define the columns
  // of the look-up table.
  gsl_vector *freestream_vels_;

  // Matrix of thrust coefficients [#] defined over a tensor grid of
  // freestream velocities and angular rates.  The thrust coefficient
  // is defined as:
  //
  //   thrust_coeff = thrust / (air_density * angular_rate_hz^2 * diameter^4)
  //
  // See Eq. 9.18, Houghton & Carpenter, 4th ed.
  gsl_matrix *thrust_coeffs_;

  // Matrix of power coefficients [#] defined over a tensor grid of
  // freestream velocities and angular rates.  The power coefficient
  // is defined as:
  //
  //   power_coeff = power / (air_density * angular_rate_hz^3 * diameter^5)
  //
  // See Eq. 9.22, Houghton & Carpenter, 4th ed.
  gsl_matrix *power_coeffs_;

  DISALLOW_COPY_AND_ASSIGN(RotorDatabase);
};

#endif  // SIM_PHYSICS_ROTOR_DATABASE_H_

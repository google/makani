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

#ifndef SIM_PHYSICS_MOTORS_H_
#define SIM_PHYSICS_MOTORS_H_

#include "sim/sim_types.h"

namespace sim {

namespace physics {

namespace motors {

struct TorqueLimits {
  TorqueLimits()
      : lower_limit(), lower_constraint(), upper_limit(), upper_constraint() {}
  double lower_limit;
  SimMotorLimit lower_constraint;
  double upper_limit;
  SimMotorLimit upper_constraint;
};

// Calculates upper and lower torque limits based on the voltage,
// rotational velocity, motor parameters, and programmed current
// limits.
TorqueLimits CalcTorqueLimits(double voltage, double rotor_vel,
                              const MotorParams &params);

// Calculate the electrical terminal power for the torque, speed, and voltage
// for the specified motor.
double CalcMotorPower(double voltage, double torque, double rotor_vel,
                      const MotorParams &params);

// Helper function for calculating motor controller loss contribution.
double CalcMotorControllerLoss(double voltage, double peak_phase_current_sq,
                               const MotorParams &params);
}  // namespace motors

}  // namespace physics

}  // namespace sim

#endif  // SIM_PHYSICS_MOTORS_H_

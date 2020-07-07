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

// The PowerSys actuator represents the entire power system including
// the inverter (or battery box) on the ground, tether electrical
// properties, and power electronics on the wing.  It takes as input
// the commanded and measured rotor angular velocities and outputs bus
// currents to each of the individual motors to achieve the commanded
// angular velocities.

#ifndef SIM_MODELS_POWER_SYS_H_
#define SIM_MODELS_POWER_SYS_H_

#include <stdint.h>

#include <array>
#include <functional>
#include <memory>
#include <vector>

#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/rotor.h"
#include "sim/models/model.h"
#include "sim/sim_messages.h"
#include "sim/sim_types.h"
#include "system/labels.h"

class PowerSys : public Model {
 public:
  PowerSys(const std::vector<std::unique_ptr<RotorBase>> &rotors,
           const RotorSensorParams (&rotor_sensor_params)[kNumMotors],
           const PowerSysParams &power_sys_params,
           const PowerSysSimParams &power_sys_sim_params,
           FaultSchedule *faults);
  ~PowerSys() __attribute__((noinline)) {}

  void Publish() const override;

  virtual double motor_torques(MotorLabel i) const = 0;
  double motor_cmds(int32_t i) const {
    return rate_limited_upper_omega_cmds_[i].val();
  }
  double motor_upper_cmds(int32_t i) const {
    return rate_limited_upper_omega_cmds_[i].val();
  }
  double motor_lower_cmds(int32_t i) const {
    return rate_limited_lower_omega_cmds_[i].val();
  }
  double flight_controller_torque_cmds(int32_t i) const {
    return rotors_[i]->TorqueCmd();
  }
  double wing_voltage() const { return wing_voltage_.val(); }
  double tether_current() const { return tether_current_.val(); }
  double motor_current(MotorLabel i) const { return motor_currents_[i].val(); }
  bool motor_connections(MotorLabel i) const {
    return motor_connections_[i].val();
  }

  void set_tether_released(bool released) {
    tether_released_.set_val(released);
  }

 protected:
  double int_motor_vel_errs(MotorLabel i) const {
    return int_motor_vel_errs_[i].val();
  }

  // Power system parameters.

  // Contains the rotor sensor parameters used to interpret the
  // ControllerCommandMessage.
  const RotorSensorParams (&rotor_sensor_params_)[kNumMotors];
  // Contains the voltage and internal resistance of the ground power
  // supply and the resistance of the tether.
  const PowerSysParams &power_sys_params_;
  // Contains the parameters that describe the transfer function of
  // the motor controllers.
  const PowerSysSimParams &power_sys_sim_params_;

  // Connections to other models.

  // Provides the current angular velocities of the rotors.
  std::vector<const RotorBase *> rotors_;
  // Connections to the FaultSchedule.
  std::vector<FaultSchedule::FaultFunc> motor_connections_fault_funcs_;

  // Continuous states.

  // Integrated motor velocity errors [Nm].
  std::vector<ContinuousState<double>> int_motor_vel_errs_;

  // Discrete states.

  // Vector of Booleans describing whether the motor is functioning.
  std::vector<DiscreteState<bool>> motor_connections_;
  // Rate limited angular rate command [rad/s].  The actual motor
  // controllers rate limit the input command to limit spikes in
  // current.
  std::vector<DiscreteState<double>> rate_limited_upper_omega_cmds_;
  std::vector<DiscreteState<double>> rate_limited_lower_omega_cmds_;

  // Derived values.

  // Current [A] passing through the tether.  This is the sum of the
  // individual bus currents.
  State<double> tether_current_;
  // Current [A] passing through each motor.  The motor current is
  // positive when generating.
  std::vector<State<double>> motor_currents_;
  // Whether the tether has been released.
  State<bool> tether_released_;
  // Voltage [V] as measured at the wing.
  State<double> wing_voltage_;

 private:
  void DiscreteStepHelper(double t) override;

  DISALLOW_COPY_AND_ASSIGN(PowerSys);
};

#endif  // SIM_MODELS_POWER_SYS_H_

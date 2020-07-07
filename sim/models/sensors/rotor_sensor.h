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

#ifndef SIM_MODELS_SENSORS_ROTOR_SENSOR_H_
#define SIM_MODELS_SENSORS_ROTOR_SENSOR_H_

#include <stdint.h>

#include <memory>
#include <vector>

#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/rotor.h"
#include "sim/models/power_sys.h"
#include "sim/models/sensors/sensor.h"
#include "sim/models/signals/measurement.h"
#include "sim/sim_messages.h"
#include "sim/sim_types.h"
#include "system/labels.h"

class RotorSensor : public Sensor {
  friend class RotorSensorTest;

 public:
  RotorSensor(const std::vector<std::unique_ptr<RotorBase>> &rotors,
              const RotorSensorParams (&params)[kNumMotors],
              FaultSchedule *faults);
  ~RotorSensor() __attribute__((noinline)) {}

  void UpdateSensorOutputs(SimSensorMessage *sensor_message,
                           TetherUpMessage * /*tether_up*/) const override;
  void Publish() const override;

  // Inputs.
  void set_is_faulted(MotorLabel m, bool v) {
    is_faulted_[static_cast<int32_t>(m)].set_val(v);
  }

  void set_bus_current(MotorLabel m, double v) {
    bus_currents_[static_cast<int32_t>(m)].set_val(v);
  }

 private:
  void DiscreteStepHelper(double t) override;

  double rotor_speeds(int32_t i) const { return rotor_speeds_[i].recorded(); }
  double omega_upper_limits(int32_t i) const {
    return omega_upper_limits_[i].recorded();
  }
  double omega_lower_limits(int32_t i) const {
    return omega_lower_limits_[i].recorded();
  }
  double torque_cmds(int32_t i) const { return torque_cmds_[i].recorded(); }
  double winding_temps(int32_t i) const { return winding_temps_[i].recorded(); }
  double board_temps(int32_t i) const { return board_temps_[i].recorded(); }
  double cap_temps(int32_t i) const { return cap_temps_[i].recorded(); }
  double core_temps(int32_t i) const { return core_temps_[i].recorded(); }
  double plate1_temps(int32_t i) const { return plate1_temps_[i].recorded(); }
  double plate2_temps(int32_t i) const { return plate2_temps_[i].recorded(); }
  double bus_voltages(int32_t i) const { return bus_voltages_[i].recorded(); }

  bool is_faulted(MotorLabel m) const {
    return is_faulted_[static_cast<int32_t>(m)].val();
  }
  double bus_currents(int32_t i) const { return bus_currents_[i].val(); }

  // Array of rotor sensor parameters.
  const RotorSensorParams (&rotor_sensor_params_)[kNumMotors];

  // Connections to other models.

  // Provides the current angular velocity of all the rotors.
  std::vector<const RotorBase *> rotors_;

  // Inputs.

  // Vector of Booleans that indicates whether a motor has faulted.
  std::vector<State<bool>> is_faulted_;

  // Motor bus current.
  std::vector<State<double>> bus_currents_;

  // Discrete state.

  // Vector of angular velocities [rad/s] of the rotors.
  std::vector<DiscreteState<double>> actual_rotor_speeds_;

  // Vectors of commands [rad/s] of the rotors.
  std::vector<DiscreteState<double>> actual_omega_upper_limits_;
  std::vector<DiscreteState<double>> actual_omega_lower_limits_;
  std::vector<DiscreteState<double>> actual_torque_cmds_;

  // Vector of temperatures [C] of the rotor windings.
  std::vector<DiscreteState<double>> actual_winding_temps_;

  // Vector of temperatures [C] of the motor controller boards.
  std::vector<DiscreteState<double>> actual_board_temps_;

  // Vector of temperatures [C] of the motor capacitors.
  std::vector<DiscreteState<double>> actual_cap_temps_;

  // Vector of temperatures [C] of the motor stator core.
  std::vector<DiscreteState<double>> actual_core_temps_;

  // Vector of temperatures [C] of the motor heat plates.
  std::vector<DiscreteState<double>> actual_plate1_temps_;
  std::vector<DiscreteState<double>> actual_plate2_temps_;

  // Vector of motor bus voltages.
  std::vector<DiscreteState<double>> actual_bus_voltages_;

  // Sub-models.
  std::vector<Measurement<double>> rotor_speeds_;
  std::vector<Measurement<double>> omega_upper_limits_;
  std::vector<Measurement<double>> omega_lower_limits_;
  std::vector<Measurement<double>> torque_cmds_;
  std::vector<Measurement<double>> winding_temps_;
  std::vector<Measurement<double>> board_temps_;
  std::vector<Measurement<double>> cap_temps_;
  std::vector<Measurement<double>> core_temps_;
  std::vector<Measurement<double>> plate1_temps_;
  std::vector<Measurement<double>> plate2_temps_;
  std::vector<Measurement<double>> bus_voltages_;

  DISALLOW_COPY_AND_ASSIGN(RotorSensor);
};

#endif  // SIM_MODELS_SENSORS_ROTOR_SENSOR_H_

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

#ifndef SIM_MODELS_SENSORS_WINCH_SENSOR_H_
#define SIM_MODELS_SENSORS_WINCH_SENSOR_H_

#include "common/macros.h"
#include "control/system_types.h"
#include "sim/models/sensors/sensor.h"
#include "sim/sim_messages.h"

class WinchSensor : public Sensor {
  friend class WinchSensorTest;

 public:
  explicit WinchSensor(const WinchParams &winch_params);
  ~WinchSensor() {}

  void UpdateSensorOutputs(SimSensorMessage * /*sensor_message*/,
                           TetherUpMessage *tether_up) const override;

  void set_winch_torque(double val) { winch_torque_.set_val(val); }
  void set_theta_winch(double val) { theta_winch_.set_val(val); }
  void set_omega_winch(double val) { omega_winch_.set_val(val); }

 private:
  void DiscreteStepHelper(double t) override;

  double winch_torque() const { return winch_torque_.val(); }
  double theta_winch() const { return theta_winch_.val(); }
  double omega_winch() const { return omega_winch_.val(); }

  double control_torque() const { return control_torque_.val(); }
  double omega_winch_servo() const { return omega_winch_servo_.val(); }
  double theta_winch_servo() const { return theta_winch_servo_.val(); }

  // Winch parameters.
  const WinchParams &winch_params_;

  // Input states.
  State<double> winch_torque_;
  State<double> theta_winch_;
  State<double> omega_winch_;

  // Discrete states.

  // Torque [N-m] from the winch applied to the winch drum.  This is
  // the torque on the drum itself after the transmission ratio.
  DiscreteState<double> control_torque_;

  // Angular rate [rad/s] of the winch servo itself before the
  // transmission ratio.
  DiscreteState<double> omega_winch_servo_;

  // Angle [rad] of the winch servo itself before the transmission
  // ratio.
  DiscreteState<double> theta_winch_servo_;

  DISALLOW_COPY_AND_ASSIGN(WinchSensor);
};

#endif  // SIM_MODELS_SENSORS_WINCH_SENSOR_H_

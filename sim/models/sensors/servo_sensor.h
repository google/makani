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

#ifndef SIM_MODELS_SENSORS_SERVO_SENSOR_H_
#define SIM_MODELS_SENSORS_SERVO_SENSOR_H_

#include <stdint.h>

#include <memory>
#include <vector>

#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/servo.h"
#include "sim/models/sensors/sensor.h"
#include "sim/models/signals/measurement.h"
#include "sim/sim_messages.h"
#include "sim/sim_types.h"

class ServoSensor : public Sensor {
 public:
  ServoSensor(const std::vector<std::unique_ptr<ServoBase>> &servos,
              const ServoParams *servo_params,
              const ServoSimParams *servo_sim_params, FaultSchedule *faults);
  ~ServoSensor() __attribute__((noinline)) {}

  void UpdateSensorOutputs(SimSensorMessage *sensor_message,
                           TetherUpMessage * /*tether_up*/) const override;
  void Publish() const override;

 private:
  void DiscreteStepHelper(double t) override;

  double shaft_angles(int32_t i) const { return shaft_angles_[i].recorded(); }
  double shaft_angular_vels(int32_t i) const {
    return shaft_angular_vels_[i].val();
  }
  double external_shaft_torques(int32_t i) const {
    return external_shaft_torques_[i].val();
  }
  double motor_powers(int32_t i) const { return motor_powers_[i].val(); }

  // Servo sensor parameters.
  const ServoParams *servo_params_;
  const ServoSimParams *servo_sim_params_;

  // Connections to other models.
  std::vector<const ServoBase *> servos_;

  // Discrete state.
  std::vector<DiscreteState<double>> actual_shaft_angles_;
  std::vector<DiscreteState<double>> shaft_angular_vels_;
  std::vector<DiscreteState<double>> external_shaft_torques_;
  std::vector<DiscreteState<double>> motor_powers_;

  // Sub-models.
  std::vector<Measurement<double>> shaft_angles_;

  DISALLOW_COPY_AND_ASSIGN(ServoSensor);
};

#endif  // SIM_MODELS_SENSORS_SERVO_SENSOR_H_

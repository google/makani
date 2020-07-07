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

#include "sim/models/sensors/servo_sensor.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

#include <memory>
#include <vector>

#include "common/c_math/util.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/signals/measurement.h"
#include "sim/sim_messages.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"
#include "system/labels.h"

ServoSensor::ServoSensor(const std::vector<std::unique_ptr<ServoBase>> &servos,
                         const ServoParams *servo_params,
                         const ServoSimParams *servo_sim_params,
                         FaultSchedule *faults)
    : Sensor("ServoSensor"),
      servo_params_(servo_params),
      servo_sim_params_(servo_sim_params),
      servos_(),
      actual_shaft_angles_(),
      shaft_angular_vels_(),
      external_shaft_torques_(),
      motor_powers_(),
      shaft_angles_() {
  std::vector<Model *> sub_models__;

  actual_shaft_angles_.reserve(kNumServos);
  shaft_angular_vels_.reserve(kNumServos);
  external_shaft_torques_.reserve(kNumServos);
  motor_powers_.reserve(kNumServos);
  shaft_angles_.reserve(kNumServos);
  for (int32_t i = 0; i < kNumServos; ++i) {
    servos_.push_back(servos[i].get());
    actual_shaft_angles_.emplace_back(
        new_discrete_state(), "actual_shaft_angles[" + std::to_string(i) + "]",
        0.0);
    shaft_angular_vels_.emplace_back(
        new_discrete_state(), "shaft_angular_vels[" + std::to_string(i) + "]",
        0.0);
    external_shaft_torques_.emplace_back(
        new_discrete_state(),
        "external_shaft_torques[" + std::to_string(i) + "]", 0.0);
    motor_powers_.emplace_back(new_discrete_state(),
                               "motor_powers[" + std::to_string(i) + "]", 0.0);
    shaft_angles_.emplace_back(full_name(),
                               "shaft_angles[" + std::to_string(i) + "]",
                               actual_shaft_angles_[i], faults);
    sub_models__.push_back(&shaft_angles_[i]);
  }
  set_sub_models(sub_models__);

  SetupDone();
}

void ServoSensor::DiscreteStepHelper(double t) {
  for (int32_t i = 0; i < kNumServos; ++i) {
    actual_shaft_angles_[i].DiscreteUpdate(t, servos_[i]->shaft_angle());
    shaft_angular_vels_[i].DiscreteUpdate(t, servos_[i]->shaft_angular_vel());
    external_shaft_torques_[i].DiscreteUpdate(
        t, servos_[i]->external_shaft_torque());
    motor_powers_[i].DiscreteUpdate(t, servos_[i]->motor_power());
  }
}

void ServoSensor::UpdateSensorOutputs(SimSensorMessage *sensor_message,
                                      TetherUpMessage * /*tether_up*/) const {
  for (int32_t i = 0; i < kNumServos; ++i) {
    sensor_message->control_input_messages_updated.servo_statuses[i] = true;
    ServoDebugMessage *servo_status =
        &sensor_message->control_input_messages.servo_statuses[i];
    memset(servo_status, 0, sizeof(*servo_status));

    servo_status->angle_estimate = static_cast<float>(shaft_angles(i));

    servo_status->angular_velocity = static_cast<float>(shaft_angular_vels(i));

    // Note:  The remainder of the servo status fields are faked so that
    // monitors display what appear to be valid data during rehearsals.
    servo_status->r22.current = 7;
    servo_status->r22.temperature = 35.0f;
    servo_status->flags.status = kServoStatusArmed;
    servo_status->state = kActuatorStateArmed;

    // This should really be 0b11111.
    servo_status->servo_mon.analog_populated = 127;

    servo_status->servo_mon.analog_data[kServoAnalogVoltageLvB] = 66.2f;
    servo_status->servo_mon.analog_data[kServoAnalogVoltageLvA] = 74.1f;
  }
}

void ServoSensor::Publish() const {
  sim_telem.servo_sensor.num_servos = kNumServos;
  for (int32_t i = 0; i < kNumServos; ++i) {
    sim_telem.servo_sensor.shaft_angles[i] = shaft_angles(i);
    sim_telem.servo_sensor.shaft_angular_vels[i] = shaft_angular_vels(i);
    sim_telem.servo_sensor.external_shaft_torques[i] =
        external_shaft_torques(i);
    sim_telem.servo_sensor.motor_powers[i] = motor_powers(i);
  }
}

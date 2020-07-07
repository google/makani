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

#include "sim/models/sensors/rotor_sensor.h"

#include <math.h>
#include <stdint.h>

#include <memory>
#include <vector>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/motor_thermal_types.h"
#include "avionics/motor/firmware/flags.h"
#include "common/c_math/util.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/rotor.h"
#include "sim/models/power_sys.h"
#include "sim/models/signals/measurement.h"
#include "sim/sim_messages.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"
#include "system/labels.h"

RotorSensor::RotorSensor(const std::vector<std::unique_ptr<RotorBase>> &rotors,
                         const RotorSensorParams (&params)[kNumMotors],
                         FaultSchedule *faults)
    : Sensor("RotorSensor"),
      rotor_sensor_params_(params),
      rotors_(),
      is_faulted_(),
      bus_currents_(),
      actual_rotor_speeds_(),
      actual_omega_upper_limits_(),
      actual_omega_lower_limits_(),
      actual_torque_cmds_(),
      actual_winding_temps_(),
      actual_board_temps_(),
      actual_cap_temps_(),
      actual_core_temps_(),
      actual_plate1_temps_(),
      actual_plate2_temps_(),
      actual_bus_voltages_(),
      rotor_speeds_(),
      omega_upper_limits_(),
      omega_lower_limits_(),
      torque_cmds_(),
      winding_temps_(),
      board_temps_(),
      cap_temps_(),
      core_temps_(),
      plate1_temps_(),
      plate2_temps_(),
      bus_voltages_() {
  std::vector<Model *> sub_models__;

  actual_rotor_speeds_.reserve(kNumMotors);
  actual_omega_upper_limits_.reserve(kNumMotors);
  actual_omega_lower_limits_.reserve(kNumMotors);
  actual_torque_cmds_.reserve(kNumMotors);
  actual_winding_temps_.reserve(kNumMotors);
  actual_board_temps_.reserve(kNumMotors);
  actual_cap_temps_.reserve(kNumMotors);
  actual_core_temps_.reserve(kNumMotors);
  actual_plate1_temps_.reserve(kNumMotors);
  actual_plate2_temps_.reserve(kNumMotors);
  actual_bus_voltages_.reserve(kNumMotors);
  is_faulted_.reserve(kNumMotors);
  bus_currents_.reserve(kNumMotors);
  rotor_speeds_.reserve(kNumMotors);
  omega_upper_limits_.reserve(kNumMotors);
  omega_lower_limits_.reserve(kNumMotors);
  torque_cmds_.reserve(kNumMotors);
  winding_temps_.reserve(kNumMotors);
  board_temps_.reserve(kNumMotors);
  cap_temps_.reserve(kNumMotors);
  core_temps_.reserve(kNumMotors);
  plate1_temps_.reserve(kNumMotors);
  plate2_temps_.reserve(kNumMotors);
  bus_voltages_.reserve(kNumMotors);
  for (int32_t i = 0; i < kNumMotors; ++i) {
    rotors_.push_back(rotors[i].get());

    actual_rotor_speeds_.emplace_back(
        new_discrete_state(), "actual_rotor_speeds[" + std::to_string(i) + "]",
        0.0);
    actual_omega_upper_limits_.emplace_back(
        new_discrete_state(),
        "actual_omega_upper_limits[" + std::to_string(i) + "]", 0.0);
    actual_omega_lower_limits_.emplace_back(
        new_discrete_state(),
        "actual_omega_lower_limits[" + std::to_string(i) + "]", 0.0);
    actual_torque_cmds_.emplace_back(
        new_discrete_state(), "actual_torque_cmds[" + std::to_string(i) + "]",
        0.0);
    actual_winding_temps_.emplace_back(
        new_discrete_state(), "actual_winding_temps[" + std::to_string(i) + "]",
        0.0);
    actual_board_temps_.emplace_back(
        new_discrete_state(), "actual_board_temps[" + std::to_string(i) + "]",
        0.0);
    actual_cap_temps_.emplace_back(
        new_discrete_state(), "actual_cap_temps[" + std::to_string(i) + "]",
        0.0);
    actual_core_temps_.emplace_back(
        new_discrete_state(), "actual_core_temps[" + std::to_string(i) + "]",
        0.0);
    actual_plate1_temps_.emplace_back(
        new_discrete_state(), "actual_plate1_temps[" + std::to_string(i) + "]",
        0.0);
    actual_plate2_temps_.emplace_back(
        new_discrete_state(), "actual_plate2_temps[" + std::to_string(i) + "]",
        0.0);
    actual_bus_voltages_.emplace_back(
        new_discrete_state(), "actual_bus_voltages[" + std::to_string(i) + "]",
        0.0);
    is_faulted_.emplace_back(new_derived_value(),
                             "is_faulted[" + std::to_string(i) + "]", false);
    bus_currents_.emplace_back(
        new_derived_value(), "bus_currents[" + std::to_string(i) + "]", false);

    rotor_speeds_.emplace_back(full_name(),
                               "rotor_speeds[" + std::to_string(i) + "]",
                               actual_rotor_speeds_[i], faults);
    omega_upper_limits_.emplace_back(
        full_name(), "omega_upper_limits[" + std::to_string(i) + "]",
        actual_omega_upper_limits_[i], faults);
    omega_lower_limits_.emplace_back(
        full_name(), "omega_lower_limits[" + std::to_string(i) + "]",
        actual_omega_lower_limits_[i], faults);
    torque_cmds_.emplace_back(full_name(),
                              "torque_cmds[" + std::to_string(i) + "]",
                              actual_torque_cmds_[i], faults);
    winding_temps_.emplace_back(full_name(),
                                "winding_temps[" + std::to_string(i) + "]",
                                actual_winding_temps_[i], faults);
    board_temps_.emplace_back(full_name(),
                              "board_temps[" + std::to_string(i) + "]",
                              actual_board_temps_[i], faults);
    cap_temps_.emplace_back(full_name(), "cap_temps[" + std::to_string(i) + "]",
                            actual_cap_temps_[i], faults);
    core_temps_.emplace_back(full_name(),
                             "core_temps[" + std::to_string(i) + "]",
                             actual_core_temps_[i], faults);
    plate1_temps_.emplace_back(full_name(),
                               "plate1_temps[" + std::to_string(i) + "]",
                               actual_plate1_temps_[i], faults);
    plate2_temps_.emplace_back(full_name(),
                               "plate2_temps[" + std::to_string(i) + "]",
                               actual_plate2_temps_[i], faults);
    bus_voltages_.emplace_back(full_name(),
                               "bus_voltages[" + std::to_string(i) + "]",
                               actual_bus_voltages_[i], faults);

    sub_models__.push_back(&rotor_speeds_[i]);
    sub_models__.push_back(&omega_upper_limits_[i]);
    sub_models__.push_back(&omega_lower_limits_[i]);
    sub_models__.push_back(&torque_cmds_[i]);
    sub_models__.push_back(&winding_temps_[i]);
    sub_models__.push_back(&board_temps_[i]);
    sub_models__.push_back(&cap_temps_[i]);
    sub_models__.push_back(&core_temps_[i]);
    sub_models__.push_back(&plate1_temps_[i]);
    sub_models__.push_back(&plate2_temps_[i]);
    sub_models__.push_back(&bus_voltages_[i]);
  }
  set_sub_models(sub_models__);

  SetupDone();
}

void RotorSensor::DiscreteStepHelper(double t) {
  double rotor_speed__, omega_upper_limit__, omega_lower_limit__;
  double torque_cmd__, winding_temp__, board_temp__, bus_voltage__;
  double cap_temp__, core_temp__, plate1_temp__, plate2_temp__;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    rotor_speed__ = rotors_[i]->omega();
    omega_upper_limit__ = rotors_[i]->omega_upper_cmd();
    omega_lower_limit__ = rotors_[i]->omega_lower_cmd();
    torque_cmd__ = rotors_[i]->torque_cmd();

    // Note:  Additional motor data are faked so that monitors
    // display what appear to be valid data during rehearsals.
    winding_temp__ = 50.0 + fabs(rotors_[i]->omega()) / 100.0;
    board_temp__ = 70.0 + fabs(rotors_[i]->omega()) / 100.0;
    cap_temp__ = 55.0 + fabs(rotors_[i]->omega()) / 100.0;
    core_temp__ = 38.0 + fabs(rotors_[i]->omega()) / 10.0;
    plate1_temp__ = 37.0 + fabs(rotors_[i]->omega()) / 10.0;
    plate2_temp__ = 23.0 + fabs(rotors_[i]->omega()) / 10.0;
    bus_voltage__ = 850.0 + fabs(rotors_[i]->omega()) / 10.0;

    actual_rotor_speeds_[i].DiscreteUpdate(t, rotor_speed__);
    actual_omega_upper_limits_[i].DiscreteUpdate(t, omega_upper_limit__);
    actual_omega_lower_limits_[i].DiscreteUpdate(t, omega_lower_limit__);
    actual_torque_cmds_[i].DiscreteUpdate(t, torque_cmd__);
    actual_winding_temps_[i].DiscreteUpdate(t, winding_temp__);
    actual_board_temps_[i].DiscreteUpdate(t, board_temp__);
    actual_cap_temps_[i].DiscreteUpdate(t, cap_temp__);
    actual_core_temps_[i].DiscreteUpdate(t, core_temp__);
    actual_plate1_temps_[i].DiscreteUpdate(t, plate1_temp__);
    actual_plate2_temps_[i].DiscreteUpdate(t, plate2_temp__);
    actual_bus_voltages_[i].DiscreteUpdate(t, bus_voltage__);
  }
}

void RotorSensor::UpdateSensorOutputs(SimSensorMessage *sensor_message,
                                      TetherUpMessage * /*tether_up*/) const {
  for (int32_t i = 0; i < kNumMotors; ++i) {
    sensor_message->control_input_messages_updated.motor_statuses[i] = true;
    MotorStatusMessage *motor_status =
        &sensor_message->control_input_messages.motor_statuses[i];
    memset(motor_status, 0, sizeof(*motor_status));

    motor_status->state = static_cast<ActuatorState>(
        is_faulted(static_cast<MotorLabel>(i)) ? kActuatorStateError
                                               : kActuatorStateRunning);
    motor_status->bus_current = static_cast<float>(bus_currents(i));
    // TODO: Consider piping in actual motor_state(i) if in HITL.
    motor_status->motor_status = static_cast<uint16_t>(
        is_faulted(static_cast<MotorLabel>(i)) ? kMotorStatusError
                                               : kMotorStatusRunning);
    motor_status->bus_voltage = static_cast<float>(bus_voltages(i));
    motor_status->temps[kMotorThermalChannelStatorCoil] =
        static_cast<float>(winding_temps(i));
    motor_status->temps[kMotorThermalChannelStatorCore] =
        static_cast<float>(core_temps(i));
    motor_status->temps[kMotorThermalChannelHeatPlate1] =
        static_cast<float>(plate1_temps(i));
    motor_status->temps[kMotorThermalChannelHeatPlate2] =
        static_cast<float>(plate2_temps(i));
    motor_status->temps[kMotorThermalChannelCapacitor] =
        static_cast<float>(cap_temps(i));
    motor_status->temps[kMotorThermalChannelBoard] =
        static_cast<float>(board_temps(i));
    motor_status->omega = static_cast<float>(
        InvertCal(rotor_speeds(i), &rotor_sensor_params_[i].omega_cal));

    // Pull the commands back from the rotor objects.
    motor_status->omega_upper_limit = static_cast<float>(
        InvertCal(omega_upper_limits(i), &rotor_sensor_params_[i].omega_cal));
    motor_status->omega_lower_limit = static_cast<float>(
        InvertCal(omega_lower_limits(i), &rotor_sensor_params_[i].omega_cal));
    motor_status->torque_cmd = static_cast<float>(
        InvertCal(torque_cmds(i), &rotor_sensor_params_[i].torque_cal));
  }
}

void RotorSensor::Publish() const {
  sim_telem.rotor_sensor.num_rotors = kNumMotors;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    sim_telem.rotor_sensor.rotor_speeds[i] = rotor_speeds(i);
  }
}

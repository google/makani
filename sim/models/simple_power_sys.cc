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

#include "sim/models/simple_power_sys.h"

#include <math.h>
#include <stdint.h>

#include <array>
#include <memory>
#include <vector>

#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/rotor.h"
#include "sim/models/power_sys.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"
#include "system/labels.h"

SimplePowerSys::SimplePowerSys(
    const std::vector<std::unique_ptr<RotorBase>> &rotors,
    const RotorSensorParams (&rotor_sensor_params)[kNumMotors],
    const PowerSysParams &power_sys_params,
    const PowerSysSimParams &power_sys_sim_params, FaultSchedule *faults)
    : PowerSys(rotors, rotor_sensor_params, power_sys_params,
               power_sys_sim_params, faults),
      motor_torques_() {
  motor_torques_.reserve(kNumMotors);
  for (int32_t i = 0; i < kNumMotors; ++i) {
    motor_torques_.emplace_back(
        new_derived_value(), "motor_torques[" + std::to_string(i) + "]", 0.0);
  }

  SetupDone();
}

void SimplePowerSys::UpdateDerivedStates() {
  double wing_electrical_power = 0.0;
  double motor_powers[kNumMotors];
  for (int32_t i = 0; i < static_cast<int32_t>(motor_torques_.size()); ++i) {
    // For simple system, tether_released and faults have same result.
    if (motor_connections_[i].val() && !tether_released_.val()) {
      motor_torques_[i].set_val(
          power_sys_sim_params_.kp_rotor_vel_err *
              (motor_cmds(i) - rotors_[i]->AngularSpeed()) +
          power_sys_sim_params_.ki_rotor_vel_err *
              int_motor_vel_errs_[i].val());
    } else {
      motor_torques_[i].set_val(0.0);
    }

    // Power is defined to be positive during generation.
    motor_powers[i] =
        -motor_torques(static_cast<MotorLabel>(i)) * rotors_[i]->AngularSpeed();
    wing_electrical_power += motor_powers[i];
  }

  tether_current_.set_val(CalcTetherCurrent(wing_electrical_power));
  wing_voltage_.set_val(CalcWingVoltage(tether_current()));

  // Assume a fully parallel harness.
  for (int32_t i = 0; i < kNumMotors; ++i) {
    motor_currents_[i].set_val(motor_powers[i] / fmax(1.0, wing_voltage()));
  }
}

void SimplePowerSys::AddInternalConnections(ConnectionStore *connections) {
  connections->Add(1, [this](double /*t*/) { UpdateDerivedStates(); });
}

// The voltage at the wing is the open circuit voltage source voltage
// (v_source_0) plus the tether current (defined to be positive during
// generation) times the sum of the voltage source's internal
// resistance (R_source) and the tether's resistance (R_tether).
//
//   v_wing = v_source_0 + i_tether * (R_source + R_tether)
//
double SimplePowerSys::CalcWingVoltage(double tether_current__) const {
  return power_sys_params_.v_source_0 +
         tether_current__ *
             (power_sys_params_.R_source + power_sys_params_.R_tether);
}

// The tether current is derived from the electrical power by solving
// the equations:
//
//   v_wing - v_source_0 = i_tether * (R_tether + R_source)
//   P_elec = v_wing * i_tether
//
// Power is defined to be positive during generation.
double SimplePowerSys::CalcTetherCurrent(double wing_electrical_power) const {
  double R_total = power_sys_params_.R_source + power_sys_params_.R_tether;
  double discriminant =
      1.0 +
      4.0 * R_total * wing_electrical_power /
          (power_sys_params_.v_source_0 * power_sys_params_.v_source_0);
  return power_sys_params_.v_source_0 / (2.0 * R_total) *
         (-1.0 + sqrt(fmax(discriminant, 0.0)));
}

void SimplePowerSys::CalcDerivHelper(double /*t*/) {
  for (int32_t i = 0; i < kNumMotors; ++i) {
    double deriv = motor_cmds(i) - rotors_[i]->AngularSpeed();

    // Zero out the derivative if it would move the error magnitude further
    // above its maximum value.
    if (fabs(int_motor_vel_errs_[i].val()) > kMotorVelIntErrorMax &&
        int_motor_vel_errs_[i].val() * deriv > 0.0) {
      deriv = 0.0;
    }

    int_motor_vel_errs_[i].set_deriv(deriv);
  }
}

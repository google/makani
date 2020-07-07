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

#include "sim/models/power_sys.h"

#include <glog/logging.h>
#include <math.h>
#include <stdint.h>

#include <array>
#include <memory>
#include <vector>

#include "common/c_math/filter.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/rotor.h"
#include "sim/models/model.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"
#include "system/labels.h"

PowerSys::PowerSys(const std::vector<std::unique_ptr<RotorBase>> &rotors,
                   const RotorSensorParams (&rotor_sensor_params)[kNumMotors],
                   const PowerSysParams &power_sys_params,
                   const PowerSysSimParams &power_sys_sim_params,
                   FaultSchedule *faults)
    : Model("PowerSys"),
      rotor_sensor_params_(rotor_sensor_params),
      power_sys_params_(power_sys_params),
      power_sys_sim_params_(power_sys_sim_params),
      rotors_(),
      motor_connections_fault_funcs_(),
      int_motor_vel_errs_(),
      motor_connections_(),
      rate_limited_upper_omega_cmds_(),
      rate_limited_lower_omega_cmds_(),
      tether_current_(new_derived_value(), "tether_current", 0.0),
      motor_currents_(),
      tether_released_(new_derived_value(), "tether_released", false),
      wing_voltage_(new_derived_value(), "wing_voltage", 0.0) {
  int_motor_vel_errs_.reserve(kNumMotors);
  motor_connections_.reserve(kNumMotors);
  rate_limited_upper_omega_cmds_.reserve(kNumMotors);
  rate_limited_lower_omega_cmds_.reserve(kNumMotors);
  motor_currents_.reserve(kNumMotors);
  for (int32_t i = 0; i < kNumMotors; ++i) {
    rotors_.push_back(rotors[i].get());
    int_motor_vel_errs_.emplace_back(
        new_continuous_state(), "int_motor_vel_errs[" + std::to_string(i) + "]",
        0.0);
    motor_currents_.emplace_back(new_derived_value(),
                                 "motor_current[" + std::to_string(i) + "]");
    motor_connections_.emplace_back(
        new_discrete_state(), "motor_connections[" + std::to_string(i) + "]",
        0.0, true);
    rate_limited_upper_omega_cmds_.emplace_back(
        new_discrete_state(),
        "rate_limited_upper_omega_cmds[" + std::to_string(i) + "]", 0.0, 0.0);
    rate_limited_lower_omega_cmds_.emplace_back(
        new_discrete_state(),
        "rate_limited_lower_omega_cmds[" + std::to_string(i) + "]", 0.0, 0.0);
    motor_connections_fault_funcs_.push_back(faults->ClaimFaultFunc(
        JoinName(full_name(), motor_connections_[i].name()),
        {{kSimFaultActuatorZero, 0U}}));
  }
}

// Updates simulator telemetry.
void PowerSys::Publish() const {
  sim_telem.power_sys.v_wing = wing_voltage();
  sim_telem.power_sys.i_teth = tether_current();
  sim_telem.power_sys.P_elec = wing_voltage() * tether_current();

  for (int32_t i = 0; i < kNumMotors; ++i) {
    sim_telem.power_sys.int_motor_vel_errs[i] = int_motor_vel_errs_[i].val();
  }
}

void PowerSys::DiscreteStepHelper(double t) {
  for (int32_t i = 0; i < kNumMotors; ++i) {
    bool fault =
        motor_connections_fault_funcs_[i](t, kSimFaultActuatorZero, nullptr);
    motor_connections_[i].DiscreteUpdate(t, !fault);

    // Rate limit the motor command.  This simulates a rate limit
    // imposed in avionics/motor/firmware/motor_stacking.c.
    // Upper
    double rate_limited_state = rate_limited_upper_omega_cmds_[i].val();
    RateLimit(rotors_[i]->AngularUpperSpeedCmd(),
              -power_sys_sim_params_.omega_cmd_rate_limit,
              power_sys_sim_params_.omega_cmd_rate_limit, t - t_z1(),
              &rate_limited_state);
    rate_limited_upper_omega_cmds_[i].DiscreteUpdate(t, rate_limited_state);

    // Lower
    rate_limited_state = rate_limited_lower_omega_cmds_[i].val();
    RateLimit(rotors_[i]->AngularLowerSpeedCmd(),
              -power_sys_sim_params_.omega_cmd_rate_limit,
              power_sys_sim_params_.omega_cmd_rate_limit, t - t_z1(),
              &rate_limited_state);
    rate_limited_lower_omega_cmds_[i].DiscreteUpdate(t, rate_limited_state);
  }
}

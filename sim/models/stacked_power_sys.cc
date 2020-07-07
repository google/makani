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

#include "sim/models/stacked_power_sys.h"

#include <stdint.h>

#include <array>
#include <memory>
#include <vector>

#include "avionics/common/motor_util.h"
#include "common/runfiles_dir.h"
#include "sim/faults/faults.h"
#include "sim/physics/motors.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"
#include "system/labels.h"

StackedPowerSys::StackedPowerSys(
    const std::vector<std::unique_ptr<RotorBase>> &rotors,
    const RotorSensorParams (&rotor_sensor_params)[kNumMotors],
    const PowerSysParams &power_sys_params,
    const PowerSysSimParams &power_sys_sim_params, FaultSchedule *faults)
    : PowerSys(rotors, rotor_sensor_params, power_sys_params,
               power_sys_sim_params, faults),
      num_blocks_(kNumMotors / 2),
      filtered_tether_current_(new_continuous_state(),
                               "filtered_tether_current", 0.0),
      ground_voltage_(new_continuous_state(), "ground_voltage",
                      power_sys_params.v_source_0),
      block_voltages_(),
      speed_correction_(),
      voltage_correction_state_x_(),
      speed_cmd_upper_filter_(),
      speed_cmd_lower_filter_(),
      motor_torque_cmds_(),
      speed_controller_state_(),
      voltage_correction_(),
      torque_limits_(),
      motor_constraints_(),
      speed_controller_int_deriv_(),
      block_powers_() {
  motor_torque_cmds_.reserve(kNumMotors);
  speed_controller_state_.reserve(kNumMotors);
  voltage_correction_.reserve(kNumMotors);
  voltage_correction_state_x_.reserve(kNumMotors);
  torque_limits_.reserve(kNumMotors);
  motor_constraints_.reserve(kNumMotors);
  speed_controller_int_deriv_.reserve(kNumMotors);
  speed_cmd_upper_filter_.reserve(kNumMotors);
  speed_cmd_lower_filter_.reserve(kNumMotors);
  for (int32_t i = 0; i < kNumMotors; ++i) {
    motor_torque_cmds_.emplace_back(
        new_derived_value(), "motor_torque_cmds[" + std::to_string(i) + "]",
        0.0);
    speed_controller_state_.emplace_back(
        new_derived_value(),
        "speed_controller_state[" + std::to_string(i) + "]",
        kSpeedControlTorque);
    voltage_correction_.emplace_back(
        new_derived_value(), "voltage_correction[" + std::to_string(i) + "]",
        0.0);
    voltage_correction_state_x_.emplace_back(
        new_continuous_state(),
        "voltage_correction_state_x[" + std::to_string(i) + "]", 0.0);
    torque_limits_.emplace_back(new_derived_value(),
                                "torque_limits[" + std::to_string(i) + "]");
    motor_constraints_.emplace_back(
        new_derived_value(), "motor_constraints[" + std::to_string(i) + "]",
        kSimMotorLimitNone);
    speed_controller_int_deriv_.emplace_back(
        new_derived_value(),
        "speed_controller_int_deriv[" + std::to_string(i) + "]", 0.0);
    speed_cmd_upper_filter_.emplace_back(
        new_continuous_state(),
        "speed_cmd_upper_filter[" + std::to_string(i) + "]", 0.0);
    speed_cmd_lower_filter_.emplace_back(
        new_continuous_state(),
        "speed_cmd_lower_filter[" + std::to_string(i) + "]", 0.0);
  }

  block_voltages_.reserve(num_blocks_);
  block_powers_.reserve(num_blocks_);
  speed_correction_.reserve(num_blocks_);
  for (int32_t i = 0; i < num_blocks_; ++i) {
    block_voltages_.emplace_back(new_continuous_state(),
                                 "block_voltages[" + std::to_string(i) + "]",
                                 power_sys_params.v_source_0 / num_blocks_);
    block_powers_.emplace_back(new_derived_value(),
                               "block_powers[" + std::to_string(i) + "]", 0.0);
    speed_correction_.emplace_back(
        new_continuous_state(), "speed_correction[" + std::to_string(i) + "]",
        0.0);
  }

  SetupDone();
}

void StackedPowerSys::Publish() const {
  PowerSys::Publish();
  for (int32_t i = 0; i < kNumMotors; ++i) {
    sim_telem.stacked_power_sys.motor_torques[i] =
        motor_torques(static_cast<MotorLabel>(i));
    sim_telem.stacked_power_sys.motor_torque_cmds[i] =
        motor_torque_cmds(static_cast<MotorLabel>(i));
    sim_telem.stacked_power_sys.motor_torque_upper_limits[i] =
        torque_limits_[i].val().upper_limit;
    sim_telem.stacked_power_sys.motor_torque_lower_limits[i] =
        torque_limits_[i].val().lower_limit;
    sim_telem.stacked_power_sys.motor_constraints[i] =
        static_cast<int32_t>(motor_constraints(static_cast<MotorLabel>(i)));
    sim_telem.stacked_power_sys.voltage_correction[i] =
        voltage_correction(static_cast<MotorLabel>(i));
    sim_telem.stacked_power_sys.voltage_correction_state_x[i] =
        voltage_correction_state_x(static_cast<MotorLabel>(i));
  }
  for (int32_t i = 0; i < num_blocks_; ++i) {
    sim_telem.stacked_power_sys.block_voltages[i] = block_voltages(i);
    sim_telem.stacked_power_sys.block_powers[i] = block_powers(i);
    sim_telem.stacked_power_sys.speed_correction[i] = speed_correction(i);
  }
  sim_telem.stacked_power_sys.tether_current = tether_current();
  sim_telem.stacked_power_sys.filtered_tether_current =
      filtered_tether_current();
  sim_telem.stacked_power_sys.ground_voltage = ground_voltage();
}

void StackedPowerSys::UpdateDerivedStates() {
  for (int32_t i = 0; i < kNumMotors; ++i) {
    torque_limits_[i].set_val(sim::physics::motors::CalcTorqueLimits(
        fabs(GetMotorBlockVoltage(static_cast<MotorLabel>(i))),
        rotors_[i]->AngularSpeed(), power_sys_sim_params_.motor));
  }

  wing_voltage_.set_val(CalcWingVoltage());
  tether_current_.set_val(CalcTetherCurrent(wing_voltage()));

  for (int32_t i = 0; i < num_blocks_; ++i) {
    MotorLabel r_bot = static_cast<MotorLabel>(GetBottomMotorIndex(i));
    MotorLabel r_top = static_cast<MotorLabel>(GetTopMotorIndex(i));

    if (IsBlockFaulted(i)) {
      // Torque is zero unless voltage has dropped low enough to load motor.
      TorqueLimits torque_limit_bottom = torque_limits_[r_bot].val();
      double bottom_torque = 0.0;
      SimMotorLimit bottom_constraint = kSimMotorLimitNone;
      if (bottom_torque > torque_limit_bottom.upper_limit) {
        bottom_torque = torque_limit_bottom.upper_limit;
        bottom_constraint = torque_limit_bottom.upper_constraint;
      } else if (bottom_torque < torque_limit_bottom.lower_limit) {
        bottom_torque = torque_limit_bottom.lower_limit;
        bottom_constraint = torque_limit_bottom.lower_constraint;
      }
      motor_torque_cmds_[r_bot].set_val(bottom_torque);
      motor_constraints_[r_bot].set_val(bottom_constraint);
      TorqueLimits torque_limit_top = torque_limits_[r_top].val();
      double top_torque = 0.0;
      SimMotorLimit top_constraint = kSimMotorLimitNone;
      if (top_torque > torque_limit_top.upper_limit) {
        top_torque = torque_limit_top.upper_limit;
        top_constraint = torque_limit_top.upper_constraint;
      } else if (top_torque < torque_limit_top.lower_limit) {
        top_torque = torque_limit_top.lower_limit;
        top_constraint = torque_limit_top.lower_constraint;
      }
      motor_torque_cmds_[r_top].set_val(top_torque);
      motor_constraints_[r_top].set_val(top_constraint);

      block_powers_[i].set_val(0.0);
      motor_currents_[r_bot].set_val(0.0);
      motor_currents_[r_top].set_val(0.0);
      voltage_correction_[r_bot].set_val(0.0);
      voltage_correction_[r_top].set_val(0.0);
      speed_controller_state_[r_bot].set_val(kSpeedControlTorque);
      speed_controller_state_[r_top].set_val(kSpeedControlTorque);
      speed_controller_int_deriv_[r_bot].set_val(0.0);
      speed_controller_int_deriv_[r_top].set_val(0.0);
    } else {
      // Calculate voltage error for each block.
      double voltage_err =
          block_voltages(i) -
          fmin(wing_voltage() / GetNumActiveBlocks(),
               power_sys_sim_params_.voltage_average_upper_sat);
      // Output step for state space implementation of stacking voltage
      // controller.  Translated from stacking.c using s-domain
      // implementation.  Correction state x is the internal state of
      // stacking controller.
      double bottom_voltage_correction =
          power_sys_sim_params_.kp_voltage_err *
          power_sys_sim_params_.voltage_control_pole /
          power_sys_sim_params_.voltage_control_zero *
          (voltage_err +
           voltage_correction_state_x(static_cast<MotorLabel>(r_bot)));

      // Calculate bottom motor torque and power.
      MotorCommand bottom_cmd = CalcMotorSpeedCommand(r_bot, tether_current());
      SimMotorLimit bottom_constraint;
      SpeedControllerState bottom_controller_state;
      double bottom_int_deriv = 0.0;
      double bottom_torque = CalcMotorTorqueCommand(
          torque_limits_[r_bot].val(), bottom_cmd,
          rotors_[r_bot]->AngularSpeed(), int_motor_vel_errs(r_bot),
          &bottom_constraint, &bottom_voltage_correction,
          &bottom_controller_state, &bottom_int_deriv);
      if (tether_current() < power_sys_sim_params_.min_tether_current) {
        bottom_constraint = kSimMotorLimitGroundPower;
      }
      motor_torque_cmds_[r_bot].set_val(bottom_torque);
      speed_controller_state_[r_bot].set_val(bottom_controller_state);
      double bottom_power = sim::physics::motors::CalcMotorPower(
          block_voltages(i), motor_torques(r_bot),
          rotors_[r_bot]->AngularSpeed(), power_sys_sim_params_.motor);
      speed_controller_int_deriv_[r_bot].set_val(bottom_int_deriv);

      // Output step for state space implementation of stacking voltage
      // controller.  Translated from stacking.c using s-domain
      // implementation.  Correction state x is the internal state of
      // stacking controller.
      double top_voltage_correction =
          power_sys_sim_params_.kp_voltage_err *
          power_sys_sim_params_.voltage_control_pole /
          power_sys_sim_params_.voltage_control_zero *
          (voltage_err +
           voltage_correction_state_x(static_cast<MotorLabel>(r_top)));

      // Calculate top motor torque and power.
      MotorCommand top_cmd = CalcMotorSpeedCommand(r_top, tether_current());
      SimMotorLimit top_constraint;
      SpeedControllerState top_controller_state;
      double top_int_deriv = 0.0;
      double top_torque = CalcMotorTorqueCommand(
          torque_limits_[r_top].val(), top_cmd, rotors_[r_top]->AngularSpeed(),
          int_motor_vel_errs(r_top), &top_constraint, &top_voltage_correction,
          &top_controller_state, &top_int_deriv);
      if (tether_current() < power_sys_sim_params_.min_tether_current) {
        top_constraint = kSimMotorLimitGroundPower;
      }
      motor_torque_cmds_[r_top].set_val(top_torque);
      speed_controller_state_[r_top].set_val(top_controller_state);
      double top_power = sim::physics::motors::CalcMotorPower(
          block_voltages(i), motor_torques(r_top),
          rotors_[r_top]->AngularSpeed(), power_sys_sim_params_.motor);
      speed_controller_int_deriv_[r_top].set_val(top_int_deriv);

      // Set derived states.
      motor_constraints_[r_bot].set_val(bottom_constraint);
      motor_constraints_[r_top].set_val(top_constraint);
      voltage_correction_[r_bot].set_val(bottom_voltage_correction);
      voltage_correction_[r_top].set_val(top_voltage_correction);
      block_powers_[i].set_val(bottom_power + top_power);
      motor_currents_[r_bot].set_val(bottom_power /
                                     fmax(1.0, block_voltages(i)));
      motor_currents_[r_top].set_val(top_power / fmax(1.0, block_voltages(i)));
    }
  }
}

void StackedPowerSys::AddInternalConnections(ConnectionStore *connections) {
  connections->Add(1, [this](double /*t*/) { UpdateDerivedStates(); });
}

// The current going into the block capacitance is equal to the
// negative tether current (current is positive flowing into the
// positive terminal of the ground-based voltage source) minus the
// current going into the motors, which we model as ideal power
// converters (e.g. P = I*V).  The time derivative of the block
// voltage is given by the standard capacitor equation (e.g. dV/dt =
// I/C).  The block current, power, and capacitance are the sum of the
// currents, powers, and capacitances of both motors in the block.
//
// Ignoring the dynamics of the voltage feedback, the motor torque's
// transfer function is:
//
//                                   2*pi*fc
//   torque = (kp + ki / (s - p)) * ----------- * (omega_cmd - omega)
//                                  s + 2*pi*fc
//
void StackedPowerSys::CalcDerivHelper(double /*t*/) {
  // Filter the tether current for the ground voltage compensation
  // controller.
  filtered_tether_current_.set_deriv(
      2.0 * M_PI * power_sys_sim_params_.current_filter_cutoff_freq *
      (tether_current() - filtered_tether_current()));

  // Model the dynamics of the ground inverter.
  double ground_voltage_cmd = CalcGroundVoltageCommand();
  ground_voltage_.set_deriv(-power_sys_sim_params_.ground_voltage_pole *
                            (ground_voltage_cmd - ground_voltage()));

  double mean_wing_voltage_correction = CalcMeanVoltageCorrection();
  for (int32_t i = 0; i < num_blocks_; ++i) {
    double block_cap_current;
    if (IsBlockFaulted(i)) {
      // During a fault, simply discharge the capacitor.
      block_cap_current =
          -block_voltages(i) * power_sys_sim_params_.cap_drain_conductance;
      // Hold speed correction during a fault.
      speed_correction_[i].set_deriv(0.0);
    } else {
      if (block_voltages(i) <= 0.0) {
        // Diode will conduct current in negative direction to drop voltage.
        block_cap_current =
            -block_voltages(i) * power_sys_sim_params_.cap_drain_conductance;
      } else if (!tether_released_.val()) {
        block_cap_current =
            -tether_current() + block_powers(i) / fmax(block_voltages(i), 1.0);
      } else {
        // When tether is released, we drop tether current from calculation.
        block_cap_current = block_powers(i) / fmax(block_voltages(i), 1.0);
      }

      // Calculate an adjustment to the speed command to unwind fighting
      // between the stacking voltage control and the speed control loops.
      double mean_pair_voltage_correction =
          (voltage_correction(static_cast<MotorLabel>(GetBottomMotorIndex(i))) +
           voltage_correction(static_cast<MotorLabel>(GetTopMotorIndex(i)))) *
          0.5;
      speed_correction_[i].set_deriv(
          2.0 * M_PI * power_sys_sim_params_.fc_stacking_speed_correction *
          (power_sys_sim_params_.kp_stacking_speed_correction *
               (mean_pair_voltage_correction - mean_wing_voltage_correction) -
           speed_correction(i)));
    }
    block_voltages_[i].set_deriv(block_cap_current / power_sys_params_.C_block);
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    // Filter of speed command.  This replicates a filter in the motor
    // controller firmware and reduces the bandwidth that the stacking voltage
    // controller and the motor speed controller will see.
    speed_cmd_upper_filter_[i].set_deriv(
        -power_sys_sim_params_.speed_cmd_pole *
        (motor_upper_cmds(i) - speed_cmd_upper_filter_[i].val()));
    speed_cmd_lower_filter_[i].set_deriv(
        -power_sys_sim_params_.speed_cmd_pole *
        (motor_lower_cmds(i) - speed_cmd_lower_filter_[i].val()));

    MotorLabel motor_label = static_cast<MotorLabel>(i);

    // Update the speed command integrator
    int_motor_vel_errs_[i].set_deriv(speed_controller_int_deriv(motor_label));

    // Apply anti-windup protection to stacking controller.  We recalculate
    // the output and calculate the disallowed control value and feedback a
    // fraction into the controller input.  This is admittedly unorthodox,
    // but is stable and does a good job preventing windup.
    double temp_gain = power_sys_sim_params_.kp_voltage_err *
                       power_sys_sim_params_.voltage_control_pole /
                       power_sys_sim_params_.voltage_control_zero;
    double voltage_err = block_voltages(i % num_blocks_) -
                         fmin(wing_voltage() / GetNumActiveBlocks(),
                              power_sys_sim_params_.voltage_average_upper_sat);
    double output =
        temp_gain * (voltage_err + voltage_correction_state_x(motor_label));
    double overage = output - voltage_correction(motor_label);
    voltage_err -= overage / temp_gain;

    // Calculate derivative of internal state x for the stacking voltage
    // controller.  This implements a filter that has a single pole
    // and a single zero.
    double temp_deriv = ((power_sys_sim_params_.voltage_control_pole -
                          power_sys_sim_params_.voltage_control_zero) *
                             voltage_err +
                         power_sys_sim_params_.voltage_control_pole *
                             voltage_correction_state_x(motor_label));

    voltage_correction_state_x_[i].set_deriv(temp_deriv);
  }
}

bool StackedPowerSys::IsBlockFaulted(int32_t i) const {
  return !(motor_connections(static_cast<MotorLabel>(GetBottomMotorIndex(i))) &&
           motor_connections(static_cast<MotorLabel>(GetTopMotorIndex(i))));
}

int32_t StackedPowerSys::GetNumActiveBlocks() const {
  int32_t num_active = 0;
  for (int32_t i = 0; i < num_blocks_; ++i) {
    if (!IsBlockFaulted(i)) ++num_active;
  }
  return num_active;
}

double StackedPowerSys::CalcWingVoltage() const {
  double v = 0.0;
  for (int32_t i = 0; i < num_blocks_; ++i) {
    if (!IsBlockFaulted(i)) v += block_voltages(i);
  }
  return v;
}

// Calculate the average of the voltage corrections for use in the speed
// correction calculation used to unwind fighting between speed and voltage
// control loops.
double StackedPowerSys::CalcMeanVoltageCorrection() const {
  double corr_sum = 0.0;
  double num_active = 0.0;
  for (int32_t i = 0; i < num_blocks_; ++i) {
    if (!IsBlockFaulted(i)) {
      corr_sum += speed_correction(i);
      ++num_active;
    }
  }
  return num_active > 0.0 ? corr_sum / static_cast<double>(num_active) : 0.0;
}

double StackedPowerSys::CalcTetherCurrent(double wing_voltage__) const {
  if (tether_released_.val()) {
    return 0.0;
  } else {
    return (wing_voltage__ - ground_voltage()) /
           (power_sys_params_.R_tether + power_sys_params_.R_source);
  }
}

// TODO(b/66951246): rotor_vel should be non-negative in principle, but it
// sometimes ends up being negative. I suspect this occurs when the ODE solver
// takes (and then presumably rejects) an overly-large time step.
double StackedPowerSys::CalcMotorTorqueCommand(
    const TorqueLimits &limits, const MotorCommand &motor_cmd, double rotor_vel,
    double int_rotor_vel_err, SimMotorLimit *constraint,
    double *local_voltage_correction, SpeedControllerState *controller_state,
    double *int_deriv) const {
  double torque_request = motor_cmd.torque_cmd;

  double omega_upper_error = motor_cmd.omega_upper_cmd - rotor_vel;
  double omega_lower_error = motor_cmd.omega_lower_cmd - rotor_vel;

  // Take care of inverted omega limits. Default to omega_upper_limit.
  if (motor_cmd.omega_lower_cmd > motor_cmd.omega_upper_cmd) {
    omega_lower_error = omega_upper_error;
  }

  double torque_upper_cmd =
      power_sys_sim_params_.kp_rotor_vel_err * (omega_upper_error) +
      int_rotor_vel_err;
  double torque_lower_cmd =
      power_sys_sim_params_.kp_rotor_vel_err * (omega_lower_error) +
      int_rotor_vel_err;

  // Pick actual torque based on suggested torque from the three competing
  // sources:
  // upper omega loop, lower omega loop, and torque request.
  double torque_cmd;
  if (torque_request >= torque_upper_cmd) {
    torque_cmd = torque_upper_cmd;
    *controller_state = kSpeedControlUpper;
  } else if (torque_request <= torque_lower_cmd) {
    torque_cmd = torque_lower_cmd;
    *controller_state = kSpeedControlLower;
  } else {
    torque_cmd = torque_request;
    *controller_state = kSpeedControlTorque;

    // If in torque mode, update integrator with requested torque.  Use
    // pole to avoid simulation issues.
    *int_deriv =
        power_sys_sim_params_.rotor_vel_err_torque_pole *
        (int_rotor_vel_err -
         Saturate(torque_request, limits.lower_limit, limits.upper_limit));
  }

  // TODO: Limits should be made identical to flight code.  Currently
  // the limits are based on physics (what motor can do) rather than using the
  // model in the flight controller.  This difference creates an opportunity for
  // windup and needs to be explored.
  // Flux weakening is buried in the limits for the simulation
  // but actually affects the closed loop response for the flight hardware.
  // Ignoring now for simplicity, but longer term, explore making sim more
  // closely match hardware.

  // Apply limits before stacking is sorted out.
  // Integrate the velocity error so the motors attain the speed command at
  // low frequencies, but use a low frequency pole rather than a pure
  // integrator so this doesn't fight the stacking voltage loop.  Allow
  // integrator to correct even if limit is exceeded if resulting update
  // unwinds the integrator.  This reduced some latching behavior.
  // TODO: Pole may no longer be necessary with implementation of
  // stacking speed correction.
  if (torque_cmd >= limits.upper_limit) {
    torque_cmd = limits.upper_limit;
    if (*controller_state != kSpeedControlTorque) *int_deriv = 0.0;
  } else if (torque_cmd <= limits.lower_limit) {
    torque_cmd = limits.lower_limit;
    if (*controller_state != kSpeedControlTorque) *int_deriv = 0.0;
  } else if (*controller_state == kSpeedControlUpper) {
    *int_deriv = (motor_cmd.omega_upper_cmd - rotor_vel) *
                     power_sys_sim_params_.ki_rotor_vel_err +
                 power_sys_sim_params_.rotor_vel_err_pole * int_rotor_vel_err;
  } else if (*controller_state == kSpeedControlLower) {
    *int_deriv = (motor_cmd.omega_lower_cmd - rotor_vel) *
                     power_sys_sim_params_.ki_rotor_vel_err +
                 power_sys_sim_params_.rotor_vel_err_pole * int_rotor_vel_err;
  }

  // Calculate a correction to the torque command to make sure voltages stay
  // balanced.  Limit correction to keep combined torque command within limits
  // and such that the correction does not lead to increased power generation.
  // Preventing additional generation helps avoid runaway propeller stall.
  *local_voltage_correction =
      fmax(*local_voltage_correction, fmin(0, -torque_cmd));
  *local_voltage_correction =
      fmin(*local_voltage_correction, limits.upper_limit - torque_cmd);
  torque_cmd += *local_voltage_correction;

  // In theory, these limits should already be applied.  This should only
  // establish which limit we are against.
  // TODO: Make sure constraints are captured correctly.
  if (torque_cmd >= limits.upper_limit) {
    torque_cmd = limits.upper_limit;
    *constraint = limits.upper_constraint;
  } else if (torque_cmd <= limits.lower_limit) {
    torque_cmd = limits.lower_limit;
    *constraint = limits.lower_constraint;
  } else {
    *constraint = kSimMotorLimitNone;
  }

  return torque_cmd;
}

double StackedPowerSys::CalcGroundVoltageCommand() const {
  if (tether_released_.val()) {
    return 0.0;
  }

  double ground_voltage_compensation =
      power_sys_params_.use_ground_voltage_compensation
          ? -power_sys_params_.R_tether * filtered_tether_current()
          : 0.0;

  ground_voltage_compensation =
      Saturate(ground_voltage_compensation,
               power_sys_sim_params_.min_ground_voltage_compensation,
               power_sys_sim_params_.max_ground_voltage_compensation);

  return power_sys_params_.v_source_0 + ground_voltage_compensation;
}

// Modify speed command with correction for excessive tether current in the
// motoring direction and stacking speed correction.
// TODO: Make tether current controller consistent with motor
// controller behavior or add test for violation of generation limit.
MotorCommand StackedPowerSys::CalcMotorSpeedCommand(
    MotorLabel i, double tether_current__) const {
  MotorCommand motor_cmd;
  double current_excess =
      power_sys_sim_params_.min_tether_current - tether_current__;
  motor_cmd.omega_upper_cmd =
      fmax(0.0, speed_cmd_upper_filter(i) -
                    power_sys_sim_params_.kp_excess_tether_current *
                        fmax(0.0, current_excess) +
                    speed_correction(static_cast<int32_t>(i) % num_blocks_));

  motor_cmd.omega_lower_cmd =
      fmax(0.0, speed_cmd_lower_filter(i) -
                    power_sys_sim_params_.kp_excess_tether_current *
                        fmax(0.0, current_excess) +
                    speed_correction(static_cast<int32_t>(i) % num_blocks_));
  motor_cmd.torque_cmd = flight_controller_torque_cmds(i);

  return motor_cmd;
}

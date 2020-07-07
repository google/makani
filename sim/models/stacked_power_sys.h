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

// StackedPowerSys implements a simplified model of the M600 power
// system, including the ground-based inverter, tether, power
// electronics, and motors on the wing.  The eight motors are grouped
// into a "stack" of four "blocks" of two as shown in the diagram
// below:
//
//        R_tether
//      ___/\/\/\___  V_wing
//     |          __|__ ____
//     |         |     |    |
//     |         M1   M5   === C_block
//     |         |_____|____|
//     |          __|__ ____
//     |         |     |    |        Layout of motors on wing:
//     |         M2   M6   ===
//     |         |_____|____|            M8     M7   x M6     M5
//    V_dc        __|__ ____         y <------------o
//     |         |     |    |            M1     M2  |  M3     M4
//     |         M3   M7   ===                      |
//     |         |_____|____|                       v z
//     |          __|__ ____
//     |         |     |    |
//     |         M4   M8   ===
//     |         |_____|____|
//     |____________|
//
// The ground based inverter is modeled as an ideal voltage source.
// The tether is modeled as a resistor (any inductance is ignored
// here).  A bandwidth limited controller attempts to compensate for
// voltage drop on the tether.
//
// The power electronics and motors are modeled as nearly ideal power
// converters (i.e. P = V*I), which are in parallel with a capacitance,
// C_block.  A loss model translates between electrical power and
// mechanical power, accounting for motor and motor controller losses.
//
// As much as possible, the control parameters have been taken from the
// avionics/motor/firmware directory and the control structure has been
// adapted to a continuous time domain implementation to ensure resonable
// simulation speed.
//
// Each motor controller is modeled as having instantaneous torque response
// within limits based on a flux weakening model of the Yasa 2.3 motor.
// Actual closed loop response of the torque control is fast relative to
// simulation time so this was deemed a necessary and acceptable compromise
// to ensure fast simulation.
//
// The power electronics attempt to equalize the block voltages with a
// lag lead controller that adjusts torque on fast time scales to minimize
// voltage error (V_block - V_wing/4).  The stacking controller also applies
// a correction on the speed command to keep the speed controller integrator
// from fighting with the stacking controller.
//
// When an element of motor_connections is false, both motors in the
// associated block are removed from the stack voltage, their torque
// commands are set to zero, and their capacitor is drained.  The
// ground-base inverters are unaffected by faults (change made with
// introduction of Ozone) leading to higher voltage on the 3 remaining
// stacking levels.
//
// The StackedPowerSys model takes as input from the controller a
// desired rotor velocity for each motor.  The power electronics has
// the freedom to adjust the speed of each motor to ensure that block
// voltages remain close to equal.
//
// TODO: Evaluate if tether inductance should impact this model.

#ifndef SIM_MODELS_STACKED_POWER_SYS_H_
#define SIM_MODELS_STACKED_POWER_SYS_H_

#include <stdint.h>

#include <memory>
#include <vector>

#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/rotor.h"
#include "sim/models/power_sys.h"
#include "sim/physics/motors.h"
#include "sim/sim_messages.h"
#include "sim/sim_types.h"
#include "system/labels.h"

struct MotorCommand {
  double omega_upper_cmd;
  double omega_lower_cmd;
  double torque_cmd;
};

class StackedPowerSys : public PowerSys {
 public:
  StackedPowerSys(const std::vector<std::unique_ptr<RotorBase>> &rotors,
                  const RotorSensorParams (&rotor_sensor_params)[kNumMotors],
                  const PowerSysParams &power_sys_params,
                  const PowerSysSimParams &power_sys_sim_params,
                  FaultSchedule *faults);
  ~StackedPowerSys() {}

  void Publish() const override;

  double motor_torques(MotorLabel i) const override {
    return motor_torque_cmds_[i].val();
  }

 private:
  enum SpeedControllerState {
    kSpeedControlTorque,
    kSpeedControlUpper,
    kSpeedControlLower,
  };

  using TorqueLimits = ::sim::physics::motors::TorqueLimits;

  void AddInternalConnections(ConnectionStore *connections) override;
  void UpdateDerivedStates();

  // Calculates the time derivative of the voltages of each block.
  void CalcDerivHelper(double t) override;

  // Tests if a given block has not faulted by checking if the
  // associated pair of motor connections are live.
  bool IsBlockFaulted(int32_t i) const;

  // Returns number of non-faulted blocks.
  int32_t GetNumActiveBlocks() const;

  // Returns the voltage [V] across all the blocks at the wing.  This
  // is on the wing side of the voltage change across the tether and
  // thus is equal to the sum of all the individual block voltages.
  double CalcWingVoltage() const;

  // Return the average of the voltage corrections.
  double CalcMeanVoltageCorrection() const;

  // Returns the current [A] in the tether, which is equal to the
  // difference between the wing and ground voltages divided by the
  // tether resistance.  This is defined to be positive during power
  // generation (i.e. when current is flowing into the positive
  // terminal of the ground-based voltage source]).  We currently do
  // not take the tether inductance or capacitance into account.
  double CalcTetherCurrent(double wing_voltage__) const;

  // Models the stacked motor controller system, which both attempts
  // to equalize the voltages across each of the blocks and deliver a
  // specific rotor velocity.  Returns the torque output from the
  // motor controller.
  double CalcMotorTorqueCommand(const TorqueLimits &limits,
                                const MotorCommand &motor_cmd, double rotor_vel,
                                double int_motor_vel_err,
                                SimMotorLimit *constraint,
                                double *local_voltage_correction,
                                SpeedControllerState *controller_state,
                                double *int_deriv) const;

  // Returns the voltage command to the ground inverter.  Currently,
  // this instantly reduces the ground-side voltage by a ratio of the
  // number of active blocks to total blocks when a block faults.
  // Also, if voltage compensation is turned on, this will attempt to
  // cancel the voltage drop across the tether.
  double CalcGroundVoltageCommand() const;

  // Calculates a motor speed command modified to account for
  // enforcing a limit on ground power draw.
  MotorCommand CalcMotorSpeedCommand(MotorLabel i, double wing_voltage__) const;

  double GetMotorBlockVoltage(MotorLabel i) const {
    return block_voltages(i % num_blocks_);
  }
  double filtered_tether_current() const {
    return filtered_tether_current_.val();
  }
  double ground_voltage() const { return ground_voltage_.val(); }
  double motor_torque_cmds(MotorLabel i) const {
    return motor_torque_cmds_[i].val();
  }
  SpeedControllerState speed_controller_state(MotorLabel i) const {
    return speed_controller_state_[i].val();
  }
  double speed_controller_int_deriv(MotorLabel i) const {
    return speed_controller_int_deriv_[i].val();
  }
  double voltage_correction(MotorLabel i) const {
    return voltage_correction_[i].val();
  }
  double voltage_correction_state_x(MotorLabel i) const {
    return voltage_correction_state_x_[i].val();
  }
  double speed_correction(int32_t i) const {
    return speed_correction_[i].val();
  }
  double speed_cmd_upper_filter(int32_t i) const {
    return speed_cmd_upper_filter_[i].val();
  }
  double speed_cmd_lower_filter(int32_t i) const {
    return speed_cmd_lower_filter_[i].val();
  }
  SimMotorLimit motor_constraints(MotorLabel i) const {
    return motor_constraints_[i].val();
  }
  double block_voltages(int32_t i) const { return block_voltages_[i].val(); }
  double block_powers(int32_t i) const { return block_powers_[i].val(); }

  // Stacked power system parameters.

  // Number of motor blocks.  For an eight rotor system with two
  // motors per block, there are four blocks.
  const int32_t num_blocks_;

  // Continuous state.

  // Measured current [A] in the tether that has been filtered for use
  // in the ground inverter voltage compensation controller.
  ContinuousState<double> filtered_tether_current_;

  // Voltage [V] of the ground inverter.
  ContinuousState<double> ground_voltage_;

  // Vector of voltages [V] across each block.
  std::vector<ContinuousState<double>> block_voltages_;

  // Speed correction controller output [rad/sec].
  std::vector<ContinuousState<double>> speed_correction_;

  // Voltage correction filter internal state x
  std::vector<ContinuousState<double>> voltage_correction_state_x_;

  // Filtered speed commands [rad/sec].
  std::vector<ContinuousState<double>> speed_cmd_upper_filter_;
  std::vector<ContinuousState<double>> speed_cmd_lower_filter_;

  // Derived values.

  // Unfiltered motor torque [N-m] commands, which are based on the
  // rotor velocity error and block voltage error signals.
  std::vector<State<double>> motor_torque_cmds_;
  std::vector<State<SpeedControllerState>> speed_controller_state_;
  std::vector<State<double>> voltage_correction_;
  std::vector<State<TorqueLimits>> torque_limits_;

  // Type of constraint, if any, that is limiting the motor command.
  std::vector<State<SimMotorLimit>> motor_constraints_;

  // Speed controller input to integrator
  std::vector<State<double>> speed_controller_int_deriv_;

  // Vector of powers [W] generated by each block.  This is the sum of
  // the powers generated by the two motors in the block and is
  // defined to be positive during generation.
  std::vector<State<double>> block_powers_;

  DISALLOW_COPY_AND_ASSIGN(StackedPowerSys);
};

#endif  // SIM_MODELS_STACKED_POWER_SYS_H_

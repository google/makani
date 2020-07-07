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

// The model of the wing servo is composed of 4 parts:
//
// 1) A stateless model of the gear box:
//
//   shaft_angular_vel = motor_angular_vel / gear_ratio
//   shaft_angle = motor_angle / gear_ratio
//   external_motor_torque = (external_shaft_torque - friction) / gear_ratio
//
// The external shaft torque may be, for example, the aerodynamic
// torque on a flap.  The friction function models the gear friction
// as a function of speed.
//
// 2) A model of the controller, which contains state:
//
//   control_voltage = f(shaft_angle_cmd, ref_model_shaft_angle,
//                       ref_model_shaft_angular_vel)
//
// where control_voltage is an analog version of PWM control and
// shaft_angle_cmd is the commanded flap angle.
//
// 3) A stateless electrical model of the motor:
//
//                   control_voltage - k_m * motor_angular_vel
//   motor_current = -----------------------------------------
//                             Z(motor_angular_vel)
//
//   Z(omega) = ||R_m + j * omega * num_elec_poles * L||
//
// where motor_current is the mean current through the windings, k_m
// is the motor torque constant, R_m is the motor resistance across
// wires (measured at the wires), and num_elec_poles is the number of
// poles (2 * pole_pairs).  Inductance is negligible on a long time
// scale, but is accounted for at the commutation time scale.
//
// 4) A mechanical model of the motor with motor angle and angular
// rate states:
//
//   d(motor_angular_vel)   k_m * motor_current + external_motor_torque
//   -------------------- = -------------------------------------------
//           dt                  J_motor + J_flap / (gear_ratio)^2
//
// where J_motor is the motor inertia and J_flap is the flap inertia.
// (The effective total moment of inertia is simply termed
// moment_of_inertia in the code.)
//
// The total electrical power (not including drive losses) is
// calculated as
//
//   P = R_m * I^2 + omega_m * (k_m * I).
//

#ifndef SIM_MODELS_ACTUATORS_SERVO_H_
#define SIM_MODELS_ACTUATORS_SERVO_H_

#include <stdint.h>

#include <vector>

#include "common/macros.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/actuator.h"
#include "sim/sim_messages.h"
#include "sim/sim_types.h"
#include "sim/state.h"

class ServoBase : public Actuator {
  friend class ServoTest;

 public:
  ServoBase(ServoLabel servo_label, const ServoParams &servo_params,
            const ServoSimParams &servo_sim_params, FaultSchedule *faults);
  virtual ~ServoBase() {}

  // Inputs.
  void set_external_flap_torque(double torque);

  void set_external_shaft_torque(double torque) {
    external_shaft_torque_.set_val(torque);
  }

  // Outputs.
  double external_shaft_torque() const { return external_shaft_torque_.val(); }
  double shaft_angle() const { return shaft_angle_.val(); }
  double shaft_angular_vel() const { return shaft_angular_vel_.val(); }
  double flap_angle() const { return FlapAngleFromServo(shaft_angle_.val()); }
  double flap_angular_vel() const {
    return FlapAngularRateFromServo(shaft_angle_.val(),
                                    shaft_angular_vel_.val());
  }
  virtual double motor_power() const { return 0.0; }

  void SetFromAvionicsPackets(const AvionicsPackets &avionics_packets) override;

 protected:
  double shaft_angle_cmd() const { return shaft_angle_cmd_.val(); }

  // Enum labeling servo (A1, A2, etc.).
  const ServoLabel servo_label_;
  // Parameters describing a dynamic model for the servo, including
  // moment of inertia.
  const ServoSimParams &servo_sim_params_;

  // Input state.

  // Total external torque [N-m] on the servo shaft from aerodynamic
  // and other forces.  This is on the servo shaft rather than the
  // flap, which may be separated by a linkage ratio.
  State<double> external_shaft_torque_;

  // Derived values.

  // Angle [rad] of the output shaft.
  State<double> shaft_angle_;
  // Angular velocity [rad/s] of the output shaft.
  State<double> shaft_angular_vel_;

  // Connection to the FaultSchedule.
  const FaultSchedule::FaultFunc fault_func_;

 private:
  void DiscreteStepHelper(double /*t*/) override {}
  double FlapAngleFromServo(double servo_shaft_angle) const;
  double FlapAngularRateFromServo(double servo_shaft_angle,
                                  double servo_shaft_rate) const;

  // Parameters containing linkage ratio.
  const ServoParams &servo_params_;

  // Actuator commands.

  // Flight controller commanded servo angle [rad].
  State<double> shaft_angle_cmd_;

  DISALLOW_COPY_AND_ASSIGN(ServoBase);
};

class Servo : public ServoBase {
  friend class ServoTest;

 public:
  Servo(ServoLabel servo_label, const ServoParams &servo_params,
        const ServoSimParams &servo_sim_params, FaultSchedule *faults);
  ~Servo() {}

  // Outputs.
  double motor_power() const override { return motor_power_.val(); }

 private:
  double motor_current() const { return motor_current_.val(); }
  double motor_angle() const { return motor_angle_.val(); }
  double motor_angular_vel() const { return motor_angular_vel_.val(); }
  double ref_model_shaft_angle() const { return ref_model_shaft_angle_.val(); }
  double ref_model_shaft_angular_vel() const {
    return ref_model_shaft_angular_vel_.val();
  }

  void AddInternalConnections(ConnectionStore *connections) override;
  void CalcDerivHelper(double t) override;

  // Continuous states.

  // Servo motor angular velocity [rad/s].
  ContinuousState<double> motor_angular_vel_;
  // Servo motor angle [rad].
  ContinuousState<double> motor_angle_;
  // Servo drive controller reference angle [rad].
  ContinuousState<double> ref_model_shaft_angle_;

  // Derived values.

  // Mean current [A] through the motor windings.
  State<double> motor_current_;
  // Electrical power [W] required at the motor.
  State<double> motor_power_;
  // Angular velocity [rad/s] of the reference model shaft command.
  State<double> ref_model_shaft_angular_vel_;

  DISALLOW_COPY_AND_ASSIGN(Servo);
};

// Updates its state from ServoDebugMessages rather than a model of the servo
// mechanics.
class HitlServo : public ServoBase {
 public:
  HitlServo(ServoLabel servo_label, const ServoParams &servo_params,
            const ServoSimParams &servo_sim_params, FaultSchedule *faults);
  ~HitlServo() {}

  void SetFromAvionicsPackets(const AvionicsPackets &avionics_packets) override;

 private:
  void CalcDerivHelper(double /*t*/) override {}

  DISALLOW_COPY_AND_ASSIGN(HitlServo);
};

#endif  // SIM_MODELS_ACTUATORS_SERVO_H_

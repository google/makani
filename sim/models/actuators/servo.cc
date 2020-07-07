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

#include "sim/models/actuators/servo.h"

#include <glog/logging.h>
#include <math.h>
#include <stdint.h>

#include <vector>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "common/c_math/util.h"
#include "common/c_math/voting.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/actuator.h"
#include "sim/sim_types.h"
#include "system/labels.h"

ServoBase::ServoBase(ServoLabel servo_label, const ServoParams &servo_params,
                     const ServoSimParams &servo_sim_params,
                     FaultSchedule *faults)
    : Actuator("Servo[" + std::to_string(static_cast<int32_t>(servo_label)) +
               "]"),
      servo_label_(servo_label),
      servo_sim_params_(servo_sim_params),
      external_shaft_torque_(new_derived_value(), "external_shaft_torque"),
      shaft_angle_(new_derived_value(), "shaft_angle", 0.0),
      shaft_angular_vel_(new_derived_value(), "shaft_angular_vel", 0.0),
      fault_func_(faults->ClaimFaultFunc(full_name(),
                                         {{kSimFaultActuatorZero, 0U},
                                          {kSimFaultServoFixValue, 1U},
                                          {kSimFaultServoHoldCurrent, 0U}})),
      servo_params_(servo_params),
      shaft_angle_cmd_(new_derived_value(), "shaft_angle_cmd") {}

// Compute the servo shaft torque produced by the flap aero moment.
void ServoBase::set_external_flap_torque(double torque) {
  double external_torque;

  external_torque = torque * (servo_params_.linear_servo_to_flap_ratio +
                              servo_params_.nonlinear_servo_to_flap_ratio *
                                  cos(shaft_angle_.val()));

  external_shaft_torque_.set_val(external_torque);
}

void ServoBase::SetFromAvionicsPackets(
    const AvionicsPackets &avionics_packets) {
  double saturated_cmd_value =
      Saturate(avionics_packets.command_message.servo_angle[servo_label_],
               servo_sim_params_.servo_drive.ref_model_min_position_limit,
               servo_sim_params_.servo_drive.ref_model_max_position_limit);
  shaft_angle_cmd_.set_val(saturated_cmd_value);
}

// Compute the flap angle from the servo angle.
double ServoBase::FlapAngleFromServo(double servo_shaft_angle) const {
  return (servo_params_.linear_servo_to_flap_ratio * servo_shaft_angle +
          servo_params_.nonlinear_servo_to_flap_ratio * sin(servo_shaft_angle));
}

// Compute the flap angular velocity from the servo angle
// and servo angular rate.
double ServoBase::FlapAngularRateFromServo(double servo_shaft_angle,
                                           double servo_shaft_rate) const {
  return (servo_params_.linear_servo_to_flap_ratio * servo_shaft_rate +
          servo_params_.nonlinear_servo_to_flap_ratio * servo_shaft_rate *
              cos(servo_shaft_angle));
}

Servo::Servo(ServoLabel servo_label, const ServoParams &servo_params,
             const ServoSimParams &servo_sim_params, FaultSchedule *faults)
    : ServoBase(servo_label, servo_params, servo_sim_params, faults),
      motor_angular_vel_(new_continuous_state(), "motor_angular_vel", 0.0),
      motor_angle_(new_continuous_state(), "motor_angle", 0.0),
      ref_model_shaft_angle_(new_continuous_state(), "ref_model_shaft_angle",
                             0.0),
      motor_current_(new_derived_value(), "motor_current", 0.0),
      motor_power_(new_derived_value(), "motor_power", 0.0),
      ref_model_shaft_angular_vel_(new_derived_value(),
                                   "ref_model_shaft_angular_vel", 0.0) {
  SetupDone();
}

namespace {

// Calculates the motor current [A] as a function of the applied
// voltage [V], motor angular rate [rad/s], and the parameters which
// describe the motor.
double CalcMotorCurrent(double voltage, double back_emf_constant,
                        double angular_vel, int32_t num_elec_poles,
                        double resistance, double inductance) {
  // Calculate the gross impedance [Ohm] of the motor due to winding
  // resistance and inductance.
  double reactance =
      inductance * static_cast<double>(num_elec_poles) * angular_vel;
  double impedance_norm = sqrt(resistance * resistance + reactance * reactance);

  return (voltage - back_emf_constant * angular_vel) / impedance_norm;
}

// Calculates the electrical power input [W] to the servo motors,
// which includes I^2 R losses.
double CalcMotorElectricalPower(double current, double back_emf_constant,
                                double angular_vel, double resistance) {
  return current * current * resistance +
         back_emf_constant * angular_vel * current;
}

// Returns the torque [N-m] acting on the servo motor after friction
// and the gear ratio have been taken into account.
double CalcExternalTorqueOnMotor(double external_torque, double gear_ratio,
                                 double shaft_angular_vel,
                                 const double friction_angular_vel_table[],
                                 const double friction_torque_table[]) {
  // Calculate the speed-dependent gear friction [N-m].
  double friction_torque =
      Sign(shaft_angular_vel) *
      Interp1(friction_angular_vel_table, friction_torque_table,
              NUM_SERVO_FRICTION_TABLE, fabs(shaft_angular_vel),
              kInterpOptionDefault);
  return (external_torque - friction_torque) / gear_ratio;
}

// Models the servo drive controller.  The servo drive controller
// takes a shaft angle command and the current servo and reference
// model states, and outputs the derivative of the reference model and
// the output control voltage of the drive.  Ignoring the non-linear
// effects from a rate limit, the reference model is equivalent to
// low-pass filtering the shaft angle command and then doing a simple
// PD controller on the error signal.
//
//                       (   w                                   )
//   V = (kp + s * kd) * ( ----- * shaft_angle_cmd - shaft_angle )
//                       ( s + w                                 )
//
// where kp and kd are the proportional and derivative gains and w is
// the reference model cutoff frequency in rad/s.  The PD controller
// outputs a voltage to the windings.  The voltage request can be
// thought of as an analog version of a PWM request.
//
// Args:
//   shaft_angle_cmd: Commanded shaft angle [rad].
//   shaft_angle: Actual shaft angle [rad].
//   shaft_angular_vel: Actual shaft angular velocity [rad/s].
//   ref_model_shaft_angle: The state variable of the reference model,
//       which may be though of as a low-pass filtered shaft angle
//       command [rad].
//   params: Parameters that describe the response and limits of the
//       servo drive.
//   control_voltage: Output point to the time-averaged control
//       voltage [V] seen at the motor windings.
//
// Returns:
//   Derivative of the state variable of the reference model, i.e. the
//   reference model shaft angular rate.
double ModelServoDrive(double shaft_angle_cmd, double shaft_angle,
                       double shaft_angular_vel, double ref_model_shaft_angle,
                       const ServoDriveSimParams &params,
                       double *control_voltage) {
  double ref_model_shaft_angular_vel =
      Saturate(2.0 * M_PI * params.ref_model_cutoff_freq *
                   (shaft_angle_cmd - ref_model_shaft_angle),
               -params.ref_model_rate_lim, params.ref_model_rate_lim);

  *control_voltage =
      Saturate(params.kp * (ref_model_shaft_angle - shaft_angle) -
                   params.kd * shaft_angular_vel,
               -params.bus_voltage, params.bus_voltage);
  return ref_model_shaft_angular_vel;
}

}  // namespace

void Servo::AddInternalConnections(ConnectionStore *connections) {
  connections->Add(1, [this](double t) {
    // Convert servo motor angle to shaft angles using the gear ratio.
    shaft_angle_.set_val(motor_angle() / servo_sim_params_.gear_ratio);
    shaft_angular_vel_.set_val(motor_angular_vel() /
                               servo_sim_params_.gear_ratio);

    // Run the servo drive model to find the derivative of the servo
    // drive state, i.e. the reference model shaft angle, and to find
    // the output voltage of the drive.
    double control_voltage;
    double shaft_angle_cmd_to_model;
    std::vector<double> parameters;
    if (fault_func_(t, kSimFaultServoFixValue, &parameters)) {
      shaft_angle_cmd_to_model = parameters[0];
    } else {
      shaft_angle_cmd_to_model = shaft_angle_cmd();
    }
    ref_model_shaft_angular_vel_.set_val(
        ModelServoDrive(shaft_angle_cmd_to_model, shaft_angle(),
                        shaft_angular_vel(), ref_model_shaft_angle(),
                        servo_sim_params_.servo_drive, &control_voltage));
    double control_current = CalcMotorCurrent(
        control_voltage, servo_sim_params_.motor_torque_constant,
        motor_angular_vel(), servo_sim_params_.num_elec_poles,
        servo_sim_params_.motor_resistance, servo_sim_params_.motor_inductance);
    // Limit servo drive current.
    control_current =
        Saturate(control_current, -servo_sim_params_.servo_drive.current_lim,
                 servo_sim_params_.servo_drive.current_lim);

    motor_current_.set_val(control_current);
    motor_power_.set_val(CalcMotorElectricalPower(
        control_current, servo_sim_params_.motor_torque_constant,
        motor_angular_vel(), servo_sim_params_.motor_resistance));
  });
}

void Servo::CalcDerivHelper(double t) {
  if (fault_func_(t, kSimFaultServoHoldCurrent, nullptr)) {
    ref_model_shaft_angle_.set_deriv(0.0);
    motor_angular_vel_.set_deriv(0.0);
    motor_angle_.set_deriv(0.0);
    return;
  }

  // Update the internal state of the servo drive reference model.
  ref_model_shaft_angle_.set_deriv(ref_model_shaft_angular_vel());

  // Calculate motor acceleration.
  double motor_external_torque = CalcExternalTorqueOnMotor(
      external_shaft_torque(), servo_sim_params_.gear_ratio,
      shaft_angular_vel(), servo_sim_params_.friction_angular_vel_table,
      servo_sim_params_.friction_torque_table);
  motor_angular_vel_.set_deriv(
      (servo_sim_params_.motor_torque_constant * motor_current() +
       motor_external_torque) /
      servo_sim_params_.moment_of_inertia);
  motor_angle_.set_deriv(motor_angular_vel());
}

HitlServo::HitlServo(ServoLabel servo_label, const ServoParams &servo_params,
                     const ServoSimParams &servo_sim_params,
                     FaultSchedule *faults)
    : ServoBase(servo_label, servo_params, servo_sim_params, faults) {
  SetupDone();
}

void HitlServo::SetFromAvionicsPackets(
    const AvionicsPackets &avionics_packets) {
  ServoBase::SetFromAvionicsPackets(avionics_packets);

  const ServoDebugMessage &status =
      avionics_packets.servo_statuses.at(servo_label_);
  shaft_angle_.set_val(status.angle_estimate);
  shaft_angular_vel_.set_val(status.angular_velocity);
}

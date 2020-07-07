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

#include "sim/models/actuators/rotor.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <vector>

#include "avionics/motor/firmware/flags.h"
#include "common/c_math/filter.h"
#include "common/c_math/force_moment.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "common/c_math/voting.h"
#include "common/runfiles_dir.h"
#include "control/system_types.h"
#include "sim/math/util.h"
#include "sim/models/environment.h"
#include "sim/physics/rotor_database.h"
#include "sim/physics/rotor_database_3d.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"
#include "system/labels.h"

RotorBase::RotorBase(MotorLabel motor_label, const RotorParams &rotor_params,
                     const RotorSimParams &rotor_sim_params,
                     const RotorSensorParams &rotor_sensor_params,
                     const Environment &environment, double ts)
    : Actuator(
          "Rotor[" + std::to_string(static_cast<int32_t>(motor_label)) + "]",
          ts),
      motor_label_(motor_label),
      rotor_params_(rotor_params),
      rotor_sim_params_(rotor_sim_params),
      rotor_sensor_params_(rotor_sensor_params),
      rotor_database_(RunfilesDir() + "/database/" +
                      rotor_sim_params_.database_names[motor_label].name),
      rotor_database_3d_(RunfilesDir() + "/database/" +
                         rotor_sim_params_.database_3d_names[motor_label].name),
      environment_(environment),
      omega_upper_cmd_(new_derived_value(), "omega_upper_cmd"),
      omega_lower_cmd_(new_derived_value(), "omega_lower_cmd"),
      torque_cmd_(new_derived_value(), "torque_cmd"),
      local_apparent_wind_b_(new_derived_value(), "local_apparent_wind_b"),
      parent_omega_(new_derived_value(), "parent_omega"),
      v_freestream_(new_derived_value(), "v_freestream"),
      v_edgewise_(new_derived_value(), "v_edgewise"),
      dcm_rp2r_(new_derived_value(), "dcm_rp2r"),
      y_prime_force_(new_derived_value(), "y_prime_force"),
      z_prime_force_(new_derived_value(), "z_prime_force"),
      y_prime_mom_(new_derived_value(), "y_prime_mom"),
      z_prime_mom_(new_derived_value(), "z_prime_mom"),
      thrust_(new_derived_value(), "thrust"),
      aero_power_(new_derived_value(), "aero_power"),
      aero_torque_(new_derived_value(), "aero_torque") {}

double RotorBase::RotorThrustCoeff() const {
  // Bound v_freestream^2 below to prevent excessively large thrust
  // coefficients.
  double bounded_v_freestream_squared =
      Square(fmax(fabs(v_freestream()),
                  rotor_sim_params_.min_freestream_vel_for_thrust_coeff));

  // Wind turbine thrust coefficient notation.
  return thrust() /
         (0.5 * environment_.air_density() * M_PI *
          pow(rotor_params_.D / 2.0, 2.0) * bounded_v_freestream_squared);
}

// The force-moment that the rotor applies to the wing has three
// components: aerodynamic, gyroscopic, and reaction forces and
// moments.
// An additional thrust vectoring force is applied to represent prop-wash
// interference on the wing (blown wing lift).
void RotorBase::CalcRotorForceMomentPos(ForceMomentPos *fmx) const {
  Vec3 aero_force_b;
  Vec3Scale(&rotor_params_.axis, thrust(), &aero_force_b);

  Vec3 aero_moment_b;
  Vec3Scale(&rotor_params_.axis, aero_torque(), &aero_moment_b);

  Vec3 gyro_moment_b;
  CalcGyroMoment(omega(), parent_omega(), &gyro_moment_b);

  Vec3 reaction_moment_b;
  CalcReactMoment(RotorAcc(), &reaction_moment_b);

  if (rotor_sim_params_.apply_3d_rotor_tables) {
    // 3D database thrust (x) force are set to zero here because they are
    // applied in AddInternalConnections.
    Vec3 rotor_prime_3d_forces = {0.0, y_prime_force(), z_prime_force()};
    Mat3 dcm_rp2r_temp = dcm_rp2r();
    Vec3 rotor_3d_forces;
    // Rotate the rotor prime axes forces to the rotor geometric axes.
    Mat3Vec3Mult(&dcm_rp2r_temp, &rotor_prime_3d_forces, &rotor_3d_forces);
    // Rotor-to-body direction cosine matrix is transpose of body-to-rotor DCM.
    Mat3 dcm_r2b;
    Mat3Trans(&rotor_params_.dcm_b2r, &dcm_r2b);
    Vec3 rotor_3d_forces_b;
    // Rotate the rotor geometric axes forces to the kite body axes.
    Mat3Vec3Mult(&dcm_r2b, &rotor_3d_forces, &rotor_3d_forces_b);

    // 3D database torque (x) moments are set to zero here because they are
    // applied in AddInternalConnections.
    Vec3 rotor_prime_3d_moments = {0.0, y_prime_mom(), z_prime_mom()};
    Vec3 rotor_3d_moments;
    // Rotate the rotor prime axes moments to the rotor geometric axes.
    Mat3Vec3Mult(&dcm_rp2r_temp, &rotor_prime_3d_moments, &rotor_3d_moments);
    Vec3 rotor_3d_moments_b;
    // Rotate the rotor geometric axes moments to the kite body axes.
    Mat3Vec3Mult(&dcm_r2b, &rotor_3d_moments, &rotor_3d_moments_b);

    Vec3Add(&aero_force_b, &rotor_3d_forces_b, &aero_force_b);
    Vec3Add(&aero_moment_b, &rotor_3d_moments_b, &aero_moment_b);
  }

  // Build force-moment-position output.  No gyroscopic force is
  // included because the wing is assumed to be a rigid body and these
  // forces will cancel.
  fmx->force = aero_force_b;
  Vec3Add3(&aero_moment_b, &gyro_moment_b, &reaction_moment_b, &fmx->moment);
  fmx->pos = rotor_params_.pos;
}

void RotorBase::CalcBlownWingForceMomentPos(ForceMomentPos *fmx_bw) const {
  // During hover flight tests, the wing experiences more tension than
  // expected by a simple flat plate drag model.  CFD studies have shown
  // the extra tension can be attributed to the blown wing effect: High-speed
  // rotor wake passes over/near the wing creating additional forces.  This is
  // represented here by a rotation of the rotor thrust force vector.
  fmx_bw->force = kVec3Zero;
  fmx_bw->moment = kVec3Zero;
  fmx_bw->pos = kVec3Zero;
  if (rotor_sim_params_.apply_blown_wing_effect) {
    Vec3Scale(&rotor_params_.axis, thrust(), &fmx_bw->force);
    fmx_bw->pos = rotor_sim_params_.thrust_vectoring_pos_b;
    double thrust_vectoring_angle = Crossfade(
        rotor_sim_params_.thrust_vectoring_angle, 0.0, CalcVFreestream(),
        rotor_sim_params_.full_blown_wing_freestream_vel,
        rotor_sim_params_.zero_blown_wing_freestream_vel);
    Mat3 thrust_vectoring_rotation;
    AngleToDcm(0.0, -thrust_vectoring_angle, 0.0, kRotationOrderZyx,
               &thrust_vectoring_rotation);
    Vec3 rotated_force;
    Mat3Vec3Mult(&thrust_vectoring_rotation, &fmx_bw->force, &rotated_force);
    // Book-keep only the difference between the rotated thrust force and the
    // original thrust force as the blown wing force contribution.
    Vec3Sub(&rotated_force, &fmx_bw->force, &fmx_bw->force);
  }
}

void RotorBase::Publish() const {
  sim_telem.rotors[motor_label_].omega = omega();
  sim_telem.rotors[motor_label_].thrust = thrust();
  sim_telem.rotors[motor_label_].aero_power = aero_power();
  sim_telem.rotors[motor_label_].aero_torque = aero_torque();
  sim_telem.rotors[motor_label_].rotor_accel = RotorAcc();
  Vec3 gyro_moment;
  CalcGyroMoment(omega(), parent_omega(), &gyro_moment);
  sim_telem.rotors[motor_label_].gyro_moment = gyro_moment;
  sim_telem.rotors[motor_label_].local_apparent_wind_b =
      local_apparent_wind_b();
  sim_telem.rotors[motor_label_].v_freestream = v_freestream();
  // motor_torque is populated by derived classes.
}

bool MotorRunCommanded(const ControllerCommandMessage &command_message) {
  return command_message.motor_command & kMotorCommandRun;
}

void RotorBase::SetFromAvionicsPackets(
    const AvionicsPackets &avionics_packets) {
  const auto &command_message = avionics_packets.command_message;

  double omega_upper_cmd__ = 0.0;
  double omega_lower_cmd__ = 0.0;
  double torque_cmd__ = 0.0;

  if (MotorRunCommanded(command_message)) {
    int32_t i = static_cast<int32_t>(motor_label_);
    omega_upper_cmd__ = command_message.motor_speed_upper_limit[i];
    omega_lower_cmd__ = command_message.motor_speed_lower_limit[i];
    torque_cmd__ = command_message.motor_torque[i];
  }

  omega_upper_cmd__ =
      ApplyCal(omega_upper_cmd__, &rotor_sensor_params_.omega_cal);
  omega_lower_cmd__ =
      ApplyCal(omega_lower_cmd__, &rotor_sensor_params_.omega_cal);

  // Upper and lower commands flip with sign changes.
  if (omega_upper_cmd__ >= omega_lower_cmd__) {
    omega_upper_cmd_.set_val(omega_upper_cmd__);
    omega_lower_cmd_.set_val(omega_lower_cmd__);
  } else {
    omega_upper_cmd_.set_val(omega_lower_cmd__);
    omega_lower_cmd_.set_val(omega_upper_cmd__);
  }
  torque_cmd_.set_val(ApplyCal(torque_cmd__, &rotor_sensor_params_.torque_cal));
}

// Projects the local apparent wind vector on the rotor axis to
// calculate the apparent wind speed (freestream velocity) used in the
// rotor aerodynamics database.
double RotorBase::CalcVFreestream() const {
  return -Vec3Dot(&local_apparent_wind_b(), &rotor_params_.axis);
}

// Finds the apparent wind magnitude normal to the rotor axis to
// use as the edgewise velocity in the 3D rotor table lookup.
// Also calculates the edgewise flow angle [rad], which is the angle
// of the edgewise flow in the rotor disk plane, positive is
// right-hand about rotor x axis, with zero aligned with rotor -z axis.
// This will be used to convert the rotor aero/prime frame back to rotor
// geometric frame and then back to the kite body frame.
double RotorBase::CalcVEdgewise(Mat3 *dcm_rp2r_local) {
  Vec3 local_apparent_wind_r;
  Mat3Vec3Mult(&rotor_params_.dcm_b2r, &local_apparent_wind_b(),
               &local_apparent_wind_r);
  double edgewise_angle =
      -atan2(-local_apparent_wind_r.y, -local_apparent_wind_r.z);
  // Create direction cosine matrix from rotor prime to rotor geometric axes.
  if (dcm_rp2r_local != nullptr) {
    AngleToDcm(-edgewise_angle, 0.0, 0.0, kRotationOrderXyz, dcm_rp2r_local);
  }
  return hypot(local_apparent_wind_r.y, local_apparent_wind_r.z);
}

// Calculates the aerodynamic torque acting on the propeller.  Since
// torque is not included in the rotor database, we calculate it from
// the aerodynamic power (i.e. mechanical shaft power) and rotor
// angular velocity:
//
//   torque = aero_power / rotor_vel
//
// Torque is defined to be negative during thrusting (regardless of
// propeller direction), so the torque will act opposite the direction
// of propeller rotation.
//
// Below a rotor velocity of 10 rad/s (a somewhat arbitrary cutoff)
// the rotor torque is set to zero, otherwise you can get really large
// torques due to inaccuracies in the rotor database.
double RotorBase::CalcAeroTorque(double rotor_vel, double aero_power__) const {
  if (static_cast<double>(rotor_params_.dir) * rotor_vel < 10.0) {
    return 0.0;
  } else {
    return aero_power__ / rotor_vel;
  }
}

// Calculates the gyroscopic moment vector (gyro_moment), which is the
// cross-product of the angular momentum vector of the rotor (L_rotor)
// and the angular velocity vector of the coordinate system (omega):
//
//   gyro_moment = L_rotor x omega
//               = (I * rotor_vel) x omega
//
// Because the rotor state model is written such that all rotors
// (regardless of which way they spin) have a positive angular rate
// under normal conditions, it is also necessary to multiply the rotor
// model's angular velocity by the rotor direction.
void RotorBase::CalcGyroMoment(double rotor_vel, const Vec3 &wing_omega,
                               Vec3 *gyro_moment) const {
  Vec3 L_rotor;
  Vec3Scale(&rotor_params_.axis, rotor_params_.I * rotor_vel, &L_rotor);
  Vec3Cross(&L_rotor, &wing_omega, gyro_moment);
}

// Calculates the reaction moment vector (react_moment), which is the
// product of moment of inertia (I) and the negative angular
// acceleration vector of the rotors (rotor_acc):
//
//   react_moment = -I * rotor_acc
//
// Because the rotor state model is written such that all rotors
// (regardless of which way they spin) have a positive angular rate
// under normal conditions, it is also necessary to multiply the rotor
// model's angular acceleration by the rotor direction.
void RotorBase::CalcReactMoment(double rotor_acc__, Vec3 *react_moment) const {
  double react_moment_mag = -rotor_params_.I * rotor_acc__;
  Vec3Scale(&rotor_params_.axis, react_moment_mag, react_moment);
}

// We do all the rotor aerodynamic database look-ups during the connection phase
// to avoid calculating these values for both CalcForceMomentPos and for
// CalcDeriv.
void RotorBase::AddInternalConnections(ConnectionStore *connections) {
  connections->Add(3, [this](double /*t*/) {
    v_freestream_.set_val(CalcVFreestream());
    if (rotor_sim_params_.apply_3d_rotor_tables) {
      Mat3 dcm_rp2r_temp;
      v_edgewise_.set_val(CalcVEdgewise(&dcm_rp2r_temp));
      dcm_rp2r_.set_val(dcm_rp2r_temp);
      y_prime_force_.set_val(rotor_database_3d_.CalcForceYPrime(
          AngularSpeed(), v_freestream(), v_edgewise(),
          environment_.air_density(), static_cast<int>(rotor_params_.dir)));
      z_prime_force_.set_val(rotor_database_3d_.CalcForceZPrime(
          AngularSpeed(), v_freestream(), v_edgewise(),
          environment_.air_density()));
      y_prime_mom_.set_val(rotor_database_3d_.CalcMomYPrime(
          AngularSpeed(), v_freestream(), v_edgewise(),
          environment_.air_density()));
      z_prime_mom_.set_val(rotor_database_3d_.CalcMomZPrime(
          AngularSpeed(), v_freestream(), v_edgewise(),
          environment_.air_density(), static_cast<int>(rotor_params_.dir)));
      double thrust_2d = rotor_database_.CalcThrust(
          AngularSpeed(), v_freestream(), environment_.air_density());
      double thrust_scale_factor = rotor_database_3d_.CalcThrustScaleFactor(
          AngularSpeed(), v_freestream(), v_edgewise());
      thrust_.set_val(thrust_2d * thrust_scale_factor);
      double aero_power_2d = rotor_database_.CalcPower(
          AngularSpeed(), v_freestream(), environment_.air_density());
      double torque_scale_factor = rotor_database_3d_.CalcTorqueScaleFactor(
          AngularSpeed(), v_freestream(), v_edgewise());
      aero_power_.set_val(aero_power_2d * torque_scale_factor);
    } else {
      v_edgewise_.set_val(0.0);
      y_prime_force_.set_val(0.0);
      z_prime_force_.set_val(0.0);
      y_prime_mom_.set_val(0.0);
      z_prime_mom_.set_val(0.0);
      dcm_rp2r_.set_val(kMat3Zero);
      thrust_.set_val(rotor_database_.CalcThrust(AngularSpeed(), v_freestream(),
                                                 environment_.air_density()));
      aero_power_.set_val(rotor_database_.CalcPower(
          AngularSpeed(), v_freestream(), environment_.air_density()));
    }
    aero_torque_.set_val(CalcAeroTorque(omega(), aero_power()));
  });
}

Rotor::Rotor(MotorLabel motor_label, const RotorParams &rotor_params,
             const RotorSimParams &rotor_sim_params,
             const RotorSensorParams &rotor_sensor_params,
             const Environment &environment)
    : RotorBase(motor_label, rotor_params, rotor_sim_params,
                rotor_sensor_params, environment, 0.0),
      motor_torque_(new_derived_value(), "motor_torque"),
      omega_(new_continuous_state(), "omega", 0.0) {
  SetupDone();
}

void Rotor::Publish() const {
  RotorBase::Publish();
  sim_telem.rotors[motor_label_].motor_torque = motor_torque();
}

void Rotor::SetMotorTorque(double val) {
  motor_torque_.set_val(val * static_cast<double>(rotor_params_.dir));
}

// The derivative of the angular velocity of the rotor (omega_) is
// simply the angular acceleration, which is calculated in a separate
// function because it is used both to calculate the state derivative
// here and to calculate the reaction torque in the CalcForceMomentPos
// function.
void Rotor::CalcDerivHelper(double /*t*/) { omega_.set_deriv(RotorAcc()); }

// Calculates the rotor's angular acceleration, which is the sum of
// the aerodynamic and motor torques divided by the rotor moment of
// inertia (I).  The rotor moment of inertia includes both the
// propeller moment of inertia (I_prop) and the motor's moment of
// inertia (I_motor).
//
//     .      motor_torque + aero_torque
//   omega = ----------------------------
//                 I_prop + I_motor
//
double Rotor::RotorAcc() const {
  return (motor_torque() + aero_torque()) / rotor_params_.I;
}

HitlRotor::HitlRotor(MotorLabel motor_label, const RotorParams &rotor_params,
                     const RotorSimParams &rotor_sim_params,
                     const RotorSensorParams &rotor_sensor_params,
                     const Environment &environment, double ts)
    : RotorBase(motor_label, rotor_params, rotor_sim_params,
                rotor_sensor_params, environment, ts),
      omega_(new_derived_value(), "omega", 0.0),
      bus_current_(new_derived_value(), "bus_current", 0.0),
      motor_status_flag_(new_derived_value(), "motor_status_flag",
                         kMotorStatusError),
      filtered_rotor_acc_(new_discrete_state(), "filtered_rotor_acc", 0.0),
      rotor_acc_filter_state_(new_discrete_state(), "rotor_acc_filter_state",
                              {0.0, 0.0}) {
  SetupDone();
}

void HitlRotor::Publish() const {
  RotorBase::Publish();
  // Since motor_torque is part of Rotor, not HitlRotor, we have to
  // back it out from the acceleration and aero torque.
  sim_telem.rotors[motor_label_].motor_torque =
      RotorAcc() * rotor_params_.I - aero_torque();
}

void HitlRotor::DiscreteStepHelper(double t) {
  std::vector<double> rotor_acc_filter_state = rotor_acc_filter_state_.val();
  filtered_rotor_acc_.DiscreteUpdate(
      t, DiffLpf2(omega(), rotor_sim_params_.fc_hitl_rotor_acc, 1.0, ts_,
                  rotor_acc_filter_state.data()));
  rotor_acc_filter_state_.DiscreteUpdate(t, rotor_acc_filter_state);
}

double HitlRotor::RotorAcc() const { return filtered_rotor_acc_.val(); }

void HitlRotor::SetFromAvionicsPackets(
    const AvionicsPackets &avionics_packets) {
  CHECK(static_cast<int32_t>(avionics_packets.motor_statuses.size()) ==
        kNumMotors);

  RotorBase::SetFromAvionicsPackets(avionics_packets);

  const MotorStatusMessage &status =
      avionics_packets.motor_statuses[static_cast<int32_t>(motor_label_)];
  omega_.set_val(ApplyCal(status.omega, &rotor_sensor_params_.omega_cal));
  bus_current_.set_val(status.bus_current);
  motor_status_flag_.set_val(status.motor_status);
}

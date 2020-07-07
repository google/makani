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

// The Rotor class models the aerodynamic and inertial effects of the
// propellers and the rotating sections of the motors.  As input, it
// takes a current from the power system, which is converted to a
// motor torque, and the local apparent wind speed, which is used to
// calculate the aerodynamic torque.  The torques are added and used
// to determine the angular acceleration of the rotor, which is
// integrated to get the angular velocity.
//
// The Rotor class also provides a ForceMomentInterface to its parent
// (the wing).  This interface is used to apply the rotor's thrust and
// its aerodynamic, gyroscopic, and reaction torques to the wing at
// the location of the rotor.

#ifndef SIM_MODELS_ACTUATORS_ROTOR_H_
#define SIM_MODELS_ACTUATORS_ROTOR_H_

#include <stdint.h>

#include <string>
#include <vector>

#include "avionics/motor/firmware/flags.h"
#include "common/c_math/force_moment.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/models/actuators/actuator.h"
#include "sim/models/environment.h"
#include "sim/physics/rotor_database.h"
#include "sim/physics/rotor_database_3d.h"
#include "sim/sim_types.h"
#include "system/labels.h"

bool MotorRunCommanded(const ControllerCommandMessage &command_message);

class RotorBase : public Actuator {
 public:
  RotorBase(MotorLabel motor_label, const RotorParams &rotor_params,
            const RotorSimParams &rotor_sim_params,
            const RotorSensorParams &rotor_sensor_params,
            const Environment &environment, double ts);
  ~RotorBase() {}
  double RotorThrustCoeff() const;

  void CalcRotorForceMomentPos(ForceMomentPos *fmx) const;
  void CalcBlownWingForceMomentPos(ForceMomentPos *fmx_bw) const;

  void Publish() const override;

  void set_local_apparent_wind_b(const Vec3 &val) {
    local_apparent_wind_b_.set_val(val);
  }
  void set_parent_omega(const Vec3 &val) { parent_omega_.set_val(val); }

  virtual double omega() const = 0;
  double aero_torque() const { return aero_torque_.val(); }

  // The power system and rotor database take speeds as inputs.  Avoiding fabs
  // will ensure that incorrectly-signed speeds from the controller cause
  // problems.
  // TODO: This supports legacy code but should be updated to
  // match flight behavior.
  double AngularSpeedCmd() const {
    return omega_upper_cmd() * static_cast<double>(rotor_params_.dir);
  }

  double AngularUpperSpeedCmd() const {
    double dir = static_cast<double>(rotor_params_.dir);
    // Upper and lower command limits flip when multiplied by negative numbers.
    if (dir > 0) {
      return omega_upper_cmd() * dir;
    } else {
      return omega_lower_cmd() * dir;
    }
  }

  double AngularLowerSpeedCmd() const {
    double dir = static_cast<double>(rotor_params_.dir);
    // Upper and lower command limits flip when multiplied by negative numbers.
    if (dir > 0) {
      return omega_lower_cmd() * dir;
    } else {
      return omega_upper_cmd() * dir;
    }
  }

  double TorqueCmd() const {
    return torque_cmd() * static_cast<double>(rotor_params_.dir);
  }

  double AngularSpeed() const {
    return omega() * static_cast<double>(rotor_params_.dir);
  }

  // Only HitlRotor's implementation of this method should be accessed.
  virtual double bus_current() const {
    LOG(FATAL) << "This implementation should never be called.";
    return 0.0;
  }

  // Only HitlRotor's implementation of this method should be accessed.
  virtual uint16_t motor_status_flag() const {
    LOG(FATAL) << "This implementation should never be called.";
    return kMotorStatusError;
  }

  virtual void SetMotorTorque(double /*val*/) {
    LOG(FATAL) << "This implementation should never be called.";
  }

  void SetFromAvionicsPackets(const AvionicsPackets &avionics_packets) override;

  // TODO: Fully expanded this to match motor controller.
  double omega_upper_cmd() const { return omega_upper_cmd_.val(); }
  double omega_lower_cmd() const { return omega_lower_cmd_.val(); }
  double torque_cmd() const { return torque_cmd_.val(); }

 protected:
  // Enum labeling rotor (R1, R2, etc...).
  const MotorLabel motor_label_;

  // Contains the rotor direction, axis, position, moment of inertia,
  // diameter, and motor constant.
  const RotorParams &rotor_params_;

  // Contains the file name of the aerodynamic database to use.
  const RotorSimParams &rotor_sim_params_;

  // Contains the rotor sensor parameters used to interpret the
  // ControllerCommandMessage.
  const RotorSensorParams &rotor_sensor_params_;

 private:
  void AddInternalConnections(ConnectionStore *connections) override;

  virtual double RotorAcc() const = 0;

  double CalcVFreestream() const;
  double CalcVEdgewise(Mat3 *dcm_rp2r_local);
  double CalcAeroTorque(double rotor_vel, double aero_power) const;
  void CalcGyroMoment(double rotor_vel, const Vec3 &wing_omega,
                      Vec3 *gyro_moment) const;
  void CalcReactMoment(double rotor_acc, Vec3 *react_moment) const;

  void DiscreteStepHelper(double /*t*/) override {}

  const Vec3 &local_apparent_wind_b() const {
    return local_apparent_wind_b_.val();
  }
  const Vec3 &parent_omega() const { return parent_omega_.val(); }

  double v_freestream() const { return v_freestream_.val(); }
  double thrust() const { return thrust_.val(); }
  double aero_power() const { return aero_power_.val(); }

  double v_edgewise() const { return v_edgewise_.val(); }
  double y_prime_force() const { return y_prime_force_.val(); }
  double z_prime_force() const { return z_prime_force_.val(); }
  double y_prime_mom() const { return y_prime_mom_.val(); }
  double z_prime_mom() const { return z_prime_mom_.val(); }

  Mat3 dcm_rp2r() const { return dcm_rp2r_.val(); }

  // Look-up table for rotor thrust and power as a function of rotor
  // angular rate and the freestream velocity of the airflow.
  const RotorDatabase rotor_database_;

  // Look-up database for all six forces and moments of a rotor (in the rotor
  // prime frame), as a function of axial velocity, edgewise velocity,
  // and rotor angular rate.
  const RotorDatabase3d rotor_database_3d_;

  const Environment &environment_;

  // Actuator command.

  // Commanded speed [rad/s] and torque (Nm) from the controller.
  State<double> omega_upper_cmd_;
  State<double> omega_lower_cmd_;
  State<double> torque_cmd_;

  // Input states.

  // Local apparent wind velocity [m/s], which is used to calculate
  // the freestream velocity for the rotor's aerodynamic databases.
  State<Vec3> local_apparent_wind_b_;

  // Angular rate vector [rad/s] of the parent coordinate system,
  // which is used to calculate the gyroscopic moment.
  State<Vec3> parent_omega_;

  // Derived values.

  // Axial freestream velocity [m/s] of the local incoming airflow parallel
  // to the propeller axis.  Defined to be positive during normal
  // flight (positive along rotor -x direction).  Can be negative in a
  // descent condition.
  State<double> v_freestream_;

  // Edgewise in-plane velocity [m/s] of the local incoming airflow normal
  // to the propeller axis.  It is the norm of the y and z components, so the
  // magnitude is always positive.
  State<double> v_edgewise_;

  // Direction cosine matrix from the rotor prime axes to the rotor geometric
  // axes.  The rotor prime axes are rotated about the rotor geometric x axis
  // such that the rotor prime z axis is aligned with the rotor edgewise flow
  // direction, but opposite sign (positive edgewise flow points in the negative
  // rotor prime z direction).
  State<Mat3> dcm_rp2r_;

  // In-plane (edgewise) forces [N] and moments [Nm] in the rotor prime frame.
  State<double> y_prime_force_;
  State<double> z_prime_force_;
  State<double> y_prime_mom_;
  State<double> z_prime_mom_;

  // Thrust [N] produced by the propeller.
  State<double> thrust_;

  // Power [W] produced by the propeller.  Defined to be positive
  // during generation.
  State<double> aero_power_;

  // Aerodynamic torque [N-m] acting on the propeller.  Defined to be
  // negative during thrusting regardless of propeller direction.
  State<double> aero_torque_;

  DISALLOW_COPY_AND_ASSIGN(RotorBase);
};

// Model of a rotor.
class Rotor : public RotorBase {
 public:
  Rotor(MotorLabel motor_label, const RotorParams &rotor_params,
        const RotorSimParams &rotor_sim_params,
        const RotorSensorParams &rotor_sensor_params,
        const Environment &environment);
  virtual ~Rotor() {}

  void Publish() const final;

  double omega() const override { return omega_.val(); }

  // Used by the power system to set torque.  The power system deals with
  // non-negative omega only; it is up to the rotor model to account for its
  // direction.
  void SetMotorTorque(double val) override;

 private:
  void CalcDerivHelper(double t) override;
  double RotorAcc() const override;

  double motor_torque() const { return motor_torque_.val(); }

  // Input state.

  // Motor torque [N-m] that drives the propeller.
  State<double> motor_torque_;

  // Continuous state.

  // Angular rate [rad/s] of the rotor.
  ContinuousState<double> omega_;

  DISALLOW_COPY_AND_ASSIGN(Rotor);
};

class HitlRotor : public RotorBase {
 public:
  HitlRotor(MotorLabel motor_label, const RotorParams &rotor_params,
            const RotorSimParams &rotor_sim_params,
            const RotorSensorParams &rotor_sensor_params,
            const Environment &environment, double ts);
  virtual ~HitlRotor() {}

  void Publish() const final;

  void SetFromAvionicsPackets(const AvionicsPackets &avionics_packets) override;

  double omega() const override { return omega_.val(); }

  double bus_current() const override { return bus_current_.val(); }

  uint16_t motor_status_flag() const override {
    return motor_status_flag_.val();
  }

 private:
  void DiscreteStepHelper(double t) override;
  double RotorAcc() const override;

  // Angular rate [rad/s] of the rotor.
  State<double> omega_;

  // Bus current [A] of the motor.
  State<double> bus_current_;

  // Status flag of the motor (Running, Winddown, Error etc.).
  State<uint16_t> motor_status_flag_;

  // Angular acceleration [rad/s^2] of the rotors and state of the filter used
  // to estimate it.
  DiscreteState<double> filtered_rotor_acc_;
  DiscreteState<std::vector<double>> rotor_acc_filter_state_;

  DISALLOW_COPY_AND_ASSIGN(HitlRotor);
};

#endif  // SIM_MODELS_ACTUATORS_ROTOR_H_

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

#ifndef SIM_MODELS_SIMPLE_POWER_SYS_H_
#define SIM_MODELS_SIMPLE_POWER_SYS_H_

#include <stdint.h>

#include <memory>
#include <vector>

#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/rotor.h"
#include "sim/models/power_sys.h"
#include "sim/sim_messages.h"
#include "sim/sim_types.h"
#include "system/labels.h"

// The goal of the simple power system is to be simple and to make
// debugging controls problems as easy as possible!  It has no motor
// controller dynamics and simply applies a torque proportional to the
// angular rate error:
//
//   motor_torque = kp * (omega_cmd - omega)
//
// There are no stacking dynamics or limitations from the power
// system.
class SimplePowerSys : public PowerSys {
 public:
  SimplePowerSys(const std::vector<std::unique_ptr<RotorBase>> &rotors,
                 const RotorSensorParams (&rotor_sensor_params)[kNumMotors],
                 const PowerSysParams &power_sys_params,
                 const PowerSysSimParams &power_sys_sim_params,
                 FaultSchedule *faults);
  ~SimplePowerSys() {}

  double motor_torques(MotorLabel i) const override {
    return motor_torques_[i].val();
  }

 private:
  // Maximum magnitude of the motor velocity integrator error [rad]. Omitted
  // from the config system since this is an artificial parameter in an
  // artifical model.
  //
  // This value is based on a realized min of about -3 rad when running a
  // default sim for 600s.
  static constexpr double kMotorVelIntErrorMax = 6.0;

  void AddInternalConnections(ConnectionStore *connections) override;
  void UpdateDerivedStates();
  void CalcDerivHelper(double /*t*/) override;

  double CalcWingVoltage(double tether_current__) const;
  double CalcTetherCurrent(double wing_electrical_power) const;

  // Vector of motor torques [N-m] output by the motor controllers.
  std::vector<State<double>> motor_torques_;

  DISALLOW_COPY_AND_ASSIGN(SimplePowerSys);
};

#endif  // SIM_MODELS_SIMPLE_POWER_SYS_H_

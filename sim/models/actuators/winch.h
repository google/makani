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

// The Winch class models the dynamics of the servo that is connected
// to the winch drum.  Importantly, the interactions between the winch
// drum and the perch are modelled in the Perch class.  Also, the
// values used in this class are all angles of the winch drum itself,
// after the transmission ratio has been applied, rather than angles
// of the winch servo.

#ifndef SIM_MODELS_ACTUATORS_WINCH_H_
#define SIM_MODELS_ACTUATORS_WINCH_H_

#include <stdint.h>

#include <vector>

#include "common/macros.h"
#include "control/system_types.h"
#include "sim/models/actuators/actuator.h"
#include "sim/sim_messages.h"
#include "sim/state.h"

class Winch : public Actuator {
 public:
  Winch(const WinchParams &winch_params,
        const WinchSimParams &winch_sim_params);
  ~Winch() {}

  // Sets the initial commanded winch drum angle.  This should be the
  // same as the actual winch drum angle (see perch.cc) if you want
  // small winch forces at the start.  This should be called before
  // beginning the simulation.
  //
  // Args:
  //   theta_winch_drum: Initial commanded winch drum angle [rad].
  //       This is the angle of the drum itself and not of the winch
  //       servo.
  void Init(double theta_winch_drum);

  void Publish() const override;
  void SetFromAvionicsPackets(const AvionicsPackets &avionics_packets) override;

  // Calculates the torque applied by the winch controller.  This
  // models with winch controller as a simple PD controller with a
  // saturation on the maximum torque.
  //
  // Args:
  //   theta_winch: Actual (as opposed to commanded) winch drum angle [rad].
  //   omega_winch: Actual (as opposed to commanded) winch drum angular
  //       rate [rad/s].
  //
  // Returns:
  //   Torque [N-m] acting on the winch drum, i.e. after the
  //   transmission ratio.  A positive torque results in a rotation of
  //   the winch drum about the positive z-axis.
  virtual double CalcTorque(double theta_winch, double omega_winch) const;

 protected:
  double theta_winch_drum_cmd() const { return theta_winch_drum_cmd_.val(); }
  double omega_winch_drum_cmd() const { return omega_winch_drum_cmd_.val(); }

  // Winch parameters.
  const WinchParams &winch_params_;
  const WinchSimParams &winch_sim_params_;

 private:
  void CalcDerivHelper(double t) override;
  void DiscreteStepHelper(double /*t*/) override {}

  // Continuous state.

  // As viewed by the wing controller, the lower-level winch control
  // is part of the plant.  Therefore, the winch controller's state is
  // included here.
  //
  // Commanded angle [rad] of the winch drum.  This is defined to be
  // 0.0 in the crosswind position and increases as the payed tether
  // length increases.  This is the angle of the drum itself and not
  // of the winch servo.
  ContinuousState<double> theta_winch_drum_cmd_;

  // Actuator commands.

  // Commanded angular rate [rad/s] for the winch drum.  This is the
  // angular rate of the drum itself and not of the winch servo.
  State<double> omega_winch_drum_cmd_;

  DISALLOW_COPY_AND_ASSIGN(Winch);
};

#endif  // SIM_MODELS_ACTUATORS_WINCH_H_

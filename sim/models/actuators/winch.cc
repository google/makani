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

#include "sim/models/actuators/winch.h"

#include <glog/logging.h>

#include <vector>

#include "common/c_math/util.h"
#include "common/c_math/voting.h"
#include "control/system_types.h"
#include "sim/sim_telemetry.h"
#include "sim/state.h"

Winch::Winch(const WinchParams &winch_params,
             const WinchSimParams &winch_sim_params)
    : Actuator("Winch"),
      winch_params_(winch_params),
      winch_sim_params_(winch_sim_params),
      theta_winch_drum_cmd_(new_continuous_state(), "theta_winch_drum_cmd",
                            0.0),
      omega_winch_drum_cmd_(new_derived_value(), "omega_winch_drum_cmd") {
  SetupDone();
}

void Winch::Init(double theta_winch_drum) {
  // This must be cleared here, rather than having an uninitialized
  // state in the constructor, because this Init function is called
  // multiple times.
  theta_winch_drum_cmd_.Clear();
  theta_winch_drum_cmd_.set_val(theta_winch_drum);
}

void Winch::Publish() const {
  sim_telem.winch.theta_cmd = theta_winch_drum_cmd();
  sim_telem.winch.omega_cmd = omega_winch_drum_cmd();
}

double Winch::CalcTorque(double theta_winch, double omega_winch) const {
  return Saturate(
      winch_sim_params_.k_theta * (theta_winch_drum_cmd() - theta_winch) +
          winch_sim_params_.k_omega * (omega_winch_drum_cmd() - omega_winch),
      -winch_sim_params_.max_torque, winch_sim_params_.max_torque);
}

void Winch::CalcDerivHelper(double /*t*/) {
  theta_winch_drum_cmd_.set_deriv(omega_winch_drum_cmd());
}

void Winch::SetFromAvionicsPackets(const AvionicsPackets &avionics_packets) {
  double winch_velocity_cmd = avionics_packets.command_message.winch_velocity;

  // The winch velocity command from the controller is in rad/s at the
  // winch drum, but this is "calibrated" to m/s because the
  // controller uses m/s internally.  Here, we apply the calibration
  // and then convert the calibrated signal back to rad/s.
  omega_winch_drum_cmd_.set_val(
      ApplyCal(winch_velocity_cmd, &winch_params_.velocity_cmd_cal) /
      winch_params_.r_drum);
}

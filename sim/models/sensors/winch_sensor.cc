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

#include "sim/models/sensors/winch_sensor.h"

#include <math.h>
#include <string.h>

#include <vector>

#include "avionics/common/tether_message_types.h"
#include "control/system_types.h"
#include "sim/models/actuators/winch.h"
#include "sim/models/perch.h"
#include "sim/sim_messages.h"

WinchSensor::WinchSensor(const WinchParams &winch_params)
    : Sensor("Winch Sensor"),
      winch_params_(winch_params),
      winch_torque_(new_input_value(), "winch_torque"),
      theta_winch_(new_input_value(), "theta_winch"),
      omega_winch_(new_input_value(), "omega_winch"),
      control_torque_(new_discrete_state(), "control_torque"),
      omega_winch_servo_(new_discrete_state(), "omega_winch_servo"),
      theta_winch_servo_(new_discrete_state(), "theta_winch_servo") {
  SetupDone();
}

void WinchSensor::DiscreteStepHelper(double t) {
  control_torque_.DiscreteUpdate(t, winch_torque());
  omega_winch_servo_.DiscreteUpdate(
      t, winch_params_.transmission_ratio * omega_winch());
  theta_winch_servo_.DiscreteUpdate(
      t, winch_params_.transmission_ratio * theta_winch());
}

void WinchSensor::UpdateSensorOutputs(SimSensorMessage * /*sensor_message*/,
                                      TetherUpMessage *tether_up) const {
  TetherPlc &plc = tether_up->plc;

  // TODO: Handle PLC sequence numbers and no_update_count.
  // plc.sequence also incremented by proximity_sensor.
  plc.sequence =
      static_cast<uint16_t>((plc.sequence + 1U) % TETHER_SEQUENCE_ROLLOVER);
  plc.no_update_count = 0;

  // Update the winch structure.
  plc.drum_angle =
      static_cast<float>(InvertCal(theta_winch_servo() * winch_params_.r_drum /
                                       winch_params_.transmission_ratio,
                                   &winch_params_.position_cal));
  plc.flags &= static_cast<uint8_t>(~kTetherPlcFlagDrumFault);
}

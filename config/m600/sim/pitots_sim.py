# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Simulated pitot parameters."""

import copy

from makani.config import mconfig
from makani.control import system_types
from makani.lib.python import units
import numpy as np


def PressureSensorParams(p_low, p_high, noise_level):
  # The HSC models we use map [p_low, p_high] linearly to [10%, 90%]
  # of the saturating range. See Figure 2 of https://goo.gl/yUZET0.
  return {
      'bias': 0.0,
      'scale': 1.0,
      'noise_level': noise_level,
      'bound_low': p_low * 1.25,
      'bound_high': p_high * 1.25,
      'quantization': 1.25 * (p_high - p_low) / float(2**14)
  }


@mconfig.Config(deps={
    'common_params': 'common.common_params',
    'pitot': 'm600.pitot',
    'rotors': mconfig.WING_MODEL + '.rotors'
})
def MakeParams(params):
  # Modify the apparent wind at the pitot to account for inflow from
  # the rotors.
  include_rotor_inflow = True

  # Calculate the total rotor area [m^2], which is used to estimate
  # the inflow velocity at the pitot tube.
  total_rotor_area = sum([np.pi * (r['D'] / 2.0)**2.0
                          for r in params['rotors']])

  # Ratio [#] of the induced velocity at the pitot to the induced
  # velocity at the rotor disk.  This was determined based on CFD
  # simulation (see "Pitot Tube Measurement Interaction with Rotors"
  # in 09_Controls/Sensors/Pitot/ in Drive).
  induced_vel_at_pitot_fraction = 0.3

  # Confirm that pitot is in the position where the
  # induced_vel_at_pitot_fraction was determined.
  assert params['pitot']['pos'] == [3.213, 0.0, 0.443]

  # Offset to the local pressure coefficient [#] at the center
  # (total pressure) port.
  local_pressure_coeff_offset = 0.0

  # The quantization and noise levels indicated here were determined
  # from test data on 2016-05-06.
  sensor_params_006mdsa = PressureSensorParams(
      -600.0, 600.0, noise_level=0.2)
  sensor_params_001pdsa = PressureSensorParams(
      units.PsiToPa(-1.0), units.PsiToPa(1.0), noise_level=2.5)
  sensor_params_015pasa = PressureSensorParams(
      0.0, units.PsiToPa(15.0), noise_level=20.0)

  pitots_sim = [None for _ in range(system_types.kNumPitotSensors)]

  pitots_sim[system_types.kPitotSensorHighSpeed] = {
      'include_rotor_inflow': include_rotor_inflow,
      'total_rotor_area': total_rotor_area,
      'induced_vel_at_pitot_fraction': induced_vel_at_pitot_fraction,
      'local_pressure_coeff_offset': local_pressure_coeff_offset,
      'rotor_axis': params['rotors'][0]['axis'],
      'ts': params['common_params']['ts'],
      'pitch_offset': 0.0,
      'yaw_offset': 0.0,
      'stat_sensor': copy.deepcopy(sensor_params_015pasa),
      'alpha_sensor': copy.deepcopy(sensor_params_001pdsa),
      'beta_sensor': copy.deepcopy(sensor_params_001pdsa),
      'dyn_sensor': copy.deepcopy(sensor_params_001pdsa),
  }

  pitots_sim[system_types.kPitotSensorLowSpeed] = {
      'include_rotor_inflow': include_rotor_inflow,
      'total_rotor_area': total_rotor_area,
      'induced_vel_at_pitot_fraction': induced_vel_at_pitot_fraction,
      'local_pressure_coeff_offset': local_pressure_coeff_offset,
      'rotor_axis': params['rotors'][0]['axis'],
      'ts': params['common_params']['ts'],
      'pitch_offset': 0.0,
      'yaw_offset': 0.0,
      'stat_sensor': copy.deepcopy(sensor_params_015pasa),
      'alpha_sensor': copy.deepcopy(sensor_params_006mdsa),
      'beta_sensor': copy.deepcopy(sensor_params_006mdsa),
      'dyn_sensor': copy.deepcopy(sensor_params_006mdsa),
  }

  return pitots_sim

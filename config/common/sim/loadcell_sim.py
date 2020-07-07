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

"""Simulated loacell parameters."""

import os

import makani
from makani.avionics.firmware.params import codec
from makani.config import mconfig
from makani.system import labels


@mconfig.Config(deps={
    'common_params': 'common.common_params'
})
def MakeParams(params):
  """Get param value for simulator."""
  # Pin loadcell capacity [N] reported on the datasheet.
  capacity = 300055.0

  calib_file = os.path.join(makani.HOME,
                            'avionics/loadcell/firmware/calib_params.yaml')
  port_params = codec.DecodeYamlFile(calib_file, 'port_sn1')
  starboard_params = codec.DecodeYamlFile(calib_file, 'starboard_sn1')

  newtons_per_count = [None] * labels.kNumLoadcellSensors
  newtons_per_count[labels.kLoadcellSensorPort0] = (
      port_params.pin_calib.strain_0_scale)
  newtons_per_count[labels.kLoadcellSensorPort1] = (
      port_params.pin_calib.strain_1_scale)
  newtons_per_count[labels.kLoadcellSensorStarboard0] = (
      starboard_params.pin_calib.strain_0_scale)
  newtons_per_count[labels.kLoadcellSensorStarboard1] = (
      starboard_params.pin_calib.strain_1_scale)

  # Representative biases [N] that could occur from mis-calibration or
  # drift with temperature.
  #
  # TODO: These were chosen in a relatively ad-hoc
  # manner.  We should do a more careful analysis of typical loadcell
  # biases.
  biases = [200.0, -600.0, -500.0, 300.0]

  sensors = []
  for i in range(labels.kNumLoadcellSensors):
    sensors.append({
        'bias': biases[i],
        'scale': 1.0,
        'noise_level': 50.0,
        'bound_low': -capacity,
        'bound_high': capacity,
        'quantization': newtons_per_count[i]
    })

  return {
      'ts': params['common_params']['ts'],
      'sensors': sensors
  }

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

"""Simulated GSG sensor parameters."""
from makani.config import mconfig
from makani.config.sensor_util import MakeEncoderParams
from makani.control import system_types
import numpy as np


@mconfig.Config(deps={
    'common_params': 'common.common_params',
    'gs_model': 'base_station.gs_model',
})
def MakeParams(params):
  """Make ground station sensor parameters."""
  # The tophat has one perch azimuth encoder; GSv1 has two.
  gsg_azi_enabled = [
      True, params['gs_model'] == system_types.kGroundStationModelGSv1]

  return {
      # Sensor parameters for GSG encoders.  The biases of one or two
      # degrees are estimated typical biases.  The noise level is
      # chosen to be pessimistic but not completely unrealistic.
      'ts': params['common_params']['ts'],
      'gsg_azi_sensor': [
          MakeEncoderParams(bias=np.deg2rad(1.0), noise_level_counts=0.25,
                            scale=1.0 if gsg_azi_enabled[0] else 0.0),
          MakeEncoderParams(bias=np.deg2rad(2.0), noise_level_counts=0.25,
                            scale=1.0 if gsg_azi_enabled[1] else 0.0)
      ],
      'gsg_ele_sensor': [
          MakeEncoderParams(bias=np.deg2rad(-1.0), noise_level_counts=0.25),
          MakeEncoderParams(bias=np.deg2rad(2.0), noise_level_counts=0.25)],
      'gsg_twist_sensor': [
          MakeEncoderParams(bias=np.deg2rad(-2.0), noise_level_counts=0.25),
          MakeEncoderParams(bias=0.0, noise_level_counts=0.25)],
  }

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

"""Ground-side gimbal parameters."""
from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      'ele_axis_z_g': -0.160,
      'ele_axis_horiz_offset_g': 0.120,

      # Encoder calibration moved to avionics/drum/firmware/config_params.py.
      # Here we specify an identity operation.
      'azi_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
      'ele_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
  }

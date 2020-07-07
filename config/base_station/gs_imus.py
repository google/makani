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

"""Ground station IMU calibration."""

from makani.config import mconfig
from makani.control import system_types


@mconfig.Config
def MakeParams():
  # TODO: Set magnetometer calibration values.
  mag_cal_a = [
      {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
      {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
      {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
  ]

  # TODO: Update IMU position, orientation and calibration.
  return [
      # IMU A.
      {
          # Parent coordinate system.
          'parent_cs': system_types.kCoordinateSystemVessel,

          # Position [m] of the IMU in parent (vessel) coordinates.  See b/130024247.
          'pos': [0.925, -1.120, -0.390],

          # Direction cosine matrix that takes parent (vessel) coordinates to
          # IMU coordinates.
          'dcm_parent2m': {'d': [[-1.0, 0.0, 0.0],
                                 [0.0, -1.0, 0.0],
                                 [0.0, 0.0, 1.0]]},

          # Calibration parameters (e.g. scales and biases) for the IMU
          # measurements.
          'acc_cal': [
              {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
              {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
              {'scale': 1.0, 'bias': 0.0, 'bias_count': 0}
          ],
          'gyro_cal': [
              {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
              {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
              {'scale': 1.0, 'bias': 0.0, 'bias_count': 0}
          ],
          'mag_cal': mag_cal_a,
          'pressure_cal': {'scale': 1e5, 'bias': 0.0, 'bias_count': 0}
      }
  ]

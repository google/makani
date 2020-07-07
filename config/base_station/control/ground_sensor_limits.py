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

"""Sensor limits."""

from makani.config import mconfig
from makani.control import control_types as m
import numpy as np


@mconfig.Config(deps={'phys': 'common.physical_constants'})
def MakeParams(params):
  """Make ground sensor limit parameters."""
  g = params['phys']['g']

  sensor_max = {
      'gs_gps': {
          # Maximum value for bool new_data flag.
          'new_data': True,

          # Maximum GPS time of week [ms] (mod one week).
          'time_of_week_ms': 24 * 7 * 3600 * 1000 - 1,

          # Maximum GPS position [m] and velocity [m/s] in ECEF.
          'pos': [7e6, 7e6, 7e6],
          'vel': [80.0, 80.0, 80.0],

          # Maximum GPS position [m] and velocity [m/s] uncertainty
          # standard deviation.
          'pos_sigma': [10.0, 10.0, 10.0],
          'vel_sigma': [3.0, 3.0, 3.0],

          # Maximum value for solution types.
          'pos_sol_type': m.kGpsSolutionTypeUnsupported,
          'vel_sol_type': m.kGpsSolutionTypeUnsupported
      },
      'imu': {
          'acc': [50.0 * g, 50.0 * g, 50.0 * g],
          'gyro': [np.deg2rad(600.0),
                   np.deg2rad(600.0),
                   np.deg2rad(600.0)],
          'mag': [2.0, 2.0, 2.0]
      },
      'gps_compass': {
          'new_data': True,
          'heading': 2.0 * np.pi,
          'heading_sigma': np.pi,
          'heading_rate': np.pi,
          'pitch': np.pi / 2.0,
          'pitch_sigma': np.pi,
          'pitch_rate': np.pi,
          'angle_sol_type': m.kGpsSolutionTypeUnsupported,
          'rate_sol_type': m.kGpsSolutionTypeUnsupported,
      }
  }

  sensor_min = {
      'gs_gps': {
          # Minimum value for bool new_data flag.
          'new_data': False,

          # Minimum GPS time of week [ms].
          'time_of_week_ms': 0,

          # Minimum GPS position [m] and velocity [m/s] in ECEF.
          'pos': [-7e6, -7e6, -7e6],
          'vel': [-80.0, -80.0, -80.0],

          # Minimum GPS position [m] and velocity [m/s] uncertainty
          # standard deviation.
          'pos_sigma': [0.0, 0.0, 0.0],
          'vel_sigma': [0.0, 0.0, 0.0],

          # Minimum value for solution types.
          'pos_sol_type': m.kGpsSolutionTypeNone,
          'vel_sol_type': m.kGpsSolutionTypeNone
      },

      # IMU minimum high-G accelerometer accelerations [m/s^2], gyro
      # angular rates [rad/s], magnetometer measurements [G], and low-G
      # accelerometer accelerations [m/s^2].
      'imu': {
          'acc': [-50.0 * g, -50.0 * g, -50.0 * g],
          'gyro': [np.deg2rad(-600.0),
                   np.deg2rad(-600.0),
                   np.deg2rad(-600.0)],
          'mag': [-2.0, -2.0, -2.0]
      },
      'gps_compass': {
          'new_data': False,
          'heading': 0.0,
          'heading_sigma': 0.0,
          'heading_rate': -np.pi,
          'pitch': -np.pi / 2.0,
          'pitch_sigma': 0.0,
          'pitch_rate': -np.pi,
          'angle_sol_type': m.kGpsSolutionTypeNone,
          'rate_sol_type': m.kGpsSolutionTypeNone,
      }
  }

  return {
      'max': sensor_max,
      'min': sensor_min,
  }

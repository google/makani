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

"""Sensor limits and standard deviation checks."""
from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      # Normal limits for the high-G accelerometer [G] when
      # stationary.
      'acc': {
          'high_diff': 0.15,
          'magnitude': {'low': 0.95, 'high': 1.15},
          'std': {'low': 0.005, 'high': 0.05}
      },

      # Normal limits for the low-G accelerometer [G] when stationary.
      'acc_lowg': {
          'magnitude': {'low': 0.97, 'high': 1.03},
          'std': {'low': 0.00, 'high': 0.02}
      },

      # Normal limits for the rate gyro [deg/s] when stationary.
      'gyro': {
          'mean': {'low': -3.0, 'high': 3.0},
          'std': {'low': 0.05, 'high': 0.5}
      },

      # Normal limits for the magnetometer [Gauss] when stationary.
      'mag': {
          'magnitude': {'low': 0.40, 'high': 0.54},
          'std': {'low': 0.0001, 'high': 0.003},
          'kiteloft': [{'low': 0.16, 'high': 0.23},
                       {'low': 0.28, 'high': 0.35},
                       {'low': -0.06, 'high': -0.04}]
      },

      # Normal limits for the loadcells [N] when stationary.
      'loadcell': {
          'outboard_mean': {'low': 5500.0, 'high': 8500.0},
          'center_mean': {'low': 21000.0, 'high': 23000.0},
          'std': {'low': 0.5, 'high': 20.0}
      },

      # Normal limits for the pitot tube pressures [Pa] when
      # stationary.
      'pitot': {
          'high_std': 20.0,
          'static_mean': {'low': 14000.0, 'high': 15000.0},
          'diff_mean': {'low': 8100.0, 'high': 8400.0}
      },

      # Normal limits for the GPS when stationary.
      'gps': {
          # Latitude-longitude limits [deg].
          'north_of_sherman': 38.0,
          'east_of_sherman': 121.0,
          'south_of_cloverdale': 37.0,
          'west_of_cloverdale': 122.4,

          # Altitude limits [m].
          'alt_offset': {'low': -20.0, 'high': 20.0},
          'sigmas': {'low': 0.0, 'high': 7.0}
      }
  }

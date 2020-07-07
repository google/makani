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

"""Environment monitoring parameters."""
import numpy as np
from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      # Limits for the wind speed [m/s].
      'wind': {
          'very_low': np.finfo('d').min,
          'low': 4.0,
          'high': 12.0,
          'very_high': np.finfo('d').max
      },

      # Limits for the wind gusts [m/s].
      'wind_gust': {
          'very_low': np.finfo('d').min,
          'low': np.finfo('d').min,
          'high': 0.4,
          'very_high': np.finfo('d').max
      },

      # Limits for the wind direction [deg].
      'rel_direction': {
          'very_low': np.finfo('d').min,
          'low': np.finfo('d').min,
          'high': 50.0,
          'very_high': np.finfo('d').max
      }
  }

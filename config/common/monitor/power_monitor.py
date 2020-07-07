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

"""Power system monitoring parameters."""
from makani.config import mconfig
import numpy as np


@mconfig.Config(deps={'power_sys': 'powertrain.power_sys'})
def MakeParams(params):
  return {
      # Resistance [Ohm] of the battery box.
      'R_source': params['power_sys']['R_source'],

      # Number of dropped packets [#] allowed before the battery box
      # communications are considered slow.
      'slow_bbox': 50,

      # Limits for the high voltage bus for Ozone controllers [V].
      'v_bus': {
          'very_low': 900.0,
          'low': 950.0,
          'high': 1150.0,
          'very_high': 1200.0
      },

      # Limits for the Ozone primary LV-12V converter output voltage [V].
      'v_12v_pri': {
          'very_low': 8.6,
          'low': 10.6,
          'high': 14.6,
          'very_high': 16.6
      },

      # Limits for the Ozone auxiliary LV-12V converter output voltage [V].
      'v_12v_aux': {
          'very_low': 8.0,
          'low': 10.0,
          'high': 14.0,
          'very_high': 16.0
      },

      # Limits for the 380 volt bus [V].
      'v_380': {
          'very_low': 160.0,
          'low': np.finfo('d').min,
          'high': np.finfo('d').max,
          'very_high': np.finfo('d').max
      },

      # Limits for the state-of-charge [V] of the battery box.
      'state_of_charge': {
          'very_low': np.finfo('d').min,
          'low': 1040.0,
          'high': np.finfo('d').max,
          'very_high': np.finfo('d').max
      }
  }

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

"""Avionics monitoring parameters."""
from makani.config import mconfig
import numpy as np


@mconfig.Config(deps={
    'servo_voltage': 'm600.monitor.servo_voltage'
})
def MakeParams(params):
  return {
      # Limits for the 48 volt bus voltage [V].
      'v_48': {
          'very_low': 40.0,
          'low': 45.0,
          'high': np.finfo('d').max,
          'very_high': np.finfo('d').max
      },

      # Limits for the avionics voltage [V].
      'v_avionics': {
          'very_low': 8.5,
          'low': 10.8,
          'high': np.finfo('d').max,
          'very_high': np.finfo('d').max
      },

      # Limits for the servo voltage [V].
      'v_servo': {
          'very_low': np.finfo('d').min,
          'low': params['servo_voltage'] * 11.0/12.0,
          'high': np.finfo('d').max,
          'very_high': np.finfo('d').max
      },

      # Limits for the tether release voltage [V].
      'v_release': {
          'very_low': np.finfo('d').min,
          'low': 15.0,
          'high': np.finfo('d').max,
          'very_high': np.finfo('d').max
      }
  }

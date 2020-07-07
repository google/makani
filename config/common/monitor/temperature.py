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

"""Temperature monitoring parameters."""
import numpy as np
from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      # Limits for the pitot temperature [#].
      'hot_pitot': 975,
      'very_hot_pitot': 1005,

      # Limits for the motor temperatures [C].
      'rotors_temp': {
          'very_low': np.finfo('d').min,
          'low': np.finfo('d').min,
          'high': 140.0,
          'very_high': 150.0
      },

      # Limits for the motor drive temperatures [C].
      'drives_temp': {
          'very_low': np.finfo('d').min,
          'low': np.finfo('d').min,
          'high': 60.0,
          'very_high': 85.0
      },

      # Limits for the servo temperatures [C].
      'servos_temp': {
          'very_low': np.finfo('d').min,
          'low': np.finfo('d').min,
          'high': 55.0,
          'very_high': 70.0
      }
  }

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

"""Tether monitoring parameters."""
import numpy as np
from makani.config import mconfig


@mconfig.Config(deps={
    'tether': mconfig.WING_MODEL + '.tether',
})
def MakeParams(params):
  # Proof load tension [N].
  proof_load = 263e3  # Performed on 2017-07-19 (b/63405679).

  # Safety factors for warnings and errors [#]. Never-exceed
  # limits are divided by these factors to get the warning and
  # error thresholds.

  # NOTE: These safety factors were fudged to achieve the desired
  # limits in b/64933954.
  yellow_line_safety_factor = 1.34
  red_line_safety_factor = 1.12

  assert yellow_line_safety_factor >= red_line_safety_factor

  return {
      # Tether proof load [N].
      'proof_load': proof_load,

      # Limits for the tether tension [N].
      'tension': {
          'very_low': np.finfo('d').min,
          'low': 0.0,
          'high': round(proof_load / yellow_line_safety_factor),
          'very_high': round(proof_load / red_line_safety_factor),
      },

      # Limits for tension in crosswind [N].
      'tension_crosswind': {
          'very_low': 10e3,
          'low': 15e3,
          'high': round(proof_load / yellow_line_safety_factor),
          'very_high': round(proof_load / red_line_safety_factor),
      },

      # Limits for tension in hover [N].
      'tension_hover': {
          'very_low': 1e3,
          'low': 2e3,
          'high': 17e3,
          'very_high': 24e3,
      },

      # Nominal tether sphere radius [m].
      'tether_sphere': {
          # According to measurements during RPX-08 and RPX-09, the nominal
          # crosswind tether sphere radius is approximately 8.2 meters greater
          # than the tether length. The approximation is valid for tether
          # lengths close to 425 meters. It should be revised for a
          # significantly different tether length (see b/78361234 for details).
          'radius_nom': params['tether']['length'] + 8.2,
          'deviation': {
              'very_low': -5.0,
              'low': -2.5,
              'high': np.finfo('d').max,
              'very_high': np.finfo('d').max,
          },
          'history_len': 4,
          'latch_duration_sec': 60.0,
      },
  }

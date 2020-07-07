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

"""Simulated tether parameters."""
from makani.config import mconfig
from makani.config import physics_util
import numpy as np


@mconfig.Config(deps={
    'phys': 'common.physical_constants',
    'tether': mconfig.WING_MODEL + '.tether',
})
def MakeParams(params):
  num_nodes = int(np.round(params['tether']['length'] / 22.0))

  if num_nodes > 0:
    # Mass [kg] of an individual tether node.
    node_mass = (params['tether']['linear_density'] * params['tether']['length']
                 / float(num_nodes))

    # Steady-state distance [m] to which a tether node will penetrate the
    # ground.
    ground_penetration = 0.01

    # Spring constant [N/m] for ground contact.
    ground_spring_const = node_mass * params['phys']['g'] / ground_penetration

    # Damping coefficient [N/(m/s)] for ground contact.
    # Damping ratio chosen somewhat arbitrarily - trying to target significant
    # damping to get visual sim results to be closer to what we would
    # expect real ground contact to do.
    ground_damping_coeff = physics_util.ConvertDampingRatioToCoeff(
        ground_spring_const, node_mass, 0.5)
  else:
    # These values are unused if there is no tether.
    ground_spring_const = 0.0
    ground_damping_coeff = 0.0

  return {
      # The number [#] of tether nodes not including the tether
      # endpoints.
      'num_nodes': num_nodes,

      # The smallest length [m] that a segment of tether is allowed to
      # be (other than the final segment on reel-in).
      'stiff_len_lim': 1.0,

      # Longitudinal damping ratio [-], for active and staged tether segments.
      'longitudinal_damping_ratio_active': 0.1,
      'longitudinal_damping_ratio_staged': 0.14,

      # Bending damping ratio [-], for active and staged tether segments.
      'bending_damping_ratio_active': 1.0,

      'ground_contactor_template': {
          'pos': [0.0, 0.0, 0.0],  # Placeholder; specified during simulation.
          'spring_const': ground_spring_const,
          'damping_coeff': ground_damping_coeff,
          'friction_coeff': 0.8,
      }
  }

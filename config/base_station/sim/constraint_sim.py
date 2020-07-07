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

"""Constraint system parameters."""
from makani.analysis.control import geometry
from makani.config import mconfig
from makani.control import system_types
import numpy as np


@mconfig.Config(deps={
    'flight_plan': 'common.flight_plan',
    'ground_station': 'base_station.ground_station',
    'phys': 'common.physical_constants',
    'wing': mconfig.WING_MODEL + '.wing',
    'wing_sim': 'common.sim.wing_sim'
})
def MakeParams(params):
  """Make constraint sim parameters."""
  anchor_pos_g = [-0.99, 14.90, -16.02]

  # Calculate the wing attachment point's initial position to get the initial
  # length.
  dcm_g2b = geometry.QuatToDcm(np.matrix(params['wing_sim']['q_0']).T)
  proboscis_pos_g = np.matrix(params['wing_sim']['Xg_0']).T + (
      dcm_g2b.T * np.matrix(params['wing']['proboscis_pos']).T)

  initial_length = np.linalg.norm(np.matrix(anchor_pos_g).T
                                  - proboscis_pos_g) + 0.05

  if params['flight_plan'] in [
      system_types.kFlightPlanHoverInPlace,
      system_types.kFlightPlanDisengageEngage]:
    tension_command = params['wing']['m'] * params['phys']['g'] * 1.5
  else:
    tension_command = 0.0

  return {
      # Constraint attachment point [m] in ground-station coordinates.
      'anchor_pos_g': anchor_pos_g,

      # Initial constraint length [m].
      'initial_length': initial_length,

      # Maximum constraint length [m].
      'maximum_length': 30.0,

      # Maximum rate of change in the constraint length [m/s].
      'maximum_rate': 1.0,

      # Desired length of slack [m] in the constraint line.
      'slack_command': 1.0,

      # Desired tension [N] if the motors turn off.  A value less than
      # or equal to zero indicates that the constraint length should
      # simply stop being adjusted if the motors are off.
      'tension_command': tension_command,

      # Frequency cutoff [Hz] for the pilot's response time.
      'fc_pilot_response': 1.0,

      # Spring constant [N/m] calculated for a 50 [m] Dyneema tether based on a
      # datasheet, which gives a 0.46% elongation at 10% of the break strength
      # of approximately 211,000 [N].
      'spring_constant': 92000.0
  }

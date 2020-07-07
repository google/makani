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

"""Geometric and inertial properties of the wing."""

from makani.config import mconfig
from makani.control import system_types as m
import numpy as np


@mconfig.Config(deps={
    'rotors': 'oktoberkite.rotors',
})
# Parameters must be updated separately in the matlab scripts in
# analysis/control/generate_hover_controllers.m and
# analysis/control/generate_crosswind_controllers.m.
def MakeParams(params):
  """Returns paramaters about the kite."""

  # Values are specified in several locations.
  # Link to airframe design spec:
  # https://docs.google.com/spreadsheets/d/1lZ7u6ad5D2tKY0jXfHQJbl4oYiS-N--W5ENZIycoigY/
  # Link to sheet of BigM600 Versions:
  # https://docs.google.com/spreadsheets/d/18eGSelEsldi6UuKKpqSIw5j9_UU21-UJzTv24IcN6oE/edit#gid=1828034640&range=AE6
  # Link to doc of Oktoberkite CSim config settings.
  # https://docs.google.com/document/d/1eS4JyUZM6VPlBcJ-xba_HyPBIbKtoZAVwTPstZm4vJs/

  # Reference mass [kg] of the wing, which is used for scaling the
  # moment-of-inertia tensor.
  reference_wing_mass = 1850.0

  # Kite mass [kg] includes airframe, bridles, wingside tether termination, etc.
  wing_mass = 1850.0

  # Center-of-mass [m] location.
  center_of_mass_pos = [-0.57, 0.0, 0.0]

  # Moment-of-inertia tensor [kg-m^2].  Ref: go/csim-config-tracker.
  I = np.array([[4.33e4, 0.0, 0.0],  # pylint: disable=invalid-name
                [0.0, 1.76e4, 0.0],
                [0.0, 0.0, 4.33e4]]) * wing_mass / reference_wing_mass

  # Set bridle geometry.
  # More details for bridle_pos at go/makani-loadcell-coordinate-system.
  # Since CalcLoadcellForces requires equal x and z coordinates,
  # their mean is used for now. True bridle position given in comments.

  # Position [m] of port and starboard bridle attachments.
  # Bridle x and z from BigM600 versions sheet for r07c_v01.
  # Y value is largely TBD, but is ~ +/- 1m.
  # Current value of 1.5 is selected to avoid triggering assert on bridle
  # load cell to tether angle calculation. See b/146515683.
  # Bridle y positions are approximately constrained to +/- 1.5m by pylons.
  bridle_pos = [[-0.3, -1.5, 0.4],
                [-0.3, 1.5, 0.4]]

  # Radial distance [m] between bridle pivot and bridle point.
  # From BigM600 config sheet for r07c_v01.
  bridle_rad = 1.0

  # Offset [m] of bridle point along y-axis.
  # From BigM600 config sheet for r07c_v01.
  bridle_y_offset = 0.0

  # A mean rotor diameter [m] is calculated for use in wing aerodynamic methods.
  rotors_diameter = [params['rotors'][r]['D'] for r in range(m.kNumMotors)]
  mean_rotor_diameter = sum(rotors_diameter) / len(rotors_diameter)

  return {
      # Wing area [m^2].
      # From Oktoberkite CSim config doc.
      'A': 54.0,

      # Wing span [m].
      # From Oktoberkite CSim config doc.
      'b': 26.0,

      # Wing chord [m].
      # From Oktoberkite CSim config doc.
      'c': 2.077,

      # Wing incidence angle [deg].
      # From Oktoberkite CSim config doc.
      'wing_i': 15.25,

      # Estimated wing mass [kg]. This incorporates both the wing and bridle
      # mass, as given by go/makanimass.
      'm': wing_mass,

      # TODO: Get Oktoberkite values.
      # Estimated tail mass [kg] (tail = tub + fuselage + empennage)
      'm_tail': 15.25,

      # Moment of inertia tensor [kg-m^2] and inverse.
      'I': {'d': I.tolist()},
      'I_inv': {'d': np.linalg.inv(I).tolist()},

      # Tail matrix of inertia [kg-m^2] about tail CG
      # (tail = tub + fuselage + empennage).
      # TODO: Get Oktoberkite values.
      'i_tail': {'d': [[1.0E-6, 0.0, 0.0],
                       [0.0, 1.0E-6, 0.0],
                       [0.0, 0.0, 1.0E-6]]},

      # Center of mass [m].
      'center_of_mass_pos': center_of_mass_pos,

      # Tail CG [m] in body coordinates (tail = tub + fuselage + empennage).
      # Current value from BigM600 versions
      # TODO: Get Oktoberkite values.
      'tail_cg_pos': [-8.5, 0.0, 0.0],

      # Position [m] of port and starboard bridle attachments,
      'bridle_pos': bridle_pos,

      # Radial distance [m] between bridle pivot and bridle point.
      'bridle_rad': bridle_rad,

      # Offset [m] of bridle point along y-axis.
      'bridle_y_offset': bridle_y_offset,

      # Position [m] of the horizontal tail (taken as the
      # center-of-pressure) in body coordinates.  This is used to
      # calculate the proximity of the horizontal tail to the remote
      # perch.
      # TODO: Get Oktoberkite values.
      'horizontal_tail_pos': [-10.0, 0.0, 0.0],

      # Position [m] of the constraint system attachment point on the
      # wing, in body coordinates.
      # TODO: Get Oktoberkite values.
      'proboscis_pos': [0.0, 0.0, 0.0],

      # Position [m] of the quarter-chord line center of each pylon. Y location
      # from ASWING input geometry file. X and Z locations approximated via
      # integration of the quarter-chord location along the pylon span, weighted
      # by chord length.
      # TODO: Get Oktoberkite values.
      'pylon_pos': [[0.8075, -4.5, 0.0],
                    [0.8075, -1.5, 0.0],
                    [0.8075, 1.5, 0.0],
                    [0.8075, 4.5, 0.0]],

      # Pylon span [m].
      # TODO: Get Oktoberkite values.
      'b_pylon': 3.26,

      # Mean rotor diameter [m].
      'mean_rotor_diameter': mean_rotor_diameter,
  }

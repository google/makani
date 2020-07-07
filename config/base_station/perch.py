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

"""Perch parameters."""
from makani.config import mconfig
from makani.control import system_types
import numpy as np


@mconfig.Config(deps={
    'gs_model': 'base_station.gs_model',
    'levelwind': 'base_station.levelwind'
})
def MakeParams(params):
  """Make ground station perch parameters."""
  if params['gs_model'] == system_types.kGroundStationModelTopHat:
    # Here, the "drum" represents the platform on top of the top
    # hat's slewing bearing on which the GSG is mounted.
    #
    # The drum angle is 0.0 when the tether is fully payed out, so it has no
    # impact on the GSG azimuth.
    gsg_pos_wd = [0.0, 0.0, -0.694]
    winch_drum_origin_p = [0.0, 0.0, 0.0]

    # This should never be used but is set for historical consistency.
    perched_wing_pos_p = [7.9, -0.14, -0.4]

  elif params['gs_model'] == system_types.kGroundStationModelGSv1:
    # Position [m] of the GSG in the wd-frame.  The z-coordinate was
    # measured by mbelani in CAD on 2015-08-18.  The x and y coordinates
    # are derived from go/makaniwiki/perch-geometry.
    gsg_pos_wd = [-1.5, 0.0, -2.38]

    # Origin [m] of the wd-frame in the p-frame.  The x and y
    # coordinates here are derived from go/makaniwiki/perch-geometry.
    winch_drum_origin_p = [-1.3, 0.0, 0.0]

    perched_wing_pos_p = [7.9, -0.14, -0.4]

  elif params['gs_model'] == system_types.kGroundStationModelGSv2:
    # These should be unused when GS02 is active (Gs02Params has its own
    # versions of these), so set them to something absurd.
    gsg_pos_wd = [1e9, 1e9, 1e9]
    winch_drum_origin_p = [1e9, 1e9, 1e9]
    perched_wing_pos_p = [1e9, 1e9, 1e9]

  else:
    assert False, 'Unsupported ground station model.'

  # Z coordinate [m] of the GSG origin in the p-frame.
  gsg_pos_z_p = gsg_pos_wd[2] + winch_drum_origin_p[2]

  # Distance [m] between the centers-of-rotation of the perch and
  # winch drum.
  dist_perch_to_drum = np.sqrt(winch_drum_origin_p[0]**2.0
                               + winch_drum_origin_p[1]**2.0)

  # Perch and winch drum inertias [kg-m^2].  The following parameters
  # are based on a conversation with hachtmann (2014-08-25):
  #
  #     Component   | Mass [kg] | I_zz [kg-m^2]
  #   --------------|-----------|---------------
  #   Frame         | 7400      | 47000 (about perch axis)
  #   Levelwind     | 1000      | 8000 (about perch axis)
  #   Boom + Panels | 3000      | 15000 (about perch axis)
  #   Winch motor   | 1500      | 13500 (about perch axis)
  #   Drum          | 4000      | 8000 (about drum axis)
  #
  # The frame, levelwind, boom, panels, and winch motor are
  # collectively called the "perch" within the simulator.
  m_drum = 4000.0
  I_drum = 8000.0  # pylint: disable=invalid-name
  I_perch_about_perch_axis = 47000.0 + 8000.0 + 15000.0 + 13500.0  # pylint: disable=invalid-name
  I_perch_and_drum = (I_drum + I_perch_about_perch_axis  # pylint: disable=invalid-name
                      + m_drum * dist_perch_to_drum**2)

  return {
      # Origin [m] of the wd-frame in the p-frame.
      'winch_drum_origin_p': winch_drum_origin_p,

      # Position [m] of the GSG in the wd-frame.  The z-coordinate was
      # measured by mbelani in CAD on 2015-08-18.  The x and y
      # coordinates are derived from go/makaniwiki/perch-geometry.
      'gsg_pos_wd': gsg_pos_wd,

      # Levelwind position [m] in the p-frame when theta_wd = 0.  Note
      # that in the simulation and controller levelwind_origin_p
      # refers to the effective anchor point of the tether, which is a
      # few centimeters from the winch drum surface.
      #
      # The x and y coordinates are derived from
      # go/makaniwiki/perch-geometry.  The z coordinate assumes that
      # the levelwind shuttle is at the GSG height before disengage.
      #
      # TODO: Update PToLW to use a direct measurement of
      # the distance between the levelwind shuttle and the perch
      # origin.
      'levelwind_origin_p_0': [
          0.97,
          0.0,
          (gsg_pos_z_p
           + np.pi * params['levelwind']['drum_angle_to_vertical_travel'])
      ],

      'I_perch_and_drum': I_perch_and_drum,
      'I_drum': I_drum,

      # Kinetic friction moment [N-m] for the perch rotation axis.  The
      # current value was derived from the log M600A-20150828-181400-bad_drop
      # using:
      #   - the measured tether tension,
      #   - the assumption that the tether force acts at the levelwind,
      #   - and the difference in wing position azimuth and perch azimuth.
      # The azimuth values were corrected for a ~3.5 [deg] bias.
      'kinetic_friction_perch': 1e3,

      # Damping coefficients [N-m/(rad/s)] of the perch linkage and the
      # drum linkage.
      #
      # The perch has no appreciable viscous friction.  This was
      # evaluated from the log M600A-20150924-160853-spin3 during
      # which the perch was spun up to 0.1 rad/s and allowed to slow
      # to a stop.
      #
      # TODO: Collect data with the perch rotating at faster
      # rates to identify viscous friction.
      'b_perch': 0.0,
      'b_drum': 50.0 * 3.294**5,

      # Wing position [m] when perched. The x-coordinate is determined by
      # choosing the largest value (at 10cm intervals) at which both talons stay
      # on the perch. Then the y-coordinate is chosen based on the position in
      # which the wing stabilizes after drop.
      'perched_wing_pos_p': perched_wing_pos_p,
  }

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

"""Levelwind parameters."""
from makani.config import mconfig
import numpy as np


@mconfig.Config
def MakeParams():
  return {
      # Conversion [m/rad] between the winch drum rotation angle and the
      # z-position of the levelwind.
      'drum_angle_to_vertical_travel': -0.033 / (2.0 * np.pi),

      # Distance [m] between the levelwind elevation axis and the
      # bridle point measured on the line between the perch
      # center-of-rotation and where the bridle point sits when
      # perched.
      #
      # The current value was derived from go/makaniwiki/perch-geometry.
      'pivot_axis_to_bridle_point': 2.139,

      # Angle [rad] between the perch y axis and the levelwind y axis
      # at zero elevation.  This value is derived from
      # go/makaniwiki/perch-geometry.
      'azimuth_offset': np.deg2rad(-32.5),

      # Drum angle [rad] when the levelwind pulley engages.
      'pulley_engage_drum_angle': np.deg2rad(-180.0),

      # Backlash [rad] in the levelwind elevation axis.  This occurs for
      # two reasons: First, in order for the tether to pass freely
      # between the skis, the skis must have a spacing slightly greater
      # than the tether diameter.  Second, static friction can prevent
      # the levelwind axis from rotating when the wing moves.
      #
      # TODO: Validate a reasonable backlash value from data.
      'elevation_backlash': np.deg2rad(0.0),

      # Nominal elevation angle [rad] of the levelwind.
      'elevation_nominal': np.deg2rad(-3.5)
  }

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

"""Wind sensor parameters."""
from makani.config import mconfig
from makani.control import system_types
import numpy as np


@mconfig.Config(deps={
    'gs': 'base_station.ground_station',
    'gs_model': 'base_station.gs_model',
})
def MakeParams(params):
  """Make wind sensor parameters."""
  if params['gs_model'] == system_types.kGroundStationModelTopHat:
    # Heading [rad] of the +u wind sensor axis relative to the +x axis
    # of the ground station coordinate system.
    heading = np.deg2rad(95.5)

    # Offset [m] of the wind sensor from the ground frame origin.
    pos = [0.202, -1.627, -3.808]

    on_perch = False

  elif params['gs_model'] == system_types.kGroundStationModelGSv1:
    # GSv1: These measurements are based off of the CAD drawings here:
    # go/makaniwiki/perch-geometry.
    #
    # The angle [rad] of the +u wind sensor axis relative to the +x axis
    # of the perch.
    heading = np.deg2rad(-30.0)
    pos = [-2.73, -2.35, -9.4]
    on_perch = True
  elif params['gs_model'] == system_types.kGroundStationModelGSv2:
    # 360 - 46.9 (ws to gps axis) - 169.84 (gps axis to the +x platform
    # coordinate system). See b/66695220.
    heading = np.deg2rad(143.26)
    pos = [2.32, 0.026, -14.9]
    on_perch = True
  else:
    assert False, 'Unsupported ground station model.'

  return {
      # Position [m] of wind sensor in the parent frame.
      'pos_parent': pos,

      # Orientation of the wind sensor relative to the parent frame.
      # Note that the +w axis of the wind sensor points upward.
      'dcm_parent2ws': {'d': [[np.cos(heading), np.sin(heading), 0.0],
                              [np.sin(heading), -np.cos(heading), 0.0],
                              [0.0, 0.0, -1.0]]},

      # Whether the wind sensor's parent frame is the perch frame, as opposed
      # to the ground frame.
      'on_perch': on_perch
  }

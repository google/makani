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

"""Ground frame parameters."""

from makani.config import mconfig
from makani.control import system_types
import numpy as np


@mconfig.Config(deps={
    'gs_model': 'base_station.gs_model',
    'test_site': 'common.test_site',
    'buoy': 'base_station.sim.buoy_sim'
})
def MakeParams(params):
  """Make ground frame parameters."""
  # Set ground station height.
  if mconfig.WING_MODEL == 'oktoberkite':
    # From BigM600 config sheet for r07c_v01.
    # https://docs.google.com/spreadsheets/d/18eGSelEsldi6UuKKpqSIw5j9_UU21-UJzTv24IcN6oE
    ground_z = 15.
    # Same ecef as ParkerRanch
    origin_ecef = [-5465412.106, -2474044.018, 2160944.798]

  else:
    if params['test_site'] == system_types.kTestSiteAlameda:
      ground_z = 15.12

    elif params['test_site'] == system_types.kTestSiteChinaLake:
      # Z-axis offset [m] of the ground (concrete base) to the top hat origin
      # (top of the slewing bearing).
      ground_z = 14.073 + 1.979

    elif params['test_site'] == system_types.kTestSiteParkerRanch:
      # Adjusted for a 5 meter tower.
      # TODO: Adjust this value once the GS is installed.
      ground_z = 6.122
      origin_ecef = [-5465412.106, -2474044.018, 2160944.798]

    elif params['test_site'] == system_types.kTestSiteNorway:
      ground_z = params['buoy']['msl_pos_z_g']
      origin_ecef = [3265751.728, 287962.154, 5452797.993]
    else:
      assert False, 'Unknown test site.'

  if params['gs_model'] == system_types.kGroundStationModelGSv2:
    # Align g-frame with NED.
    heading = 0.0
  else:
    # China Lake east/west radial.
    heading = np.deg2rad(269.0)

  return {
      'ground_z': ground_z,

      # Ground frame heading [rad].
      'heading': heading,

      # Origin [m, ECEF].
      'origin_ecef': origin_ecef,
  }

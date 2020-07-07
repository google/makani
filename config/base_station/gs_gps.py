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

"""GS GPS parameters."""

from makani.config import mconfig
from makani.control import system_types
import numpy as np


@mconfig.Config(deps={
    'gs_model': 'base_station.gs_model',
    'test_site': 'common.test_site',
})
def MakeParams(params):
  """Make ground station gps parameters."""
  if params['gs_model'] == system_types.kGroundStationModelTopHat:
    gps_primary_antenna_dir = [0.0, 0.0, -1.0]
    gps_primary_pos = [1.418, -1.657, -2.417]

    # TopHat doesn't actually have a secondary gps.
    gps_secondary_antenna_dir = gps_primary_antenna_dir
    gps_secondary_pos = gps_primary_pos

    # Angle [rad] from the GPS compass baseline to the zero-azimuth
    # reference of the perch frame. Note: The TopHat does not have a
    # GPS compass, but this value is set for historical consistency.
    gps_compass_to_perch_azi = -2.440

  elif params['gs_model'] == system_types.kGroundStationModelGSv1:
    gps_primary_antenna_dir = [0.0, 0.0, -1.0]
    # Position measured on 2015-06-15.
    gps_primary_pos = [0.0, 0.0, -2.94]
    # GSv1 doesn't actually have a secondary gps.
    gps_secondary_antenna_dir = gps_primary_antenna_dir
    gps_secondary_pos = gps_primary_pos

    # Angle [rad] from the GPS compass baseline to the zero-azimuth
    # reference of the perch frame
    gps_compass_to_perch_azi = -2.440

  elif params['gs_model'] == system_types.kGroundStationModelGSv2:
    gps_primary_antenna_dir = [0.0, 0.0, -1.0]
    gps_secondary_antenna_dir = [0.0, 0.0, -1.0]
    if params['test_site'] == system_types.kTestSiteParkerRanch:
      # See b/137283974 for details.
      gps_primary_pos = [-0.002, 0.011, -6.7]
      gps_secondary_pos = [-2.450, -0.428, -6.827]
    elif params['test_site'] == system_types.kTestSiteNorway:
      # See b/137660975 for details.
      gps_primary_pos = [-0.002, 0.011, -6.7]
      gps_secondary_pos = [-2.450, -0.428, -6.757]
    else:
      assert False, 'Unsupported test site.'
    # Angle [rad] from the GPS compass baseline to the zero-azimuth
    # reference of the platform frame.  See b/118710931.
    gps_compass_to_perch_azi = np.deg2rad(169.84)

  else:
    assert False, 'Unsupported ground station model.'

  return {
      # Position [m] of the GS GPS antenna in the platform frame.
      # NOTE: The direction of the antennae is currently not used.
      'primary_antenna_p': {
          'antenna_dir': gps_primary_antenna_dir,
          'pos': gps_primary_pos,
      },
      'secondary_antenna_p': {
          'antenna_dir': gps_secondary_antenna_dir,
          'pos': gps_secondary_pos,
      },

      # Calibration for the ground station compass ([#], [rad], [#]).
      # The bias is used to account for the angle between the perch
      # frame and the NovAtel differential GPS receiver.
      # TODO: Remove this parameter once the computation of
      # compass heading from the primary and secondary antennae is implemented.
      'heading_cal': {
          'scale': 1.0, 'bias': gps_compass_to_perch_azi, 'bias_count': 0}
  }

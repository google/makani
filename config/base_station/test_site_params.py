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

"""Test site parameters."""

from makani.config import mconfig
from makani.control import system_types as m

import numpy as np


@mconfig.Config(deps={
    'flight_plan': 'common.flight_plan',
    'ground_frame': 'base_station.ground_frame',
    'gs_model': 'base_station.gs_model',
    'test_site': 'common.test_site',
})


def MakeParams(params):
  """Return test site parameters."""

  # Minimum and maximum azimuth angles [rad], useful
  # if test site layout does not permit operation at any azimuth.  The
  # azimuth angle wraps at +/- pi; setting azi_allow_start to -pi and
  # azi_allow_end to +pi effectively removes this limit.
  #
  # Parker Ranch has azimuth limits for Command Center safety.
  # GS02 check is to ensure other changes tied to GS02 also occur.
  if (params['test_site'] == m.kTestSiteParkerRanch
      and params['gs_model'] == m.kGroundStationModelGSv2
      and params['flight_plan'] != m.kFlightPlanHoverInPlace):
    assert params['ground_frame']['heading'] == 0.0, (
        'Ground station coordinate system must align with NED for Parker '
        'Ranch test site.')
    # The Command Center (CC) is at 45 deg azimuth. The no-go zone extends
    # +/-53 deg from the CC location.  The allowed zone in this file is less
    # than 180 deg to prevent the snap through issue described in b/116805392.
    # We bias slightly clockwise of the prevailing downwind direction at
    # Parker Ranch (230-240 degrees) to make the most use of this 180 degree
    # range.
    azi_allow_start = np.deg2rad(161.0)
    azi_allow_end = np.deg2rad(-20.0)
  elif params['test_site'] == m.kTestSiteChinaLake:

    assert abs(
        params['ground_frame']['heading'] - np.deg2rad(269.0)) < 1e-5, (
            'Ground station coordinate system must align with East-West '
            'radial for China Lake test site.')

    # Restrict the azimuth so we will not penetrate the ground frame yz plane.
    # Downwind is -x, so allowable zone crosses the wrap.
    # This restriction is for top hat hardware limitations at China Lake.
    # The 10 deg offsets are for buffer.
    azi_allow_start = np.deg2rad(90.0 + 10.0)
    azi_allow_end = np.deg2rad(-90.0 - 10.0)
  elif params['test_site'] == m.kTestSiteNorway:
    # The crosswind azimuth is unrestricted in software at the Norway test site
    # because this is handled through flight envelope and operational
    # restrictions instead (see b/137571714).
    azi_allow_start = -np.pi
    azi_allow_end = np.pi
  else:
    # The crosswind azimuth is currently unrestricted for all other configs.
    azi_allow_start = -np.pi
    azi_allow_end = np.pi

  # Find size of no-go zone (which is opposite of allow zone, so it's specified
  # from azi_allow_end to azi_allow_start).
  if azi_allow_start < azi_allow_end:
    azi_no_go_size = azi_allow_start + 2.0 * np.pi - azi_allow_end
  else:
    azi_no_go_size = azi_allow_start - azi_allow_end

  # No-go zone size must be zero or greater than min size.
  assert (azi_no_go_size == 0.0
          or azi_no_go_size >= m.MIN_AZI_NO_GO_SIZE)

  return {
      'azi_allow_start': azi_allow_start,
      'azi_allow_end': azi_allow_end,
      'azi_no_go_size': azi_no_go_size
  }

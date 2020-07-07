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

"""Reference wind speeds and dimensions from the specification."""

from makani.config import mconfig
from makani.sim import sim_types
import numpy as np


@mconfig.Config(deps={
    'ground_frame': 'base_station.ground_frame',
})
def MakeParams(params):
  # We place our hub-height assuming a 30 degree angle with the ground.
  # We neglect the GSG's height above the ground station origin.
  hub_elevation = np.deg2rad(30.0)
  reference_tether_length = 370.0
  hub_height_agl = (np.sin(hub_elevation) * reference_tether_length
                    + params['ground_frame']['ground_z'])

  return {
      # Cut-in (i.e. zero-power) wind speed [m/s].
      'v_in': 4.0,

      # Cut-out wind speed [m/s].
      'v_out': 25.0,

      # Wind speed [m/s] at first rated power point (i.e. 2/3 P_nom).
      'v_r1': 9.2,

      # Wind speed [m/s] at second rated power point (i.e. P_nom).
      'v_r2': 11.0,

      # Hub height above ground level [m].
      'hub_height_agl': hub_height_agl,

      # Wind turbine diameter [m] when comparing our system to a HAWT.
      'rotor_diameter': 220.0,

      # Time [s] at which extreme wind events start.
      'event_t_start': 10.0,

      # IEC load case to simulate.
      'load_case': sim_types.kIecCaseNormalWindProfile
  }

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

"""Physical parameters relevant in simulation."""

import sys

from makani.config import mconfig
from makani.control import system_types
from makani.sim import sim_types
import numpy as np


@mconfig.Config(deps={
    'ground_station': 'base_station.ground_station',
    'gs_model': 'base_station.gs_model',
    'physical_constants': 'common.physical_constants',
    'test_site': 'common.test_site',
    'wind_sensor': 'base_station.wind_sensor'
})
def MakeParams(params):
  if params['gs_model'] == system_types.kGroundStationModelGSv2:
    # For GS02, the g-frame is equivalent to NED.
    if params['test_site'] == system_types.kTestSiteParkerRanch:
      # The prevailing wind at Parker Ranch is approximately north-easterly.
      wind_direction = np.deg2rad(40.0)
    elif params['test_site'] == system_types.kTestSiteNorway:
      # The prevailing wind at Norway is approximately north-westerly.
      wind_direction = np.deg2rad(335.0)
    else:
      # This specifies a westerly wind.
      wind_direction = np.deg2rad(270.0)
  else:
    # Wind comes from the +x-direction.
    wind_direction = 0.0

  if params['test_site'] == system_types.kTestSiteNorway:
    # TurbSim at 10 m/s @ 32 m ASL, 0 shear, 10% turbulence intensity,
    # and offshore.
    wind_database_name = '20190619-074534-012-00_10mps_00shear.h5'
  else:
    # Default to TurbSim at 10 m/s @ 21 m AGL, 0 shear, category 'B' Turbulence,
    # and onshore.
    wind_database_name = '20181011-165906-006-00_10mps_00shear.h5'

  # Delta [m/s] applied to the baseline mean wind speed taking place at time
  # t_delta [s].
  wind_speed_updates = []
  no_wind_speed_update = {
      't_update': sys.float_info.max,
      'offset': 0.0,
  }

  return {
      # Air density [kg/m^3].
      #
      # This air density is used by the simulator and can be different
      # from that used by the controller.
      'air_density': params['physical_constants']['rho'],

      # Dynamic viscosity [Pa-s] at sea level.  See:
      # https://en.wikipedia.org/wiki/Standard_sea_level.
      'dynamic_viscosity': 1.789e-5,

      # Local path to the wind database. run_sim supports auto-download of a
      # specific online database if an appropriate filename is set at the
      # command line using the -o flag or in the parameters *.json file using
      # the -p flag.
      'wind_database': {'name': wind_database_name},

      # Initial time index [s] into the wind database.
      # Recommend setting the initial time on the turbulence database to
      # be slightly greater than zero to avoid data being fetched from
      # outside the database range.
      'wind_database_initial_time': 30.0,

      # Y offset [m] of the wind database, from center.
      'wind_database_y_offset': 0.0,

      # Model to use for wind and turbulence.
      # If this is set to use a database, then many of the parameters in this
      # file (such as wind_elevation, wind_shear_exponent) are ignored.
      'wind_model': sim_types.kWindModelDrydenTurbulence,

      # Mean wind speed [m/s] measured at wind sensor altitude.
      'wind_speed': 10.0,
      'wind_speed_update': {
          'num_updates': len(wind_speed_updates),
          'offsets': wind_speed_updates
                     + ([no_wind_speed_update]
                        * (sim_types.MAX_WIND_SPEED_UPDATES
                           - len(wind_speed_updates)))
      },
      'wind_speed_update_rate_limit': 0.05,

      # Nominal wind azimuth angle [rad] relative to g-coordinate system.
      # At zero, the wind comes from the positive x direction.
      # At pi/2.0, the wind comes from the positive y direction.
      'wind_direction': wind_direction,

      # Nominal wind elevation angle [rad] relative to g-coordinate system.
      # Positive angles indicate a downward flow.
      'wind_elevation': 0.0,

      # The wind shear exponent, alpha [#], determines how the mean wind
      # speed varies with height according to:
      #
      #   V(z) = V_hub * (z/z_hub)^alpha.
      #
      # A common value is 1/7. The IEC 61400-1 (sec. 6.3.1.2) normal wind
      # profile recommends a value of 0.2.
      'wind_shear_exponent': 0.0,

      # Above-ground-level height [m] at which the mean wind applies.
      #
      # Wind shear is calculated relative to this height AGL.  This
      # value was set to the height of the wind sensor at Parker Ranch,
      # and then hard-coded to prevent the value from changing.
      # Wind databases ignore this parameter.
      'wind_shear_ref_height_agl': 21.0,

      # Wind azimuth change [rad] that takes place between the start and end
      # heights [m] above-ground-level.
      'wind_veer': 0.0,
      'wind_veer_start_height_agl': 100.0,
      'wind_veer_end_height_agl': 200.0,
  }

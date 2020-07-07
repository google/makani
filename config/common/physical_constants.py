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

"""Physical constants.

Nothing that depends on the orientation of the ground station (or any
other parameter of the system) should be included here (i.e. only
mag_ned, not mag_g).
"""
from makani.config import mconfig
from makani.control import system_types


@mconfig.Config(deps={'test_site': 'common.test_site'})
def MakeParams(params):
  # Initialize physical parameters lists.
  air_densities = [None] * system_types.kNumTestSites
  atmospheric_pressures = [None] * system_types.kNumTestSites
  magnetic_fields_ned = [[None, None, None]] * system_types.kNumTestSites

  # Use standard atmosphere density and pressure at 0 m altitude and
  # no temperature offset (15 C).
  air_densities[system_types.kTestSiteAlameda] = 1.225
  atmospheric_pressures[system_types.kTestSiteAlameda] = 1.01325e5

  # Use standard atmosphere density and pressure at 696 m altitude and
  # 22 degrees C temperature offset (about 32 C).
  air_densities[system_types.kTestSiteChinaLake] = 1.063
  atmospheric_pressures[system_types.kTestSiteChinaLake] = 93238.5

  # Use standard atmosphere pressure at 922 m altitude,
  # air temperature of 31 C, and a relative humidity of 70%.
  # See b/80543226.
  air_densities[system_types.kTestSiteParkerRanch] = 1.026
  atmospheric_pressures[system_types.kTestSiteParkerRanch] = 90728.30

  # A summer day in Stavanger, Norway: 18 degrees Celcius, 60%
  # humidity.
  air_densities[system_types.kTestSiteNorway] = 1.220
  atmospheric_pressures[system_types.kTestSiteNorway] = 102404.39

  # The following values come from NOAA's magnetic field calculator
  # (see http://www.ngdc.noaa.gov/geomag-web/#igrfwmm) using the
  # WMM2015 model on 2016-02-29.
  #
  # Latitude:  37.7730361 degrees N
  # Longitude: -122.2978066 degrees W
  # Elevation: -11.6269 m WGS84 (from GPS base station)
  magnetic_fields_ned[system_types.kTestSiteAlameda] = [0.226452,
                                                        0.055025,
                                                        0.424603]
  # Latitude:  35.6830816 degrees N
  # Longitude: -117.6898297 degrees W
  # Elevation: 696 m above mean sea level (from Wikipedia)
  magnetic_fields_ned[system_types.kTestSiteChinaLake] = [0.231583,
                                                          0.050742,
                                                          0.417817]
  # Latitude:  19.9320889 degrees N
  # Longitude: -155.6450528 degrees W
  # Elevation: 817 m above mean sea level (from Google Maps)
  magnetic_fields_ned[system_types.kTestSiteParkerRanch] = [0.274242,
                                                            0.046062,
                                                            0.206902]

  # Stavanger, Norway (58.88 N, 6.63 E, 0 meters), using the WMM
  # evaluated for the summer solstice, 2019.
  magnetic_fields_ned[system_types.kTestSiteNorway] = [0.156859,
                                                       0.004375,
                                                       0.483152]

  return {
      # Magnitude of gravitational acceleration [m/s^2].
      'g': 9.81,

      # Density of air [kg/m^3].
      'rho': air_densities[params['test_site']],

      # Atmospheric pressure [N/m^2].
      'P_atm': atmospheric_pressures[params['test_site']],

      # Specific gas constant [J/kg/K] for dry air.
      'R_dry_air': 287.058,

      # Specific gas constant [J/kg/K] for water vapor.
      'R_water_vapor': 461.5,

      # Gravitational acceleration vector [m/s^2].
      'g_g': [0.0, 0.0, 9.81],

      # Earth's magnetic field [G] in North-East-Down coordinates.
      'mag_ned': magnetic_fields_ned[params['test_site']],
  }

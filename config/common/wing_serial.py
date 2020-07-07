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

"""Wing serial number."""

from makani.config import mconfig
from makani.control import system_types


@mconfig.Config(deps={'flight_plan': 'common.flight_plan',
                      'test_site': 'common.test_site'})
def MakeParams(params):
  """Make parameters for wing serial based on flight plan and test site."""

  # The Oktoberkite only has one serial number.
  if mconfig.WING_MODEL == 'oktoberkite':
    return system_types.kWingSerialOktoberKite01

  if params['flight_plan'] in [system_types.kFlightPlanManual]:
    if params['test_site'] == system_types.kTestSiteParkerRanch:
      return system_types.kWingSerial04Hover
    elif params['test_site'] == system_types.kTestSiteNorway:
      return system_types.kWingSerial05Hover
    else:
      assert False, 'No wing serial specified for this test site.'
  elif params['flight_plan'] in (system_types.kFlightPlanHoverInPlace,
                                 system_types.kFlightPlanDisengageEngage,
                                 system_types.kFlightPlanHighHover):
    return system_types.kWingSerial01
  else:
    if params['test_site'] == system_types.kTestSiteParkerRanch:
      return system_types.kWingSerial06Crosswind
    elif params['test_site'] == system_types.kTestSiteNorway:
      return system_types.kWingSerial05Crosswind
    else:
      assert False, 'No wing serial specified for this test site.'

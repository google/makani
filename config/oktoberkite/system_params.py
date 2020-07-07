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

"""System parameters."""

from makani.config import mconfig
from makani.control import system_types


@mconfig.Config(deps={
    'common_params': 'common.common_params',
    'comms': 'common.comms',
    'flight_plan': 'common.flight_plan',
    'ground_frame': 'base_station.ground_frame',
    'buoy': 'base_station.buoy',
    'ground_station': 'base_station.ground_station',
    'gsg': 'base_station.gsg',
    'gs_gps': 'base_station.gs_gps',
    'gs_imus': 'base_station.gs_imus',
    'gs_model': 'base_station.gs_model',
    # TODO: OktKite. Migrate to its config when there is hardware layout.
    'hitl': 'm600.hitl',
    'joystick': 'common.joystick',
    'levelwind': 'base_station.levelwind',
    'limits': 'oktoberkite.limits',
    # TODO: OktKite. Migrate to its config when there is hardware layout.
    'loadcells': 'm600.loadcells',
    'perch': 'base_station.perch',
    'phys': 'common.physical_constants',
    # TODO: OktKite. Migrate to its config when there is hardware layout.
    'pitot': 'm600.pitot',
    'power_sensor': 'common.power_sensor',
    'power_sys': 'powertrain.power_sys',
    # TODO: OktKite. Migrate to its config when there is hardware layout.
    'rotor_sensors': 'm600.rotor_sensors',
    'rotors': 'oktoberkite.rotors',
    # TODO: OktKite. Migrate to its config when there is hardware layout.
    'sensor_layout': 'm600.sensor_layout',
    # TODO: OktKite. Migrate to its config when there is hardware layout.
    'servos': 'm600.servos',
    'test_site': 'common.test_site',
    'test_site_params': 'base_station.test_site_params',
    'tether': 'oktoberkite.tether',
    'winch': 'base_station.winch',
    'wind_sensor': 'base_station.wind_sensor',
    'wing': 'oktoberkite.wing',
    # TODO: OktKite. Migrate to its config when there is hardware layout.
    'wing_gps': 'm600.wing_gps',
    # TODO: OktKite. Migrate to its config when there is hardware layout.
    'wing_imus': 'm600.wing_imus',
    'wing_serial': 'common.wing_serial',
    'wing_serial_status': 'common.wing_serial_status',
})
def MakeParams(params):
  """Make system parameters."""

  system_params = {
      'test_site': params['test_site'],
      'test_site_params': params['test_site_params'],
      'wing_model': system_types.kWingModelOktoberKite,
      'wing_serial': params['wing_serial'],
      'wing_serial_is_active': params['wing_serial_status'],
      'gs_model': params['gs_model'],
      'ts': params['common_params']['ts'],
      'flight_plan': params['flight_plan'],
      'phys': params['phys'],
      'ground_frame': params['ground_frame'],
      'buoy': params['buoy'],
      'ground_station': params['ground_station'],
      'perch': params['perch'],
      'winch': params['winch'],
      'gs_imus': params['gs_imus'],
      'gs_gps': params['gs_gps'],
      'tether': params['tether'],
      'wing': params['wing'],
      'wing_imus': params['wing_imus'],
      'wing_gps': params['wing_gps'],
      'rotors': params['rotors'],
      'loadcells': params['loadcells'],
      'pitot': params['pitot'],
      'gsg': params['gsg'],
      'wind_sensor': params['wind_sensor'],
      'servos': params['servos'],
      'joystick': params['joystick'],
      'power_sys': params['power_sys'],
      'power_sensor': params['power_sensor'],
      'rotor_sensors': params['rotor_sensors'],
      'levelwind': params['levelwind'],
      'comms': params['comms'],
      'sensor_layout': params['sensor_layout'],
      'hitl': params['hitl'],
      'offshore': (True if params['test_site'] == system_types.kTestSiteNorway
                   else False),
      'limits': params['limits'],
  }

  assert mconfig.MatchesCStruct(system_params, system_types.SystemParams)
  return system_params

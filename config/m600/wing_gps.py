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

"""Wing GPS parameters."""
from makani.config import mconfig
from makani.control import system_types


@mconfig.Config(deps={'wing_serial': 'common.wing_serial'})
def MakeParams(params):
  if params['wing_serial'] == system_types.kWingSerial01:
    crosswind_mass_balance_tube = False
    pitot_tube_cover_actuator = False
  elif params['wing_serial'] == system_types.kWingSerial04Hover:
    crosswind_mass_balance_tube = False
    pitot_tube_cover_actuator = False
  elif params['wing_serial'] == system_types.kWingSerial04Crosswind:
    crosswind_mass_balance_tube = True
    pitot_tube_cover_actuator = False
  elif params['wing_serial'] == system_types.kWingSerial05Hover:
    crosswind_mass_balance_tube = False
    pitot_tube_cover_actuator = False
  elif params['wing_serial'] == system_types.kWingSerial05Crosswind:
    crosswind_mass_balance_tube = True
    pitot_tube_cover_actuator = True
  elif params['wing_serial'] == system_types.kWingSerial06Hover:
    crosswind_mass_balance_tube = False
    pitot_tube_cover_actuator = False
  elif params['wing_serial'] == system_types.kWingSerial06Crosswind:
    crosswind_mass_balance_tube = True
    pitot_tube_cover_actuator = False
  elif params['wing_serial'] == system_types.kWingSerial07Hover:
    crosswind_mass_balance_tube = False
    pitot_tube_cover_actuator = False
  elif params['wing_serial'] == system_types.kWingSerial07Crosswind:
    crosswind_mass_balance_tube = True
    pitot_tube_cover_actuator = True
  # TODO: OktKite. Migrate to its own config folder once it makes sense.
  elif params['wing_serial'] == system_types.kWingSerialOktoberKite01:
    crosswind_mass_balance_tube = True
    pitot_tube_cover_actuator = False
  else:
    assert False, 'Unknown wing serial.'

  config = {}

  if crosswind_mass_balance_tube:
    # Locations and directions of the GPS antennas on the SN02 crosswind
    # mass-balance tube provided by seanchou on 2016-08-12 (b/30795963).
    config[system_types.kWingGpsReceiverCrosswind] = {
        # Vector [#] giving the orientation of the antenna in body
        # coordinates.
        'antenna_dir': [0.0, -0.342, -0.940],

        # Antenna position [m] in body coordinates.
        'pos': [2.780, -0.005, 0.376]
    }

    # Pitot tube cover actuator correction provided by horton (b/136198483).
    if pitot_tube_cover_actuator:
      hover_receiver_pos = [2.969, 0.005, 0.361]
    else:
      hover_receiver_pos = [2.969, 0.005, 0.391]

    config[system_types.kWingGpsReceiverHover] = {
        # Vector [#] giving the orientation of the antenna in body
        # coordinates.
        'antenna_dir': [0.707, 0.242, -0.665],

        # Antenna position [m] in body coordinates.
        'pos': hover_receiver_pos
    }
  else:
    # Aluminum mass-balance tube with forward-facing side-by-side
    # GPS antennas.
    config[system_types.kWingGpsReceiverCrosswind] = {
        # Vector [#] giving the orientation of the antenna in body
        # coordinates.
        'antenna_dir': [1.0, 0.0, 0.0],

        # Antenna position [m] in body coordinates.
        'pos': [1.46, -0.105, 0.0]
    }

    config[system_types.kWingGpsReceiverHover] = {
        # Vector [#] giving the orientation of the antenna in body
        # coordinates.
        'antenna_dir': [1.0, 0.0, 0.0],

        # Antenna position [m] in body coordinates.
        'pos': [1.46, 0.105, 0.0]
    }

  # Wingtip GPS receivers. Data provided by seanchou 2018-08-22 (see ECR 336).
  config[system_types.kWingGpsReceiverPort] = {
      'pos': [-0.550, -12.680, -0.435],
      'antenna_dir': [0.0, 0.0, -1.0],
  }

  config[system_types.kWingGpsReceiverStar] = {
      'pos': [-0.550, 12.680, -0.435],
      'antenna_dir': [0.0, 0.0, -1.0],
  }

  return [config[i] for i in range(system_types.kNumWingGpsReceivers)]

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

"""Status (active/inactive) of wing serials."""

from makani.config import mconfig
from makani.control import system_types


@mconfig.Config(deps={'wing_serial': 'common.wing_serial'})
def MakeParams(params):
  """Make parameter for whether a wing serial number is still useable."""

  def CheckModelAndSerialCompatibility(model, serial):
    """Checks that the wing model and serial numbers are compatible."""
    incompatible = False
    if (model == 'oktoberkite' and system_types.WingSerialToModel(serial) !=
        system_types.kWingModelOktoberKite):
      incompatible = True
    elif (model == 'm600' and system_types.WingSerialToModel(serial) !=
          system_types.kWingModelYm600):
      incompatible = True
    assert not incompatible, (
        'Model %s and serial number %i are not compatible.' % (model, serial))

  CheckModelAndSerialCompatibility(mconfig.WING_MODEL, params['wing_serial'])

  wing_serial_is_active = {
      system_types.kWingSerial01: True,
      system_types.kWingSerial02: False,
      system_types.kWingSerial02Final: False,
      system_types.kWingSerial03Hover: False,
      system_types.kWingSerial03Crosswind: False,
      system_types.kWingSerial04Hover: True,
      system_types.kWingSerial04Crosswind: True,
      system_types.kWingSerial05Hover: True,
      system_types.kWingSerial05Crosswind: True,
      system_types.kWingSerial06Hover: True,
      system_types.kWingSerial06Crosswind: True,
      system_types.kWingSerial07Hover: True,
      system_types.kWingSerial07Crosswind: True,
      system_types.kWingSerialOktoberKite01: True,
  }

  assert len(wing_serial_is_active.keys()) == system_types.kNumWingSerials

  return wing_serial_is_active[params['wing_serial']]

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

import unittest

from makani.avionics.bootloader import system_config
from makani.config import mconfig
from makani.control import system_types
from makani.lib.python import c_helpers


_WING_SERIAL_HELPER = c_helpers.EnumHelper('WingSerial', system_types,
                                           prefix='kWingSerial')


class SystemConfigTest(unittest.TestCase):
  # Verify that all wing serials have an associated configuration.

  def testAllWingSerials(self):
    # TODO: OktKite. When the Oktoberkite has a valid yaml system_config
    # entry remove the restriction which models are run.
    for wing_serial in _WING_SERIAL_HELPER.Values():
      if (system_types.WingSerialToModel(wing_serial) ==
          system_types.kWingModelOktoberKite):
        continue
      is_active = mconfig.MakeParams(
          'common.wing_serial_status', overrides={'wing_serial': wing_serial},
          override_method='derived')
      if not is_active:
        continue

      config = system_config.SystemConfig.GetSystemConfigBySerial(
          wing_serial)
      self.assertIsNotNone(config)

if __name__ == '__main__':
  unittest.main()

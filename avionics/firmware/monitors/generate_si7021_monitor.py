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

"""Generate SI7021 ambient temperature and humidity monitor configuration.

This module looks for an 'si7021_config' tuple in the configuration file
(specified on command line). The si7021_config tuple should contain the
hardware revision EnumHelper and a dictionary that maps the board's hardware
revision to a list of dictionaries. Each dictionary in this list specifies
the configuration for each SI7021 chip.
"""

import sys
import textwrap

from makani.avionics.firmware.drivers import si7021_types
from makani.avionics.firmware.monitors import generate_monitor_base
from makani.lib.python import c_helpers


resolution_helper = c_helpers.EnumHelper('Si7021Resolution', si7021_types)


class Si7021DeviceConfig(generate_monitor_base.DeviceConfigBase):
  """Generate SI7021 monitor configuration."""
  # TODO: Add unit tests.

  def __init__(self, config):
    expected_parameters = {
        'name',
        'address',
        'resolution'}

    super(Si7021DeviceConfig, self).__init__(config, expected_parameters)

  def CheckParameterValues(self, config):
    name = config['name']
    if config['resolution'] not in resolution_helper:
      assert ValueError('Invalid resolution value for %s.' % name)

  def ComputeParameters(self, config):
    config['user_reg1'] = si7021_types.Si7021BuildUserReg1(config['resolution'])

  def GetConfigAsString(self, config, enum_values, index):
    """Generate an initialization array for the Si7021Monitors structure."""
    string = textwrap.dedent("""\
        [{index}] = {{
          .monitor = {monitor_name},
          .config = {{
            .addr = 0x{address:02X},
            .user_reg1 = 0x{user_reg1:02X}}}}},
        """).format(
            index=index,
            monitor_name=enum_values['monitor'],
            **config)
    return string

  def GetHeaderFiles(self):
    return ['avionics/firmware/monitors/si7021_types.h']


def Main(argv):
  """Entry point."""
  flags = generate_monitor_base.ParseFlags(argv)
  config_module = generate_monitor_base.GetConfigModule(flags)
  gen = generate_monitor_base.GenerateMonitorConfig(
      'si7021', flags.prefix, Si7021DeviceConfig)
  gen.LoadConfig(config_module.si7021_config)
  generate_monitor_base.WriteOutputFiles(flags, gen)


if __name__ == '__main__':
  Main(sys.argv)

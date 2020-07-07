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

"""Generate LTC4151 voltage/current monitor configuration.

This module looks for an 'ltc4151_config' tuple in the configuration file
(specified on command line). The ltc4151_config tuple should contain the
hardware revision EnumHelper and a dictionary that maps the board's hardware
revision to a list of dictionaries. Each dictionary in this list specifies
the configuration for each LTC4151 chip.
"""

import sys
import textwrap

from makani.avionics.firmware.monitors import generate_monitor_base


class Ltc4151DeviceConfig(generate_monitor_base.DeviceConfigBase):
  """Generate LTC4151 voltage/current monitor configuration."""
  # TODO: Add unit tests.

  def __init__(self, config):
    expected_parameters = {
        'name',
        'address',
        'binary_config',
        'shunt_resistor',
        'current_max',
        'voltage_max',
        'voltage_min'}

    super(Ltc4151DeviceConfig, self).__init__(config, expected_parameters)

  def GetConfigAsString(self, config, enum_values, index):
    """Generate an initialization array for the Ltc4151Monitors structure."""
    string = textwrap.dedent("""\
        [{index}] = {{
          .monitor = {monitor_name},
          .config = {{
            .addr = 0x{address:02X},
            .binary_config = 0x{binary_config:02X},
            .shunt_resistor = {shunt_resistor}f}},
          .current_max = {current_max}f,
          .voltage_max = {voltage_max}f,
          .voltage_min = {voltage_min}f}},
        """).format(
            index=index,
            monitor_name=enum_values['monitor'],
            **config)
    return string

  def GetHeaderFiles(self):
    return ['avionics/firmware/monitors/ltc4151_types.h']


def Main(argv):
  """Entry point."""
  flags = generate_monitor_base.ParseFlags(argv)
  config_module = generate_monitor_base.GetConfigModule(flags)
  gen = generate_monitor_base.GenerateMonitorConfig(
      'ltc4151', flags.prefix, Ltc4151DeviceConfig)
  gen.LoadConfig(config_module.ltc4151_config)
  generate_monitor_base.WriteOutputFiles(flags, gen)


if __name__ == '__main__':
  Main(sys.argv)

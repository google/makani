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

"""Generate BQ34Z100 voltage/current monitor configuration.

This module looks for an 'bq34z100_config' tuple in the configuration file
(specified on command line). The bq34z100_config tuple should contain the
hardware revision EnumHelper and a dictionary that maps the board's hardware
revision to a list of dictionaries. Each dictionary in this list specifies
the configuration for each BQ34Z100 chip.
"""

import sys
import textwrap

from makani.avionics.firmware.monitors import generate_monitor_base


class Bq34z100DeviceConfig(generate_monitor_base.DeviceConfigBase):
  """Generate BQ34Z100 voltage/current monitor configuration."""
  # TODO: Add unit tests.

  def __init__(self, config):
    expected_parameters = {
        'name',
        'address',
        'cell_mult',
        'current_max',
        'voltage_max',
        'voltage_min',
        'soc_min'}

    super(Bq34z100DeviceConfig, self).__init__(config, expected_parameters)

  def GetConfigAsString(self, config, enum_values, index):
    """Generate an initialization array for the Bq34z100Monitors structure."""
    string = textwrap.dedent("""\
        [{index}] = {{
          .monitor = {monitor_name},
          .config = {{
            .addr = 0x{address:02X},
            .cell_mult = {cell_mult}f}},
          .current_max = {current_max}f,
          .voltage_max = {voltage_max}f,
          .voltage_min = {voltage_min}f,
          .soc_min = {soc_min}}},
        """).format(
            index=index,
            monitor_name=enum_values['monitor'],
            **config)
    return string

  def GetHeaderFiles(self):
    return ['avionics/firmware/monitors/bq34z100_types.h']


def Main(argv):
  """Entry point."""
  flags = generate_monitor_base.ParseFlags(argv)
  config_module = generate_monitor_base.GetConfigModule(flags)
  gen = generate_monitor_base.GenerateMonitorConfig(
      'bq34z100', flags.prefix, Bq34z100DeviceConfig)
  gen.LoadConfig(config_module.bq34z100_config)
  generate_monitor_base.WriteOutputFiles(flags, gen)


if __name__ == '__main__':
  Main(sys.argv)

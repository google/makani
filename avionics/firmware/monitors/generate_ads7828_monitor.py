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

"""Generate ADS7828 voltage/current monitor configuration.

This module looks for an 'ads7828_config' tuple in the configuration file
(specified on command line). The ads7828_config tuple should contain the
hardware revision EnumHelper and a dictionary that maps the board's hardware
revision to a list of dictionaries. Each dictionary in this list specifies
the configuration for each ADS7828 chip.
"""

import sys
import textwrap

from makani.avionics.firmware.drivers import ads7828_types
from makani.avionics.firmware.monitors import generate_monitor_base
from makani.lib.python import c_helpers


select_helper = c_helpers.EnumHelper('Ads7828Select', ads7828_types)
convert_helper = c_helpers.EnumHelper('Ads7828PowerConverter', ads7828_types)
ref_helper = c_helpers.EnumHelper('Ads7828PowerReference', ads7828_types)


class Ads7828DeviceConfig(generate_monitor_base.DeviceConfigBase):
  """Generate ADS7828 voltage/current monitor configuration."""
  # TODO: Add unit tests.

  def __init__(self, config):
    expected_parameters = {
        'name',
        'address',
        'channel',
        'converter',
        'reference',
        'input_divider',
        'input_offset',
        'nominal',
        'min',
        'max'}

    super(Ads7828DeviceConfig, self).__init__(config, expected_parameters)

  def CheckParameterValues(self, config):
    name = config['name']
    if config['input_divider'] == 0.0:
      assert ValueError('Invalid input_divider specified for %s.' % name)

  def ComputeParameters(self, config):
    """Update per-input configuration with computed data."""
    input_divider = config['input_divider']
    config['volts_per_count'] = 2.5 / 2**12 / input_divider
    config['offset'] = config['input_offset'] / input_divider

    config['binary'] = ads7828_types.Ads7828BuildCommand(
        config['channel'], config['converter'], config['reference'])

  def GetConfigAsString(self, config, enum_values, index):
    """Generate an initialization array for the Ads7828Monitors structure."""
    string = textwrap.dedent("""\
        [{index}] = {{
          .monitor = {monitor_name},
          .config = {{
            .addr = 0x{address:02X},
            .command = 0x{binary:02X}}},
          .volts_per_count = {volts_per_count}f,
          .offset = {offset}f,
          .nominal = {nominal}f,
          .min = {min}f,
          .max = {max}f}},
        """).format(
            index=index,
            monitor_name=enum_values['monitor'],
            **config)
    return string

  def GetHeaderFiles(self):
    return ['avionics/firmware/monitors/ads7828_types.h']


def Main(argv):
  """Entry point."""
  flags = generate_monitor_base.ParseFlags(argv)
  config_module = generate_monitor_base.GetConfigModule(flags)
  gen = generate_monitor_base.GenerateMonitorConfig(
      'ads7828', flags.prefix, Ads7828DeviceConfig)
  gen.LoadConfig(config_module.ads7828_config,
                 multiple_configs_per_device=True)
  generate_monitor_base.WriteOutputFiles(flags, gen)


if __name__ == '__main__':
  Main(sys.argv)

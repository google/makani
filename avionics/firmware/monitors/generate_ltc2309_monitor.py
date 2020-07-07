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

"""Generate LTC2309 voltage/current monitor configuration.

This module looks for an 'ltc2309_config' tuple in the configuration file
(specified on command line). The ltc2309_config tuple should contain the
hardware revision EnumHelper and a dictionary that maps the board's hardware
revision to a list of dictionaries. Each dictionary in this list specifies
the configuration for each LTC2309 chip.
"""

import sys
import textwrap

from makani.avionics.firmware.drivers import ltc2309_types
from makani.avionics.firmware.monitors import generate_monitor_base
from makani.lib.python import c_helpers


select_helper = c_helpers.EnumHelper('Ltc2309Select', ltc2309_types)
mode_helper = c_helpers.EnumHelper('Ltc2309ConversionMode', ltc2309_types)
power_helper = c_helpers.EnumHelper('Ltc2309PowerSavingMode', ltc2309_types)


class Ltc2309DeviceConfig(generate_monitor_base.DeviceConfigBase):
  """Generate LTC2309 voltage/current monitor configuration."""
  # TODO: Add unit tests.

  def __init__(self, config):
    expected_parameters = {
        'name',
        'address',
        'channel',
        'conversion_mode',
        'power_saving',
        'input_divider',
        'input_offset',
        'nominal',
        'min',
        'max'}

    super(Ltc2309DeviceConfig, self).__init__(config, expected_parameters)

  def CheckParameterValues(self, config):
    name = config['name']
    if config['input_divider'] == 0.0:
      assert ValueError('Invalid input_divider specified for %s.' % name)

  def ComputeParameters(self, config):
    """Update per-input configuration with computed data."""
    input_divider = config['input_divider']
    config['volts_per_count'] = 4.096 / 2**12 / input_divider
    config['offset'] = config['input_offset'] / input_divider

    config['binary'] = ltc2309_types.Ltc2309BuildCommand(
        config['channel'], config['conversion_mode'], config['power_saving'])

  def GetConfigAsString(self, config, enum_values, index):
    """Generate an initialization array for the Ltc2309Monitors structure."""
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
    return ['avionics/firmware/monitors/ltc2309_types.h']


def Main(argv):
  """Entry point."""
  flags = generate_monitor_base.ParseFlags(argv)
  config_module = generate_monitor_base.GetConfigModule(flags)
  gen = generate_monitor_base.GenerateMonitorConfig(
      'ltc2309', flags.prefix, Ltc2309DeviceConfig)
  gen.LoadConfig(config_module.ltc2309_config,
                 multiple_configs_per_device=True)
  generate_monitor_base.WriteOutputFiles(flags, gen)


if __name__ == '__main__':
  Main(sys.argv)

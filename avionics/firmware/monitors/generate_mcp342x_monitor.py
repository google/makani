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

"""Generate MCP342X voltage/current monitor configuration.

This module looks for an 'mcp342x_config' tuple in the configuration file
(specified on command line). The mcp342x_config tuple should contain the
hardware revision EnumHelper and a dictionary that maps the board's hardware
revision to a list of dictionaries. Each dictionary in this list specifies
the configuration for each MCP342X chip.
"""

import sys
import textwrap

from makani.avionics.firmware.drivers import mcp342x_types
from makani.avionics.firmware.monitors import generate_monitor_base
from makani.lib.python import c_helpers


channel_helper = c_helpers.EnumHelper('Mcp342xChannel', mcp342x_types)
polarity_helper = c_helpers.EnumHelper('Mcp342xPolarity', mcp342x_types)
mode_helper = c_helpers.EnumHelper('Mcp342xMode', mcp342x_types)
sps_helper = c_helpers.EnumHelper('Mcp342xSps', mcp342x_types)
gain_helper = c_helpers.EnumHelper('Mcp342xGain', mcp342x_types)


class Mcp342xDeviceConfig(generate_monitor_base.DeviceConfigBase):
  """Generate MCP342X ADC monitor configuration."""

  def __init__(self, config):
    expected_parameters = [
        'name',
        'address',
        'channel',
        'polarity',
        'mode',
        'sps',
        'gain']

    super(Mcp342xDeviceConfig, self).__init__(config, expected_parameters)

  def GetConfigAsString(self, config, enum_values, index):
    """Generate initialization code for the Mcp342xMonitorConfig structure."""
    string = textwrap.dedent("""\
        [{index}] = {{
          .monitor = {monitor_name},
          .addr = 0x{address:02X},
          .config = {{
            .channel = {channel_str},
            .polarity = {polarity_str},
            .mode = {mode_str},
            .sps = {sps_str},
            .gain = {gain_str}}}}},
        """).format(
            index=index,
            monitor_name=enum_values['monitor'],
            channel_str=channel_helper.Name(config['channel']),
            polarity_str=polarity_helper.Name(config['polarity']),
            mode_str=mode_helper.Name(config['mode']),
            sps_str=sps_helper.Name(config['sps']),
            gain_str=gain_helper.Name(config['gain']),
            **config)
    return string

  def GetHeaderFiles(self):
    return ['avionics/firmware/monitors/mcp342x_types.h']


def Main(argv):
  """Entry point."""
  flags = generate_monitor_base.ParseFlags(argv)
  config_module = generate_monitor_base.GetConfigModule(flags)
  gen = generate_monitor_base.GenerateMonitorConfig(
      'mcp342x', flags.prefix, Mcp342xDeviceConfig)
  gen.LoadConfig(config_module.mcp342x_config,
                 multiple_configs_per_device=True)
  generate_monitor_base.WriteOutputFiles(flags, gen)


if __name__ == '__main__':
  Main(sys.argv)

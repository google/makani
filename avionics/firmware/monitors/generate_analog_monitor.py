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

"""Generate analog monitor configuration.

This module looks for an 'analog_config' tuple in the configuration file
(specified on command line). The analog_config tuple should contain the
hardware revision EnumHelper and a dictionary that maps the board's hardware
revision to a list of dictionaries. Each dictionary in this list specifies
the configuration for each analog input configuration.
"""

import sys
import textwrap

from makani.avionics.firmware.monitors import analog_types
from makani.avionics.firmware.monitors import generate_monitor_base
from makani.lib.python import c_helpers


type_helper = c_helpers.EnumHelper('AnalogType', analog_types)


class AnalogDeviceConfig(generate_monitor_base.DeviceConfigBase):
  """Generate analog monitor configuration."""
  # TODO: Add unit tests.

  def __init__(self, config):
    expected_parameters = {
        'name',
        'channel',
        'type',
        'input_divider',
        'input_offset',
        'nominal',
        'nominal',
        'min',
        'max'}

    super(AnalogDeviceConfig, self).__init__(config, expected_parameters)

  def CheckParameterValues(self, config):
    name = config['name']
    if config['channel'] < 0 or 23 < config['channel']:
      raise ValueError('Invalid channel specified for %s.' % name)
    if config['type'] not in type_helper:
      raise ValueError('Invalid type specified for %s.' % name)
    if (config['type'] == analog_types.kAnalogTypeVoltage
        and config['input_divider'] == 0.0):
      raise ValueError('Invalid input_divider specified for %s.' % name)

  def ComputeParameters(self, config):
    """Update per-input configuration with computed data."""
    if config['type'] == analog_types.kAnalogTypeVoltage:
      input_divider = config['input_divider']
      volts_per_count = 3.0 / 2**12 / input_divider
      offset = config['input_offset'] / input_divider
    else:
      volts_per_count = 0.0
      offset = 0.0
    config['volts_per_count'] = volts_per_count
    config['offset'] = offset

  def GetConfigAsString(self, config, enum_values, index):
    """Generate an initialization array for the AnalogMonitors structure."""
    string = textwrap.dedent("""\
        [{index}] = {{
          .input = {input_name},
          .voltage = {voltage_name},
          .type = {type_str},
          .channel = {channel},
          .volts_per_count = {volts_per_count}f,
          .offset = {offset}f,
          .nominal = {nominal}f,
          .min = {min}f,
          .max = {max}f}},
        """).format(
            index=index,
            input_name=enum_values['input'],
            voltage_name=enum_values['voltage'],
            type_str=type_helper.Name(config['type']),
            **config)
    return string

  def GetHeaderFiles(self):
    return ['avionics/firmware/monitors/analog_types.h']


def GenerateRevisionFields(devices):
  channel_mask = 0x0
  for device in devices:
    config = device.GetConfig()
    channel_mask |= 1 << config['channel']
  string = textwrap.dedent("""\
      .channel_mask = 0x{channel_mask:08X},
      """)[:-1].format(channel_mask=channel_mask)
  return string


def Main(argv):
  """Entry point."""
  flags = generate_monitor_base.ParseFlags(argv)
  config_module = generate_monitor_base.GetConfigModule(flags)
  gen = generate_monitor_base.GenerateMonitorConfig(
      'analog', flags.prefix, AnalogDeviceConfig, common_group='input',
      get_revision_fields=GenerateRevisionFields)
  is_voltage = lambda x: x['type'] == analog_types.kAnalogTypeVoltage
  gen.CreateGroup('voltage', is_member_function=is_voltage)
  gen.LoadConfig(config_module.analog_config)
  generate_monitor_base.WriteOutputFiles(flags, gen)


if __name__ == '__main__':
  Main(sys.argv)

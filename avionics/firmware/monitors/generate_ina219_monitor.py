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

"""Generate INA219 voltage/current monitor configuration.

This module looks for an 'ina219_config' tuple in the configuration file
(specified on command line). The ina219_config tuple should contain the
hardware revision EnumHelper and a dictionary that maps the board's hardware
revision to a list of dictionaries. Each dictionary in this list specifies
the configuration for each INA219 chip.
"""

import sys
import textwrap

from makani.avionics.firmware.drivers import ina219_types
from makani.avionics.firmware.monitors import generate_monitor_base
from makani.lib.python import c_helpers


adc_helper = c_helpers.EnumHelper('Ina219Adc', ina219_types)
bus_voltage_helper = c_helpers.EnumHelper('Ina219BusVoltage', ina219_types)
mode_helper = c_helpers.EnumHelper('Ina219Mode', ina219_types)
range_helper = c_helpers.EnumHelper('Ina219Range', ina219_types)


class Ina219DeviceConfig(generate_monitor_base.DeviceConfigBase):
  """Generate INA219 voltage/current monitor configuration."""
  # TODO: Add unit tests.

  def __init__(self, config):
    expected_parameters = {
        'name',
        'address',
        'shunt_resistor',
        'bus_voltage',
        'range',
        'bus_adc',
        'shunt_adc',
        'mode',
        'current_max',
        'voltage_limits_percent'}

    super(Ina219DeviceConfig, self).__init__(config, expected_parameters)

  def CheckParameterValues(self, config):
    name = config['name']
    if config['bus_voltage'] not in bus_voltage_helper:
      raise ValueError('Invalid bus_voltage value for %s.' % name)
    if config['range'] not in range_helper:
      raise ValueError('Invalid range value for %s.' % name)
    if config['bus_adc'] not in adc_helper:
      raise ValueError('Invalid bus_adc value for %s.' % name)
    if config['shunt_adc'] not in adc_helper:
      raise ValueError('Invalid shunt_adc value for %s.' % name)
    if config['mode'] not in mode_helper:
      raise ValueError('Invalid mode value for %s.' % name)
    if config['shunt_resistor'] <= 0:
      raise ValueError('Invalid shunt resistor value for %s.' % name)

  def ComputeParameters(self, config):
    # Compute nominal voltage from device name.
    voltage = float(config['name'].split('_', 1)[0].replace('v', '.'))
    voltage_limits_percent = config['voltage_limits_percent']
    config['voltage_nominal'] = voltage
    config['voltage_min'] = voltage * float(voltage_limits_percent[0]) / 100.0
    config['voltage_max'] = voltage * float(voltage_limits_percent[1]) / 100.0
    config['current_max'] = float(config['current_max'])

    # Compute configuration to send to the device.
    config['binary'] = ina219_types.Ina219BuildConfig(
        config['bus_voltage'],
        config['range'],
        config['bus_adc'],
        config['shunt_adc'],
        config['mode'])

  def GetConfigAsString(self, config, enum_values, index):
    string = textwrap.dedent("""\
        [{index}] = {{
          .monitor = {monitor_name},
          .config = {{
            .addr = 0x{address:02X},
            .shunt_resistor = {shunt_resistor}f,
            .config = 0x{binary:04X}}},
          .current_max = {current_max}f,
          .voltage_max = {voltage_max}f,
          .voltage_min = {voltage_min}f,
          .voltage_nominal = {voltage_nominal}f}},
        """).format(
            index=index,
            monitor_name=enum_values['monitor'],
            **config)
    return string

  def GetHeaderFiles(self):
    return ['avionics/firmware/monitors/ina219_types.h']


def Main(argv):
  """Entry point."""
  flags = generate_monitor_base.ParseFlags(argv)
  config_module = generate_monitor_base.GetConfigModule(flags)
  gen = generate_monitor_base.GenerateMonitorConfig(
      'ina219', flags.prefix, Ina219DeviceConfig)
  gen.LoadConfig(config_module.ina219_config)
  generate_monitor_base.WriteOutputFiles(flags, gen)


if __name__ == '__main__':
  Main(sys.argv)

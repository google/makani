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

"""Generate LTC6804 voltage/current monitor configuration.

This module looks for an 'ltc6804_config' tuple in the configuration file
(specified on command line). The ltc6804_config tuple should contain the
hardware revision EnumHelper and a dictionary that maps the board's hardware
revision to a list of dictionaries. Each dictionary in this list specifies
the configuration for each LTC6804 chip.
"""

import sys
import textwrap

from makani.avionics.firmware.drivers import ltc6804_types
from makani.avionics.firmware.monitors import generate_monitor_base
from makani.lib.python import c_helpers

rate_helper = c_helpers.EnumHelper('Ltc6804Rate', ltc6804_types)
cell_ch_helper = c_helpers.EnumHelper('Ltc6804CellCh', ltc6804_types)
aux_ch_helper = c_helpers.EnumHelper('Ltc6804AuxCh', ltc6804_types)
stat_ch_helper = c_helpers.EnumHelper('Ltc6804StatCh', ltc6804_types)
dcto_helper = c_helpers.EnumHelper('Ltc6804Dcto', ltc6804_types)
self_test_helper = c_helpers.EnumHelper('Ltc6804SelfTest', ltc6804_types)


class Ltc6804DeviceConfig(generate_monitor_base.DeviceConfigBase):
  """Generate LTC6804 voltage/current monitor configuration."""
  # TODO: Add unit tests.

  def __init__(self, config):
    expected_parameters = {
        'name',
        'address',
        'input_mask',
        'balance_min_cutoff',
        'balance_thres',
        'balance_hysteresis',
        'max_simult_balance',
        'num_series_cells',
        'under_volt_thres',
        'over_volt_thres',
        'reference_on',
        'discharge_permitted',
        'rate',
        'cell_ch',
        'aux_ch',
        'stat_ch',
        'discharge_timeout',
        'self_test_mode'}

    super(Ltc6804DeviceConfig, self).__init__(config, expected_parameters)

  def GetConfigAsString(self, config, enum_values, index):
    """Generate an initialization array for the Ltc6804Monitors structure."""
    string = textwrap.dedent("""\
        [{index}] = {{
          .monitor = {monitor_name},
          .stack_level = 0x{address:02X},
          .input_mask = 0x{input_mask:03X},
          .v_balance_min = {balance_min_cutoff}f,
          .v_balance_thres = {balance_thres}f,
          .v_balance_hyst = {balance_hysteresis}f,
          .num_max_simult_bal = {max_simult_balance}L,
          .num_series_cells = {num_series_cells}L,
          .control = {{
            .under_volt_thres = {under_volt_thres}f,
            .over_volt_thres = {over_volt_thres}f,
            .reference_on = {reference_on},
            .discharge_permitted = {discharge_permitted},
            .rate = {rate_str},
            .cell_channels = {cell_ch_str},
            .aux_channels = {aux_ch_str},
            .stat_channels = {stat_ch_str},
            .discharge_timeout = {dcto_str},
            .self_test_mode = {self_test_str}}}}},
        """).format(
            index=index,
            monitor_name=enum_values['monitor'],
            rate_str=rate_helper.Name(config['rate']),
            cell_ch_str=cell_ch_helper.Name(config['cell_ch']),
            aux_ch_str=aux_ch_helper.Name(config['aux_ch']),
            stat_ch_str=stat_ch_helper.Name(config['stat_ch']),
            dcto_str=dcto_helper.Name(config['discharge_timeout']),
            self_test_str=self_test_helper.Name(config['self_test_mode']),
            **config)
    return string

  def GetHeaderFiles(self):
    return ['avionics/firmware/monitors/ltc6804_types.h']


def Main(argv):
  """Entry point."""
  flags = generate_monitor_base.ParseFlags(argv)
  config_module = generate_monitor_base.GetConfigModule(flags)
  gen = generate_monitor_base.GenerateMonitorConfig(
      'ltc6804', flags.prefix, Ltc6804DeviceConfig)
  gen.LoadConfig(config_module.ltc6804_config)
  generate_monitor_base.WriteOutputFiles(flags, gen)


if __name__ == '__main__':
  Main(sys.argv)


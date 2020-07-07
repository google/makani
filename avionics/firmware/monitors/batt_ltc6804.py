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

"""Batt LTC6804 hardware monitor configuration."""

from makani.avionics.firmware.drivers import ltc6804_types
from makani.avionics.firmware.serial import batt_serial_params as rev

ltc6804_default = {
    'name': '',
    'address': 0x00,
    'input_mask': 0x000,
    'balance_min_cutoff': 4.0,
    'balance_thres': 0.005,
    'balance_hysteresis': 0.0005,
    'max_simult_balance': 17,
    'num_series_cells': 18,
    'under_volt_thres': 3.0,
    'over_volt_thres': 4.2,
    'reference_on': 'true',
    'discharge_permitted': 'false',
    'rate': ltc6804_types.kLtc6804Rate7kHz,
    'cell_ch': ltc6804_types.kLtc6804CellChAll,
    'aux_ch': ltc6804_types.kLtc6804AuxChVref2,
    'stat_ch': ltc6804_types.kLtc6804StatChAll,
    'discharge_timeout': ltc6804_types.kLtc6804DctoDisable,
    'self_test_mode': ltc6804_types.kLtc6804SelfTest1,
}

cell15 = [
    dict(ltc6804_default, name='stack_level_0', input_mask=0x3BE,
         max_simult_balance=14, num_series_cells=15),
    dict(ltc6804_default, name='stack_level_1', address=0x01,
         input_mask=0x1F6, max_simult_balance=14, num_series_cells=15),
]

cell18 = [
    dict(ltc6804_default, name='stack_level_0', input_mask=0x3FE),
    dict(ltc6804_default, name='stack_level_1', address=0x01,
         input_mask=0x3FE),
]

cell17 = [
    dict(ltc6804_default, name='stack_level_0', input_mask=0x3FE,
         max_simult_balance=16, num_series_cells=17),
    dict(ltc6804_default, name='stack_level_1', address=0x01,
         input_mask=0x1FE, max_simult_balance=16, num_series_cells=17),
]

ltc6804_config = (rev.BattHardware, {
    rev.BattHardware.SMALL_CELL15_V1: cell15,
    rev.BattHardware.BIG_CELL18_V1: cell18,
    rev.BattHardware.SMALL_CELL15_AA: cell15,
    rev.BattHardware.BIG_CELL18_AA: cell18,
    rev.BattHardware.SMALL_CELL15_AB: cell15,
    rev.BattHardware.BIG_CELL18_AB: cell18,
    rev.BattHardware.SMALL_CELL17_AB: cell17,
    rev.BattHardware.SMALL_CELL15_AC: cell15,
    rev.BattHardware.BIG_CELL18_AC: cell18,
    rev.BattHardware.SMALL_CELL17_AC: cell17,
    rev.BattHardware.SMALL_CELL17_AD: cell17,
})

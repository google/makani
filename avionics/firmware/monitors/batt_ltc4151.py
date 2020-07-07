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

"""Batt LTC4151 hardware monitor configuration."""

from makani.avionics.firmware.serial import batt_serial_params as rev

ltc4151_default = {
    'name': '',
    'address': 0x6A,
    'binary_config': 0x0C,
    'current_max': 15.0,
    'voltage_max': 75.0,
    'voltage_min': 0.0,
}

small_cell15_v1 = [
    dict(ltc4151_default, name='charger_output', shunt_resistor=0.02),
]

big_cell18_v1 = small_cell15_v1

small_cell15_aa = [
    dict(ltc4151_default, name='charger_output', shunt_resistor=0.005),
]

big_cell18_aa = small_cell15_aa

empty_set = []

ltc4151_config = (rev.BattHardware, {
    rev.BattHardware.SMALL_CELL15_V1: small_cell15_v1,
    rev.BattHardware.BIG_CELL18_V1: big_cell18_v1,
    rev.BattHardware.SMALL_CELL15_AA: small_cell15_aa,
    rev.BattHardware.BIG_CELL18_AA: big_cell18_aa,
    rev.BattHardware.SMALL_CELL15_AB: empty_set,
    rev.BattHardware.BIG_CELL18_AB: empty_set,
    rev.BattHardware.SMALL_CELL17_AB: empty_set,
    rev.BattHardware.SMALL_CELL15_AC: empty_set,
    rev.BattHardware.BIG_CELL18_AC: empty_set,
    rev.BattHardware.SMALL_CELL17_AC: empty_set,
    rev.BattHardware.SMALL_CELL17_AD: empty_set,
})

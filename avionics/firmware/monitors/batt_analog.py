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

"""Battery management board analog hardware monitor configuration."""

from makani.avionics.firmware.monitors import analog_types
from makani.avionics.firmware.serial import batt_serial_params as rev

# Output value = (input - input_offset) / input_divider.
analog_default = {
    'name': '',
    'channel': -1,
    'type': analog_types.kAnalogTypeVoltage,
    'input_divider': 1.0,
    'input_offset': 0.0,
    'nominal': 0.0,
    'min': 0.0,
    'max': 0.0,
}

voltage = dict(analog_default, type=analog_types.kAnalogTypeVoltage)

# LV limits based on bus range of {min=65, nominal=70, max=80} or
# (Below 65V, you're either close to depleting the big box, or running off
# the small box -- time to land the wing immediately.)
v_lv = dict(voltage, nominal=70.0, max=80.0)  # [V]
i_lv_or = dict(voltage, nominal=0.0, min=0.0, max=4.0)  # [A]
i_chg = dict(voltage, nominal=0.0, min=0.0, max=15.0)  # [A]
v_12v = dict(voltage, nominal=12.0, min=12.0 * 0.95, max=12.0 * 1.05)  # [V]
v_5v = dict(voltage, nominal=5.0, min=5.0 * 0.95, max=5.0 * 1.05)  # [V]
i_hall = dict(voltage, nominal=0.0, min=-1.5, max=50.0)  # [A]

common_set = [
    dict(v_lv, name='v_lv_or', min=65.0, channel=16,
         input_divider=10.0 / 497.0),
    dict(v_12v, name='12v', channel=18, input_divider=23.2 / 123.2),
    dict(v_5v, name='5v', channel=19, input_divider=23.2 / 123.2),
    dict(i_hall, name='i_hall', channel=21, input_divider=3.3 * 0.8 / 100,
         input_offset=0.12 * 3.3),  # Allegro ACS758 datasheet.
]

small_cell15 = common_set + [
    dict(v_lv, name='lv_a', min=66.0, channel=14, input_divider=10.0 / 497.0),
    dict(v_lv, name='lv_b', min=55.0, channel=15, input_divider=10.0 / 497.0),
]

small_v1_aa = small_cell15 + [
    dict(i_lv_or, name='i_lv_or', channel=17, input_divider=0.132,
         input_offset=0.357),
]

big_cell18 = common_set + [
    dict(v_lv, name='lv_a', min=55.0, channel=14, input_divider=10.0 / 497.0),
    dict(v_lv, name='lv_b', min=66.0, channel=15, input_divider=10.0 / 497.0),
]

big_v1_aa = big_cell18 + [
    dict(i_lv_or, name='i_lv_or', channel=17, input_divider=0.132,
         input_offset=0.357),
]

small_cell15_ab = small_cell15 + [
    dict(i_chg, name='i_chg', channel=17, input_divider=0.132,
         input_offset=0.357),
]

big_cell18_ab = big_cell18 + [
    dict(i_chg, name='i_chg', channel=17, input_divider=0.132,
         input_offset=0.357),
]

cell17_common = common_set + [
    dict(v_lv, name='lv_a', min=66.0, channel=14, input_divider=10.0 / 497.0),
    dict(v_lv, name='lv_b', min=62.3, channel=15, input_divider=10.0 / 497.0),
]

small_cell17_ab = cell17_common + [
    dict(i_chg, name='i_chg', channel=17, input_divider=0.132,
         input_offset=0.357),
]

small_cell17_ad = cell17_common + [
    dict(i_chg, name='i_chg', channel=17, input_divider=0.143,
         input_offset=0.357),
]

analog_config = (rev.BattHardware, {
    rev.BattHardware.SMALL_CELL15_V1: small_v1_aa,
    rev.BattHardware.BIG_CELL18_V1: big_v1_aa,
    rev.BattHardware.SMALL_CELL15_AA: small_v1_aa,
    rev.BattHardware.BIG_CELL18_AA: big_v1_aa,
    rev.BattHardware.SMALL_CELL15_AB: small_cell15_ab,
    rev.BattHardware.BIG_CELL18_AB: big_cell18_ab,
    rev.BattHardware.SMALL_CELL17_AB: small_cell17_ab,
    # The AC revs are functionally identical to the AB revs.
    rev.BattHardware.SMALL_CELL15_AC: small_cell15_ab,
    rev.BattHardware.BIG_CELL18_AC: big_cell18_ab,
    rev.BattHardware.SMALL_CELL17_AC: small_cell17_ab,
    rev.BattHardware.SMALL_CELL17_AD: small_cell17_ad,
})

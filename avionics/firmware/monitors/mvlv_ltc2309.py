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

"""Mvlv LTC2309 hardware monitor configuration."""

from makani.avionics.firmware.drivers import ltc2309_types
from makani.avionics.firmware.serial import mvlv_serial_params as rev

default_config = {
    'name': '',
    'address': -1,
    'channel': ltc2309_types.kLtc2309SelectSingleCh0,
    'conversion_mode': ltc2309_types.kLtc2309Unipolar,
    'power_saving': ltc2309_types.kLtc2309NapMode,
    'input_divider': 1.0,
    'input_offset': 0.0,
    'nominal': 0.0,
    'min': 0.0,
    'max': 0.0,
}

addr_0x18 = dict(default_config, address=0x18)

v_diff = {'conversion_mode': ltc2309_types.kLtc2309Unipolar,
          'input_divider': 54.9 / (1000 + 54.9) / 100,
          'nominal': 3400.0, 'min': 3000.0, 'max': 3800.0}  # [V]
v_com = {'conversion_mode': ltc2309_types.kLtc2309Unipolar,
         'input_divider': 21.0 / 20 * 54.9 / (1000 + 2 * 54.9) / 100,
         'nominal': 1700.0, 'min': 1500.0, 'max': 1900.0}  # [V]
i_peak = {'conversion_mode': ltc2309_types.kLtc2309Unipolar,
          'input_divider': 0.05,
          'nominal': 15.0, 'min': 0.0, 'max': 30.0}  # [A]

common = [
    dict(dict(default_config, **i_peak), address=0x18, name='i_pos_peak',
         channel=ltc2309_types.kLtc2309SelectDiffCh2Ch3),
    dict(dict(default_config, **i_peak), address=0x18, name='i_neg_peak',
         channel=ltc2309_types.kLtc2309SelectDiffCh1Ch0),
    dict(dict(default_config, **v_com), address=0x18, name='v_pos',
         channel=ltc2309_types.kLtc2309SelectDiffCh4Ch5),
    dict(dict(default_config, **v_diff), address=0x18, name='v_diff',
         channel=ltc2309_types.kLtc2309SelectDiffCh6Ch7),
]

ltc2309_config = (rev.MvlvHardware, {
    rev.MvlvHardware.SYNC_RECT_REV_A1: common,
})

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

"""Core switch analog hardware monitor configuration."""

from makani.avionics.firmware.monitors import analog_types
from makani.avionics.firmware.serial import cs_serial_params as rev

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
logic_low = dict(analog_default, type=analog_types.kAnalogTypeLogicLow)
logic_high = dict(analog_default, type=analog_types.kAnalogTypeLogicHigh)
port_detect = dict(analog_default, type=analog_types.kAnalogTypePortDetect)

v_in = dict(voltage, nominal=12.0, min=10.0, max=14.0)  # [V]

rev_aa = [
    dict(v_in, name='v_in', channel=6, input_divider=14.3 / 114.3),
    dict(v_in, name='v_aux', channel=7, input_divider=14.3 / 114.3),
    dict(logic_low, name='power_not_good_3v3', channel=8),
    dict(logic_low, name='power_not_good_2v5', channel=9),
    dict(logic_low, name='power_not_good_1v2', channel=10),
    dict(logic_low, name='hilt_detect', channel=12),
    dict(logic_high, name='sfp_aux_mod_abs', channel=15),
    dict(logic_high, name='sfp_mod_abs', channel=16),
]

rev_ac = [
    dict(v_in, name='v_in', channel=6, input_divider=14.3 / 114.3),
    dict(logic_low, name='power_not_good_3v3', channel=8),
    dict(logic_low, name='power_not_good_2v5', channel=9),
    dict(logic_low, name='power_not_good_1v2', channel=10),
    dict(logic_low, name='hilt_detect', channel=12),
    dict(logic_high, name='sfp_aux_mod_abs', channel=15),
    dict(logic_high, name='sfp_mod_abs', channel=16),
    dict(logic_high, name='radio_signal_1', channel=20),
    dict(logic_high, name='radio_signal_2', channel=21),
    dict(logic_high, name='radio_signal_3', channel=22),
    dict(logic_high, name='radio_status', channel=23),
]

analog_config = (rev.CsHardware, {
    rev.CsHardware.REV_AA: rev_aa,
    rev.CsHardware.REV_AB: rev_aa,  # No changes from rev_aa.
    rev.CsHardware.REV_AC: rev_ac,
    rev.CsHardware.REV_AD_CLK8: rev_ac,  # No changes from rev_ac
    rev.CsHardware.REV_AD_CLK16: rev_ac,  # No changes from rev_ac
})

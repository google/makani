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

"""Loadcell analog hardware monitor configuration."""

from makani.avionics.firmware.monitors import analog_types
from makani.avionics.firmware.serial import loadcell_serial_params as rev

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
logic_high = dict(analog_default, type=analog_types.kAnalogTypeLogicHigh)

v_5v = dict(voltage, nominal=5.0, min=5.0 * 0.95, max=5.0 * 1.05)  # [V]
# Allowable current range based on the allowable zero error range over temp of
# +/- 60 mV on the current sensor.
i_batt = dict(voltage, nominal=0.0, min=-0.5, max=0.5)  # [A]
v_aoa = dict(voltage, nominal=1.5, min=0.0, max=3.0)  # [V]
v_release = dict(voltage, nominal=0.0, min=0.0, max=30.0)  # [V]

rev_ab = [
    dict(i_batt, name='i_batt', channel=14, input_divider=0.095,
         input_offset=0.357),
    dict(v_release, name='v_release', channel=15, input_divider=9.31 / 109.31),
    dict(v_aoa, name='v_aoa_1', channel=16),
    dict(v_aoa, name='v_aoa_2', channel=17),
    dict(v_5v, name='5v', channel=19, input_divider=14.3 / 43.0),
    dict(v_release, name='v_arm', channel=20, input_divider=9.31 / 109.31),
    dict(voltage, name='v_batt_test', channel=21, input_divider=9.31 / 109.31),
    dict(logic_high, name='eeprom_wp', channel=23),
]

rev_aa = rev_ab + [
    dict(v_5v, name='v_loadcell_bias', channel=18, input_divider=14.3 / 43.0),
]

analog_config = (rev.LoadcellHardware, {
    rev.LoadcellHardware.REV_AA: rev_aa,
    rev.LoadcellHardware.REV_AB: rev_ab
})

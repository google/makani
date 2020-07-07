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

"""Joystick analog hardware monitor configuration."""

from makani.avionics.firmware.monitors import analog_types
from makani.avionics.firmware.serial import joystick_serial_params as rev

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

v_in = dict(voltage, nominal=12.0, min=10.0, max=14.0)  # [V]

rev_aa = [
    dict(v_in, name='lv_a', channel=14, input_divider=23.2 / 123.2),
    dict(v_in, name='lv_b', channel=15, input_divider=23.2 / 123.2),
    dict(logic_high, name='eeprom_wp', channel=23),
]

analog_config = (rev.JoystickHardware, {
    rev.JoystickHardware.REV_AA: rev_aa,
})

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

"""Recorder analog hardware monitor configuration."""

from makani.avionics.firmware.monitors import analog_types
from makani.avionics.firmware.serial import recorder_serial_params as rev

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

v_3v3 = dict(voltage, nominal=3.3, min=3.3 * 0.95, max=3.3 * 1.05)  # [V]

rev_ba = [
    dict(v_3v3, name='3v3_sata', channel=14, input_divider=14.3 / 43.0),
    dict(logic_low, name='q7_thermal_trip', channel=16),
    dict(logic_high, name='eeprom_wp', channel=23),
]

analog_config = (rev.RecorderHardware, {
    rev.RecorderHardware.REV_AA: [],
    rev.RecorderHardware.REV_BA: rev_ba,
})

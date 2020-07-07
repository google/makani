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

"""Flight computer analog hardware monitor configuration."""

from makani.avionics.firmware.monitors import analog_types
from makani.avionics.firmware.serial import fc_serial_params as rev

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
v_3v3 = dict(voltage, nominal=3.3, min=3.3 * 0.95, max=3.3 * 1.05)  # [V]
v_6v = dict(voltage, nominal=6.0, min=6.0 * 0.95, max=6.0 * 1.05)  # [V]

rev_ab = [
    dict(logic_high, name='power_not_good', channel=5),
    dict(v_in, name='v_in', channel=6, input_divider=14.3 / 114.3),
    dict(v_in, name='v_aux', channel=7, input_divider=14.3 / 114.3),
    dict(port_detect, name='port_detect_0', channel=8),
    dict(port_detect, name='port_detect_1', channel=9),
    dict(logic_low, name='q7_thermal_trip', channel=11),
    dict(logic_low, name='hilt_detect', channel=12),
    dict(logic_low, name='inst_detect', channel=13),
]

rev_ba = [
    dict(v_6v, name='6v_lna', channel=14, input_divider=14.3 / 43.0),
    dict(v_3v3, name='3v3_gps', channel=15, input_divider=14.3 / 43.0),
    dict(logic_low, name='q7_thermal_trip', channel=16),
    dict(v_3v3, name='3v3_imu', channel=17, input_divider=14.3 / 43.0),
]

analog_config = (rev.FcHardware, {
    rev.FcHardware.REV_AB: rev_ab,
    rev.FcHardware.REV_BA: rev_ba,
    rev.FcHardware.REV_BB: rev_ba,  # No changes from REV_BA.
    rev.FcHardware.REV_BC: rev_ba,  # No changes from REV_BA.
    rev.FcHardware.REV_BD: rev_ba,  # No changes from REV_BA.
})

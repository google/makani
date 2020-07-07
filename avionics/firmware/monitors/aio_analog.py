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

"""AIO module analog hardware monitor configuration."""

from makani.avionics.firmware.monitors import analog_types
from makani.avionics.firmware.serial import aio_serial_params as rev

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
logic_low = dict(analog_default, type=analog_types.kAnalogTypeLogicLow)
logic_high = dict(analog_default, type=analog_types.kAnalogTypeLogicHigh)
port_detect = dict(analog_default, type=analog_types.kAnalogTypePortDetect)
v_2v5 = dict(voltage, nominal=2.5, min=2.5 * 0.95, max=2.5 * 1.05)  # [V]
v_5v = dict(voltage, nominal=5.0, min=5.0 * 0.95, max=5.0 * 1.05)  # [V]
v_rssi = dict(voltage, nominal=0.1, min=0.0, max=1.0, input_divider=0.9)  # [mW]

rev_aa = [
    dict(logic_low, name='watchdog_enabled', channel=3),
    dict(port_detect, name='port_detect_0', channel=8),
    dict(port_detect, name='port_detect_1', channel=9),
    dict(port_detect, name='port_detect_2', channel=10),
    dict(port_detect, name='port_detect_3', channel=11),
    dict(logic_low, name='gti_detect', channel=12),
    dict(v_2v5, name='2v5', channel=13, input_divider=1.0),
]

rev_ac = [
    dict(port_detect, name='port_detect_0', channel=8),
    dict(port_detect, name='port_detect_1', channel=9),
    dict(port_detect, name='port_detect_2', channel=10),
    dict(port_detect, name='port_detect_3', channel=11),
    dict(logic_low, name='gti_detect', channel=12),
    dict(v_2v5, name='2v5', channel=13, input_divider=1.0),
]

rev_ad = [
    dict(v_5v, name='5v', channel=3, input_divider=10.0 / 20.0),
    dict(port_detect, name='port_detect_0', channel=8),
    dict(port_detect, name='port_detect_1', channel=9),
    dict(port_detect, name='port_detect_2', channel=10),
    dict(port_detect, name='port_detect_3', channel=11),
    dict(logic_low, name='gti_detect', channel=12),
    dict(v_2v5, name='2v5', channel=13, input_divider=1.0),
]

rev_ba = [
    dict(v_5v, name='5v', channel=3, input_divider=10.0 / 20.0),
    dict(v_rssi, name='port_rssi_0', channel=8),
    dict(v_rssi, name='port_rssi_1', channel=9),
    dict(v_rssi, name='port_rssi_2', channel=10),
    dict(v_rssi, name='port_rssi_3', channel=11),
    dict(logic_low, name='gti_detect', channel=12),
    dict(v_2v5, name='2v5', channel=13, input_divider=1.0),
]

analog_config = (rev.AioHardware, {
    rev.AioHardware.REV_AA: rev_aa,
    rev.AioHardware.REV_AB: rev_aa,  # No changes from rev_aa.
    rev.AioHardware.REV_AC: rev_ac,
    rev.AioHardware.REV_AD: rev_ad,
    rev.AioHardware.REV_BA: rev_ba,
})

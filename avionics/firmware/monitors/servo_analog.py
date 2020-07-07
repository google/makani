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

"""Servo analog hardware monitor configuration."""

from makani.avionics.firmware.monitors import analog_types
from makani.avionics.firmware.serial import servo_serial_params as rev

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
port_detect = dict(analog_default, type=analog_types.kAnalogTypePortDetect)

# LV limits based on bus range of {min=65, nominal=75, max=77} and battery
# range of {min=55, nominal=60, max=67.2}. Added on 2016-01-29.
v_lv = dict(voltage, nominal=75.0, min=55.0, max=77.0)  # [V]
# Vservo minimum based on 10 RPM (~1 rad/sec) at 50V for SPA-25-160-SP4041.
# Maximum based on clamp circuit turn-on voltage.
v_servo = dict(voltage, nominal=70.0, min=50.0, max=85.0)  # [V]
i_servo = dict(voltage, nominal=0.0, min=0.0, max=10.0)  # [A]
v_12v = dict(voltage, nominal=12.0, min=12.0 * 0.95, max=12.0 * 1.05)  # [V]
v_5v = dict(voltage, nominal=5.0, min=5.0 * 0.95, max=5.0 * 1.05)  # [V]

rev_aa = [
    dict(v_lv, name='lv_a', channel=6, input_divider=10.0 / 400.0),
    dict(v_lv, name='lv_b', channel=7, input_divider=10.0 / 400.0),
    dict(v_servo, name='v_servo', channel=8, input_divider=10.0 / 400.0),
    dict(i_servo, name='i_servo', channel=9, input_divider=0.060,
         input_offset=1.5),
    dict(port_detect, name='port_detect_0', channel=10),
    dict(port_detect, name='port_detect_1', channel=11),
    dict(port_detect, name='port_detect_2', channel=12),
    dict(port_detect, name='port_detect_3', channel=13),
    dict(logic_low, name='hilt_detect', channel=14),
]

rev_ba = [
    dict(v_lv, name='lv_a', channel=14, input_divider=10.0 / 497.0),
    dict(v_lv, name='lv_b', channel=15, input_divider=10.0 / 497.0),
    dict(v_servo, name='v_servo', channel=16, input_divider=10.0 / 497.0),
    dict(i_servo, name='i_servo', channel=17, input_divider=0.132,
         input_offset=0.357),
    dict(v_12v, name='12v', channel=18, input_divider=23.2 / 123.2),
    dict(v_5v, name='5v', channel=19, input_divider=23.2 / 123.2),
    dict(voltage, name='clamp_resistor', channel=20),
    dict(port_detect, name='port_detect_4', channel=21),
]

rev_bb = [
    dict(v_lv, name='lv_a', channel=14, input_divider=10.0 / 497.0),
    dict(v_lv, name='lv_b', channel=15, input_divider=10.0 / 497.0),
    dict(v_servo, name='v_servo', channel=16, input_divider=10.0 / 497.0),
    dict(i_servo, name='i_servo', channel=17, input_divider=0.132,
         input_offset=0.357),
    dict(v_12v, name='12v', channel=18, input_divider=23.2 / 123.2),
    dict(v_5v, name='5v', channel=19, input_divider=23.2 / 123.2),
    dict(voltage, name='clamp_resistor', channel=20),
]

analog_config = (rev.ServoHardware, {
    rev.ServoHardware.REV_AA: rev_aa,
    rev.ServoHardware.REV_BA: rev_ba,
    rev.ServoHardware.REV_BB: rev_bb,
    rev.ServoHardware.REV_BC: rev_bb,
})

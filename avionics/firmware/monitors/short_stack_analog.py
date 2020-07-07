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

"""Short-stack board analog hardware monitor configuration."""

from makani.avionics.firmware.monitors import analog_types
from makani.avionics.firmware.serial import short_stack_serial_params as rev

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

v_72vfire = dict(voltage, nominal=70.0, min=4.0, max=80.0)  # [V]
v_5v = dict(voltage, nominal=5.0, min=5.0 * 0.95, max=5.0 * 1.05)  # [V]
v_3v3 = dict(voltage, nominal=3.3, min=3.3 * 0.95, max=3.3 * 1.05)  # [V]
v_block = dict(voltage, nominal=1200.0, min=0.0, max=2000.0,  # [V]
               input_divider=(20.0 / (10000.0 + 20.0)) * (20.0 / (20.0 + 10.0)))
v_full_stack = dict(voltage, nominal=4400.0, min=0.0, max=6000.0,  # [V]
                    input_divider=(20.0 / 10020.0) * (1.03 / (1.03 + 3.09)))
v_frame = dict(voltage, nominal=0.0, min=-3000.0, max=3000.0,  # [V]
               input_divider=(20.0 / 10020.0) * (1.03 / (1.03 + 3.09)),
               input_offset=1.37)

common_set = [
    dict(v_block, name='block0', channel=17),  # +MV/2 to +MV.
    dict(v_block, name='block1', channel=15),  # -MV/2 to MID.
    dict(v_block, name='block2', channel=14),  # -MV to -MV/2.
    dict(v_block, name='block3', channel=16),  # MID to +MV/2.
    dict(v_full_stack, name='main', channel=18),
    dict(v_frame, name='frame', channel=19),
    dict(v_72vfire, name='72vfire', channel=20, input_divider=10.0 / 497.0),
    dict(v_3v3, name='3v3', channel=21, input_divider=23.2 / 123.2),
    dict(v_5v, name='5v', channel=22, input_divider=23.2 / 123.2),
]

rev_01 = common_set

analog_config = (rev.ShortStackHardware, {
    rev.ShortStackHardware.REV01: rev_01,
})

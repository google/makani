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

"""MV-LV synchronous rectifier board analog hardware monitor configuration."""

from makani.avionics.firmware.monitors import analog_types
from makani.avionics.firmware.serial import mvlv_serial_params as rev

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

# LV limits based on schematics (https://goo.gl/CKHdNW).
# LV buses {min=31, nominal=72, max=100} as shown on page 13.
# LV OR {min=31, nominal=72, max=87.5} as shown on page 15.
# LV is regulated to +-2V so {min=70V, nominal=72, max=75}.
v_lv = dict(voltage, nominal=72.0, min=70.0, max=75.0)  # [V].
v_lv_bus = dict(v_lv, min=31.0, max=100.0)  # [V].
v_lv_or = dict(v_lv_bus, max=87.5)  # [V].
v_12v = dict(voltage, nominal=12.0, min=12.0 * 0.95, max=12.0 * 1.05)  # [V].
v_5v = dict(voltage, nominal=5.0, min=5.0 * 0.95, max=5.0 * 1.05)  # [V].
v_3v3 = dict(voltage, nominal=3.3, min=3.3 * 0.95, max=3.3 * 1.05)  # [V].
i_hall = dict(voltage, nominal=0.0, min=-1.5, max=80.0)  # [A] output current.

common_set = [
    # Divider setting is based on schematic (https://goo.gl/CKHdNW) page 14.
    dict(v_lv_bus, name='v_lv_pri', channel=16,
         input_divider=10.0 / 497.0),  # Primary LV bus.
    dict(v_lv_or, name='v_lv_or', channel=18,
         input_divider=10.0 / 497.0),  # Secondary LV ORed with MV-LV bus.
    dict(v_lv_bus, name='v_lv_sec', channel=20,
         input_divider=10.0 / 497.0),  # Secondary LV bus.
    dict(v_lv, name='v_lv', channel=21,
         input_divider=10.0 / 497.0),  # MV-LV local LV bus.
    dict(v_12v, name='12v', channel=22, input_divider=23.2 / 123.2),
    dict(v_5v, name='5v', channel=14, input_divider=23.2 / 123.2),
    dict(v_3v3, name='3v3', channel=17, input_divider=1.65 / 3.30),
    # Allegro ACS758 offset = 3v3 * 0.12, divider = 3v3 * 0.040 / 5.
    # Datasheet (page 16) : http://goo.gl/abqaiy.
    # Schematic (page 7) : https://goo.gl/CKHdNW.
    dict(i_hall, name='i_hall', channel=19, input_divider=3.3 * 0.8 / 100,
         input_offset=0.12 * 3.3),  # Allegro ACS758 datasheet.
]

sync_rect_rev_a1 = common_set + [
    dict(voltage, name='v_ext', channel=15,
         input_divider=1.0, min=-1.0, max=5.5),  # ADC on extension connector.
]

analog_config = (rev.MvlvHardware, {
    rev.MvlvHardware.SYNC_RECT_REV_A1: sync_rect_rev_a1,
})

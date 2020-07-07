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

"""Ground I/O ADS7828 hardware monitor configuration."""

from makani.avionics.firmware.drivers import ads7828_types
from makani.avionics.firmware.serial import ground_io_serial_params as rev

default_config = {
    'name': '',
    'address': -1,
    'channel': ads7828_types.kAds7828SelectSingleCh0,
    'converter': ads7828_types.kAds7828PowerConverterOn,
    'reference': ads7828_types.kAds7828PowerReferenceOn,
    'input_divider': 4.99 / (4.99 + 45.3),
    'input_offset': 0.0,
    'nominal': 0.0,
    'min': 0.0,
    'max': 0.0,
}

addr_0x49 = dict(default_config, address=0x49)
addr_0x4b = dict(default_config, address=0x4B)

v_in = {'nominal': 12.0, 'min': 10.0, 'max': 14.0}  # [V]

rev_aa = [
    dict(dict(addr_0x49, **v_in), name='lv_a',
         channel=ads7828_types.kAds7828SelectSingleCh0),
    dict(dict(addr_0x49, **v_in), name='lv_b',
         channel=ads7828_types.kAds7828SelectSingleCh1),
    dict(addr_0x49, name='can2_power',
         channel=ads7828_types.kAds7828SelectSingleCh2),
    dict(addr_0x49, name='can3_power',
         channel=ads7828_types.kAds7828SelectSingleCh3),
    dict(addr_0x49, name='uart1_power',
         channel=ads7828_types.kAds7828SelectSingleCh4),
    dict(addr_0x49, name='uart2_power',
         channel=ads7828_types.kAds7828SelectSingleCh5),
    dict(addr_0x49, name='analog_in1',
         channel=ads7828_types.kAds7828SelectSingleCh6),
    dict(addr_0x49, name='analog_in2',
         channel=ads7828_types.kAds7828SelectSingleCh7),
    dict(addr_0x4b, name='analog_in3',
         channel=ads7828_types.kAds7828SelectSingleCh0),
    dict(addr_0x4b, name='analog_in4',
         channel=ads7828_types.kAds7828SelectSingleCh1),
    dict(addr_0x4b, name='enc_power1',
         channel=ads7828_types.kAds7828SelectSingleCh2),
    dict(addr_0x4b, name='enc_power2',
         channel=ads7828_types.kAds7828SelectSingleCh3),
    dict(addr_0x4b, name='enc_power3',
         channel=ads7828_types.kAds7828SelectSingleCh4),
    dict(addr_0x4b, name='enc_power4',
         channel=ads7828_types.kAds7828SelectSingleCh5),
    dict(addr_0x4b, name='enc_power5',
         channel=ads7828_types.kAds7828SelectSingleCh6),
    dict(addr_0x4b, name='enc_power6',
         channel=ads7828_types.kAds7828SelectSingleCh7),
]

ads7828_config = (rev.GroundIoHardware, {
    rev.GroundIoHardware.REV_AA: rev_aa,
})

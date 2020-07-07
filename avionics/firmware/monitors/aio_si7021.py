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

"""AIO module SI7021 hardware monitor configuration."""

from makani.avionics.firmware.drivers import si7021_types
from makani.avionics.firmware.serial import aio_serial_params as rev

si7021_default = {
    'name': '',
    'address': -1,
    'resolution': si7021_types.kSi7021ResolutionRh12BitTemp14Bit,
}

rev_ab = [
    dict(si7021_default, name='board', address=0x40),
]

si7021_config = (rev.AioHardware, {
    rev.AioHardware.REV_AA: [],  # Not supported.
    rev.AioHardware.REV_AB: rev_ab,
    rev.AioHardware.REV_AC: rev_ab,  # Same as rev_ab.
    rev.AioHardware.REV_AD: rev_ab,  # Same as rev_ab.
    rev.AioHardware.REV_BA: rev_ab,  # Same as rev_ab.
})

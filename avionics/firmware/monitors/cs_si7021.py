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

"""Core switch SI7021 hardware monitor configuration."""

from makani.avionics.firmware.drivers import si7021_types
from makani.avionics.firmware.serial import cs_serial_params as rev

si7021_default = {
    'name': '',
    'address': -1,
    'resolution': si7021_types.kSi7021ResolutionRh12BitTemp14Bit,
}

rev_ac = [
    dict(si7021_default, name='board', address=0x40),
]

si7021_config = (rev.CsHardware, {
    rev.CsHardware.REV_AA: [],  # Not supported.
    rev.CsHardware.REV_AB: [],  # Not supported.
    rev.CsHardware.REV_AC: rev_ac,
    rev.CsHardware.REV_AD_CLK8: rev_ac,  # No changes from rev_ac
    rev.CsHardware.REV_AD_CLK16: rev_ac,  # No changes from rev_ac
})

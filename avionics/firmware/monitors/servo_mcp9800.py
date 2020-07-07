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

"""Servo MCP9800 hardware monitor configuration."""

from makani.avionics.firmware.drivers import mcp9800_types
from makani.avionics.firmware.serial import servo_serial_params as rev

mcp9800_default = {
    'name': '',
    'address': -1,
    'resolution': mcp9800_types.kMcp9800Resolution0C0625,
}

rev_ba = [
    dict(mcp9800_default, name='cold_junction', address=0x48),
]

mcp9800_config = (rev.ServoHardware, {
    rev.ServoHardware.REV_AA: [],  # Not supported.
    rev.ServoHardware.REV_BA: rev_ba,
    rev.ServoHardware.REV_BB: rev_ba,  # Same as rev_ba.
    rev.ServoHardware.REV_BC: rev_ba,  # Same as rev_ba.
})

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

"""Motor MCP9800 hardware monitor configuration."""

from makani.avionics.firmware.drivers import mcp9800_types
from makani.avionics.firmware.serial import motor_serial_params as rev

mcp9800_default = {
    'name': '',
    'address': -1,
    'resolution': mcp9800_types.kMcp9800Resolution0C0625,
}

gin_a1 = [
    dict(mcp9800_default, name='board', address=0x48),
    dict(mcp9800_default, name='controller_air', address=0x4D),
]

gin_a2 = gin_a1

gin_a3 = [
    dict(mcp9800_default, name='controller_air', address=0x4D),
]

mcp9800_config = (rev.MotorHardware, {
    rev.MotorHardware.GIN_A1: gin_a1,
    rev.MotorHardware.GIN_A2: gin_a2,
    rev.MotorHardware.GIN_A3: gin_a3,
    rev.MotorHardware.GIN_A4_CLK16: gin_a3,
    rev.MotorHardware.GIN_A4_CLK8: gin_a3,
    rev.MotorHardware.OZONE_A1: gin_a3,
})

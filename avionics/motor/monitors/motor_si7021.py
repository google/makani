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

"""Motor controller SI7021 hardware monitor configuration."""

from makani.avionics.firmware.drivers import si7021_types
from makani.avionics.firmware.serial import motor_serial_params as rev

si7021_default = {
    'name': '',
    'address': -1,
    'resolution': si7021_types.kSi7021ResolutionRh12BitTemp14Bit,
}

gin_a3 = [
    dict(si7021_default, name='board', address=0x40),
]

si7021_config = (rev.MotorHardware, {
    rev.MotorHardware.GIN_A1: [],  # Not supported.
    rev.MotorHardware.GIN_A2: [],  # Not supported.
    rev.MotorHardware.GIN_A3: gin_a3,
    rev.MotorHardware.GIN_A4_CLK16: gin_a3,
    rev.MotorHardware.GIN_A4_CLK8: gin_a3,
    rev.MotorHardware.OZONE_A1: gin_a3,
})

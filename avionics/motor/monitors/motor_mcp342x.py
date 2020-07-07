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

"""Motor MCP342X hardware monitor configuration."""

from makani.avionics.firmware.drivers import mcp342x_types
from makani.avionics.firmware.serial import motor_serial_params as rev
from makani.avionics.motor.firmware import config_params

mcp342x_default = {
    'name': '',
    'address': 0x0,
    'channel': mcp342x_types.kMcp342xChannel1,
    'polarity': mcp342x_types.kMcp342xPolarityPositive,
    'mode': mcp342x_types.kMcp342xModeSingle,
    'sps': mcp342x_types.kMcp342xSps15,
    'gain': mcp342x_types.kMcp342xGain1X,
}

ch1_pos = dict(mcp342x_default, **{
    'channel': mcp342x_types.kMcp342xChannel1,
    'polarity': mcp342x_types.kMcp342xPolarityPositive,
})

ch1_neg = dict(mcp342x_default, **{
    'channel': mcp342x_types.kMcp342xChannel1,
    'polarity': mcp342x_types.kMcp342xPolarityNegative,
})

ch2_pos = dict(mcp342x_default, **{
    'channel': mcp342x_types.kMcp342xChannel2,
    'polarity': mcp342x_types.kMcp342xPolarityPositive,
})

ch2_neg = dict(mcp342x_default, **{
    'channel': mcp342x_types.kMcp342xChannel2,
    'polarity': mcp342x_types.kMcp342xPolarityNegative,
})

ch3_pos = dict(mcp342x_default, **{
    'channel': mcp342x_types.kMcp342xChannel3,
    'polarity': mcp342x_types.kMcp342xPolarityPositive,
})

ch3_neg = dict(mcp342x_default, **{
    'channel': mcp342x_types.kMcp342xChannel3,
    'polarity': mcp342x_types.kMcp342xPolarityNegative,
})

ch4_pos = dict(mcp342x_default, **{
    'channel': mcp342x_types.kMcp342xChannel4,
    'polarity': mcp342x_types.kMcp342xPolarityPositive,
})

ch4_neg = dict(mcp342x_default, **{
    'channel': mcp342x_types.kMcp342xChannel4,
    'polarity': mcp342x_types.kMcp342xPolarityNegative,
})

controller = [
    dict(ch3_pos, name='capacitor', address=0x68),
    dict(ch1_pos, name='heat_plate_1', address=0x68),
    dict(ch4_pos, name='heat_plate_2', address=0x68),
]

protean = [
    dict(ch1_neg, name='protean_stator_1', address=0x6C),
    dict(ch2_neg, name='protean_stator_2', address=0x6C),
    dict(ch2_pos, name='protean_stator_3', address=0x68),
]

yasa = [
    dict(ch2_pos, name='yasa_pylon_ambient', address=0x68),
    dict(ch3_neg, name='yasa_rotor', address=0x6C),
    dict(ch2_neg, name='yasa_stator_coil', address=0x6C),
    dict(ch1_neg, name='yasa_stator_core', address=0x6C),
]

protean_config = (rev.MotorHardware, {
    rev.MotorHardware.GIN_A1: controller + protean,
    rev.MotorHardware.GIN_A2: controller + protean,
    rev.MotorHardware.GIN_A3: controller + protean,
    rev.MotorHardware.GIN_A4_CLK16: controller + protean,
    rev.MotorHardware.GIN_A4_CLK8: controller + protean,
    rev.MotorHardware.OZONE_A1: controller + protean,
})

yasa_config = (rev.MotorHardware, {
    rev.MotorHardware.GIN_A1: controller + yasa,
    rev.MotorHardware.GIN_A2: controller + yasa,
    rev.MotorHardware.GIN_A3: controller + yasa,
    rev.MotorHardware.GIN_A4_CLK16: controller + yasa,
    rev.MotorHardware.GIN_A4_CLK8: controller + yasa,
    rev.MotorHardware.OZONE_A1: controller + yasa,
})

mcp342x_config = (config_params.MotorType, {
    config_params.MotorType.PROTEAN: protean_config,
    config_params.MotorType.YASA: yasa_config,
})

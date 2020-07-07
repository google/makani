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

"""Mvlv MCP342X hardware monitor configuration."""

from makani.avionics.firmware.drivers import mcp342x_types
from makani.avionics.firmware.serial import mvlv_serial_params as rev

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

ch2_pos = dict(mcp342x_default, **{
    'channel': mcp342x_types.kMcp342xChannel2,
    'polarity': mcp342x_types.kMcp342xPolarityPositive,
})

ch3_pos = dict(mcp342x_default, **{
    'channel': mcp342x_types.kMcp342xChannel3,
    'polarity': mcp342x_types.kMcp342xPolarityPositive,
})

ch4_pos = dict(mcp342x_default, **{
    'channel': mcp342x_types.kMcp342xChannel4,
    'polarity': mcp342x_types.kMcp342xPolarityPositive,
})

common = [
    dict(ch1_pos, name='output_switch', address=0x68),
    dict(ch2_pos, name='filter_cap', address=0x68),
    dict(ch3_pos, name='sync_rect_mosfet_side', address=0x68),
    dict(ch4_pos, name='sync_rect_pcb', address=0x68),
    dict(ch1_pos, name='sync_rect_mosfet_top', address=0x69),
    dict(ch2_pos, name='hv_resonant_cap', address=0x69),
    dict(ch3_pos, name='igbt', address=0x69),
    dict(ch4_pos, name='enclosure_air', address=0x69),
]

mcp342x_config = (rev.MvlvHardware, {
    rev.MvlvHardware.SYNC_RECT_REV_A1: common,
})

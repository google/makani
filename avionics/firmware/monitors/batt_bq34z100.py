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

"""Batt BQ34Z100 hardware monitor configuration."""

from makani.avionics.firmware.serial import batt_serial_params as rev

bq34z100_default = {
    'name': 'coul_count',
    'address': 0x75,  # Different from Bq34z100 default bc address translator.
    'cell_mult': 3,
    'current_max': 50.0,
    'voltage_max': 75.0,  # TODO: Update for big and small box.
    'voltage_min': 50.0,  # TODO: Update for big and small box.
    'soc_min': 95,  # Warning stoplight if state-of-charge < 95%.
}

# The bq34z100 chip won't let us calibrate it to expect the full number
# of cells (15+), so I calibrate it to expect a fraction of that amount (5 cells
# for small box, 6 cells for big box). As such, the 15 and 18-cell boxes
# can use a multiplier of 3 in the avionics code, while 17-cell uses 3.4.
cells_15_or_18 = [
    dict(bq34z100_default, cell_mult=3.0),
]

cells_17 = [
    dict(bq34z100_default, cell_mult=3.4),
]

bq34z100_config = (rev.BattHardware, {
    rev.BattHardware.SMALL_CELL15_V1: cells_15_or_18,
    rev.BattHardware.BIG_CELL18_V1: cells_15_or_18,
    rev.BattHardware.SMALL_CELL15_AA: cells_15_or_18,
    rev.BattHardware.BIG_CELL18_AA: cells_15_or_18,
    rev.BattHardware.SMALL_CELL15_AB: cells_15_or_18,
    rev.BattHardware.BIG_CELL18_AB: cells_15_or_18,
    rev.BattHardware.SMALL_CELL17_AB: cells_17,
    rev.BattHardware.SMALL_CELL15_AC: cells_15_or_18,
    rev.BattHardware.BIG_CELL18_AC: cells_15_or_18,
    rev.BattHardware.SMALL_CELL17_AC: cells_17,
    rev.BattHardware.SMALL_CELL17_AD: cells_17,
})

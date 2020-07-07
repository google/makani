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

"""Information about the location of sensors on the network."""

from makani.config import mconfig
from makani.control import control_types


@mconfig.Config
def MakeParams():
  # Flight computers responsible for the high speed (large range) and
  # low speed (small range) Pitot pressure sensors.
  pitot_fc_labels = [None for _ in range(control_types.kNumPitotSensors)]

  pitot_fc_labels[control_types.kPitotSensorHighSpeed] = (
      control_types.kFlightComputerA)

  pitot_fc_labels[control_types.kPitotSensorLowSpeed] = (
      control_types.kFlightComputerC)

  return {
      'pitot_fc_labels': pitot_fc_labels
  }

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

"""Power system parameters."""

from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      # Boolean describing whether the ground inverters attempt to
      # cancel the voltage drop across the tether.
      'use_ground_voltage_compensation': True,

      # Ground power available [W].  By convention, supplied power is
      # negative.  See the comments in config/m600/control/rotor_control.py
      # about the current ground power limitations.
      'P_source': -1.1e6,

      # Tether resistance [Ohm].  See b/29272896 and b/29247490.
      # Updated 2016-06-17.
      'R_tether': 1.0,

      # Voltage source resistance [Ohm].
      'R_source': 0.1,

      # Inverter voltage [V].
      'v_source_0': 4200.0,

      # Block capacitance [F]. The actual value should be 1.5e-3, however the
      # stacked power simulation is unstable with this value.
      #
      # TODO: Determine the source of the voltage instability.
      'C_block': 6e-3
  }

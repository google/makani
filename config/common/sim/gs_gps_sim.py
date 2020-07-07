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

"""GS GPS parameters."""

from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      'ts': 0.05,  # Update period [s].

      # The following noise model was determined empirically; see b/136181998.
      'pos_sigma': [2.5, 2.5, 2.5],  # Random error [m, ECEF] on position.
      'vel_sigma': [0.5, 0.5, 0.5],  # Random error [m/s, ECEF] on velocity.
      'heading_sigma': 1e-2,         # Random error [rad] on heading [NED].
      'pitch_sigma': 1e-2,           # Random error [rad] on pitch [NED].
      'length_sigma': 1e-2,          # Random error [m] on baseline length.
  }

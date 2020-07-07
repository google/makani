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

"""Autonomous planner control parameters."""

from makani.config import mconfig
import numpy as np


@mconfig.Config
def MakeParams():
  return {
      # The takeoff delay is the time during which all launch criteria must be
      # met before the autonomous controller commands a takeoff.
      "takeoff_delay": 30.0,  # [s]

      # The takeoff and landing windows are described using a min and max wind
      # speed and min and max azimuth.  The playbook estimated wind speed must
      # be within both of these ranges to meet the criteria of the window.

      # The takeoff window indicates the criteria required to begin a flight.
      # These limits are currently set based on the CW-11/12 test plan.
      "takeoff": {
          "wind_speed_min": 5.0,
          "wind_speed_max": 11.0,
          "wind_azi_min": np.deg2rad(20.0),
          "wind_azi_max": np.deg2rad(135.0),
      },
      # The landing window indicates the criteria under which flight will be
      # allowed to continue.  If these limits are exceeded, a landing will be
      # commanded.
      "landing": {
          "wind_speed_min": 4.0,
          "wind_speed_max": 13.0,
          "wind_azi_min": np.deg2rad(0.0),
          "wind_azi_max": np.deg2rad(155.0),
      },
  }

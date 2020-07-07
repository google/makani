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

"""Winch parameters."""

from makani.config import mconfig
from makani.control import system_types


@mconfig.Config(deps={
    'gs_model': 'base_station.gs_model',
    'ground_station': 'base_station.ground_station',
})
def MakeParams(params):
  if params['gs_model'] == system_types.kGroundStationModelGSv2:
    r_drum = params['ground_station']['gs02']['drum_radius']

    return {
        # Drum radius [m].
        'r_drum': r_drum,

        # Unused.
        'transmission_ratio': 0.0,

        # Calibration for winch velocity command. Identity; PLC takes commands
        # in [m/s].
        'velocity_cmd_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},

        # Winch position calibration [m/rad], [rad], [#].
        'position_cal': {'scale': r_drum, 'bias': 0.0, 'bias_count': 0},

        # Reported winch velocity calibration. Identity; provided in [rad/s] by
        # PLC.
        'drum_velocity_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
    }
  else:
    r_drum = 1.75

    return {
        # Drum radius [m].
        'r_drum': r_drum,

        # Transmission ratio [#] between winch servo and drum. There is a 16.7
        # planetery transmission followed by a sprocket ratio of 120 / 19.
        'transmission_ratio': 16.7 * 120.0 / 19.0,

        # Calibration for winch velocity command [m/rad], [rad/s], [#].  The
        # winch reports the angular rate of the winch drum.  This converts the
        # measurement to a velocity.
        'velocity_cmd_cal': {'scale': r_drum, 'bias': 0.0, 'bias_count': 0},

        # Winch position calibration [m/rad], [rad], [#].  The winch
        # reports the angle of the winch drum rather than the winch
        # itself.  This converts the measurement to a distance.
        'position_cal': {'scale': r_drum, 'bias': 0.0, 'bias_count': 0},

        # Winch velocity calibration. Identity; provided in [rad/s] by PLC.
        'drum_velocity_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
    }

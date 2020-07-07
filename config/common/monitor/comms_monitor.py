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

"""Communications monitoring parameters."""
from makani.config import mconfig
import numpy as np


@mconfig.Config
def MakeParams():
  return {
      # Number of samples [#] of not receiving UDP data where the
      # monitor considers there to be no UDP or slow UDP.
      'no_udp': 50,
      'slow_udp': 20,

      # Number of samples [#] of not receiving RF data where the
      # monitor considers there to be no RF link or a slow RF link.
      'no_rf_link': 100,
      'slow_rf_link': 6,

      # Number of samples [#] of not receiving data from the GS, where
      # the monitor will warn about the GS communications.
      'no_gs_comms': 125,

      # Number of samples [#] of not receiving data from the OMAP,
      # where the monitor will warn about the OMAP.
      'high_omap_timeout': 10,

      # Number of samples [#] of not receiving data from the joystick,
      # where the monitor will warn about the joystick.
      'high_joystick_hold': 50,

      # Limits for the received signal strength indicator [dBm].
      'rssi': {
          'very_low': np.finfo('d').min,
          'low': -70.0,
          'high': np.finfo('d').max,
          'very_high': np.finfo('d').max
      },

      # Limits for the rate of invalid packets [#/s].
      'invalid_rate': {
          'very_low': np.finfo('d').min,
          'low': np.finfo('d').min,
          'high': 0.3,
          'very_high': np.finfo('d').max
      },

      # Limits for the loop time [s].
      'max_loop_time': {
          'very_low': np.finfo('d').min,
          'low': np.finfo('d').min,
          'high': 4.9,
          'very_high': np.finfo('d').max
      },

      # Limits for the mean loop time [s].
      'mean_loop_time': {
          'very_low': np.finfo('d').min,
          'low': np.finfo('d').min,
          'high': 4.2,
          'very_high': np.finfo('d').max
      },

      'aio': {
          # Number of samples [#] of not receiving status messages before
          # the monitor will provide a warning. The AIO snapshotting tool is
          # sending messages at 100Hz.
          # TODO: Replace with timeouts in seconds.
          'telemetry_timeout': 10,
          'control_telemetries_timeout': 25,  # Message sent at 10 Hz.
          'control_slow_telemetries_timeout': 250,
          'controller_q7_slow_statuses_timeout': 250,  # Message sent at 1 Hz.
          'core_switch_statuses_timeout': 50,
          'core_switch_slow_statuses_timeout': 250,  # Message sent at 1 Hz.
          'drum_sensors_timeout': 50,
          'flight_computer_sensors_timeout': 50,
          'ground_station_weather_timeout': 50,
          'gs_gps_compass_timeout': 50,
          'gs_gps_observations_timeout': 50,
          'gs_gps_solution_timeout': 50,
          'joystick_timeout': 50,
          'joystick_monitor_timeout': 50,
          'loadcell_statuses_timeout': 50,
          'motor_statuses_timeout': 50,
          'platform_sensors_timeout': 50,
          'recorder_q7_slow_statuses_timeout': 250,  # Message sent at 1 Hz.
          'recorder_statuses_timeout': 50,
          'self_test_timeout': 250,  # Message sent at 1 Hz.
          'servo_statuses_timeout': 50,
          'slow_statuses_timeout': 250,  # Message sent at 1 Hz.
          'tether_down_timeout': 50,
          'winch_plc_timeout': 50,
          'wing_gps_novatel_observations_timeout': 50,
          'wing_gps_novatel_solutions_timeout': 50,
          'wing_gps_septentrio_observations_timeout': 50,
          'wing_gps_septentrio_solutions_timeout': 50,
      }
  }

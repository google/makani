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

"""Sensor limits."""

from makani.avionics.common import plc_messages
from makani.config import mconfig
from makani.control import control_types as m
from makani.system import labels
import numpy as np


@mconfig.Config(deps={'phys': 'common.physical_constants'})
def MakeParams(params):
  g = params['phys']['g']

  sensor_max = {
      # No limits should be applied to the synchronization messages.
      'sync': [{'sequence': 255, 'flight_mode': 100000}
               for _ in range(m.kNumControllers)],

      # Maximum GSG azimuth, elevation [rad].
      'gsg': [{
          'azi': 1e10, 'ele': 1e10,
      } for _ in range(m.kNumDrums)],

      'gs_sensors': {
          # Maximum ground station mode (unenforced).
          'mode': plc_messages.kNumGroundStationModes - 1,

          # Maximum ground station stage number (unenforced).
          'transform_stage': 4,

          # Maximum winch pos [m].
          'winch_pos': 1.0,

          # Maximum detwist position [rad] (unenforced).
          'detwist_pos': np.pi,

          # Maximum proximity sensor value.
          'proximity': True,
      },

      'wing_gps': [{
          # Maximum value for bool new_data flag.
          'new_data': True,

          # Maximum GPS time of week [ms] (mod one week).
          'time_of_week_ms': 24 * 7 * 3600 * 1000 - 1,

          # Maximum GPS position [m] and velocity [m/s] in ECEF.
          'pos': [7e6, 7e6, 7e6],
          'vel': [80.0, 80.0, 80.0],

          # Maximum GPS position [m] and velocity [m/s] uncertainty
          # standard deviation.
          'pos_sigma': [10.0, 10.0, 10.0],
          'vel_sigma': [3.0, 3.0, 3.0],

          # Maximum value for solution types.
          'pos_sol_type': m.kGpsSolutionTypeUnsupported,
          'vel_sol_type': m.kGpsSolutionTypeUnsupported
      } for _ in range(m.kNumWingGpsReceivers)],

      # Maximum values [#] for the joystick sticks and switches.  The
      # values for the joystick's sticks are between [0, 1] for the
      # throttle and [-1, 1] for the roll, pitch, and yaw sticks.
      'joystick': {
          'throttle': 2.0,
          'roll': 2.0,
          'pitch': 2.0,
          'yaw': 2.0,
          'switch_position': m.kJoystickSwitchPositionDown,
          'release': True,
          'engage_auto_glide': True,
      },

      # Maximum loadcell forces [N].
      'loadcells': [500000.0, 500000.0, 500000.0, 500000.0],

      # Placeholder; this value is not used.
      'tether_released': True,

      # IMU maximum high-G accelerometer accelerations [m/s^2], gyro
      # angular rates [rad/s], magnetometer measurements [G], and low-G
      # accelerometer accelerations [m/s^2].
      'imus': [{
          'acc': [50.0 * g, 50.0 * g, 50.0 * g],
          'gyro': [np.deg2rad(600.0),
                   np.deg2rad(600.0),
                   np.deg2rad(600.0)],
          'mag': [2.0, 2.0, 2.0]
      } for _ in range(m.kNumWingImus)],

      # Maximum pitot tube static pressure [Pa] and alpha, beta, and
      # dynamic differential pressures [Pa].
      #
      # NOTE: I increased the stat_press rate limit to the
      # somewhat unreasonable 2000 m/s because it helps the pressure
      # Kalman filter converge faster after a pressure glitch.  The
      # "right thing" is to have a fancy rate limit function that is
      # slow away from the expected value and fast toward it.
      'pitots': [{
          'stat_press': 108570.0,
          'diff': {
              'alpha_press': 1e10,
              'beta_press': 1e10,
              'dyn_press': 1e10
          }
      } for _ in range(labels.kNumPitotSensors)],

      # Maximum flap positions [rad].
      'flaps': [np.pi for _ in range(m.kNumFlaps)],

      # Maximum rotor angular rate [rad/s].
      'rotors': [2000.0 for _ in range(m.kNumMotors)],

      # Maximum enum describing the fault state of the stacked motor
      # system.
      'stacking_state': m.kNumStackingStates - 1,

      # Maximum wind speed [m/s] in ground coordinates.
      'wind_ws': [30.0, 30.0, 30.0],

      # Ground station GPS limits.
      'gs_gps': {
          # Maximum GPS position [m] in ECEF.
          'pos': [7e6, 7e6, 7e6],
          'pos_sigma': 10.0
      },

      'perch': {
          # Maximum winch position [m].
          'winch_pos': 1.0,

          # Maximum heading [rad] of the ground station.
          'perch_heading': 2.0 * np.pi,

          # Maximum perch azimuth angle [rad].
          'perch_azi': [np.pi for _ in range(m.kNumPlatforms)],

          # Maximum levelwind elevation angle [rad].
          'levelwind_ele': [np.pi / 2.0 for _ in range(m.kNumPlatforms)],
      },

      'weather': {
          'temperature': 60.0,  # [deg C]
          'pressure': 1500e2,   # [Pa]
          'humidity': 1.0,      # [fraction]
      },

      # Placeholder; unused, but required by code generation.
      'force_hover_accel': True,
      'force_high_tension': True,
      'force_reel': True,
      'gs_unpause_transform': True,
      'force_detwist_turn_once': True,
      'ground_estimate': {
          'time': 0.0,
          'dcm_g2p': {
              'd': [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
          },
          'pqr': [0.0, 0.0, 0.0],
          'Xg': [0.0, 0.0, 0.0],
          'Vg': [0.0, 0.0, 0.0],
          'attitude_valid': True,
          'position_valid': True,
      },

      # Experiment configs
      'experiment_type': 7,
      'experiment_case_id': 63,
  }

  sensor_min = {
      # No limits should be applied to the synchronization messages.
      'sync': [{'sequence': 0, 'flight_mode': -100000}
               for _ in range(m.kNumControllers)],

      # Minimum GSG azimuth, elevation [rad].
      'gsg': [{
          'azi': -1e10, 'ele': -1e10,
      } for _ in range(m.kNumDrums)],

      'gs_sensors': {
          # Minimum ground station mode (unenforced).
          'mode': 0,

          # Minimum ground station stage number (unenforced).
          'transform_stage': 0,

          # Minimum winch pos [m].
          'winch_pos': -1000.0,

          # Minimum detwist position [rad] (unenforced).
          'detwist_pos': -np.pi,

          # Minimum proximity sensor value.
          'proximity': False,
      },

      'wing_gps': [{
          # Minimum value for bool new_data flag.
          'new_data': False,

          # Minimum GPS time of week [ms].
          'time_of_week_ms': 0,

          # Minimum GPS position [m] and velocity [m/s] in ECEF.
          'pos': [-7e6, -7e6, -7e6],
          'vel': [-80.0, -80.0, -80.0],

          # Minimum GPS position [m] and velocity [m/s] uncertainty
          # standard deviation.
          'pos_sigma': [0.0, 0.0, 0.0],
          'vel_sigma': [0.0, 0.0, 0.0],

          # Minimum value for solution types.
          'pos_sol_type': m.kGpsSolutionTypeNone,
          'vel_sol_type': m.kGpsSolutionTypeNone
      } for _ in range(m.kNumWingGpsReceivers)],

      # Minimum values [#] for the joystick sticks and switches.  The
      # values for the joystick's sticks are between [0, 1] for the
      # throttle and [-1, 1] for the roll, pitch, and yaw sticks.
      'joystick': {
          'throttle': -2.0,
          'roll': -2.0,
          'pitch': -2.0,
          'yaw': -2.0,
          'switch_position': 0,
          'release': False,
          'engage_auto_glide': False,
      },

      # Minimum loadcell forces [N].
      'loadcells': [-500000.0, -500000.0, -500000.0, -500000.0],

      # Placeholder; this value is not used.
      'tether_released': False,

      # IMU minimum high-G accelerometer accelerations [m/s^2], gyro
      # angular rates [rad/s], magnetometer measurements [G], and low-G
      # accelerometer accelerations [m/s^2].
      'imus': [{
          'acc': [-50.0 * g, -50.0 * g, -50.0 * g],
          'gyro': [np.deg2rad(-600.0),
                   np.deg2rad(-600.0),
                   np.deg2rad(-600.0)],
          'mag': [-2.0, -2.0, -2.0]
      } for _ in range(m.kNumWingImus)],

      # Minimum pitot tube static pressure [Pa] and alpha, beta, and
      # dynamic differential pressures [Pa].
      #
      # NOTE: I increased the stat_press rate limit to the
      # somewhat unreasonable 2000 m/s because it helps the pressure
      # Kalman filter converge faster after a pressure glitch.  The
      # "right thing" is to have a fancy rate limit function that is
      # slow away from the expected value and fast toward it.
      'pitots': [{
          'stat_press': 80000.0,
          'diff': {
              'alpha_press': -1e10,
              'beta_press': -1e10,
              'dyn_press': -1e10
          }
      } for _ in range(labels.kNumPitotSensors)],

      # Minimum flap positions [rad].
      'flaps': [-np.pi for _ in range(m.kNumFlaps)],

      # Minimum rotor angular rate [rad/s].
      'rotors': [0.0 for _ in range(m.kNumMotors)],

      # Minimum enum describing the fault state of the stacked motor
      # system.
      'stacking_state': m.kStackingStateNormal,

      # Minimum wind speed [m/s] in ground coordinates.
      'wind_ws': [-30.0, -30.0, -30.0],

      # Ground station GPS limits.
      'gs_gps': {
          # Minimum GPS position [m] in ECEF.
          'pos': [-7e6, -7e6, -7e6],
          'pos_sigma': 0.0
      },

      'perch': {
          # Minimum winch position [m].
          'winch_pos': -1000.0,

          # Minimum heading [rad] of the ground station.
          'perch_heading': 0.0,

          # Minimum perch azimuth angle [rad].
          'perch_azi': [-np.pi for _ in range(m.kNumPlatforms)],

          # Minimum levelwind elevation angle [rad].
          'levelwind_ele': [-np.pi / 2.0 for _ in range(m.kNumPlatforms)],
      },

      'weather': {
          'temperature': -60.0,  # [deg C]
          'pressure': 500e2,     # [Pa]
          'humidity': 0.0,       # [fraction]
      },

      # Placeholder; unused, but required by code generation.
      'force_hover_accel': False,
      'force_high_tension': False,
      'force_reel': False,
      'gs_unpause_transform': False,
      'force_detwist_turn_once': False,
      'ground_estimate': {
          'time': 0.0,
          'dcm_g2p': {
              'd': [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
          },
          'pqr': [0.0, 0.0, 0.0],
          'Xg': [0.0, 0.0, 0.0],
          'Vg': [0.0, 0.0, 0.0],
          'attitude_valid': False,
          'position_valid': False,
      },

      # Experiment configs
      'experiment_type': 0,
      'experiment_case_id': 0,
  }

  return {
      'max': sensor_max,
      'min': sensor_min,
  }

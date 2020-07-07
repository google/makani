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

"""Estimation monitoring parameters."""
from makani.config import mconfig
import numpy as np


@mconfig.Config(deps={
    'estimator': 'm600.control.estimator',
})
def MakeParams(params):
  gsg_bias_low = params['estimator']['nav']['position']['glas']['bias_low']
  gsg_bias_high = params['estimator']['nav']['position']['glas']['bias_high']

  return {
      # Maximum difference [rad] in the fixed GS heading versus the
      # computed one.
      'max_gs_heading_error': np.deg2rad(5.0),

      # Maximum difference [m] in the fixed GS position versus the
      # computed one.
      'max_gs_pos_ecef_error': 0.2,

      # Maximum difference [G] between magnetometer measurements in the body
      # frame.
      'max_mag_diff': 0.03,

      # Maximum difference [rad] between attitude estimates computed
      # from the redundant IMU data.
      'max_attitude_diff': np.deg2rad(3.0),

      # Warning/error magnitude [rad/s] of the gyro biases.
      'gyro_bias': {
          'very_low': 0.0,
          'low': 0.0,
          'high': 0.0047,
          'very_high': 0.0094,
      },

      # Warning/error magnitude [rad/s] of the gyro bias drifts in crosswind.
      # NOTE(b/77811299): These limits are only applied to the X-axis gyros.
      'gyro_bias_crosswind_drift': {
          'very_low': 0.0,
          'low': 0.0,
          'high': 0.002,
          'very_high': 0.003,
      },

      # Maximum distance [m] to start acceleration.
      'far_accel_start': 100.0,

      # Payout [m] should be close to zero when the wing is perched or
      # ascending.
      'far_payout_at_perch': 0.02,

      # During payout and reel-in, the winch position [m] should agree
      # with the estimated distance from the perched position.
      'winch_distance_disagreement_threshold': 1.0,

      # The winch position [m] should be close to its nominal fully
      # reeled-in value on the perch.
      'winch_position_max_perched_disagreement': 0.5,

      # Distances from perch [m] that affect chart display ranges.
      'close_range': 10.0,
      'mid_range': 50.0,
      'full_range': 160.0,

      # Limits for GSG biases. To be sure of getting a red indication
      # upon saturation, we back off from the saturation limits by one
      # degree to set the indicator limits.
      'gsg_bias_azi': {
          'very_low': gsg_bias_low['azi'] + np.deg2rad(1.0),
          'low': 0.75 * gsg_bias_low['azi'] + 0.25 * gsg_bias_high['azi'],
          'high': 0.25 * gsg_bias_low['azi'] + 0.75 * gsg_bias_high['azi'],
          'very_high': gsg_bias_high['azi'] - np.deg2rad(1.0),
      },
      'gsg_bias_ele': {
          'very_low': gsg_bias_low['ele'] + np.deg2rad(1.0),
          'low': 0.75 * gsg_bias_low['ele'] + 0.25 * gsg_bias_high['ele'],
          'high': 0.25 * gsg_bias_low['ele'] + 0.75 * gsg_bias_high['ele'],
          'very_high': gsg_bias_high['ele'] - np.deg2rad(1.0),
      },

      # Limits for the magnitude of the difference between the GLAS
      # position and wing_pos_g.
      'glas_pos_diff': {
          'very_low': np.finfo('d').min,
          'low': np.finfo('d').min,
          'high': 20.0,
          'very_high': 100.0,
      },

      # Maximum GPS sigma [m].
      'max_gps_pos_sigma': 0.2,
      'max_gps_vel_sigma': 0.3,

      # Time [s] after switching to a new GPS that a warning will be shown.
      'min_current_gps_receiver_time': 5.0,

      # Acceptable difference [rad] between various perch azi readings.
      'perch_azimuth_max_disagreement': np.deg2rad(0.5),

      # Acceptable difference [rad] between various detwist readings.
      'detwist_max_disagreement': np.deg2rad(0.5),

      # Acceptable difference [rad] between various winch readings.
      'winch_max_disagreement': np.deg2rad(0.5),

      # Acceptable difference [rad] between various levelwind encoder readings.
      'lvlwind_max_disagreement': np.deg2rad(0.5),

      # Acceptable difference [m] between various levelwind shuttle readings.
      'lvlwind_shuttle_max_disagreement': 10.0/1000.0,

      # Acceptable difference [rad] between various gsg encoder readings.
      'gsg_max_disagreement': np.deg2rad(0.5),

      # Limits for difference between counted kite loops and detwist loops.
      'detwist_kite_loop_diff': {
          'very_low': -9.9,  # Red at 10.
          'low': -1.9,  # Yellow at 2.
          'high': 1.9,
          'very_high': 9.9,
      },

      # Limits for GS Azimuth error in high tension mode.
      'gs_azi_error_ht': {
          'very_low': np.deg2rad(-23.0),
          'low': np.deg2rad(-15.0),
          'high': np.deg2rad(15.0),
          'very_high': np.deg2rad(23.0),
      },

      # Limits for GS Azimuth error in reel mode.
      'gs_azi_error_reel': {
          'very_low': np.deg2rad(-1.5),
          'low': np.deg2rad(-1.2),
          'high': np.deg2rad(1.2),
          'very_high': np.deg2rad(1.5),
      },

      # Limits for the hover Euler angles [rad] in the order: roll,
      # pitch, yaw.
      'hover_angles': [{
          'very_low': np.deg2rad(-35.0),
          'low': np.deg2rad(-20.0),
          'high': np.deg2rad(20.0),
          'very_high': np.deg2rad(35.0),
      }, {
          'very_low': np.deg2rad(-15.0),
          'low': np.deg2rad(-5.0),
          'high': np.deg2rad(15.0),
          'very_high': np.deg2rad(22.0),
      }, {
          'very_low': np.deg2rad(-20.0),
          'low': np.deg2rad(-15.0),
          'high': np.deg2rad(15.0),
          'very_high': np.deg2rad(20.0),
      }],

      # Limits for hover tether angles.
      # Monitors choose which limits based on flight mode and transform stage.
      'hover_tether_angles': [{  # Reel-like.
          'very_low': np.deg2rad(-2.0),
          'low': np.deg2rad(1.0),
          'high': np.deg2rad(12.0),
          'very_high': np.deg2rad(16.0),
      }, {  # High-tension-like TransformGsUp & Down stages 1-3.
          'very_low': np.deg2rad(1.0),
          'low': np.deg2rad(3.0),
          'high': np.deg2rad(12.0),
          'very_high': np.deg2rad(16.0),
      }, {  # High-tension-like.
          'very_low': np.deg2rad(-2.0),
          'low': np.deg2rad(0.0),
          'high': np.deg2rad(70.0),
          'very_high': np.deg2rad(75.0),
      }],

      # Limits for crosswind tether angles[deg] in order: yoke, termination.
      'crosswind_tether_angles': [{
          'very_low': -73.0,
          'low': -65.0,
          'high': 65.0,
          'very_high': 73.0,
      }, {
          'very_low': -35.0,
          'low': -31.0,
          'high': 31.0,
          'very_high': 35.0,
      }],

      # Limits for tether pitch [rad].
      'tether_pitch': {
          'very_low': np.deg2rad(-55.0),
          'low': np.deg2rad(-50.0),
          'high': np.deg2rad(16.0),
          'very_high': np.deg2rad(17.0),
      },

      # Limits for the angle-of-attack [rad].
      'alpha': {
          'very_low': np.deg2rad(-6.0),
          'low': np.deg2rad(-4.0),
          'high': np.deg2rad(7.0),
          'very_high': np.deg2rad(10.0)
      },

      # Limits for the sideslip [rad].
      'beta': {
          'very_low': np.deg2rad(-10.0),
          'low': np.deg2rad(-7.0),
          'high': np.deg2rad(7.0),
          'very_high': np.deg2rad(10.0)
      },

      # Limits for the airspeed [m/s].
      # TODO: Consider signed apparent_wind.x, not just absolute speed.
      'airspeed': {
          'very_low': -10.0,
          'low': 25.0,
          'high': 65.0,
          'very_high': 70.0
      },
      'airspeed_error': {
          'very_low': np.finfo('d').min,
          'low': -5.0,
          'high': 5.0,
          'very_high': np.finfo('d').max
      },
      'alpha_error': {
          'very_low': np.finfo('d').min,
          'low': np.deg2rad(-3.0),
          'high': np.deg2rad(3.0),
          'very_high': np.finfo('d').max
      },
      'beta_error': {
          'very_low': np.finfo('d').min,
          'low': np.deg2rad(-4.0),
          'high': np.deg2rad(4.0),
          'very_high': np.finfo('d').max
      },
      'crosswind_altitude_agl': {
          'very_low': 80.0,
          'low': 100.0,
          'high': np.finfo('d').max,
          'very_high': np.finfo('d').max
      }
  }

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

"""Simulated servo parameters."""

import copy
import numpy as np

from makani.config import mconfig
from makani.sim import sim_types


@mconfig.Config
def MakeParams():
  # Values are from Harmonic Drive SPA-25 specification sheet,
  # SPA-25-160-SP4041, drawing number 7379481.
  spa25_160 = {
      # Motor torque constant [N-m/A] or [V/(rad/s)].
      'motor_torque_constant': 0.239,

      # Motor inductance [H].
      'motor_inductance': 1.36e-3,

      # Phase-to-phase resistance [Ohm].
      'motor_resistance': 0.886,

      # Number of electrical poles [#].
      'num_elec_poles': 12,

      # Servo motor moment-of-inertia [kg-m^2].  Flap inertia is small
      # in comparison so it is ignored for now but could be accounted
      # for in this constant.
      'moment_of_inertia': 2.3e-4,

      # Gear ratio [#] of the RX64 transmission.
      'gear_ratio': 160.0,

      # The procedure used to find gear friction is:
      #
      #   1) Servo power off.
      #   2) Lift the laden servo arm.
      #   3) Drop the arm.
      #   4) Record asymptotic speed.
      #
      # By running this procedure at several loads a table of friction
      # versus speed is built.  Implicit in this lookup table are the
      # two assumptions:
      #
      #   1) Backdriving the transmission results in the same friction
      #      as forward driving it.
      #   2) Friction is load independent.
      #
      # Friction lookup table of angular rates [rad/s] and torques
      # [N-m].
      'friction_angular_vel_table': [0.0, 0.1780, 0.3874, 0.6492, 1.466],
      'friction_torque_table': [0.0, 27.2727, 49.0909, 63.6363, 100.0],

      'servo_drive': {
          # The reference model rate limit, and the feedback gains
          # kp and kd, have been updated so the sim model matches
          # parameter id tests performed in Feb. 2016.

          # Reference model cutoff frequency [Hz] and a rate limit on
          # the reference model shaft angular velocity [rad/s].
          'ref_model_cutoff_freq': 15.91,
          'ref_model_rate_lim': 0.85,
          'ref_model_min_position_limit': None,
          'ref_model_max_position_limit': None,

          # Servo drive voltage controller proportional and derivative
          # gains, [V/rad] and [V/(rad/s)] respectively, referenced to
          # shaft angle and angular velocity.
          'kp': 120.0,
          'kd': -34.0,

          # Servo bus voltage [V].
          'bus_voltage': 70.0,

          # Maximum continuous rated current [A] for the servo motors.
          'current_lim': 6.0
      }
  }

  servo_params = [
      copy.deepcopy(spa25_160) for _ in range(sim_types.kNumServos)
  ]

  # Hard-code the position limits for each individual servo.
  # These values are taken from the file:
  #   //avionics/servo/firmware/config_params.yaml
  # The values here must be updated when the values in the above file change.
  # TODO: auto-populate the values.
  servo_params[sim_types.kServoA1]['servo_drive'][
      'ref_model_min_position_limit'] = np.deg2rad(-81.0)
  servo_params[sim_types.kServoA1]['servo_drive'][
      'ref_model_max_position_limit'] = np.deg2rad(10.0)

  servo_params[sim_types.kServoA2]['servo_drive'][
      'ref_model_min_position_limit'] = np.deg2rad(-81.0)
  servo_params[sim_types.kServoA2]['servo_drive'][
      'ref_model_max_position_limit'] = np.deg2rad(10.0)

  servo_params[sim_types.kServoA4]['servo_drive'][
      'ref_model_min_position_limit'] = np.deg2rad(-97.0)
  servo_params[sim_types.kServoA4]['servo_drive'][
      'ref_model_max_position_limit'] = np.deg2rad(46.0)

  servo_params[sim_types.kServoA5]['servo_drive'][
      'ref_model_min_position_limit'] = np.deg2rad(-97.0)
  servo_params[sim_types.kServoA5]['servo_drive'][
      'ref_model_max_position_limit'] = np.deg2rad(46.0)

  servo_params[sim_types.kServoA7]['servo_drive'][
      'ref_model_min_position_limit'] = np.deg2rad(-81.0)
  servo_params[sim_types.kServoA7]['servo_drive'][
      'ref_model_max_position_limit'] = np.deg2rad(10.0)

  servo_params[sim_types.kServoA8]['servo_drive'][
      'ref_model_min_position_limit'] = np.deg2rad(-81.0)
  servo_params[sim_types.kServoA8]['servo_drive'][
      'ref_model_max_position_limit'] = np.deg2rad(10.0)

  servo_params[sim_types.kServoE1]['servo_drive'][
      'ref_model_min_position_limit'] = np.deg2rad(-90.0)
  servo_params[sim_types.kServoE1]['servo_drive'][
      'ref_model_max_position_limit'] = np.deg2rad(16.0)

  servo_params[sim_types.kServoE2]['servo_drive'][
      'ref_model_min_position_limit'] = np.deg2rad(-90.0)
  servo_params[sim_types.kServoE2]['servo_drive'][
      'ref_model_max_position_limit'] = np.deg2rad(16.0)

  servo_params[sim_types.kServoR1]['servo_drive'][
      'ref_model_min_position_limit'] = np.deg2rad(-78.0)
  servo_params[sim_types.kServoR1]['servo_drive'][
      'ref_model_max_position_limit'] = np.deg2rad(78.0)

  servo_params[sim_types.kServoR2]['servo_drive'][
      'ref_model_min_position_limit'] = np.deg2rad(-78.0)
  servo_params[sim_types.kServoR2]['servo_drive'][
      'ref_model_max_position_limit'] = np.deg2rad(78.0)

  return servo_params

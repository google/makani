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

"""Manual control parameters."""

from makani.config import mconfig
import numpy as np


# TODO: Automatically tune auto-glide gains.
@mconfig.Config
def MakeParams():
  auto_glide = {
      # Orientation of the autoglide frame with respect to the ground [rad].
      #
      # Auto-glide angle-of-attack.
      # This is used as an input to the trim calculation for off-tether kite.
      # Based on analysis in makani/analysis/util/autoglide_analysis.py;
      # Best L/D (range) is at kite alpha = 1 deg,
      # Best L^1.5/D (minimum sink speed) is at kite alpha = 9 deg.
      # For CW-01 the recommended maximum alpha is 4 deg, b/116745911.
      # 'angle_of_attack' is chosen closer to minimum sink speed alpha but with
      # some margin from recommend maximum alpha, b/112266199.
      # Once 'angle_of_attack' is updated here, please run:
      #   bazel run analysis/control:manual
      # and update 'pitch_angle', 'roll_angle' and 'flap_offsets' below.
      'angle_of_attack': np.deg2rad(1.0),

      # These are determined by analysis/control/manual.py.
      'pitch_angle': np.deg2rad(-3.946),
      'roll_angle': np.deg2rad(2.164),

      # Auto-glide gains.  The pitch proportional [rad/rad] and derivative
      # [rad/rad/s] gains are applied to the aircraft pitch and pitch rate
      # with respect to the autoglide frame, to command the elevator.  The
      # integral gain is not used and must be zero.
      'pitch_pid': {
          'kp': -0.5236,
          'ki': 0.0,
          'kd': -0.5236,
          'int_output_min': 0.0,
          'int_output_max': 0.0
      },

      # The roll proportional [rad/rad] and derivative [rad/rad/s] gains are
      # applied to the aircraft roll and roll rate to command the ailerons.
      # The integral gain is not used and must be zero.
      'roll_pid': {
          'kp': -1.047,
          'ki': 0.0,
          'kd': -0.1047,
          'int_output_min': 0.0,
          'int_output_max': 0.0
      },

      # Yaw rate and roll-to-yaw_rate gains to the rudder.
      # The yaw rate gain is based on linear analysis.  The closed-loop
      # lateral roots in the manual mode are approximately:
      # s = -5 +/- j3 and -0.85 +/- j0.1 rad/sec
      # with this gain.
      #
      # The roll_crossfeed_gain was loosely based on a coordinated turn
      # relationship between roll angle and yaw rate, but it is currently
      # disabled due to stability concerns in simulation.
      'yaw_rate_gain': 0.8,
      'roll_crossfeed_gain': 0.0
  }

  output = {
      # Thrust and moment weights [#] for the least squares solver
      # used to set rotor velocities.
      'thrust_moment_weights': {
          'thrust': 1.0,
          'moment': [1e-4, 1.0, 1.0]
      },

      # Offsets [rad] for flaps in standard order.
      #
      # These are determined by analysis/control/manual.py.
      'flap_offsets': np.deg2rad(
          [-4.892, -4.892, -11.5, -11.5, -2.108, -2.108, -0.555, -3.472]
      ).tolist(),

      # Limits [rad] applied to the total flap deflections.
      # Keeping them consistent with crosswind limits.
      'lower_flap_limits': np.deg2rad(
          [-25.0, -25.0, -90.0, -90.0, -25.0, -25.0, -10.0, -22.0]).tolist(),
      'upper_flap_limits': np.deg2rad(
          [5.0, 5.0, 0.0, 0.0, 5.0, 5.0, 15.0, 22.0]).tolist(),

      # Kondensatorentladesystem (KES) delay [sec] and torque [Nm].
      'kes_delay': 3.0,
      'kes_torque': 100.0
  }

  return {
      # Joystick to flap deflection gain [rad/#].  Note that an
      # additional roll->rudder contribution was considered, as
      # aileron control of roll is fairly weak, but it was omitted at
      # ssteiner's request.
      'joystick_flap_gains': {
          'aileron': -0.54,
          'elevator': -0.25,
          'rudder': -0.61,
          'inboard_flap': 0.0,
          'midboard_flap': 0.0,
          'outboard_flap': 0.0
      },

      # Joystick to motor thrust-moment gain [N/#, N-m/#].
      'joystick_motor_gains': {
          'thrust': 30000.0,
          'moment': [0.0, 20000.0, 20000.0],
      },

      'auto_glide': auto_glide,
      'output': output
  }

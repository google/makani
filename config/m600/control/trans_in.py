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

"""Transition-in controller parameters."""
from makani.analysis.control import simple_aero
from makani.config import mconfig
from makani.config.m600.control import trans_in_controllers
import numpy as np


@mconfig.Config(deps={
    'phys': 'common.physical_constants',
    'rotors': mconfig.WING_MODEL + '.rotors',
    'tether': mconfig.WING_MODEL + '.tether',
    'wing': mconfig.WING_MODEL + '.wing',
    'wing_serial': 'common.wing_serial',
})
def MakeParams(params):
  controllers = trans_in_controllers.GetControllers(params['wing_serial'])

  flap_offset = np.deg2rad(-11.5)

  simple_aero_model = {
      'dCL_dalpha': 7.23857325,
      'CL_0': 1.69398499,
      'base_flaps': [flap_offset, flap_offset, flap_offset, flap_offset,
                     flap_offset, flap_offset, 0.0, 0.0],
      'dCL_dflap': [0.31226199, 0.37013073, 0.37070369, 0.37013073,
                    0.36783890, 0.31111608, 0.54488286, 0.00229183],
      'dCY_dbeta': -1.419025,
      'CY_0': -0.051620,
      'dCD_dalpha': 0.68301,
      'CD_0': 0.076590,
  }
  simple_aero.CheckSimpleAeroModel(
      'm600/m600_aswing_baseline_zero_angular_rate.json',
      simple_aero_model, 1e-3, flap_offset=flap_offset)

  # Maximum aerodynamic climb angle [rad] commanded by the
  # longitudinal loop.  This value is also used to provide a lower
  # limit on the expected lift in lateral control.
  max_aero_climb_angle_cmd = np.deg2rad(55.0)

  # Compute the maximum CL.
  max_delta_flap_cmd = np.deg2rad(10.0)
  max_angle_of_attack_cmd = np.deg2rad(1.0)

  # TODO: Consider whether trans-in should use the dCL_dflap that was
  # added to the simple aero model.
  CL_max = (simple_aero_model['CL_0']
            + controllers['dCL_dflap'] * max_delta_flap_cmd
            + simple_aero_model['dCL_dalpha'] * max_angle_of_attack_cmd)

  # Calculate the angle [rad] between the body x-axis and the total
  # thrust line.
  thrust_axis_b = np.zeros((3,))
  for i in range(len(params['rotors'])):
    thrust_axis_b += params['rotors'][i]['axis']
  thrust_axis_b /= len(params['rotors'])
  assert thrust_axis_b[1] == 0.0
  thrust_pitch = np.arctan2(-thrust_axis_b[2], thrust_axis_b[0])

  return {
      # Airspeed bias [m/s] and thresholds [m/s] for handling propeller inflow.
      #
      # We subtract a bias from the airspeed measurement to account for the
      # effect of propeller inflow on the Pitot sensor.
      'prop_inflow_airspeed_bias': 1.4,
      'prop_inflow_low_airspeed': 20.0,
      'prop_inflow_high_airspeed': 50.0,

      # X position [m] at which to start an early turn.
      'turn_start_pos_ti_x': -params['tether']['length'] * np.cos(np.pi/ 4.0),

      # Turning radius [m] for the early turn.
      'turn_radius': 200.0,

      # Course angle [rad] at which to resume straight flight.
      'turn_course_angle': -np.pi / 6.0,

      'mode': {
          # Minimum estimated dynamic pressure [Pa] before starting trans-in.
          # TODO: This value sets a minimum airspeed above which
          # we can reasonably trust the Pitot pressure sensors.
          #
          # This threshold was reduced to avoid entering trans-out.
          'min_dynamic_pressure': 30.0,

          # Minimum time [s] spent in kFlightModeHoverAccel before a
          # transition is allowed.
          'min_time_in_accel': 2.5,

          # Measured acceleration [m/s^2] threshold and time to keep
          # accelerating [s].  Transition to kFlightModeTransIn
          # requires the measured specific force to drop below this
          # value or for max_time_keep_accelerating time to elapse.
          'acc_stopped_accelerating_threshold': params['phys']['g'] * 1.05,
          'max_time_keep_accelerating': 5.0,

          # Minimum pitch angle [rad] before forcing a transition from
          # kFlightModeHoverAccel to kFlightModeTransIn.
          'min_pitch_angle': -0.5
      },

      'longitudinal': {
          # Aerodynamic climb angle limits [rad].
          'min_aero_climb_angle_cmd': np.deg2rad(-10.0),
          'max_aero_climb_angle_cmd': max_aero_climb_angle_cmd,

          # Angle [rad] between the body x-axis and the thrust line.
          'thrust_pitch': thrust_pitch,

          # Minimum airspeed [m/s].  Below this airspeed, a linear
          # gain is applied to pitch the kite forward.
          #
          # TODO: This should be a dynamic pressure.
          'min_airspeed': 24.5,

          # Natural frequency [Hz] for the position loop.
          'radial_tracking_freq_hz': 0.1,

          # Damping ratio for the radial tracking loop.
          'radial_tracking_damping_ratio': 1.25,

          # Threshold [m] on the radial error below which additional
          # normal force is applied to establish tension.
          'tension_control_radial_error_threshold': 20.0,

          # Threshold [rad] on the elevation angle above which additional
          # normal force is applied to establish tension.
          'tension_control_elevation_angle_threshold': 0.7,

          # Desired tension [N] on the tether sphere.
          'min_tension_cmd': 6000.0,

          # Zero angle-of-attack lift coefficient [#], lift slope
          # [#/rad] with respect to change in angle-of-attack and flap
          # commands.
          'CL_0': simple_aero_model['CL_0'],
          'dCL_dalpha': simple_aero_model['dCL_dalpha'],
          'dCL_dflap': controllers['dCL_dflap'],

          # Minimum and maximum delta flap command [rad].
          'min_delta_flap_cmd': np.deg2rad(-10.0),
          'max_delta_flap_cmd': max_delta_flap_cmd,

          # Minimum and maximum angle-of-attack command [rad].
          'min_angle_of_attack_cmd': np.deg2rad(-6.0),
          'max_angle_of_attack_cmd': max_angle_of_attack_cmd,

          # Maximum absolute feed-forward pitch rate [rad/s] to be commanded.
          'max_pitch_rate_b_cmd': 0.5,

          # Maximum thrust [N].
          #
          # This thrust command attempts to saturate thrust assuming
          # Rev3 propellers, an 800 N-m torque limit, a per-motor
          # power limit of 108 kW and a total aerodynamic power limit
          # of 730 kW.
          'thrust_cmd': 28000.0,
      },

      'lateral': {
          # Maximum expected lift coefficient [#].
          'CL_max': CL_max,

          'max_aero_climb_angle': max_aero_climb_angle_cmd,

          # Reference length [m] for the lateral tracking loop.
          #
          # The desired lateral tracking bandwidth is given by
          #     airspeed / (2.0 * pi * lateral_tracking_ref_length).
          'lateral_tracking_ref_length': 150.0,

          # Maximum bandwidth [Hz] for the lateral tracking loop.
          'max_lateral_tracking_freq_hz': 0.035,

          # Damping ratio [#] for the desired wing lateral position
          # response.
          'lateral_tracking_damping_ratio': 0.5,

          # Maximum lateral position error [m].
          'max_pos_ti_y_err': 30.0,

          # Maximum feed-forward yaw-rate [rad/s] that can be commanded.
          'max_yaw_rate_ti_cmd': 0.3,

          # Maximum absolute roll angle command [rad].
          'max_delta_roll_ti_cmd': 0.3,

          # Trim angle-of-sideslip, roll angle and yaw angle [rad] in
          # the transition-in frame.
          'angle_of_sideslip_cmd': controllers['angle_of_sideslip_cmd'],
          'roll_ti_cmd': controllers['roll_ti_cmd'],
          'yaw_ti_cmd': controllers['yaw_ti_cmd']
      },

      'attitude': {
          # Minimum feed-forward pitch moment [N-m] to carry-over from
          # the final kFlightModeHoverAccel commands.  The upper bound
          # on this moment is zero.
          'min_initial_pitch_moment': -5000.0,

          # Maximum feed-forward yaw moment magnitude [N-m] to
          # carry-over from the final kFlightModeHoverAccel commands.
          'max_initial_yaw_moment': 5000.0,

          # Maximum angular acceleration [rad/s^2] for the initial
          # pitch-forward trajectory.
          'pitch_forward_max_pitch_accel': 1.0,

          # Maximum pitch rate [rad/s] for the initial pitch-forward
          # trajectory.
          'pitch_forward_max_pitch_rate': 0.5,

          # Pitch angle error [rad] above which no explicit pitch
          # forward maneuver is attempted.
          'pitch_forward_max_pitch_error': np.deg2rad(-15.0),

          # Maximum duration [s] allowed for the pitch-forward maneuver.
          'pitch_forward_max_duration': 4.0,

          # Elevator trim [rad] for zero angle-of-attack assuming a
          # slack tether.
          'delta_elevator_alpha_zero': controllers['delta_elevator_alpha_zero'],

          # Change in elevator trim per angle-of-attack [rad/rad] assuming
          # a slack tether.
          'ddelta_elevator_dalpha': controllers['ddelta_elevator_dalpha'],

          # Airspeed [m/s] and angle-of-attack [rad] thresholds for
          # holding the integrator.  The integrator is allowed to
          # operate after the first moment where the pitch forward is
          # complete, the airspeed is above the first threshold and
          # the angle-of-attack is below the second threshold.
          'int_release_airspeed_threshold': 15.0,
          'int_release_alpha_threshold': 0.15,

          # Maximum rate of change of the integrated angle-of-attack
          # error [rad].
          'max_int_angle_of_attack_rate': 0.1,

          # Maximum integrated angle-of-attack error [rad-s].
          'max_int_angle_of_attack': 0.25,

          # Maximum integrated roll error [rad-s].
          'max_int_roll': 0.2,

          # Ratio [#] of delta_flap_cmd to apply to the midboard flaps.
          'midboard_flap_ratio': 0.5,

          # Tension cutoffs [N] for when to use gains for airplane-like
          # control (low tension) and on-tether control (high-tension).
          'low_tension': 10000.0,
          'high_tension': 20000.0,

          # Gain matrices for low and high tension.  The lateral gains
          # are given as:
          #
          #   [motor yaw; ailerons; rudder] =
          #       K * [roll; yaw; p; r; angle_of_sideslip].
          #
          # and the longitudinal gains as:
          #
          #   [motor pitch; elevator] = K * [pitch; q; int_angle_of_attack].
          #
          # The low tension gains are trimmed by placing the kite 20
          # meters from full tether extension and trimming to to have
          # 2.0 [deg] roll angle, 45 [deg] aerodynamic climb angle, in 0
          # [m/s] wind using the m600_vsaero_zero_angular_rate.json
          # database.  For the high tension gains, the kite's trim
          # position was moved outward until until the tether tension
          # reached 12 [kN], and merge_databases was set to false.
          'lat_gains_pitch_forward': controllers['lat_gains_pitch_forward'],
          'lat_gains_low_tension': controllers['lat_gains_low_tension'],
          'lat_gains_high_tension': controllers['lat_gains_high_tension'],
          'long_gains_pitch_forward': [
              [5.08e+03, 10.04e+03, 0.0],
              [-0.35, -0.15, 0.0]
          ],
          'long_gains_low_tension': controllers['long_gains_low_tension'],
          'long_gains_high_tension': controllers['long_gains_high_tension'],
      },

      'output': {
          # Thrust and moment weights [#] for the least squares solver
          # used to set rotor velocities.  The relatively high weight
          # on roll is to avoid exciting the symmetric torsional mode;
          # the trans-in controller always commands zero roll moment.
          'thrust_moment_weights': {
              'thrust': 1e-3,
              'moment': [3.0, 1.0, 1.0]
          },

          # Flap offsets and lower and upper limits [rad] in the
          # standard order: port aileron, center flap, starboard
          # aileron, elevator, rudder.  The aileron offsets are
          # slightly up so there is still some room left for control
          # during acceleration as the 0 degree flap position
          # corresponds to maximum lift.
          #
          # The offsets are chosen based on trimming the kite to have
          # 2 [deg] roll angle, 45 [deg] aerodynamic climb angle, in 0
          # [m/s] wind using the database
          # m600_vsaero_zero_angular_rate.json.
          #
          # Note that the elevator trim command is handled in the
          # attitude control parameters.
          'flap_offsets': controllers['flap_offsets'],

          'lower_flap_limits': [
              np.deg2rad(angle) for angle in
              [-20.0, -20.0, -20.0, -20.0, -20.0, -20.0, -10.0, -22.0]
          ],
          'upper_flap_limits': [
              np.deg2rad(angle) for angle in
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0, 22.0]
          ]
      }
  }

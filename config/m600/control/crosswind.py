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

"""Crosswind controller parameters."""

from makani.config import mconfig
from makani.config.m600.control import crosswind_airspeed_controller
from makani.config.m600.control import crosswind_controllers
from makani.config.m600.control import crosswind_playbook
from makani.config.m600.control import crosswind_playbook_utils
from makani.config.m600.control.experiments import crosswind as crosswind_exp
from makani.control import control_types as m
from makani.lib.python import dict_util

import numpy as np


@mconfig.Config(deps={
    'ground_station': 'base_station.ground_station',
    'gs_model': 'base_station.gs_model',
    'phys': 'common.physical_constants',
    'test_site': 'common.test_site',
    'tether': mconfig.WING_MODEL + '.tether',
    'trans_in': 'm600.control.trans_in',
    'wing': mconfig.WING_MODEL + '.wing',
    'wing_serial': 'common.wing_serial',
})


def MakeParams(params):
  """Retrieves controller parameters and calls MakeParamsHelper."""
  with dict_util.MustConsumeAllDictEntries(
      crosswind_airspeed_controller.GetControllers(
          params['wing_serial'])) as airspeed_controller:
    with dict_util.MustConsumeAllDictEntries(
        crosswind_controllers.GetControllers(
            params['wing_serial'])) as controllers:
      with dict_util.MustConsumeAllDictEntries(
          crosswind_playbook.GetPlaybook(
              params['test_site'])) as playbook_params:
        return MakeParamsHelper(
            params, airspeed_controller, controllers, playbook_params)


def MakeParamsHelper(params, airspeed_controller, controllers, playbook_params):
  """Return crosswind controller parameters."""

  # Minimum and maximum elevation angles [rad] above ground.
  # The crosswind controller is saturated between ele_min and ele_max.
  ele_min = np.deg2rad(25.0)
  ele_max = np.deg2rad(55.0)

  # Ailerons trim angle offset [rad]. Here, we shift the zero aileron deflection
  # point closer to the center of the ailerons actuation window (see ECR 388).
  ailerons_trim_angle_offset = np.deg2rad(-2.5)

  # Generate and smooth playbook from selected parameters.
  playbook_wind_adjust_params = playbook_params['wind_adjust_params']
  playbook_smoothing_params = playbook_params['smoothing_params']
  playbook_parameterized = playbook_params['playbook_parameterized']
  transout_airspeeds = playbook_params['transout_airspeeds']

  # TODO(b/137710129) The process for merging transout_airspeed schedules with
  # the rest of the playbook should be more robust, to ensure that they are
  # aligned at the proper wind speed.
  for i in range(len(playbook_parameterized)):
    playbook_parameterized[i]['transout_airspeed'] = transout_airspeeds[i]

  playbook = crosswind_playbook_utils.MakePlaybook(
      playbook_parameterized, playbook_wind_adjust_params)

  playbook = crosswind_playbook_utils.SmoothPlaybook(
      playbook, playbook_smoothing_params)

  playbook = crosswind_playbook_utils.FillPlaybook(playbook)

  fallback_params = {
      # Constant alpha and beta commands [rad].
      'alpha_cmd': np.deg2rad(3.0),
      'beta_cmd': np.deg2rad(0.0),

      # Airspeed schedule.
      #
      # The following parameters are based on an airspeed versus loop
      # angle optimization run performed around 2015.  See
      # go/makaniwiki/controls/optimal-airspeeds-around-the-loop for
      # more information.

      # Loop angle [rad] when the airspeed should be at its maximum.
      # TODO: Obsolete. Remove.
      'airspeed_variation_loop_angle_offset': 1.74,

      # Amplitude [m/s] (center-to-peak) of the variation in airspeed
      # around the loop, and the slope of the change in amplitude
      # [m/s/(m/s)] as a function of mean airspeed.
      # TODO: Obsolete. Remove.
      'airspeed_variation_0': 14.62,
      'airspeed_variation_slope': -0.11,

      # Mean airspeed [m/s].
      'airspeed_mean': 47.0,

      # Loop radius [m].
      'path_radius_target': 165.0,

      # Loop center elevation [rad].
      'elevation': np.deg2rad(45.0),

      # Azimuth offset from downwind [rad].
      'azi_offset': 0.1,
  }

  playbook_fallback = {
      'entry': crosswind_playbook_utils.MakeFallbackPlaybook(fallback_params),
      'crossfade_rate': 0.05,  # [1/s]
      'crossfade_throttle': 0.9,
  }

  power = {
      # Timer [s] to smooth transition from transition-in to crosswind.
      'transition_smooth_time': 15.0,

      # Minimum and maximum airspeeds [m/s].
      # The minimum airspeed measured during CrosswindNormal were:
      # CW-01: 37.3 m/s, CW-02: 33.2 m/s, CW-03: 36.7 m/s, CW-04: 31.1 m/s.
      # Flight qualities during CW-01 and 03 were found to be robust,
      # whereas they were weak during CW-04, because of the slow airspeed.
      # (b/129574396)
      'min_airspeed': 25.0,
      # The upper end of the airspeed range is driven by the propeller whirl
      # flutter limit (see b/67742392).
      'max_airspeed': 80.0,

      # Minimum, safe, maximum, and prep-trans-out elevation angles
      # [rad] above ground.
      'ele_min': ele_min,
      'ele_max': ele_max,

      # All changes to elevation are rate limited by ele_rate_lim
      # [rad/s].
      'ele_rate_lim': np.deg2rad(0.3),

      # To protect against sudden changes in wind direction, we
      # rate limit how fast the path center azimuth can move.
      'azi_rate_lim': np.deg2rad(0.1),

      # These loop angles [rad] define the region in which you can switch
      # predictor path shapes and start the final trans-out sequence.
      'loop_angle_path_switch_max': 0.55 * np.pi,
      'loop_angle_path_switch_min': 0.50 * np.pi,

      # Angle [rad] over which to crossfade from the PrepTransOut airspeed
      # to the sinusoidal deceleration command.
      'transout_path_switch_crossfade_distance': np.deg2rad(5.0),

      # Crosswind loop elevation to reach before trans-out maneuver.
      'transout_elevation_cmd': np.deg2rad(31.0),

      # How close [m] the instantaneous radius command must be to
      # its preptransout target before the path type will switch
      # to trigger the final deceleration.
      'transout_path_radius_target_threshold': 0.1,

      # Kite altitude [m] below which the path type will switch to
      # kCrosswindPathPrepareTransitionOut. This triggers the deceleration
      # and subsequent switch to hover.
      'min_transout_altitude': 65.0,

      # Trans-out airspeed command which defines the deceleration profile.
      'transout_airspeed_target': 32.0,

      # Loop angle [rad] at which the speed command finishes
      # crossfading to the trans-out airspeed target. There is a
      # value for low wind and a value for high wind. The actual
      # angle used is crossfaded between the two.
      'low_wind_transout_airspeed_corner_angle': np.deg2rad(13.0),
      'high_wind_transout_airspeed_corner_angle': np.deg2rad(28.0),

      # Wind speeds [m/s] across which to crossfade the above corner angles
      'corner_angle_low_wind_speed': 3.0,
      'corner_angle_high_wind_speed': 13.0,

      # Time [s] over which to crossfade from the normal airspeed command
      # to the PrepTransOut airspeed command.
      'transout_airspeed_crossfade_time': 5.0,

      # Airspeed [m/s] and loop angle [rad] for the final crossfade of
      # Trans-out airspeed commands. This final crossfade is to ensue
      # that kites which are overspeeding receive a decelerating command
      # after they pass transout_airspeed_crossfade_angle_end
      'transout_final_airspeed_target': 25.0,
      'transout_final_airspeed_crossfade_angle': 0.0,

      # Maximum horizontal distance [m] of the kite from the crosswind
      # plane center line where a path azimuth slew is allowed to begin.
      'max_crosswind_y_position_for_slew': 2.0,
  }

  path = {
      # Direction to travel around the loop.
      'loop_dir': m.kLoopDirectionCw,

      # List of potential "aerodynamic" curvatures [1/m] to follow.
      'k_tab': (np.linspace(-0.015, 0.002, 9)).tolist(),

      # Time [s] at future curvature set-point.
      'commanded_curvature_time': 1.4,

      # Time [s] at current measured curvature.
      'current_curvature_time': 0.6,

      # Maximum velocity [m/s] at which time-look-ahead applies.
      'time_horizon_speed_limit': 100.0,

      # Minimum turning radius [m].
      'min_turning_radius': 293.0,

      # Loop radius command during PrepTransOut [m],
      'preptransout_radius_cmd': 170.0,

      # Crosswind Plane z position [m] where the TransOut path becomes
      # purely vertical in the final loop. z positions are positive
      # in the "down" direction of the Crosswind Plane.
      'cw_z_for_vertical_paths': 50.0,

      # Cutoff frequency [Hz] for geometric curvature.
      'fc_k_geom_curr': 10.0,

      # Cutoff frequency [Hz] for velocity filter.  Filter velocity at
      # slow rate to find an average around the loop.
      'fc_speed': 0.1,

      # Cutoff frequencies [Hz] for curvature filters.
      'fc_k_aero_cmd': 1.5,
      'fc_k_geom_cmd': 1.5,

      # Integral gain for crosstrack error. This linear integral
      # feedback is summed with the output of the nonlinear path
      # controller in order to achieve zero steady state loop radius
      # error.  Proportional and derivative gains are not used and
      # must be zero.
      'crosstrack_pid': {
          'kp': 0.0,
          'ki': 3e-6,
          'kd': 0.0,
          'int_output_min': -0.0020,
          'int_output_max': 0.0020,
      },

      # Maximum rate of change [m/s] of the path radius target when entering and
      # exiting CrosswindNormal.
      'path_radius_rate': 0.5,

      # Maximum allowable crosstrack radius error [m]
      'crosswind_max_radius_error': 100.0,

      # Maximum allowable error [1/m] for the curvature command.
      'max_k_aero_error': 0.004,
  }

  curvature = {
      # The alpha set-point [rad] fades from nominal to
      # alpha_min at a rate of dalpha_dairspeed as the airspeed [m/s] increases
      # to alpha_min_airspeed.
      'alpha_min': -np.deg2rad(2.0),
      'dalpha_dairspeed': -np.deg2rad(8.0/15.0),
      'alpha_min_airspeed': 75.0,

      # Rate limits on alpha command [rad/s].
      'alpha_cmd_rate_min': -np.deg2rad(4.0),
      'alpha_cmd_rate_max': np.deg2rad(4.0),

      # Rate limits on beta command [rad/s].
      'beta_cmd_rate_min': -np.deg2rad(1.0),
      'beta_cmd_rate_max': np.deg2rad(1.0),

      # These are the maximum tether roll angles [rad] that we command
      # at low and high tensions [N].  See the note in the controller
      # for why the tight roll command constraints are necessary.
      'tether_roll_max_excursion_low': 0.2,
      'tether_roll_max_excursion_high': 0.48,
      'tether_roll_nom': np.arctan2(-params['wing']['bridle_y_offset'],
                                    params['wing']['bridle_rad']
                                    + params['wing']['bridle_pos'][1][2]),
      'tether_roll_tension_low': 5000.0,
      'tether_roll_tension_high': 20000.0,

      # Feedforward amplitude (rad) and phase (rad) for the tether roll cmd
      # Structure is amplitude * cos(loop_angle + phase)
      'tether_roll_ff_amplitude': np.deg2rad(5.0),
      'tether_roll_ff_phase': np.deg2rad(45.0),

      # Final saturations on angle commands [rad] and C_L commands [#]. The
      # saturation on the trans-out flare angle is intentionally deep within
      # stall.
      'alpha_cmd_min': -0.2,
      'alpha_cmd_max': np.deg2rad(12.0),
      'alpha_cmd_max_flare': np.pi / 3.0,
      'dCL_cmd_max': 0.1,
      'beta_cmd_min': np.deg2rad(-4.0),
      'beta_cmd_max': np.deg2rad(4.0),

      # Cutoff frequency [Hz] for tension high-pass filter.  We try to
      # cancel out this tension noise with the flaps.
      'fc_tension': 1.0,

      # Gain on the high frequency tension noise reduction feature.
      # This should be set to 0 until we have a metric for proving
      # its utility.
      'kp_tension_hf': 0.0,

      # Nominal tether tension [N] before flaring.
      'transout_tether_tension_cmd': 12000.0,

      # The steady command [rad] and its rate limit [rad/sec] for
      # tether roll angle prior to the TransOut flare. This command is
      # applied at the start of the path switch which occurs at
      # CrosswindCurvatureParams->loop_angle_path_switch_max
      # TODO: Should force this to finish before the
      # transout_flare_airspeed is reached. The closed loop response to
      # this command is on the slow side compared to the TransOut
      # timescales. The goal is to achieve this steady tether roll angle
      # and have zero roll rate on the kite just before the flare and so
      # a large rate limit on the command is used.
      'transout_tether_roll_cmd': np.deg2rad(15.0),
      'transout_tether_roll_cmd_rate_limit': np.deg2rad(60.0),

      # Airspeed [m/s] at which to begin the trans-out flare. This is loosely
      # coupled to the trans-out speed command schedule; the nominal and flare
      # airspeeds need to be set so that vehicle actually reaches / measures an
      # airspeed of transout_flare_airspeed. Otherwise, the values are
      # independent with this controlling how aggressive the flare is.
      'transout_flare_airspeed': 38.0,

      # Alpha command  and rate limit for CrosswindPrepTransOut. We don't want
      # propulsive lift the way CrosswindNormal does. We are trying to
      # slow down so we command a lower angle of attack.
      'preptransout_alpha_cmd': np.deg2rad(1.5),
      'preptransout_alpha_rate': np.deg2rad(0.5),

      # Alpha and beta angles [rad] and angular rates [rad/s] for the trans-out
      # flare.
      'transout_flare_alpha_cmd': np.pi / 3.0,
      'transout_flare_alpha_rate': np.pi / 7.0,
      'transout_flare_beta_cmd': np.deg2rad(0.0),
      'transout_flare_beta_rate': np.pi / 5.0
  }

  # Calculate the maximum lateral integrator values assuming a maximum
  # rudder and aileron deflection of 0.3 radians.
  inputs = [m.kCrosswindLateralInputAileron, m.kCrosswindLateralInputRudder]
  states = [m.kCrosswindLateralStateIntegratedTetherRoll,
            m.kCrosswindLateralStateIntegratedSideslip]
  min_lateral_integrators = np.dot(
      np.linalg.inv(np.array(controllers['lateral_gains_min_airspeed'])
                    [np.ix_(inputs, states)]),
      np.array([[0.0], [0.3]]))
  assert all(min_lateral_integrators < 0.0)

  max_lateral_integrators = -min_lateral_integrators

  delta_spoiler = np.deg2rad(-25.0)

  # Maximum desired state error or actuator deflection, used to define the LQR
  # weights for the longitudinal and lateral controllers synthesis.
  longitudinal_states_max = np.zeros((m.kNumCrosswindLongitudinalStates,))
  longitudinal_states_max[m.kCrosswindLongitudinalStatePositionGroundZ] = 10.0
  longitudinal_states_max[m.kCrosswindLongitudinalStateVelocityGroundZ] = 10.0
  longitudinal_states_max[m.kCrosswindLongitudinalStateAngleOfAttack] = 0.03
  longitudinal_states_max[m.kCrosswindLongitudinalStatePitchRate] = 0.2
  longitudinal_states_max[
      m.kCrosswindLongitudinalStateIntegratedAngleOfAttack] = 0.1
  longitudinal_inputs_max = np.zeros((m.kNumCrosswindLongitudinalInputs,))
  longitudinal_inputs_max[m.kCrosswindLongitudinalInputElevator] = 0.05
  longitudinal_inputs_max[m.kCrosswindLongitudinalInputMotorPitch] = 500.0

  lateral_states_max = np.zeros((m.kNumCrosswindLateralStates,))
  lateral_states_max[m.kCrosswindLateralStateTetherRoll] = 0.1
  lateral_states_max[m.kCrosswindLateralStateSideslip] = 0.04
  lateral_states_max[m.kCrosswindLateralStateRollRate] = 0.4
  lateral_states_max[m.kCrosswindLateralStateYawRate] = 0.4
  lateral_states_max[m.kCrosswindLateralStateIntegratedTetherRoll] = 0.1
  lateral_states_max[m.kCrosswindLateralStateIntegratedSideslip] = 0.2
  lateral_inputs_max = np.zeros((m.kNumCrosswindLateralInputs,))
  lateral_inputs_max[m.kCrosswindLateralInputAileron] = 0.2
  lateral_inputs_max[m.kCrosswindLateralInputRudder] = 0.2
  lateral_inputs_max[m.kCrosswindLateralInputMotorYaw] = 10000.0

  inner = {
      # Moving the flaps down creates an additional downward pitching
      # moment, which must be balanced by the elevator.  This creates a
      # feed-forward term that accounts for this effect.
      #
      # TODO: Automatically calculate this from the
      # aerodynamic database.
      'elevator_flap_ratio': -0.039,

      # This ratio [rad/rad] is used to create a feed-forward term for
      # the elevator deflection.  It was necessary to limit high
      # tensions by smoothing the transition to crosswind in the high
      # wind IEC cases.
      'delevator_dalpha': -0.5,

      # Gain [rad/#] between a change in C_L command and a delta.
      'kp_flap': 0.5,

      # Table of airspeeds [m/s] that are used to schedule and limit
      # gains for the inner loops.
      'airspeed_table': controllers['airspeed_table'],

      # Approximate maximum desired state error or actuator deflection.
      'longitudinal_states_max': longitudinal_states_max.tolist(),
      'longitudinal_inputs_max': longitudinal_inputs_max.tolist(),
      'lateral_states_max': lateral_states_max.tolist(),
      'lateral_inputs_max': lateral_inputs_max.tolist(),

      # Control matrix. This is the 3x3 matrix that, when multiplying a
      # vector of {aileron, elevator, rudder} flap deflections [rad],
      # produces a vector of angular accelerations, {p_dot, q_dot,
      # r_dot} [rad/s^2].
      'B_flaps_to_pqr': {'d': controllers['B_flaps_to_pqr_min_airspeed']},

      # Gain matrices to control the longitudinal plant calculated at
      # the minimum, nominal, and maximum airspeeds.  The state and
      # input vectors are:
      #
      #   x = [z, dz/dt, alpha, q, int_alpha]'
      #   u = [elevator, motor_pitch]'
      #
      'longitudinal_gains_min_airspeed': (
          controllers['longitudinal_gains_min_airspeed']),
      'longitudinal_gains_nominal_airspeed': (
          controllers['longitudinal_gains_nominal_airspeed']),
      'longitudinal_gains_max_airspeed': (
          controllers['longitudinal_gains_max_airspeed']),

      # Minimum and maximum values [rad-s] for the integrated alpha
      # error.  These roughly correspond to an integrated elevator
      # deflection between [-0.1, 0.15] rad assuming an integral gain
      # around 0.3.
      'int_alpha_min': -0.333,
      'int_alpha_max': 0.5,

      # Gain matrices to control the lateral plant calculated at the
      # minimum, nominal, and maximum airspeeds.  The state and input
      # vectors are:
      #
      #   x = [tether_roll, beta, p, r, int_tether_roll, int_beta]'
      #   u = [aileron, rudder, motor_yaw]'
      #
      'lateral_gains_min_airspeed': controllers['lateral_gains_min_airspeed'],
      'lateral_gains_nominal_airspeed': (
          controllers['lateral_gains_nominal_airspeed']),
      'lateral_gains_max_airspeed': controllers['lateral_gains_max_airspeed'],

      # Minimum and maximum values [rad-s] for the integrated beta
      # error and tether roll error.
      'int_tether_roll_min': (
          min_lateral_integrators[m.kCrosswindLateralInputAileron, 0]),
      'int_tether_roll_max': (
          max_lateral_integrators[m.kCrosswindLateralInputAileron, 0]),
      'int_beta_min': (
          min_lateral_integrators[m.kCrosswindLateralInputRudder, 0]),
      'int_beta_max': (
          max_lateral_integrators[m.kCrosswindLateralInputRudder, 0]),

      # Airspeed error [m/s] is saturated to plus/minus this value.
      'max_airspeed_error': 5.0,

      # Gains for the airspeed loop.  The negative integrator limit
      # [N] must be relatively small to avoid windup when the
      # propellers are at their maximum advance ratio.
      'airspeed_pid': {
          'kp': airspeed_controller['airspeed']['kp'],
          'ki': airspeed_controller['airspeed']['ki'],
          'kd': airspeed_controller['airspeed']['kd'],
          'int_output_min': -5000.0,
          'int_output_max': 5000.0
      },

      # Maximum magnitude of the aerodynamic power [W] that the airspeed
      # controller is allowed to command in generation and motoring.  This is
      # used to roughly bound the thrusts.  Detailed constraints on the motor
      # power are used in the motor solver.
      'max_airspeed_control_power_gen': 1000e3,
      'max_airspeed_control_power_motor': 650e3,

      # Maximum approximate slew rate of the thrust command [N/s].
      'max_airspeed_control_thrust_rate': 1.50e4,

      # Thrust command at crosswind initialization.
      'initial_thrust': params['trans_in']['longitudinal']['thrust_cmd'],

      # Airspeed errors [m/s] at which the spoiler turns on/off and
      # how far / fast it deflects [rad] / [rad/s].
      'airspeed_error_spoiler_on': 8.0,
      'airspeed_error_spoiler_off': 4.0,
      'delta_spoiler': delta_spoiler,
      'delta_spoiler_on_rate': 0.4 * delta_spoiler,
      'delta_spoiler_off_rate': -1.0 * delta_spoiler,

      # Harmonic integrator parameters.  Gain [#] from harmonics
      # components of beta error to harmonic integrator.
      'beta_harmonic_gain': 1.0,
      # Maximum value of the components of the harmonic integrator for
      # beta error [rad-s].
      'beta_harmonic_integrator_max': 0.1,

      # Whether to feed-forward the calculated acceleration necessary
      # to track the airspeed command, to the thrust command.
      'enable_acceleration_ff': True,
  }

  def LowerFlapLimits(delta_e):
    return [-30.0, -30.0, -90.0, -90.0, -30.0, -30.0, delta_e, -22.0]

  output = {
      # The rudder deflection limits [rad] are scheduled on beta
      # [radians] and airspeed [m/s] as indicated in the following
      # tables, provided in go/makanicl/43323 and documented in ECR 328.
      'rudder_limit_betas': np.deg2rad(
          np.array([-20.0, -10.0, 0.0, 10.0, 20.0])).tolist(),
      'rudder_limit_airspeeds': [50.0, 55.0, 60.0, 65.0, 70.0, 75.0],

      'rudder_limits_lower': np.deg2rad(
          np.array([[-22.00, -22.00, -22.00, -22.00, -22.00],
                    [-22.00, -22.00, -22.00, -22.00, -22.00],
                    [-22.00, -22.00, -22.00, -22.00, -16.42],
                    [-22.00, -22.00, -22.00, -22.00, -10.57],
                    [-22.00, -22.00, -22.00, -19.54, -5.92],
                    [-22.00, -22.00, -22.00, -15.72, -2.16]])).tolist(),

      'rudder_limits_upper': np.deg2rad(
          np.array([[22.00, 22.00, 22.00, 22.00, 22.00],
                    [17.67, 22.00, 22.00, 22.00, 22.00],
                    [9.87, 22.00, 22.00, 22.00, 22.00],
                    [3.80, 17.64, 22.00, 22.00, 22.00],
                    [-1.02, 12.83, 22.00, 22.00, 22.00],
                    [-4.91, 8.94, 21.97, 22.00, 22.00]])).tolist(),

      # Thrust and moment weights [#] for the least squares solver
      # used to set rotor velocities.  To optimize for power
      # generation, thrust should take a similar weight to the pitch
      # and yaw moments.  However, for the first flights, we will rely
      # significantly on motor pitch and yaw control, so the weights
      # have been modified to reflect this.
      #
      # TODO: Is there a good analytic way to determine
      # these weights?
      'thrust_moment_weights': {
          'thrust': 1.0,
          'moment': [3.0, 10.0, 2.0]
      },

      # Flap offsets and lower and upper limits [rad] for normal flight.
      'flap_offsets': (
          np.array(controllers['flap_offsets']) +
          ailerons_trim_angle_offset * np.array([1.0, 1.0, 0.0, 0.0, 1.0, 1.0,
                                                 0.0, 0.0])
          ).tolist(),
      'lower_flap_limits': [
          np.deg2rad(angle) for angle in LowerFlapLimits(-10.0)
      ],
      'upper_flap_limits': [
          np.deg2rad(angle)
          for angle in [10.0, 10.0, 0.0, 0.0, 10.0, 10.0, 15.0, 22.0]
      ],

      # Flap lower and upper limits [rad] for the trans-out stall.
      'lower_flap_limits_flare': [
          np.deg2rad(angle) for angle in LowerFlapLimits(-25.0)
      ],

      # Delay [sec] in tether release command to first reduce load on the wing.
      'release_wait_period': 0.5,

      # Aileron command [rad] to reduce load on the wing prior to
      # tether release.
      'release_aileron_cmd': -0.3,

      # Flag to indicate whether to use a detwist command that uses the
      # tether departure direction to reduce the termination angle
      # excursions (True), or a command that is equal to the current kite
      # loop angle (False).
      'adaptive_detwist_cmd': True,
  }

  mode = {
      # Minimum velocity [m/s] for the switch to crosswind.
      'min_wing_speed': 15.0,

      # Minimum tension [N] for the switch to crosswind.  The wing
      # must achieve a lift coefficient near 2 at the minimum velocity
      # for the switch to crosswind.
      'min_tension': 10000.0,

      # Vertical position [m] below which we cannot switch to
      # crosswind.  This is defined to be vertical position of the
      # path center at the specified elevation.
      # See b/117524509 for discussion of this value.
      'max_wing_pos_g_z': (
          -params['tether']['length'] * np.sin(np.deg2rad(45.0))),

      # Critical angles [rad] and speeds [m/s] for transition-out.
      # Note that the maximum transition-out speed actually increases
      # past the middle of the upstroke because the controller should
      # be strongly committed to transitioning out.
      'loop_angle_table': [2.0 * np.pi * x
                           for x in [0.0, 0.1, 0.3, 0.75, 0.8, 1.0]],
      'max_trans_out_speed_table': [15.0, 10.0, 0.0, 0.0, 25.0, 15.0],

      # Minimum airspeed for returning to crosswind normal once the final
      # trans-out sequence has begun.
      'min_airspeed_return_to_crosswind': 38.0,

      # Maximum time the vehicle is allowed to flare. In extreme cases where
      # there's relatively little elevator effectiveness and/or large side
      # slips, the vehicle may not reach the airspeed or alpha threshold. As a
      # last ditch effort, we hand over to the hover controller after a certain
      # amount of time.
      'transout_max_time_in_flare': 3.0,

      # Maximum airspeed [m/s] at which to allow hover trans-out.
      'transout_airspeed': 32.0,

      # Minimum angle of attack [rad] at which to allow hover trans-out.
      'transout_alpha': np.deg2rad(20.0),

      # Minimum time [s] spent in kFlightModeHoverAccel before a transition
      # to kFlightModeHoverTransOut is allowed.
      'min_time_in_accel': 5.0,

      # Maximum time [s] spent in kFlightModeHoverAccel before a transition
      # to kFlightModeHoverTransOut is forced.
      'max_time_in_accel': 15.0,

      # Minimum specific-force vector magnitude [m/s^2] in between the
      # minimum and maximum acceleration times.  If acc_norm_f drops
      # below this threshold during kFlightModeHoverAccel, we assume
      # we are decelerating, and enter kFlightModeHoverTransOut.
      'acc_slow_down_threshold': params['phys']['g'] * 1.0,
  }

  experiments = crosswind_exp.GetExperiments()

  return {
      'loop_dir': path['loop_dir'],
      'power': power,
      'path': path,
      'curvature': curvature,
      'inner': inner,
      'output': output,
      'mode': mode,
      'playbook': playbook,
      'playbook_fallback': playbook_fallback,
      'enable_new_pitch_rate_cmd': True,
      'experiments': experiments,
  }

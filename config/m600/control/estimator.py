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

"""Estimation parameters."""

from makani.config import mconfig
from makani.control import system_types as m
import numpy as np
from scipy import signal


def _CalcUpwashAnglePerCL(pos, wing_params):
  """Computes the upwash angle detected by the pitot due to wing lift.

  The upwash is computed assuming a constant strength bound vortex
  in the wing. See Houghton & Carpenter Eq. 5.9 and the
  Kutta-Joukowsky theorem.

  Args:
    pos: position of the pitot sensors
    wing_params: dict containing wing parameters

  Returns:
    delta_alpha: the amount that alpha changes due to upwash of the wing.
  """
  d = np.hypot(pos[0], pos[2])
  y1 = pos[1] + wing_params['b'] / 2.0
  y2 = pos[1] - wing_params['b'] / 2.0
  # We neglect the change in freestream velocity due to the induced
  # velocity and make a small angle approximation.
  delta_alpha = 0.5 * wing_params['c'] * (pos[0] / d) * (
      y1 / np.hypot(y1, d) - y2 / np.hypot(y2, d)) / (4.0 * np.pi * d)
  assert 0.0 <= delta_alpha and delta_alpha <= 0.1
  return delta_alpha


def _CheckFilter(b, a, pass_band, max_phase):
  (_, freq_resp) = signal.freqz(b, a, np.linspace(0.0, pass_band, 1000))
  return (np.angle(freq_resp[0]) == 0.0
          and np.max(np.abs(np.angle(freq_resp))) < max_phase)


@mconfig.Config(deps={
    'common_params': 'common.common_params',
    'phys': 'common.physical_constants',
    'pitot': 'm600.pitot',
    'simple_aero': 'm600.control.simple_aero_model',
    'tether': mconfig.WING_MODEL + '.tether',
    'wing': mconfig.WING_MODEL + '.wing',
    'test_site': 'common.test_site'
})
def MakeParams(params):
  upwash_angle_per_cl = _CalcUpwashAnglePerCL(
      params['pitot']['pos'], params['wing'])

  # Coefficients for IIR filters on the rate gyros and accelerometers.
  # These filters attenuate motor vibrations while avoiding reducing
  # phase margin significantly (which is tested approximately by
  # _CheckFilter).
  #
  # The filter cutoff frequencies were hand-optimized to provide
  # sufficient rejection at the vibration frequencies without adding
  # significant phase lag.
  #
  # NOTE: The coefficients generated here should be updated in
  # generate_hover_controllers.m whenever these filters are adjusted.
  vibration_filter_b, vibration_filter_a = signal.butter(
      2, 8.3 * 2.0 * params['common_params']['ts'])

  assert _CheckFilter(vibration_filter_b, vibration_filter_a,
                      0.6 * 2.0 * np.pi * params['common_params']['ts'],
                      np.deg2rad(10.0))

  # Coefficients for IIR filters on the wing velocity.
  Vb_filter_b, Vb_filter_a = signal.butter(
      2, 3.3 * 2.0 * params['common_params']['ts'])

  assert _CheckFilter(Vb_filter_b, Vb_filter_a,
                      0.4 * 2.0 * np.pi * params['common_params']['ts'],
                      np.deg2rad(10.0))

  # Adjust the detwist axis offset depending on the test site.
  # This parameter is only used if adaptive_detwist_cmd in crosswind.py is set
  # to True. This axis is raised to ensure the tether departure direction
  # encircles the axis and also stays far to avoid fast commanded detwist
  # angles. Offshore, the axis needs to be higher up to compensate for the buoy
  # pitching motion.
  if params['test_site'] in [m.kTestSiteNorway]:
    detwist_axis_offset_deg = 20.0
  else:
    detwist_axis_offset_deg = 10.0

  return {
      # Times [s] to initialize the estimator.
      't_initialize': 11.0,

      # Initial payout [m] to set if kControlOptHardCodeInitialPayout
      # is enabled.
      'hard_coded_initial_payout': 0.0,

      'apparent_wind': {
          # Bias [rad] subtracted from the angle-of-attack estimate to
          # compensate for upwash effects.
          'pitot_upwash_alpha_bias': (
              params['simple_aero']['CL_0'] * upwash_angle_per_cl
          ),

          # Scale factor [rad/rad] to divide the angle-of-attack estimate by
          # to compensate for upwash effects.
          'pitot_upwash_alpha_scale': (
              1.0 + params['simple_aero']['dCL_dalpha'] * upwash_angle_per_cl
          ),

          # Speed [m/s] below/above which to not/fully trust the pitot.
          'v_low': 12.0,
          'v_high': 17.0,

          # Estimated alpha and beta [rad] below/above which to fully/not
          # trust the pitot.
          'ang_est_low': np.deg2rad(30.0),
          'ang_est_high': np.deg2rad(40.0),

          # Pitot or loadcell measured alpha and beta [rad] below/above
          # which to fully/not trust the pitot or loadcell.
          'ang_fly_low': np.deg2rad(20.0),
          'ang_fly_high': np.deg2rad(30.0),

          # Cutoff frequencies [Hz] for filtered apparent wind estimate.
          'fc_v': 1.0,
          'fc_alpha': 5.0,
          'fc_beta': 5.0,

          # Cutoff frequency [Hz] for complementary apparent wind filter.
          'fc_comp': 0.5,
      },

      'nav': {
          'attitude': {
              # Initial attitude quaternion.
              'q_g2b_0': [1.0, 0.0, 0.0, 0.0],

              # Coarse initialization proportional gains [rad/s], [#], [#].
              'coarse_init_kp': 10.0,
              'coarse_init_kp_acc': 0.2,
              'coarse_init_kp_mag': 0.8,

              # Initial standard deviation [rad] for attitude errors.
              'sigma_attitude_0': 0.5,

              # Initial standard deviation [rad/s] for gyro biases.
              'sigma_gyro_bias_0': 0.01,

              # Gyro angle random walk [rad/s/rtHz].  The ADIS16488 datasheet
              # indicates a typical gyro angle random walk of 0.26 deg/sqrt(hr).
              # 0.26 deg/sqrt(hour)
              #   = 0.26 * (pi/180 rad) / sqrt(hour)
              #   = 0.26 * pi/180 rad/sqrt(hour) * sqrt(hour)/sqrt(3600 s)
              #   = 0.26 * pi/180 / sqrt(3600) * rad/sqrt(s) * s/s
              #   = 0.26 * pi/180 / sqrt(3600) * rad/s/sqrt(Hz)
              #   = 7.5631e-05 rad/s/sqrt(Hz)
              'sigma_gyro_noise': 0.00075,  # Tuned.

              # Gyro bias instability [rad/s].  The ADIS16488 datasheet
              # indicates a typical gyro bias instability of 5.1 deg/hr.
              # 5.1 deg/hr
              #   = 5.1 * pi/180 rad / 3600 s
              #   = 2.4725e-05 rad/s
              'sigma_gyro_bias_instability': 4.9e-6,  # Tuned.

              # Time constant [s] for the gyro biases. This parameter relates
              # to sigma_gyro_bias_instability to describe a Gauss-Markov
              # process noise.
              'gyro_bias_time_constant': 3600.0,

              # Nominal angle-of-attack [rad] and angle-of-sideslip [rad]
              # for computing the apparent wind vector.
              'nominal_angle_of_attack': 0.0,
              'nominal_angle_of_sideslip': 0.0,

              # Multiplier [#] on the airspeed to give the standard
              # deviation for apparent wind vector updates.
              # Set per b/64558980#comment3 on 2017-10-06.
              'v_app_relative_err': 2.5,

              # Minimum airspeed [m/s] when calculating the standard
              # deviation for apparent wind vector updates.
              'v_app_relative_err_min_airspeed': 15.0,

              # Multiplier [#] on the norm of mag_g to give the
              # standard deviation for magnetometer updates.
              # Set per b/64558980#comment3 on 2017-10-06.
              'mag_relative_err': 0.75,

              # Multiplier [#] on the norm of g to give the standard deviation
              # for plumb bob gravity updates.
              'plumb_bob_relative_err': 0.4,

              # Multiplier [#] to account for the difference between
              # the magnitude of the accelerometer reading and the
              # expected value of g.
              'plumb_bob_g_err_scale': 7.0,

              # Maximum magnitude [rad/s] for gyro biases.
              'max_gyro_bias': 0.05,

              # Cutoff frequency [Hz] for the pre-filter applied before
              # treating the accelerometers as a vector measurement of gravity.
              'fc_acc': 5.0,

              # Correct attitude by comparing the apparent wind vector rotated
              # into ground coordinates to the ground station wind sensor. This
              # setting only affects crosswind attitude estimation.
              'enable_apparent_wind_correction': True,

              # Correct attitude by comparing the port-to-starboard (or
              # wingtip-to-center) GPS position vector rotated into body
              # coordinates against the body frame antenna locations.
              'enable_gps_vector_correction': True,

              # For the GPS vector attitude correction, this parameter defines
              # additional uncertainty to account for mounting and wing flex
              # errors in the physical location of each antenna.
              'wing_wingtip_to_center_sigma': 0.2,  # [m]

              # For the GPS vector attitude correction, this parameter defines
              # additional uncertainty to account for mounting and wing flex
              # errors in the physical location of each antenna.
              'wing_port_to_star_sigma': 0.2,  # [m]

              # For the GPS vector attitude correction, this parameter defines
              # the required ratio between the baseline and standard deviation
              # to apply the measurement correction. A ratio of 9.0 means that
              # the baseline must be 9.0 times the standard deviation. For a
              # 26 m baseline, sigma must be less than 2.89 m.
              'wing_vector_sigma_ratio': 9.0,

              # Reject measurements with a GPS position standard deviation
              # greater than this threshold.
              'max_gps_position_sigma': 0.1,  # [m]

              # Reject measurements by comparing the measured distance and the
              # known physical distance between the two antennas. When choosing
              # this parameter consider the scenario when the measurement error
              # is perpendicular to baseline: a 0.5 m error on a 12.8 m baseline
              # corresponds to a atan2(0.5, 12.8) = 2.2 deg error.
              'max_gps_vector_error': 0.1,  # [m]

              # Scale the difference between the measured distance and the known
              # physical distance between antennas to obtain a measurement
              # standard deviation.
              'gps_vector_relative_err': 1.0,  # [m]

              # Number of estimator cycles to consider the GPS vector solution
              # valid and therefore prevent application of the apparent wind
              # correction.
              'gps_vector_timeout_cycles': 500,

              # Reject measurements with a GPS compass standard deviation
              # greater than this threshold.
              'max_gps_compass_sigma': np.deg2rad(10.0)
          },

          'position': {
              'baro': {
                  'kalman_est': {
                      # Minimum and maximum filter cutoff-frequencies [Hz].
                      'fc_min': 0.0,
                      'fc_max': 1.0,

                      # Random walk variance for barometric altitude bias [m^2].
                      'Q': 0.01,
                  },

                  # Initial standard deviation [m] for the unknown
                  # altitude bias.
                  'sigma_Xg_z_bias_0': 100.0,

                  # Standard deviation [m] for the instantaneous
                  # altitude error for each pressure measurement.
                  'sigma_Xg_z': 4.0
              },

              'filter': {
                  # Initial velocity standard deviation [m/s].
                  'sigma_vel_g_0': 0.1,

                  # Multipliers [#] to apply to the GPS sigmas.
                  #
                  # Currently the position filter re-uses stale GPS
                  # data.  We multiply the reported standard
                  # deviations by the square-root of the GPS update
                  # period divided by the controller update period to
                  # approximately account for this re-use.
                  #
                  # TODO: Signal to the estimator when fresh
                  # GPS data arrives and remove this multiplier.
                  'gps_sigma_multiplier': (0.05 / 0.01)**0.5,

                  # Minimum GPS velocity [m/s] and position [m]
                  # standard deviations.
                  'min_gps_sigma_vel_g': 0.3,
                  'min_gps_sigma_pos_g': 0.1,

                  # Acceleration standard deviation [m/s^2].
                  #
                  # These values set the acceleration standard
                  # deviation (used for the process noise in the
                  # Kalman filter) for hover and all other flight
                  # modes.  These values are set by assuming a maximum
                  # of 12 degrees of attitude error and 60 [m/s^2] of
                  # crosswind acceleration.
                  'sigma_wing_accel_hover': 1.96,
                  'sigma_wing_accel_dynamic': 11.0,

                  # Whether or not to enable barometric altitude.
                  'baro_enabled': False,

                  # Whether or not to enable GLAS aiding.
                  'glas_enabled': False,

                  # Number of cycles to wait after losing GPS to begin applying
                  # GLAS corrections.
                  'gps_position_timeout_cycles': int(
                      10.0 / params['common_params']['ts']),
              },

              'glas': {
                  # Multiplier [#] on tether length to give a minimum
                  # position standard deviation.
                  'min_relative_sigma_pos_g': 0.025,

                  # Position sigma per weight-over-tension ratio [m].
                  #
                  # The reported position standard deviation from the GLAS
                  # estimate is taken to be inversely proportional to
                  # the tether tension.
                  #
                  # TODO: Investigate models that more
                  # accurately account for drag and inertial effects.
                  # One such implementation was removed by I91657020.
                  'sigma_per_weight_over_tension_ratio': 0.45,

                  # Limit on tether weight over tension [#] for use
                  # in the uncertainty model mentioned above.
                  'max_weight_over_tension_ratio': 0.5,

                  # Cutoff frequency [Hz] for updating the GSG biases.
                  'gsg_bias_fc': 0.003,

                  # Minimum tension [N] for updating the GSG biases.
                  #
                  # The bias correction uses a straight line tether
                  # model.  This threshold is set high enough that
                  # catenary effects are negligible.
                  'gsg_bias_tension_lim': 10.0 * (
                      params['tether']['length'] *
                      params['tether']['linear_density'] * params['phys']['g']),

                  # Limits on the GSG biases [rad].
                  'bias_low': {'azi': -np.deg2rad(40.0), 'ele': -0.1},
                  'bias_high': {'azi': np.deg2rad(40.0), 'ele': 0.1}
              },

              'gps': {
                  # Number [#] of control cycles to ignore GPS data for after
                  # a thrown error.
                  'outage_hold_num': 3,

                  # Distance [m] below which GPS position estimates
                  # are guaranteed to be considered in agreement.  The
                  # first number is for hover flight modes, and is
                  # selected to avoid jumps that could endanger a
                  # successful ascend or descend.  The
                  # second number is for all other flight modes, and is
                  # set to be twice the difference in position
                  # expected due to a 0.05 second delay between the
                  # receivers when travelling 60 m/s.
                  'min_disagreement_distance_hover': 0.3,
                  'min_disagreement_distance': 6.0,

                  # Fraction of the minimum disagreement distance [#]
                  # from the current estimate that a given receiver
                  # must be before switching away from using its
                  # solution.
                  'disagreement_hysteresis_ratio': 0.5,

                  # Excess position uncertainty [m] added to the alternate GPS
                  # receiver to avoid chatter between receivers.
                  'sigma_hysteresis': 0.2,

                  # Relative threshold [#] used to compare GPS sigma's
                  # when deciding to switch between receivers.
                  'relative_sigma_threshold': 1.5,
              },
          },

          # Filter frequency [Hz] for velocity to determine when to use the
          # V_app vector.
          'fc_Vg': 5.0,

          # Cutoff frequency [Hz] for the norm of the acclerometer reading.
          'fc_acc_norm': 0.4,

          # See definition above.
          'vibration_filter_a': vibration_filter_a.tolist(),
          'vibration_filter_b': vibration_filter_b.tolist(),
          'Vb_filter_a': Vb_filter_a.tolist(),
          'Vb_filter_b': Vb_filter_b.tolist(),

          # Maximum value [m] for the norm of the sigma of position of the
          # position filter.  Once we exceed this value, we declare the estimate
          # to be invalid.
          'max_valid_position_sigma_norm': 3.0,
      },

      'tether_anchor': {
          # Filter parameters for the tether anchor.
          'fc_near_perch': 0.1,
          'fc_far_from_perch': 0.01,

          # Payout values [m] over which the filter cutoff frequency is faded.
          'payout_near_perch': 0.0,
          'payout_far_from_perch': 100.0,

          'fc_lateral': 0.1,
          'zeta_lateral': np.sqrt(2.0) / 2.0,
      },

      'ground_station': {
          # Number [#] of controller iterations to debounce the ground station
          # mode.
          'num_debounce': 20,
      },

      'joystick': {
          # Number [#] of controller iterations to debounce joystick switches.
          'joystick_num_debounce': 11,

          # Throttle filter frequency [Hz].
          'fc_throttle': 1.0,

          # Pitch filter frequency [Hz].
          'fc_pitch': 1.0,
      },

      'perch_azi': {
          # Bound on the angular rate [rad/s] estimate for the perch azimuth.
          'max_angle_vel': 1.0,

          # Filter cutoff [Hz] for the perch azimuth angular rate.
          'fc_angle_vel': 1.0,

          # Damping ratio [#] for the filter on perch azimuth angular
          # rate.
          'damping_ratio_angle_vel': 1.0,
      },

      'tether_force': {
          # Cutoff frequency [Hz] for the filtered tension.
          'fc_tension': 1.0
      },

      'tether_ground_angles': {
          # Half-angle [rad] of the cone about the detwist axis in which to hold
          # the estimate of the tether detwist angle.
          'hold_cone_half_angle': np.deg2rad(1.0),

          # Angle [rad] that the axis used to compute the detwist angle is
          # offset upwards to ensure the tether departure direction encircles
          # this detwist axis.
          'detwist_axis_offset': np.deg2rad(detwist_axis_offset_deg),
      },

      'weather': {
          # Filter cutoff frequency [Hz] and damping ratio [#] for a
          # low-pass filter applied to air density measurements.
          'fc_rho': 1.0/60.0,
          'zeta_rho': 1.0,
      },

      'wind': {
          # Wind vector [m/s] to use when the kControlOptHardCodeWind
          # is enabled.
          #
          # TODO: This should probably be a system parameter.
          'hard_coded_wind_g': [-8.19152, -5.73576, 0.0],

          # Filter cutoff frequencies [Hz].
          'fc_initialize': 1.0,
          'fc_vector': 0.2,
          'fc_vector_slow': 0.01,
          'fc_speed': 0.01,
          'fc_speed_playbook': 0.005,
          'zeta_speed_playbook': 1.0,
          'fc_dir': 0.01,
          'zeta_dir': 1.0,
          'fc_dir_playbook': 0.005,
          'zeta_dir_playbook': 1.0,
          # Maximum allowable difference between wind aloft and ground wind.
          # Wind aloft direction will be saturated if greater.
          'playbook_aloft_azi_offset_max': np.deg2rad(20.0)
      },
  }

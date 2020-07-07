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

"""Ground station v2 (GS02) simulator parameters."""
from makani.analysis.control import geometry
from makani.config import mconfig
import numpy as np


@mconfig.Config(deps={
    'buoy_sim': 'base_station.sim.buoy_sim',
    'ground_station': 'base_station.ground_station',
    'mclaren_params': 'base_station.sim.mclaren_matlab_params',
    'phys_sim': 'common.sim.phys_sim',
    'wing_serial': 'common.wing_serial',
})
def MakeParams(params):
  """Make ground station GS02 parameters."""
  # Origin of the panel frame in platform coordinates [m].
  origin_pos_p = [0.288, 7.15, -4.332]

  # Radius [m] of the cylinders that define the panels.
  radius = 4.0

  # For each panel, this is the angle [rad] the panel_origin-to-cylinder_center
  # vector makes with the +x_panel direction, about the panel z-axis.
  port_panel_angle = np.deg2rad(180.0 - 11.0)
  star_panel_angle = np.deg2rad(180.0 + 16.0)

  mclaren = params['mclaren_params']
  mc = mclaren['mclaren']

  d2r = lambda x: np.deg2rad(x).tolist()

  # The setpoint for the detwist angle at start [deg].
  detwist_setpoint = 203.0

  # Tether free length [m] below which the proximity sensor will fire.
  # This is hand-tuned in the sim to be 10cm past the free length at which the
  # kite initializes on the perch, and depends on the bridle geometry.
  if mconfig.WING_MODEL == 'oktoberkite':
    # Manually tuned to get kite to perch.
    prox_sensor_tether_free_length = 6.52
  else:
    prox_sensor_tether_free_length = 3.02

  return {
      # These parameters correspond to the model described in
      # the "Azimuth response" section of go/makani-controls-gs02.
      'mclaren': {
          # Sample time [s] for the McLaren controller. The true sample time is
          # 0.0096s, but that's close enough to the simulator sample time that
          # we ignore the difference.
          'ts': 0.01,
          'detwist_setpoint': np.deg2rad(detwist_setpoint),

          'reel': {
              # Natural Frequency [rad/sec] and damping ratio [Nondim] for the
              # azimuth command prefilter.
              'azi_cmd_filter_omega': 5.0,
              'azi_cmd_filter_zeta': 1.0,

              # Gain [nondim] for the azimuth velocity feedforward path.
              'azi_vel_cmd_ff_gain': 0.75,

              # Saturation value for azimuth error [rad].
              'azi_error_max': np.deg2rad(2.0),

              # Proportional gain [1/s] for the McLaren controller.
              'azi_vel_cmd_kp': 1.5,

              # Rate limit [rad/s^2] for the McLaren controller.
              'azi_vel_cmd_rate_limit': 0.11,

              # Angle [rad] by which the ground station azimuth is offset from
              # the wing azimuth.
              'azi_offset_from_wing': (params['ground_station']['gs02']
                                       ['reel_azi_offset_from_wing']),

              # Maximum drum angle [rad].
              'drum_angle_upper_limit': -1.977 * np.pi,

              # Max commanded drum acceleration [rad/s^2].
              'max_drum_accel': (params['ground_station']['gs02']
                                 ['max_drum_accel_in_reel']),

              # Fraction of dead zone at which to zero azimuth position
              # error. See the MAT controller parameter rAziLittleDeadzone.
              'little_dead_zone': 1.0 / 3.0,
          },

          'transform': {
              # Half-width [rad] of the dead zone to apply about azimuth target
              # angles.
              'azi_dead_zone_half_width': d2r(mc['aDead_Azi_Transform']),

              # Fraction [#] of the maximum acceleration to use when attenuating
              # azimuth velocity commands.
              'azi_decel_ratio': mc['rDecel_Azi_Transform'],

              # Maximum acceleration [rad/s^2] for azimuth commands.
              'azi_max_accel': mc['dnMax_Azi_Transform'],

              # Nominal velocity command [rad/s] for the azimuth.
              'azi_nominal_vel': mc['nNominal_Azi_Transform'],

              # Base offset angle [rad] of the platform from the wing azimuth.
              'azi_offset_from_wing': -np.pi,

              # Target azimuth angles [rad] relative to azi_offset_from_wing for
              # each stage of a HighTension to Reel transform.
              'azi_targets_ht2reel': d2r(mc['aTargetHT2Reel_Azi_Transform']),

              # Target azimuth angles [rad] for each stage of a Reel to
              # HighTension transform.
              'azi_targets_reel2ht': d2r(mc['aTargetReel2HT_Azi_Transform']),

              # Tolerances [rad] for which to consider the azimuth target
              # satisfied for each stage of a HighTension to Reel transform.
              'azi_tols_ht2reel': d2r(mc['aTolHT2Reel_Azi_Transform']),

              # Tolerances [rad] for which to consider the azimuth target
              # satisfied for each stage of a Reel to HighTension transform.
              'azi_tols_reel2ht': d2r(mc['aTolReel2HT_Azi_Transform']),

              # Half-width [rad] of the dead zone to apply about winch target
              # angles.
              'winch_dead_zone_half_width': d2r(mc['aDead_Winch_Transform']),

              # Fraction [#] of the maximum acceleration to use when attenuating
              # winch velocity commands.
              'winch_decel_ratio': mc['rDecel_Winch_Transform'],

              # Maximum acceleration [rad/s^2] for winch commands.
              'winch_max_accel': mc['dnMax_Winch_Transform'],

              # Nominal velocity command [rad/s] for the winch.
              'winch_nominal_vel': mc['nNominal_Winch_Transform'],

              # Target winch angles [rad] for each stage of a HighTension to
              # Reel transform.
              'winch_targets_ht2reel': d2r(
                  mc['aTargetHT2Reel_Winch_Transform']),

              # Target winch angles [rad] for each stage of a Reel to
              # HighTension transform.
              'winch_targets_reel2ht': d2r(
                  mc['aTargetReel2HT_Winch_Transform']),

              # Tolerances [rad] for which to consider the winch target
              # satisfied for each stage of a HighTension to Reel transform.
              'winch_tols_ht2reel': d2r(mc['aTolHT2Reel_Winch_Transform']),

              # Tolerances [rad] for which to consider the winch target
              # satisfied for each stage of a Reel to HighTension transform.
              'winch_tols_reel2ht': d2r(mc['aTolReel2HT_Winch_Transform']),

              # Target detwist angles [rad] for each stage of a HighTension to
              # Reel transform.
              # The targets are ordered as [0, 1, 2, 3, 4]
              'detwist_targets_ht2reel': d2r([0.0, 165.0, 165.0,
                                              detwist_setpoint,
                                              detwist_setpoint]),

              # Target detwist angles [rad] for each stage of a Reel to
              # HighTension transform.
              # The targets are ordered as [0, 4, 3, 2, 1]
              'detwist_targets_reel2ht': d2r([detwist_setpoint, 0.0, 165.0,
                                              165.0, detwist_setpoint]),

              # Max velocity [rad/s] for the detwist.
              # See DetwistPro:Configuration.MaxTrackingVelocity_rps in file
              # configuration.csv in
              # Makani_Groundstation/UserPartition/permBackup/GS02-0x/
              'detwist_max_vel': 1.257,
          },

          'high_tension': {
              # Threshold brake torque [N-m] to switch from state machine
              # case 0 to either case 1 or 6.
              'm_max_azi_ht': 5000.0,

              # Threshold azimuth error [rad] to switch from state machine
              # case 0 to either case 1 or 6.
              'a_control_threshold_azi_ht': 0.0873,

              # Threshold total torque [N-m] to switch from state machine case 1
              # to 2 or from case 6 to 7.
              'm_control_threshold_azi_ht': 500.0,

              # Nominal commanded azimuth angular rate [rad/s] in state machine
              # cases 3 or 8.
              'n_demand_azi_ht': 1.0,

              # Threshold azimuth error [rad] to command state machine case 5
              # from either cases 3 or 8.
              'a_control_tolerance_azi_ht': 0.0262,

              # Threshold azimuth rate [rad/s] to command state machine case 0.
              'n_control_tolerance_azi_ht': 0.05,

              # Maximum time [s] in state machine case 5 before switching to
              # case 0.
              't_threshold_wait_azi_ht': 10.0,

              # Steady state non-zero azimuth rate [rad/s] based on GS02 test
              # data.
              'omega_nom': 0.122,

              # Threshold azimuth rate [rad/s] for comparing the difference
              # between azimuth rates of type double.
              'test_threshold': 0.001,

              # Desired azimuth 1st order spin up loop time constant [s].
              'tau_spin': 0.6,

              # Desired azimuth 1st order stopping loop time constant [s].
              'tau_stop': 0.3,

              # Ground station inertia about the azimuth axis [kg-m^2].
              'Iz_gndstation': 9.44e4,

              # 1st order spin up loop gain: Iz_gndstation/tau_spin [kg-m^2/s].
              'k_spin': 1.573e5,

              # 1st order stopping loop gain: Iz_gndstation/tau_stop [kg-m^2/s].
              'k_stop': 3.147e5,

              # Max velocity [rad/s] for the detwist.
              # See DetwistPro:Configuration.MaxTrackingVelocity_rps in file
              # configuration.csv in
              # Makani_Groundstation/UserPartition/permBackup/GS02-0x/
              'detwist_max_vel': 1.257,
          },
      },

      # Proportional gain [1/s] for the drive controller model.
      'azi_accel_kp': 9.5,

      # Proportional gain [1/s] for the detwist controller model.
      # The kp is retrieved from the flight log, by comparing detwist motor
      # command and the detwist angle.
      'detwist_angle_kp': 46.8,

      # Natural frequency [Hz] and damping ratio [#] for a second-order model of
      # the winch drive response. See the "Winch response" section of
      # go/makani-controls-gs02.
      'winch_drive_natural_freq': np.sqrt(250.0),
      'winch_drive_damping_ratio': 0.55,

      # Rates [m/rad] at which the main- and wide-wrap sections of the
      # tether track traverse the drum x-direction with respect to drum
      # angle.  These are derived from the camming chart:
      # https://docs.google.com/spreadsheets/d/1Zbgf6RQRHo1KDB0dMvGqBsKi1cp5f8RNb6y5Gb4Mz6I"
      #
      # For a given lead angle alpha, a positive drum rotation of dtheta
      # corresponds to a translation of the wrapping along the negative
      # x-direction by r*tan(alpha).
      'dx_dtheta_main_wrap': -0.00696,
      'dx_dtheta_wide_wrap': -0.0733,

      'panel': {
          # For each panel, the center [m] in the panel frame and radius [m]
          # describe the cylinder modeling it. The z_extents_p [m] specify the
          # planes, parallel to the platform xy-plane, at which the cylinders
          # are cut.
          'port': {
              'center_panel': [radius * np.cos(port_panel_angle),
                               radius * np.sin(port_panel_angle)],
              'radius': radius,
              'z_extents_p': [origin_pos_p[2], origin_pos_p[2] + 3.0],
          },

          'starboard': {
              'center_panel': [radius * np.cos(star_panel_angle),
                               radius * np.sin(star_panel_angle)],
              'radius': radius,
              'z_extents_p': [origin_pos_p[2], origin_pos_p[2] + 3.0],
          },

          # These parameters define the panel frame relative to the platform
          # frame. Its origin is the top of the panels along the bookseam (the
          # seam between the panels). Then we apply a yaw rotation so that +x
          # points along the nominal boom direction (see note below), and
          # finally a pitch rotation corresponding to the inclination of the
          # panels.
          #
          # Note: The actual boom direction is another 4 degrees from the
          # platform +x-direction, but this is compensated up to 0.1 degrees
          # by rotation of the panels. We ignore both the boom misalignment and
          # the compensation.
          'origin_pos_p': origin_pos_p,
          'dcm_p2panel': {'d': geometry.AngleToDcm(
              np.deg2rad(90.0), np.deg2rad(-14.0), 0.0).tolist()},

          # Y extents [m] of the panel apparatus in the panel coordinate
          # system. Visualizer only.
          'y_extents_panel': [-1.9, 2.0],
      },

      # Radius [m] of the visualized platform.
      'platform_radius': 3.0,

      # Drum frame x-coordinate [m] at which the tether wrapping on the main
      # body of the drum begins.
      'wrap_start_posx_drum': -1.985,

      # Drum frame x-coordinate [m] of the transition between the wide-wrap
      # section and the main-wrap section.
      'wrap_transition_posx_drum': -1.634,

      # Initial angle [rad] for the platform azimuth.
      'initial_platform_azi': (
          params['phys_sim']['wind_direction'] + np.deg2rad(90.0) -
          params['buoy_sim']['mooring_lines']['yaw_equilibrium_heading']),

      # Initial angles [rad] for the drum. Has no effect if init_tether_tension
      # is true.
      'initial_drum_angle': np.deg2rad(-170.0),

      # Whether to choose an initial drum angle that yields a reasonable tether
      # tension. Pre-empts initial_drum_angle.
      'init_tether_tension': True,

      # Tether free length [m] below which the proximity sensor will fire.
      'prox_sensor_tether_free_length': prox_sensor_tether_free_length,

      # The minimum tether elevation [deg] in the platform frame to engage the
      # levelwind.
      'min_levelwind_angle_for_tether_engagement': np.deg2rad(-2.0),
  }

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

"""Hover controller parameters."""
import copy

from makani.config import mconfig
from makani.config.m600.control.experiments import hover as hover_experiments
from makani.config.oktoberkite.control import hover_controllers
from makani.control import system_types as m
from makani.lib.python import dict_util
import numpy as np


@mconfig.Config(deps={
    'crosswind': mconfig.WING_MODEL + '.control.crosswind',
    'flight_plan': 'common.flight_plan',
    'ground_frame': 'base_station.ground_frame',
    'ground_station': 'base_station.ground_station',
    'gs_model': 'base_station.gs_model',
    'levelwind': 'base_station.levelwind',
    'perch': 'base_station.perch',
    'phys': 'common.physical_constants',
    'rotors': mconfig.WING_MODEL + '.rotors',
    'system': mconfig.WING_MODEL + '.system_params',
    'tether': mconfig.WING_MODEL + '.tether',
    'winch_sys': 'base_station.winch',
    'wing': mconfig.WING_MODEL + '.wing',
    'wing_serial': 'common.wing_serial',
})
def MakeParams(params):
  """Returns hover controller parameters."""
  with dict_util.MustConsumeAllDictEntries(
      hover_controllers.GetHoverControllers(
          params['wing_serial'])) as controllers:
    return MakeParamsHelper(params, controllers)


def MakeParamsHelper(params, controllers):
  """Helper to return hover controller parameters."""

  gs02_drum_radius = params['ground_station']['gs02']['drum_radius']
  using_gs02 = (params['gs_model'] == m.kGroundStationModelGSv2)

  if using_gs02:
    perched_wing_pos_p = params['ground_station']['gs02']['perched_wing_pos_p']
  else:
    perched_wing_pos_p = params['perch']['perched_wing_pos_p']

  tether_mass = params['tether']['length'] * params['tether']['linear_density']

  gps_error_tolerance = 1.0

  # Set offset in ground coordinates [m] to ascend off perch.
  ascend_offset_g_z = -0.7 - gps_error_tolerance

  # This is obtained from flight data (HH-01/02 and CW-01/02).
  target_above_perch_tether_elevation = np.deg2rad(-0.0)
  if using_gs02:
    perched_wing_pos_anchor = copy.copy(perched_wing_pos_p)
    perched_wing_pos_anchor[2] -= gs02_drum_radius
    perched_tether_length = np.linalg.norm(perched_wing_pos_anchor)
  else:
    perched_tether_length = np.linalg.norm(perched_wing_pos_p)

  launch_perch_elevation_max = max(
      np.deg2rad(8.0),
      target_above_perch_tether_elevation + gps_error_tolerance /
      perched_tether_length)
  launch_perch_elevation_min = min(
      np.deg2rad(0.0),
      target_above_perch_tether_elevation - gps_error_tolerance /
      perched_tether_length)

  # Confirm that the ascend offset is negative because it is in ground
  # coordinates.
  assert ascend_offset_g_z < 0.0

  altitude = {
      # Maximum thrust-to-weight ratio [#] that the pilot can command
      # during pilot hover.  The weight is defined as the wing weight
      # plus the payed tether weight.
      'max_pilot_thrust_to_weight_ratio': 1.6,

      # Proportional, integral, and derivative loop gains ([N/m],
      # [N/m-s], and [N/(m/s)] respectively) and the integrated
      # thrust saturation limits [N].
      #
      # The integrator saturations are set to 50% of the maximum
      # nominal thrust.
      'low_altitude_pid': {
          'kp': controllers['low_altitude']['kp'],
          'ki': controllers['low_altitude']['ki'],
          'kd': controllers['low_altitude']['kd'],
          'int_output_min': -((params['wing']['m'] + tether_mass)
                              * params['phys']['g'] * 0.5),
          'int_output_max': ((params['wing']['m'] + tether_mass)
                             * params['phys']['g'] * 0.5)
      },

      'high_altitude_pid': {
          'kp': controllers['high_altitude']['kp'],
          'ki': controllers['high_altitude']['ki'],
          'kd': controllers['high_altitude']['kd'],
          'int_output_min': -((params['wing']['m'] + tether_mass)
                              * params['phys']['g'] * 0.5),
          'int_output_max': ((params['wing']['m'] + tether_mass)
                             * params['phys']['g'] * 0.5)
      },

      # Crossover frequency [Hz] for the boost integrator.  During
      # HoverAscend we expand the PID controller to include a (1/s^2)
      # double-integrator term such that the controller transfer
      # function is
      #
      #   [(boost_fc * 2 pi)/s + 1] * (ki/s + kp + kd*s)
      #
      # where boost_fc is the crossover frequency of the contribution
      # of the boost term compared to the existing PID.
      'boost_fc': 0.1,

      # Minimum and maximum thrust [N] contributions from the boost
      # stage.
      'boost_output_min': -5e3,
      'boost_output_max': 5e3,

      # Whether the boost feature is enabled [bool].
      'boost_enabled': True,

      # Altitudes [m] (above the ground station origin) below and
      # above which the low altitude gains and the high altitude gains
      # are used.
      'low_altitude': 50.0,
      'high_altitude': 100.0,

      # Maximum combined thrust [N] command from all the motors.  This
      # is the thrust commanded during acceleration and also the final
      # saturation to the thrust command.  It is set conservatively
      # high right now for the Rev3 propellers, assuming that the
      # constrained least squares solver will also handle the
      # saturation.
      'max_thrust': 28000.0,

      # Rate limit for applying max thrust [N/s]. This rate limit
      # prevents discontinuities in the rotor speed commands which
      # the rotor speed controllers do not like.
      'max_thrust_rate': 20000.0,

      # Flight mode times [sec] to start and end crossfading the
      # thrust feedforward term from 0.0 to 1.0 during HoverTransOut.
      'transout_thrust_fade_start': 1.0,
      'transout_thrust_fade_end': 2.0
  }

  angles = {
      # Pitch moment [N-m] to apply when we are in contact with the
      # perch.  This increases the stability of the constrained pitch
      # system, and also accounts for the fact that the pitch
      # integrator is turned off during this time.
      'perch_contact_extra_pitch_moment_min': 3057.8,
      'perch_contact_extra_pitch_moment_max': 10057.8,
      'perch_contact_total_pitch_moment_min': -7000.0,
      'perch_contact_total_pitch_moment_max': 7000.0,

      # Pitch angle errors [rad] where the perch contact pitch moment fades
      # from full moment (angle_min) to zero (angle_max).
      'perch_contact_extra_pitch_moment_fade_angle_min': -0.15,
      'perch_contact_extra_pitch_moment_fade_angle_max': -0.05,

      # Ratio [#] of the cross-coupling gain between roll rate and yaw
      # moment over the normal gain between yaw rate and yaw moment.
      # This uses a yaw moment to damp roll oscillations through the
      # bridle coupling.  LQR typically chooses values between -0.1
      # and -0.3 for this term, depending on wind speed and tether
      # weight.
      #
      # TODO(b/25647658): On the 2015-11-10 flight test, there was
      # significant coupling of roll vibrations to the yaw command, so
      # we have zeroed this term.  We should revisit this issue.
      'bridle_roll_damping_factor': 0.0,

      # Proportional, integral, and derivative loop gains ([N-m/rad],
      # [N-m/rad-s], and [N-m/(rad/s)] respectively) and the
      # integrated moment saturation limits [N-m] for the roll, pitch
      # and yaw angle control loops.
      'roll_pid': {
          'kp': controllers['roll']['kp'],
          'ki': controllers['roll']['ki'],
          'kd': controllers['roll']['kd'],
          'int_output_min': -500.0,
          'int_output_max': 500.0
      },

      # Gain on roll rate to blown flaps moment request [N-m/(rad/s)]
      # to the inner ailerons. See delta_blown_aileron_per_roll_moment.
      'blown_flaps_roll_rate_gain': 0.0,

      # The integrated error limits are set to 5000 N-m, which roughly
      # corresponds to the center-of-mass being off by 30 cm.
      'low_thrust_pitch_pid': {
          'kp': controllers['low_thrust_pitch']['kp'],
          'ki': controllers['low_thrust_pitch']['ki'],
          'kd': controllers['low_thrust_pitch']['kd'],
          'int_output_min': -5000.0,
          'int_output_max': 5000.0
      },

      'pitch_pid': {
          'kp': controllers['pitch']['kp'],
          'ki': controllers['pitch']['ki'],
          'kd': controllers['pitch']['kd'],
          'int_output_min': -5000.0,
          'int_output_max': 5000.0
      },

      # The integrated error limits correspond to 5000 N-m, which
      # roughly corresponds to the center-of-mass being off by 35 cm.
      'yaw_pid': {
          'kp': controllers['yaw']['kp'],
          'ki': controllers['yaw']['ki'],
          'kd': controllers['yaw']['kd'],
          'int_output_min': -5000.0,
          'int_output_max': 5000.0
      },

      # Minimum and maximum commanded moments [N-m].  It is necessary
      # to saturate these because a large moment command can cause a
      # limit cycle due to the rate limit on the motor speed command
      # imposed by the stacking controller (for an example see
      # 20160107-180836-motor_hitl_pilot_hover.h5).
      'min_moment': [-5e3, -15e3, -25e3],
      'max_moment': [5e3, 35e3, 25e3],

      # Minimum and maximum commanded moments [N-m] during
      # hover-accel.  The pitch moment is opened from the normal
      # limits to fight propwash over the elevator causing too quick
      # of a pitch forward.  See b/31313922.
      'min_accel_moment': [-5e3, -10e3, -25e3],
      'max_accel_moment': [5e3, 20e3, 25e3],

      # Nominal pitch moment [N-m] to request from the elevator in
      # order to unload the top propellers for the high-tail.  Note
      # that to evenly distribute the forces between the top and
      # bottom propellers, this should be even lower; however the
      # elevator is a very unreliable and unsteady source of pitch
      # moment so we keep the nominal request small.
      'nominal_elevator_pitch_moment': 0.0,

      # Proportional, integral, and derivative loop gains
      # ([N-m/N-m-s], [N-s/N-m-s^2], [N-m/N-m]) and the saturations on
      # the output [N-m] for the integrated pitch error loop.  We use
      # the elevator deflection to reduce the integrated pitch moment
      # from the motors.
      'int_pitch_pid': {
          'kp': controllers['int_pitch']['kp'],
          'ki': controllers['int_pitch']['ki'],
          'kd': controllers['int_pitch']['kd'],
          'int_output_min': -2000.0,
          'int_output_max': 2000.0
      },

      # Proportional, integral, and derivative loop gains
      # ([N-m/N-m-s], [N-s/N-m-s^2], [N-m/N-m]) and the saturations on
      # the output [N-m] for the integrated yaw error loop.  We use
      # the rudder deflection to reduce the integrated yaw moment from
      # the motors.
      'int_yaw_pid': {
          'kp': controllers['int_yaw']['kp'],
          'ki': controllers['int_yaw']['ki'],
          'kd': controllers['int_yaw']['kd'],
          'int_output_min': -2000.0,
          'int_output_max': 2000.0
      }
  }

  # To confirm system response, we inject step inputs in position and
  # angle commands during autonomous hover-in-place.
  inject = {
      'use_signal_injection': False,

      # Amplitude [m] and start and stop times [s] of a rectangle
      # function along each axis of position in hover coordinates,
      # i.e. positive x is up, positive y is toward the starboard
      # wing, and positive z is toward the ground station.
      'position_amplitude': [2.0, 0.0, 0.0],
      'position_start_time': [240.0, 0.0, 0.0],
      'position_stop_time': [270.0, 0.0, 0.0],

      # Amplitude [rad] and start and stop times [s] of a rectangle
      # function along each of the hover Euler angles: roll, pitch,
      # and yaw.
      'angles_amplitude': [0.2, 0.05, 0.05],
      'angles_start_time': [60.0, 120.0, 180.0],
      'angles_stop_time': [90.0, 150.0, 210.0],

      # Amplitude [rad] and start and stop times [s] of a triangle
      # function differential input to the center flaps.  This uses
      # the full triangle wave, so the signal will have positive and
      # negative sections.
      'blown_flaps_amplitude': 0.2,
      'blown_flaps_period': 30.0,
      'blown_flaps_start_time': 300.0,
      'blown_flaps_stop_time': 450.0,

      # Start and stop times [s], period [s], and up and down
      # positions [rad] to place the outer ailerons fethered to, and
      # normal to, the wind (respectively).
      'drag_flaps_start_time': 540.0,
      'drag_flaps_stop_time': 660.0,
      'drag_flaps_period': 30.0,
      'drag_flaps_low_drag_pos': np.deg2rad(-80.0),
      'drag_flaps_high_drag_pos': 0.0,

      # Amplitude [rad] and start and stop times [s] of a triangle
      # function input to the elevator.  This only uses a half-cycle
      # of a triangle wave, so the entire signal will have the same
      # sign as the amplitude.
      'elevator_amplitude': 0.0,
      'elevator_start_time': 0.0,
      'elevator_stop_time': 0.0,

      # Amplitude [rad] and start and stop times [s] of a triangle
      # function input to the rudder.  This uses the full triangle
      # wave, so the signal will have positive and negative sections.
      'rudder_amplitude': 0.3,
      'rudder_start_time': 480.0,
      'rudder_stop_time': 540.0
  }

  # Leave the altitude gate (in platform frame) to ascent-complete gate.
  max_z_for_pay_out = perched_wing_pos_p[2]

  mode = {
      # Angle [rad] from the perch/platform azimuth to the wind direction for
      # which the ground station is considered aligned for HoverAscend.
      'aligned_perch_azi_to_wind_angle_for_ascend': (
          np.pi / 2.0 if params['gs_model'] == m.kGroundStationModelGSv2
          else np.pi),

      # Maximum angle [rad] between the perch azimuth and the
      # downwind direction allowed before ascend.
      'max_perch_wind_misalignment_for_ascend': np.deg2rad(10.0),

      # Maximum angle [rad] to allow between the kite azimuth and the ideal
      # platform azimuth prior to descent onto GS02. This corresponds to
      # ~20 cm misalignment on the panels.
      'max_platform_misalignment_for_descend': np.deg2rad(1.4),

      # Maximum z position [m], i.e. minimum altitude, before starting
      # payout.
      'max_z_for_pay_out': max_z_for_pay_out,

      # Maximum yaw error [rad] and yaw rate [rad/s] allowed before we
      # go to payout from ascend.  This prevents dramatic position
      # movements near the perch for low bandwidth controllers.
      'max_yaw_angle_error_for_pay_out': 0.05,
      'max_yaw_rate_for_pay_out': 0.025,

      # Minimum winch position [m] at which to enter HoverFullLength.
      'min_winch_pos_for_transform_gs_up':
          np.deg2rad(-360.0) * gs02_drum_radius,

      # Maximum azimuth error [rad] required before initiating the
      # ground station transform.
      'max_azimuth_error_for_transform': np.deg2rad(1.0),

      # Maximum z position error [m] required before initiating the
      # ground station transform.
      'max_z_error_for_transform': 1.0,

      # Maximum roll angle error [rad] allowed before acceleration.
      'max_roll_angle_error_for_accel': 0.4,

      # Maximum yaw angle error [rad] allowed before acceleration.
      'max_yaw_angle_error_for_accel': 0.15,

      # Maximum yaw rate [rad/s] allowed before acceleration.
      'max_yaw_rate_for_accel': 0.1,

      # Maximum total angular rate [rad/s] allowed before acceleration.
      'max_angular_rate_for_accel': 0.2,

      # Maximum azimuth error [rad] from the acceleration start
      # location allowed before acceleration.
      'max_azimuth_error_for_accel': np.deg2rad(10.0),

      # Maximum z position [m] error allowed before acceleration.
      'max_z_error_for_accel': 4.0,

      # Maximum wing speed [m/s] allowed before acceleration.
      'max_speed_for_accel': 3.0,

      # Maximum y velocity [m/s] in body coordinates allowed
      # before acceleration.
      'max_body_y_vel_for_accel': 2.0,

      # Minimum tension [N] before ascend.  See b/113601392.
      'min_tension_for_ascend': 1500.0,

      # Minimum tension [N] allowed before acceleration.
      'min_tension_for_accel': 4000.0,

      # Minimum time [s] the controller is allowed to spend in
      # transition-out.  This is a safe guard against the pilot
      # unintentionally commanding reel-in immediately following
      # transition-out.
      'min_time_in_trans_out': 12.0,

      # Maximum tether elevation error [rad] that must be sustained,
      # and which must be attained instantaneously, for the ground station
      # transform.
      'max_tether_elevation_error_for_gs_transform_staging': np.deg2rad(1.5),
      'max_tether_elevation_error_for_gs_transform_kickoff': np.deg2rad(0.5),

      # The duration [s] to sustain the tether elevation before
      # initiating a ground station transform.
      'min_gs_transform_staging_time': 10.0,
  }

  # Propwash velocity [m/s] at the elevator x position assuming zero
  # apparent wind speed.  The positive z component accounts for the
  # vectoring of the propwash about the main wing and is determined
  # empirically.
  propwash_b = [-24.0, 0.0, 5.0]

  # Apparent wind speed [m/s] at which the middle of the propwash
  # impinges on the tail.
  center_propwash_wind_speed = -4.0

  output = {
      # Weights [#] for the relative importance of total thrust and
      # moments when saturations are applied.  In general, meeting
      # pitch and yaw requests is favored over meeting thrust
      # requests.
      'weights': {
          'thrust': 1.0,
          'moment': [3.0, 20.0, 1.0]
      },

      # Time [s] to slowly ramp the motor thrusts and moments up after
      # the throttle has been brought above the software e-stop
      # threshold (if the throttle was held low for long enough to
      # latch the software e-stop), and for ramp down after perching.
      # This is set to be conservatively long to start testing, and
      # for gentle perching.
      'gain_ramp_time': 2.5,

      'propwash_b': propwash_b,

      # Apparent wind speeds [m/s], along the negative body z-axis, at
      # which there is no interaction of the propwash with the
      # elevator, full interaction, and in the center of the full
      # interaction.
      'zero_propwash_wind_speed': center_propwash_wind_speed - 8.0,
      'full_propwash_wind_speed': center_propwash_wind_speed - 3.0,
      'center_propwash_wind_speed': center_propwash_wind_speed,

      # Additional elevator deflection [rad] applied during
      # transition-out.
      'delta_ele_trans_out': np.pi / 180.0 * -30.0,

      # Conversion [rad/N-m] between the elevator pitch moment and the
      # elevator deflection, assuming that the airspeed is the nominal
      # propwash speed.
      #
      # This assumes a dCL/dalpha of 4.8, a horizontal tail area of
      # 3.5 m^2, and a lever arm of 6.25 m.
      'delta_elevator_per_pitch_moment': (
          -1.0 / (0.5 * params['phys']['rho'] * 4.8 * 3.5 * 6.25
                  * np.linalg.norm(propwash_b)**2.0)),

      # Minimum and maximum elevator feedback deflections [rad].  This
      # assumes that the elevator feed-forward term places the
      # elevator at zero lift.  The minimum delta is intended to keep
      # the elevator from stalling.  The maximum delta has a larger
      # range because there is significant uncertainty in the angle of
      # the propwash.  We think it is more important to ensure that
      # the elevator applies some forward pitching moment, rather than
      # protect against stall, so we increase the limit on this side.
      'min_delta_elevator_fb': np.pi / 180.0 * -9.0,
      'max_delta_elevator_fb': np.pi / 180.0 * 15.0,

      # Low-pass filter cutoff frequency [Hz].  We filter the
      # feed-forward component of the elevator command to reduce servo
      # wear and to reduce a feedback path between the elevator motion
      # through the gyros back to the elevator.
      'elevator_cutoff_freq': 1.0,

      # Forward velocities [m/s] of the wing used to schedule flap
      # gains and cutoff frequencies.
      'no_aileron_rudder_speed': 4.0,
      'full_aileron_rudder_speed': 10.0,

      # Roll and yaw moment control derivatives [#/rad] for the
      # aileron and rudder.
      #
      # TODO: Calculate these from the aero database.
      'cl_da': -0.0071 * 180.0 / np.pi,
      'cn_dr': -0.0012 * 180.0 / np.pi,

      # Inner aileron deflections per roll moment [rad/N-m] assuming
      # they are in the propwash.  This number should be determined
      # empirically, and hence we do not use configuration parameters
      # here.  For now we estimate it as:
      #
      #   1 / (0.5 * rho * propwash^2 * cl_da * (lever arm ratio) * (area ratio)
      #                                       * (wing area) * (wing span))
      #
      # assuming:
      #
      #   cl_da [1/rad]       : -0.0071 * 180 / pi
      #   propwash [m/s]      : 24
      #   lever arm ratio [#] : 0.5 / 3.5
      #   area ratio [#]      : 1 / 2
      #
      # TODO: Estimate from flight data.
      'delta_blown_aileron_per_roll_moment': -1.18e-4,

      # Forward speeds [m/s] at which to fade the blown flaps roll
      # actuation to zero. The intent is to only use blown flaps when
      # the airflow over the flaps is dominated by propwash, not
      # freestream, such as may occur during trans-out when the kite
      # has significant forward velocity.
      'zero_blown_flaps_forward_speed': 5.0,
      'full_blown_flaps_forward_speed': 2.5,

      # Flap offsets and lower and upper limits [rad] in the standard
      # order: port ailerons A1 and A2, center flaps A4 and A5,
      # starboard ailerons A7 and A8, elevator, rudder.  The aileron
      # offsets are slightly up so there is still some room left for
      # control during acceleration as the 0 degree flap position
      # corresponds to maximum lift.
      'flap_offsets': [
          np.pi / 180.0 * angle
          for angle in [-11.5, -11.5, -11.5, -11.5, -11.5, -11.5, 0.0, 0.0]
      ],
      'lower_flap_limits': [
          np.pi / 180.0 * angle
          for angle in [-80.0, -80.0, -35.0, -35.0, -80.0, -80.0, -89.0, -22.0]
      ],
      'upper_flap_limits': [
          np.pi / 180.0 * angle
          for angle in [0.0, 0.0, 15.0, 15.0, 0.0, 0.0, 15.0, 22.0]
      ],

      # Deadzone values for the gs02 azimuth [rad] while perched and
      # during all other flight modes.
      'gs02_deadzone_while_perched': 0.0,
      'gs02_deadzone_during_flight': np.deg2rad(0.5),

      # Flap angle [rad] when ailerons are spoiled.
      'spoiled_aileron_angle': -np.deg2rad(75.0),
  }

  # Expected heeling angle [rad] of the vessel away from the kite.
  # This angle describes how much the vessel is tilted towards the kite,
  # away from being upright.
  # It is measured as a rotational angle over the axis defined by
  # z_hat_g X kite_hat_g.
  vessel_heel_ff = np.deg2rad(-0.5) if params['system']['offshore'] else 0.0

  # The target tether elevation angle [rad] in ground frame.
  tether_elevation_target_g = np.deg2rad(6.0)
  tether_elevation_target_p = tether_elevation_target_g - vessel_heel_ff

  path = {
      # Maximum accelerations [m/s^2] allowed in the path.
      'max_acceleration_g': [1.0, 1.0, 1.0],

      # Perched position of the wing [m] in the perch frame (GSv1) or the
      # platform frame (GSv2).
      'perched_wing_pos_p': perched_wing_pos_p,

      # Maximum speeds [m/s] that the path will ever move by in the
      # radial, tangential, and z directions in autonomous hover
      # during various phases of flight.
      'max_normal_radial_speed': 5.0,

      # During the RPX and CW programs, robust operation with tangential speeds
      # of 5.0 m/s was demonstrated.  Following the loss-of-vehicle in FCW-01
      # this was reduced to 2.0 m/s due to concerns about the possible coupling
      # of large hover sideslips to roll moments.
      #
      # TODO(b/143181116): Revisit once roll moments in hover are better
      # understood.
      'max_normal_tangential_speed': 2.0,
      'max_ascend_perch_z_speed': 0.45,
      'max_ascend_near_perch_z_speed': 0.5,
      'max_ascend_normal_z_speed': 2.0,
      'max_descend_perch_z_speed': 0.3,
      'max_descend_near_perch_z_speed': 0.5 if using_gs02 else 1.0,
      'max_descend_normal_z_speed': 2.0,
      'max_accel_z_speed': 45.0,

      'gps_error_tolerance': gps_error_tolerance,

      # Ascend / Descend.

      # Vertical offset [m] in ground coordinates from the perched
      # position that the wing ascends to before paying out.
      'ascend_offset_g_z': ascend_offset_g_z,

      # Vertical offset [m] in ground coordinates from the perched
      # position that the wing attempts to descend to after reel-in.
      # (This should be positive to be below the perched position).
      # This is conservatively large in case there is GPS drift.
      'descend_offset_g_z': 10.0,

      # Cutoff frequency [Hz] and damping ratio [#] for the
      # low-pass-filter that smooths the velocity command in
      # SmoothRawPositionCommand.
      'velocity_cutoff_freq': 10.0,
      'velocity_damping_ratio': 1.0 / np.sqrt(2.0),

      # Pay-out / Reel-in.

      # The angle [rad] at the levelwind that we attempt to achieve
      # with the tether based on the position of the wing and a
      # horizontal tension in the tether (negative means down).

      # The target tether elevation [rad] for reel-in and payout.
      'target_reel_tether_elevation': tether_elevation_target_p,

      # The allowed range of tether elevation in GS02v2 during transform
      # modes is 0 - 17 degrees (relative to the GS). The GS is expected to tilt
      # 1 degree from hover tension in a steady hover state. So the center of
      # the tether elevation window, relative to ground, is 7.5 degrees.
      # We further reduce it by 1 degree. See b/131630581 for more details.
      'target_transform_tether_elevation': tether_elevation_target_p,

      # Target tether elevation above the perch at the end of ascend
      # or at the beginning of descend.
      'target_above_perch_tether_elevation':
          target_above_perch_tether_elevation,

      # Expected heeling angle [rad] of the vessel away from the kite.
      'vessel_heel_ff': vessel_heel_ff,

      # Proportional gains [rad/rad], integral gains [rad/rad-s],
      # derivative gains [rad/(rad/s)], and integrated angle saturations
      # [rad] for the PID controlling tether elevation in PayOut and
      # ReelIn.  Note that a separate controller is used in the
      # PrepGsTransform modes.
      'reel_tether_elevation_pid': {
          'kp': controllers['reel_tether_elevation']['kp'],
          'ki': controllers['reel_tether_elevation']['ki'],
          'kd': controllers['reel_tether_elevation']['kd'],
          'int_output_min': -0.4,
          'int_output_max': 0.4
      },

      # Maximum payout [m] within which to prepare for acending/descending.
      'max_payout_for_perching_prep': 7.0,

      # Cutoff frequency [Hz] and damping ratio [#] for filter applied
      # to tether elevation error. This is part of the total open loop
      # gain of the tether elevation controller and supplies
      # additional roll-off to avoid exciting tether modes above the
      # controller's unity gain frequency.
      'tether_elevation_error_fc': 0.05,
      'tether_elevation_error_zeta': 1.0,

      # Proportional, integral, and derivative loop gains ([m/rad],
      # [(m/rad)/s], and [m/(rad/s)] respectively) and the integrated
      # tether elevation limits [m] for the PrepGsTransform flight modes.
      'transform_tether_elevation_pid': {
          'kp': controllers['transform_tether_elevation']['kp'],
          'ki': controllers['transform_tether_elevation']['ki'],
          'kd': controllers['transform_tether_elevation']['kd'],
          # TODO(b/116036191): Add protection from unreasonable altitude
          # offsets.
          'int_output_min': -100.0,
          'int_output_max': 100.0,
      },

      # Azimuth offset [rad] from downwind at which the transform should be
      # completed.
      'transform_azimuth_offset': 0.0,

      # The elevation angle [rad] limits of the launch/perch trajectory when
      # the wing is near the perch.
      # It is also necessary to make sure the window is compatible to
      # near perch tether elevation control and the GPS error tolerance.
      'launch_perch_elevation_max': launch_perch_elevation_max,
      'launch_perch_elevation_min': launch_perch_elevation_min,

      # Tether length thresholds [m] for when to use .
      'reel_short_tether_length': 0.1 * params['tether']['length'],
      'reel_long_tether_length': 0.8 * params['tether']['length'],

      # Azimuth offset [rad] used in pay-out and reel-in at short
      # tether lengths, formerly used to bias perching to one side.
      # This is cross-faded to transform_azimuth_offset at long tether lengths.
      'reel_azimuth_offset': 0.0,

      # Minimum and maximum elevations [rad] commanded by the hover
      # controller at long tether lengths.  These limits are
      # cross-faded with the launch_perch_elevation at short tether
      # lengths.
      'reel_elevation_min': 0.0,
      'reel_elevation_max': 0.5,

      # Full length.

      # Elevation above the horizion (positive is up) where the wing begins its
      # acceleration into crosswind.
      'accel_start_elevation': 0.3,

      # Reel-in before engage.

      # Duration [s] of the crossfade from the estimated inertial velocity to
      # the nominal velocity command in trans-out.
      # TODO: This can probably be tightened up.
      'transout_vg_cmd_crossfade_duration': 6.0,

      # Multiplier to velocity command during TransOut to
      # reduce the jump in thrust and power at the transition to TransOut
      # The lower this gain, the smaller the jump in thrust, the higher
      # the kite deceleration and the lower the final hover altitude. Values
      # too high would violate the min thrust threshold (T > 0 N) and
      # create tether motion.
      'transout_vel_cmd_multiplier': 0.8,

      # Minimum altitude (negative Xg.z position) commanded in HoverTransOut.
      'transout_min_altitude': 150.0,

      # Maximum altitude for safe horizontal translation.
      'max_altitude_error_for_translation': 5.0
  }

  position = {
      # Feed-forward Euler angles [rad] in XYZ order that result in no
      # position movement.  The roll angle is due to the asymmetric
      # bridling.  This is positive when the nominal tether direction
      # has a negative y component in the body axes.
      #
      # The pitch angle accounts for the rotors being tilted forward by 3
      # degrees, in addition to a substantial "blown lift" effect experienced
      # by the propwash in interaction with the main wing.
      #
      # There are several effects that determine the yaw set-point:
      #  - Because the wing is rolled from the roll bridling and pitched
      #    back, it is necessary to add a positive yaw to point the
      #    thrust vector up.
      #  - Because the pylons are cambered and develop lift in the -y
      #    direction, it is necessary to cancel this lift by yawing in the
      #    positive direction.
      #  - If the propellers are all rotating in the same direction (not
      #    currently the case), then it is necessary to cancel the reaction
      #    moment of the propellers with a yaw offset.
      #
      # The yaw set-point was determined empirically; see b/72126270.
      'eulers_ff': [np.arctan2(params['wing']['bridle_y_offset'],
                               params['wing']['bridle_rad'] +
                               params['wing']['bridle_pos'][0][2]),
                    np.arctan2(params['rotors'][0]['axis'][2],
                               params['rotors'][0]['axis'][0]) - 0.30,
                    np.deg2rad(4.0)],

      # Tether lengths [m] that we use to schedule the radial and
      # tangential controller gains.
      'short_tether': 100.0,
      'long_tether': 200.0,

      # Altitudes [m] above ground-station origin (positive is up)
      # that are used to schedule the long tether tangential gains.
      # The low altitude is chosen to approximately correspond to the
      # normal altitude at the short tether length.  The high value is
      # chosen to be approximately when the tether no longer touches
      # the ground.
      'low_altitude': 15.0,
      'high_altitude': 30.0,

      # Proportional gains [rad/m], integral gains [rad/m-s],
      # derivative gains [rad/(m/s)], and integrated angle saturations
      # [rad] for the radial and tangential position PID controllers.
      'short_tether_radial_pid': {
          'kp': 0.0,
          'ki': 0.0,
          'kd': 0.0,
          'int_output_min': 0.0,
          'int_output_max': 0.0
      },

      'long_tether_radial_pid': {
          'kp': 0.0,
          'ki': 0.0,
          'kd': controllers['radial']['kd'],
          'int_output_min': 0.0,
          'int_output_max': 0.0
      },

      # The integrator saturations are chosen so the yaw command from
      # the integrator will be less than 6 degrees.
      'short_tether_tangential_pid': {
          'kp': controllers['tangential_short_tether']['kp'],
          'ki': controllers['tangential_short_tether']['ki'],
          'kd': controllers['tangential_short_tether']['kd'],
          'int_output_min': -0.3,
          'int_output_max': 0.3
      },

      'low_altitude_long_tether_tangential_pid': {
          'kp': controllers['tangential_low_altitude_long_tether']['kp'],
          'ki': controllers['tangential_low_altitude_long_tether']['ki'],
          'kd': controllers['tangential_low_altitude_long_tether']['kd'],
          'int_output_min': -0.3,
          'int_output_max': 0.3
      },

      'high_altitude_long_tether_tangential_pid': {
          'kp': controllers['tangential_high_altitude_long_tether']['kp'],
          'ki': controllers['tangential_high_altitude_long_tether']['ki'],
          'kd': controllers['tangential_high_altitude_long_tether']['kd'],
          'int_output_min': -0.3,
          'int_output_max': 0.3
      },

      # Maximum angles [rad] that the proportional and derivative
      # feedback is allowed to command.
      'max_pos_angle_fb': 0.1,
      'max_vel_angle_fb': 0.1,

      # Gains on joysticks [rad/#] in pilot hover mode.
      'k_pilot': [0.8, 0.3, 0.3],

      # Starting and ending times [s] for crossfading between the current
      # attitude and the nominal attitude command during the first few seconds
      # of trans-out from crosswind. Yaw is especially prone to saturating
      # motors and can rob a substantial amount of thrust which is why it is
      # crossfaded more slowly.
      'transout_angles_cmd_crossfade_start_times': [0.0, 0.0, 0.0],
      'transout_angles_cmd_crossfade_end_times': [1.0, 1.0, 1.0],

      # Constant hover pitch command [rad] for the kite during HoverTransOut
      'transout_low_wind_pitch_cmd': np.deg2rad(4.0),
      'transout_high_wind_pitch_cmd': np.deg2rad(-5.0),

      # Wind speeds [m/s] across which to crossfade the transout pitch commands.
      'transout_pitch_low_wind_speed': 3.0,
      'transout_pitch_high_wind_speed': 11.0,

      # Ending time [s] for crossfading between the current body angular rates
      # and the nominal body angular rates.
      # TODO: Examine making this consistent with the angle
      # command crossfade.
      'transout_pqr_cmd_crossfade_duration': [1.0, 1.0, 1.0],

      # Minimum and maximum command angles [rad] that are passed to
      # the hover angle controller.
      'min_angles': [-0.4, -0.3, -0.4],
      'max_angles': [0.4, 0.332, 0.4],

      # Time [sec] to crossfade from the constant pitch angle command in
      # HoverTransOut to regular tension regulation in HoverPrepTransformGsDown
      'transformdown_pitch_cmd_crossfade_time': 3.0,
  }

  tension = {
      # Minimum allowed tension [N] on the tether during hover.  The
      # M600 specification requires a minimum of 2.5 kN, so we stay a
      # safe margin above this.
      'tension_min_set_point': 8000.0,

      # Proportional gains [rad/N] and integral gains [rad/N-s], and
      # integrated pitch saturation [rad] for the tension loop.  Note
      # that the derivative term is effectively handled by the radial
      # position loop.  See generator_hover_controllers.m.
      # TODO:  Decrease int_output_min to allow high hover in
      #                      higher winds.
      'tension_hard_pid': {
          'kp': controllers['tension_hard']['kp'],
          'ki': controllers['tension_hard']['ki'],
          'kd': controllers['tension_hard']['kd'],
          'int_output_min': np.deg2rad(-7.0),
          'int_output_max': np.deg2rad(14.0)
      },
      'tension_soft_pid': {
          'kp': controllers['tension_soft']['kp'],
          'ki': controllers['tension_soft']['ki'],
          'kd': controllers['tension_soft']['kd'],
          'int_output_min': np.deg2rad(-7.0),
          'int_output_max': np.deg2rad(14.0)
      },

      # The hard and soft tension controllers are calculated based on
      # spring constants.  However, we schedule these controllers as a
      # function of payout.  The equation for the catenary spring
      # constant as a function of payout is:
      #
      #                mu g               cosh(r / 2a)
      #   k_catenary = ---- * --------------------------------------
      #                 2      (r / 2a) cosh(r / 2a) - sinh(r / 2a)
      #
      #   a = t0 / (mu * g)
      #
      #   mu: Linear mass density [kg/m].
      #   g: Acceleration from gravity [m/s^2].
      #   r: Horizontal distance of tether [m].
      #   t0: Horizontal tension [N/m].
      #
      # From this equation, the 10 kN/m and 1 kN/m spring constants,
      # using a horizontal tension of about 3400 N, occur around 80 m
      # and 170 m of payout respectively.  To be conservative, we use
      # significantly lower values of payout for now.
      #
      # Payouts [m] below and above which we use the hard-spring
      # tensions gains and the soft-spring tension gains.  Between
      # these values linearly crossfade the controllers.
      'hard_spring_payout': 10.0,
      'soft_spring_payout': 50.0,

      # Rate limit [rad/s] on pitch command adjustments applied based
      # on current flight mode.  This avoids sudden changes in pitch
      # request which can lead to large pitching moments and a
      # potential reduction in overall thrust.
      'additional_pitch_cmd_rate_limit': 0.1,

      # Use payout [m] to schedule minimum and maximum pitch angles
      # [rad].  Near the perch, we hold pitch constant to preserve
      # perching geometry and avoid dramatic movements while perching.
      #
      # TODO: Move this pitch table to the attitude or position
      # controllers, which control absolute pitch; the tension controller
      # references pitch as an offset from the neutral attitude eulers_ff.
      'payout_table': [0.0, 1.5, 5.0, 77.0],
      'min_pitch_table': (np.deg2rad([16.0, 16.0, 3.0, -20.0]) + 0.30).tolist(),
      'max_pitch_table': (np.deg2rad([16.0, 16.0, 11.5, 17.0]) + 0.30).tolist(),

      # The coefficient of drag [#] during hover.  From the way this
      # is used in the code, it should be the drag coefficient of the
      # wing at the nominal Euler angles, but referenced to the full
      # wing area.
      'hover_drag_coeff': 2.2,

      # Maximum change in tension [N] that may be commanded by the
      # pilot in autonomous modes using the pitch stick.
      'max_pilot_extra_tension': 10000.0,

      # The rate limit for the horizontal tension command [N/s].
      # TODO(b/116036824): Check that this rate limit doesn't interfere with
      # the pilot's ability to manually command extra tension.
      'horizontal_tension_cmd_rate_limit': 250.0,

      # Threshold value [#] that the joystick roll must exceed (positive or
      # negative) to increment the horizontal tension command.
      'horizontal_tension_joystick_roll_threshold': 0.8,

      # Number of cycles [#] that the joystick roll must exceed its threshold
      # value to increment the horizontal tension command.
      'horizontal_tension_num_cycles_for_increment': 10,

      # Horizontal tension increment [N] that may be applied by the pilot using
      # the roll stick.
      'horizontal_tension_pilot_increment': 500.0,

      # Filter parameters for the pilot horizontal tension offset.
      'horizontal_tension_pilot_offset_fc': 0.1,  # [Hz]
      'horizontal_tension_pilot_offset_zeta': 0.7,  # [#]

      # Maximum horizontal tension offset [N] that may be applied by the pilot.
      'horizontal_tension_max_pilot_offset': 10e3,
  }

  # Zone [m] where wing-perch contact is possible.
  contact_zone = 2.2

  # Final winch speeds [m/s].
  final_winch_speed = 0.1

  if params['flight_plan'] == m.kFlightPlanDisengageEngage:
    # For the first GS02 transform HITL, we will reel out slowly from slightly
    # below the transform angle. After transforming up to high-tension, then
    # back down to reel, we will reel in 1.5 m and stop.
    upper_limit = -1.90 * np.pi * gs02_drum_radius
    lower_limit = upper_limit - 1.5

    winch_position_pay_out_table = np.linspace(
        lower_limit, upper_limit, 6).tolist()
    winch_speed_pay_out_table = [
        final_winch_speed,
        final_winch_speed,
        final_winch_speed,
        final_winch_speed,
        final_winch_speed,
        final_winch_speed,
    ]

    winch_position_reel_in_table = np.linspace(
        lower_limit, upper_limit, 5).tolist()
    winch_speed_reel_in_table = [
        0.0,
        final_winch_speed,
        final_winch_speed,
        final_winch_speed,
        final_winch_speed
    ]
  else:
    # Low, high, and high pitch groove winch speeds [m/s].  The high
    # winch speed is decreased for the launch-perch flight plan
    # because this flight plan is typically used when we are under
    # constraints and have a limited range of movement. The high pitch
    # groove winch speed is set to ensure the levelwind shuttle can keep
    # up with the position of the tether on the drum in the last 1/2
    # turn before starting the transform when paying out.
    low_winch_speed = 0.5
    high_pitch_groove_winch_speed = 0.15
    if params['flight_plan'] in (m.kFlightPlanLaunchPerch,
                                 m.kFlightPlanHoverInPlace):
      high_winch_speed = low_winch_speed
    else:
      high_winch_speed = 2.0

    # Reel-in is slowed down in the "perch approach zone."  This zone
    # [m] prevents a slightly shortened tether from causing excessive
    # perch approach speed (a "shortened tether" could be caused by low
    # tension during reel-in).
    perch_approach_zone = 0.04 * params['tether']['length']

    # Distance [m] over which we distribute acceleration and
    # deceleration.
    acc_dec_zone = 8.0 * high_winch_speed

    # Transform slow zone [m].
    transform_zone = 1.5 * 2.0 * np.pi * params['winch_sys']['r_drum']

    # The predicted winch position [m] when the wing is perched includes
    # the tether length and a 180 degree tether wrap around the drum
    # during the perch transformation.  We also add a little extra
    # margin (1.0 m) for pay-out and reel-in.
    winch_position_perched = (-params['tether']['length']
                              - np.pi * params['winch_sys']['r_drum'] + 1.0)

    # Table of winch positions [m] that are used to schedule pay-out
    # velocities.
    winch_position_pay_out_table = [
        winch_position_perched,
        winch_position_perched + contact_zone + acc_dec_zone,
        -acc_dec_zone - transform_zone,
        -transform_zone,
        -0.1,
        0.0
    ]
    # Table of winch speeds [m/s] (sign does not matter) at
    # different points during pay-out.
    winch_speed_pay_out_table = [
        high_winch_speed,
        high_winch_speed,
        high_winch_speed,
        high_pitch_groove_winch_speed,
        high_pitch_groove_winch_speed,
        high_pitch_groove_winch_speed
    ]
    # Table of winch positions [m] that are used to schedule reel-in
    # velocities.
    winch_position_reel_in_table = [
        winch_position_perched + perch_approach_zone,
        winch_position_perched + perch_approach_zone + acc_dec_zone,
        -acc_dec_zone - transform_zone,
        -transform_zone,
        0.0
    ]

    # Table of winch speeds [m/s] (sign does not matter) at
    # different points during reel-in.
    winch_speed_reel_in_table = [
        low_winch_speed,
        high_winch_speed,
        high_winch_speed,
        high_pitch_groove_winch_speed,
        high_pitch_groove_winch_speed,
    ]

  gs02 = params['ground_station']['gs02']
  winch = {
      'winch_position_pay_out_table': winch_position_pay_out_table,
      'winch_speed_pay_out_table': winch_speed_pay_out_table,
      'winch_position_reel_in_table': winch_position_reel_in_table,
      'winch_speed_reel_in_table': winch_speed_reel_in_table,

      # Payout [m] below which we could contact the perch.  The winch
      # slows to its contact speed here.
      'contact_payout': contact_zone,

      # Final speed [m/s] when the winch is contacting the perch.
      'contact_winch_speed': final_winch_speed,

      # Maximum allowable winch speed [m/s] command.  This may be
      # higher than the winch speeds in the payout and reel-in tables
      # because the pilot may choose to increase the winch speed.
      'max_winch_speed': 3.0,

      # Maximum linear acceleration [m/s^2] of the winch drum.
      'max_winch_accel': gs02['max_drum_accel_in_reel'] * gs02['drum_radius'],

      # Tension [N] at which the winch stops reel-in (presumably
      # because it is trying to reel the wing in while it is perched).
      # This is set to the maximum hover tension from the
      # specification.
      'max_tension': 55e3,
  }

  assert mconfig.IsStrictlyIncreasing(winch['winch_position_pay_out_table'])
  assert mconfig.IsStrictlyIncreasing(winch['winch_position_reel_in_table'])

  experiments = hover_experiments.GetExperiments()

  return {
      'altitude': altitude,
      'angles': angles,
      'inject': inject,
      'mode': mode,
      'output': output,
      'path': path,
      'position': position,
      'tension': tension,
      'winch': winch,
      'experiments': experiments
  }

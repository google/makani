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

"""Rotor control parameters."""

from makani.analysis.control import simple_aero
from makani.config import mconfig
from makani.control import system_types
import numpy as np


def _CheckDoubleReverseRainbow(rotors):
  for r in range(system_types.kNumMotors):
    if r in [system_types.kMotorPto, system_types.kMotorSto,
             system_types.kMotorPbi, system_types.kMotorSbi]:
      if rotors[r]['dir'] != system_types.kPositiveX:
        return False
    else:
      if rotors[r]['dir'] != system_types.kNegativeX:
        return False
  return True


@mconfig.Config(deps={
    'control_opt': 'common.control.control_options',
    'flight_plan': 'common.flight_plan',
    'motors': 'powertrain.motors',
    'phys': 'common.physical_constants',
    'power_sys': 'powertrain.power_sys',
    'propellers': 'prop.propellers',
    'rotors': mconfig.WING_MODEL + '.rotors',
    'system': mconfig.WING_MODEL + '.system_params',
    'wing': mconfig.WING_MODEL + '.wing',
})
def MakeParams(params):
  """Make rotor control params."""
  # The current state of the motor solver parameters requires a double-reverse
  # rainbow configuration. See go/makanicl/26340 for an example of supporting a
  # "top spins negative, bottom spins positive" configuration.
  _CheckDoubleReverseRainbow(params['rotors'])

  # The simplified rotor model has difficulty fitting the rotor
  # databases as the Mach number approaches the critical Mach number
  # of the rotors.  Here we set a tip speed limit for the fit that
  # is intentionally fast.
  tip_speed_limit = 340.0 * 0.65

  # The advance ratio stall margin [#] reduces the maximum allowed
  # advance ratio from the stall advance ratio.
  #
  # TODO: Revisit these values.  This was set to 0.3 for
  # the Rev1 and Rev2 propellers because the simulated stacked power
  # system diverged when the propeller stalled.  (It is not clear if
  # this is a physical effect).  This was set to 0.25 for the Rev3
  # propellers because otherwise the advance ratio and power
  # constraints conflicted.
  advance_ratio_stall_margins = [None for _
                                 in range(system_types.kNumPropVersions)]
  advance_ratio_stall_margins[system_types.kPropVersionRev1] = 0.3
  advance_ratio_stall_margins[system_types.kPropVersionRev1Trimmed] = 0.3
  advance_ratio_stall_margins[system_types.kPropVersionRev2] = 0.3
  advance_ratio_stall_margins[system_types.kPropVersionRev3NegativeX] = 0.1
  advance_ratio_stall_margins[system_types.kPropVersionRev3PositiveX] = 0.1
  advance_ratio_stall_margins[system_types.kPropVersionRev4NegativeX] = 0.1
  advance_ratio_stall_margins[system_types.kPropVersionRev4PositiveX] = 0.1

  nominal_motor_voltage = (params['power_sys']['v_source_0'] /
                           (system_types.kNumMotors / 2))
  rotor_models = [
      simple_aero.GetRotorModel(  # pylint: disable=g-complex-comprehension
          params['phys']['rho'],
          params['propellers'][i]['database']['name'],
          params['motors'], nominal_motor_voltage, tip_speed_limit,
          advance_ratio_stall_margin=advance_ratio_stall_margins[i])
      for i in range(system_types.kNumPropVersions)
  ]

  simple_models = [
      rotor_model.CalcSimpleRotorModel() for rotor_model in rotor_models
  ]

  for rotor_model, simple_model in zip(rotor_models, simple_models):
    assert rotor_model.CheckSimpleRotorModel(simple_model, 150.0, 1e-1)

  # Converts a vector of individual rotor thrusts into the
  # thrust-moment vector:
  #
  #   [F, tx, ty, tz] = thrust_to_thrust_moment * [T1, T2, T3, T4, ...]'
  #
  # The thrust-moment vector is calculated in a frame centered at the
  # center-of-mass and rotated so its thrust vector is along the axes
  # of the propellers.
  mass_pos = np.array(params['wing']['center_of_mass_pos'])
  unit_moments = [None] * len(params['rotors'])
  for i in range(len(params['rotors'])):
    rotor_pos = np.array(params['rotors'][i]['pos'])
    rotor_axis = params['rotors'][i]['axis']
    unit_moments[i] = np.cross(rotor_pos - mass_pos, rotor_axis)
    # The above calculation is performed in body coordinates, but the
    # thrust-moment vector should be expressed in the frame aligned
    # with the propeller axes.  So, here we rotate the z-moment to the
    # axis of the propeller.  We are assuming that the propeller axes
    # are collinear and that they are not angled about the body
    # z-axis.
    unit_moments[i][2] /= params['rotors'][i]['axis'][0]

  thrusts_to_thrust_moment = np.matrix(
      [[1.0 for _ in params['rotors']],
       [rotor_models[r['version']].CalcStaticTorquePerThrust()
        * float(r['dir']) for r in params['rotors']],
       [unit_moment[1] for unit_moment in unit_moments],
       [unit_moment[2] for unit_moment in unit_moments]])

  thrust_moment_to_thrusts = np.linalg.pinv(thrusts_to_thrust_moment)

  # Convert a vector of common and differential thrusts to individual
  # thrusts:
  #
  #   [T1, T2, ...] = comm_and_diff_thrusts_to_thrusts * [T_c, T_d1, ... ].
  #
  comm_and_diff_thrusts_to_thrusts = np.matrix(
      [[1.0, 1.0, 0.0, 0.0, 0.0],
       [1.0, 0.0, 1.0, 0.0, 0.0],
       [1.0, 0.0, 0.0, 1.0, 0.0],
       [1.0, 0.0, 0.0, 0.0, 1.0],
       [1.0, -1.0, 0.0, 0.0, 0.0],
       [1.0, 0.0, -1.0, 0.0, 0.0],
       [1.0, 0.0, 0.0, -1.0, 0.0],
       [1.0, 0.0, 0.0, 0.0, -1.0]])

  # Maximum rotor speeds [rad/s] set by tip Mach number
  # considerations.
  #
  # TODO: Speed of sound varies with temperature.  We may
  # want to set a maximum rotor Mach number, not absolute speed.
  speed_of_sound = 340.0
  max_mach_number = 0.75
  max_speeds = [
      max_mach_number * speed_of_sound / (
          simple_models[rotor['version']]['D'] / 2.0)
      for rotor in params['rotors']
  ]

  # Convert a vector of common and differential thrusts into the
  # force/moment vector:
  #
  #   [F, tx, ty, tz] = common_and_diff_thrusts_to_thrust_moment
  #                     * [T_comm, T_diff1, ... ].
  #
  # For the four stacking fault situations, we create a reduced common
  # and differential thrust to thrust-moment matrix with the column
  # associated with the differential thrust from the faulted motor
  # block set to zeros.
  comm_and_diff_thrusts_to_thrust_moment = [
      (thrusts_to_thrust_moment * comm_and_diff_thrusts_to_thrusts).tolist()
  ]
  # TODO: Iterate using kNumStackingStates instead of integers.
  for i in range(4):
    comm_and_diff_thrusts_to_thrust_moment.append(
        (thrusts_to_thrust_moment
         * np.diag([0.0 if (c == i or c == i + 4) else 1.0 for c in range(8)])
         * comm_and_diff_thrusts_to_thrusts).tolist())

  # Constraint matrix for the constrained least squares solver.  The
  # first eight constraints are individual rotor thrust constraints
  # that may be used to limit torque in hover or rotor speed in
  # crosswind.  The final constraint is used as a power constraint in
  # hover and transition-in.  The entries of this last constraint row
  # and the lower and upper bounds for all constraints are calculated
  # within the control loop.
  # Include constraint matrices for each of the five stacking states.
  constraint_matrix = [np.concatenate((  # pylint: disable=g-complex-comprehension
      comm_and_diff_thrusts_to_thrusts,
      np.matrix(comm_and_diff_thrusts_to_thrust_moment[i])[0, :])).tolist()
                       for i in range(5)]

  # A total ground power constraint is roughly enforced by capping the
  # the total thrust that can be commanded at any given airspeed.
  # This maximum total thrust is calculated around a trim set of rotor
  # speeds that balance to body pitch and yaw moments.  The limits
  # imposed by the controller neglect the rotor inertia and varying
  # aerodynamic effects, and we rely on the stacking controller to
  # ensure this limit is met on fast time scales.

  # The approximate efficiency from electrical to shaft power.
  motor_efficiency = 0.88

  min_tether_current = (
      params['power_sys']['P_source'] / params['power_sys']['v_source_0'])
  total_source_resistance = (
      params['power_sys']['R_source'] + params['power_sys']['R_tether'])
  min_aero_power_electrical = (
      params['power_sys']['P_source'] +
      min_tether_current**2.0 * total_source_resistance)
  min_aero_power = motor_efficiency * min_aero_power_electrical

  # Note that tether current and aero power are negative by convention
  # when motoring.
  assert min_aero_power < 0.0, 'Aero power limit must be negative.'

  # Freestream velocities [m/s] at which to calculate power train
  # limits.
  freestream_vel_table = [0.0, 15.0, 30.0, 45.0, 60.0]

  # Set the tether resistance to zero (for determining the voltage
  # based thrust limit) if voltage compensation is active.
  tether_resistance = (0.0
                       if params['power_sys']['use_ground_voltage_compensation']
                       else params['power_sys']['R_tether'])

  # Maximum thrust [N] available for each motor at a given
  # freestream velocity given torque and power limits.
  #
  # Thrust, torque, and power vary almost linearly with air
  # density.  A maximum speed is used below to capture set maximum
  # thrust limits determined by Mach number considerations.
  max_thrusts_no_fault = np.array([
      simple_aero.CalcVoltageThrustLimit(  # pylint: disable=g-complex-comprehension
          params['power_sys']['v_source_0'], 0, tether_resistance,
          motor_efficiency,
          [rotor_models[rotor['version']] for rotor in params['rotors']],
          v_freestream)
      for v_freestream in freestream_vel_table
  ]).T

  max_thrusts_with_fault = np.array([
      simple_aero.CalcVoltageThrustLimit(  # pylint: disable=g-complex-comprehension
          params['power_sys']['v_source_0'], 1, tether_resistance,
          motor_efficiency,
          [rotor_models[rotor['version']] for rotor in params['rotors']],
          v_freestream)
      for v_freestream in freestream_vel_table
  ]).T

  max_thrusts = np.array([
      max_thrusts_no_fault.tolist(),
      max_thrusts_with_fault.tolist()
  ]).tolist()

  # Torque limit calculated from fundamental motor params.  This does
  # not take into speed related limits to achievable torque.
  max_torque_command = (
      1.5 * params['motors']['iq_cmd_upper_limit'] *
      params['motors']['flux_linkage'] * params['motors']['num_pole_pairs'])

  min_torque_command = (
      1.5 * params['motors']['iq_cmd_lower_limit'] *
      params['motors']['flux_linkage'] * params['motors']['num_pole_pairs'])

  return {
      # Angular rate [rad/s] of rotors during idling.  Even when the
      # wing is perched, the rotors should spin a little so that an
      # open-loop restart is not required.
      'idle_speed': 30.0,

      'max_speeds': max_speeds,

      # Limits [Nm] to torque commands.  Max is motoring, min is generation.
      'max_torque_command': max_torque_command,
      'min_torque_command': min_torque_command,

      # Minimum total aerodynamic power [W], defined to be negative during
      # thrusting.
      'min_aero_power': min_aero_power,

      # Whether to penalize the symmetric torsion mode. Penalization is not
      # required for the double reverse-rainbow rotor configuration, which
      # suppresses this mode by commanding zero roll moment with high weight.
      'penalize_symmetric_torsion_mode': False,

      # Weight [#] used to regularize the rotor calculation QP.  This
      # small weight is used to penalize differential thrust commands
      # for being non-zero.  It helps ensure there is a unique
      # minimizer for the resulting quadratic program.
      'regularization_weight': 1e-6,

      # Weight [#] used to avoid exciting the symmetric torsion mode
      # along the main wing.  This mode was excited in the 2016-04-06
      # flight tests due to a coupling between motor saturation and
      # pitch rate at the gyro due to this mode.  The current value
      # was chosen by setting a trade-off of 3 kN of thrust for 1 kN
      # of the value (T1 - T5) - (T2 - T6) - (T3 - T7) + (T4 - T8),
      # where Ti is the thrust of the ith motor, the value at which we
      # started to experience issues.
      #
      # TODO: This trade-off relies on the 1e-3 weight
      # used in the hover controller.  Figure out how to remove this
      # relationship.
      'symmetric_torsion_weight': 1e-2,

      'thrust_moment_to_thrusts': thrust_moment_to_thrusts.tolist(),
      'thrusts_to_thrust_moment': thrusts_to_thrust_moment.tolist(),

      'comm_and_diff_thrusts_to_thrusts': (
          comm_and_diff_thrusts_to_thrusts.tolist()),
      'comm_and_diff_thrusts_to_thrust_moment': (
          comm_and_diff_thrusts_to_thrust_moment),

      'constraint_matrix': constraint_matrix,

      'freestream_vel_table': freestream_vel_table,

      'max_thrusts': max_thrusts,

      # Maximum thrust [N] for any given motor mount in normal operation
      # and in motor out condition.  First number is structural, second number
      # is based on available thrust with 3 stack and 4200 V.
      # TODO: This needs to improve if we want to survive motor out
      # condition but is tied to structural improvements.  Motor fault overload
      # condition needs validation.
      'motor_mount_thrust_limit': [3670.0, 4700.0],

      # Maximum total thrust [N] that can be achieved while balancing pitch and
      # yaw moments and respecting min_aero_power at a given freestream
      # velocity.
      'total_power_limit_thrusts': [
          simple_aero.CalcMaxTotalThrustForFreestreamVelocity(  # pylint: disable=g-complex-comprehension
              min_aero_power, freestream_vel,
              [rotor_models[rotor['version']] for rotor in params['rotors']],
              np.matrix(comm_and_diff_thrusts_to_thrust_moment[0]),
              np.matrix(comm_and_diff_thrusts_to_thrusts))
          for freestream_vel in freestream_vel_table
      ],

      'simple_models': [simple_models[r['version']] for r in params['rotors']]
  }

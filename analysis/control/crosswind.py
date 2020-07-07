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

"""Crosswind analysis module."""

from __future__ import absolute_import

import collections
import copy
import itertools
import json
import os
import sys

import control
import gflags
import makani
import makani.analysis.control
from makani.analysis.control import dynamics
from makani.analysis.control import geometry
from makani.analysis.control import type_util
from makani.analysis.control import write_controllers
from makani.config import mconfig
from makani.config import overrides_util
from makani.control import control_types
from makani.control import system_types
from makani.lib.python import c_helpers
from makani.lib.python import wing_flag
import numpy as np
from scipy import io
from scipy import optimize

wing_flag.AppeaseLintWhenImportingFlagOnly()
makani.SetRunfilesDirFromBinaryPath()

gflags.DEFINE_boolean('display_trim', True, 'Displays trim state and inputs.')
gflags.DEFINE_boolean('display_stability', False,
                      'Dispays poles and stability status.',
                      short_name='s')
gflags.DEFINE_string('export_json_filename', None,
                     'Name of JSON file to export the controller gains.')
gflags.DEFINE_string('export_matlab_filename', None,
                     'Name of file to export data to in .mat format.')
gflags.DEFINE_string('overrides', None,
                     'JSON string of overrides.',
                     short_name='o')
gflags.DEFINE_integer('wing_serial_enum', None,
                      'Enum of the WingSerial to be processed')

FLAGS = gflags.FLAGS


_WING_SERIAL_HELPER = c_helpers.EnumHelper('WingSerial', system_types)

# State of the wing in ground coordinates.
#
# Attributes:
#   omega_b: Body angular rate.
#   eulers_g: Euler angles about the ground coordinate frame.
#   wing_vel_g: Velocity of the wing in ground coordinates.
#   wing_pos_g: Position of the wing in ground coordinates.
CrosswindState = type_util.MakeFlatStateClass(  # pylint: disable=invalid-name
    'CrosswindState', [('omega_b', range(0, 3)),
                       ('eulers_g', range(3, 6)),
                       ('wing_vel_g', range(6, 9)),
                       ('wing_pos_g', range(9, 12))])

# Output vector of the linearized crosswind state-space model.
CrosswindOutputs = type_util.MakeNamedVectorClass(  # pylint: disable=invalid-name
    'CrosswindOutputs', [('tether_roll', [0]),
                         ('sideslip', [1]),
                         ('roll_rate', [2]),
                         ('yaw_rate', [3]),
                         ('angle_of_attack', [4]),
                         ('pitch_rate', [5])])


class CrosswindFrame(object):
  """Representation of the transformation from WingState to CrosswindState."""

  def StateTangentFromWingStateTangent(self, wing_state, tangent):
    """Convert a WingState.Tangent to a CrosswindState tangent vector."""
    yaw, pitch, roll = geometry.DcmToAngle(wing_state.dcm_g2b)
    eulers_g = np.matrix([[roll], [pitch], [yaw]])
    return CrosswindState.Tangent(domega_b=tangent.domega_b,
                                  deulers_g=geometry.GetAngleDerivative(
                                      eulers_g, tangent.ddcm_g2b),
                                  dwing_vel_g=tangent.dwing_vel_g,
                                  dwing_pos_g=tangent.dwing_pos_g)

  def StateToWingState(self, state):
    """Converts a CrosswindState to a WingState."""
    return dynamics.WingState(omega_b=state.omega_b,
                              dcm_g2b=geometry.AngleToDcm(state.eulers_g[2],
                                                          state.eulers_g[1],
                                                          state.eulers_g[0]),
                              wing_vel_g=state.wing_vel_g,
                              wing_pos_g=state.wing_pos_g)


class CrosswindInputs(type_util.MakeNamedVectorClass(
    'CrosswindInputs', [('thrust', range(0, 1)),
                        ('motor_moment', range(1, 4)),
                        ('aileron', range(4, 5)),
                        ('elevator', range(5, 6)),
                        ('rudder', range(6, 7)),
                        ('wind_g', range(7, 10))])):
  """Structure crosswind inputs to the wing.

  Attributes:
    thrust: Motor thrust [N] (1-by-1 np.matrix).
    motor_moment: Motor moments [N-m] (3-by-1 np.matrix).
    aileron: Aileron deflection [rad].
    elevator: Elevator deflection [rad].
    rudder: Rudder deflection [rad].
    wind_g: Wind speed [m/s] in ground coordinates (3-by-1 np.matrix).
  """

  @type_util.RequireMatrixArguments(None, (control_types.kNumFlaps, 1))
  def ToWingInputs(self, flap_offsets):
    flaps = copy.copy(flap_offsets)
    flaps[control_types.kFlapA1] -= self.aileron
    flaps[control_types.kFlapA2] -= self.aileron
    flaps[control_types.kFlapA7] += self.aileron
    flaps[control_types.kFlapA8] += self.aileron
    flaps[control_types.kFlapEle] += self.elevator
    flaps[control_types.kFlapRud] += self.rudder
    return dynamics.WingInputs(thrust=copy.copy(self.thrust),
                               motor_moment=copy.copy(self.motor_moment),
                               flaps=flaps,
                               wind_g=copy.copy(self.wind_g))


# pylint doesn't like capital letters in variable names, in contrast
# to control systems conventions.
# pylint: disable=invalid-name


class ControlDesign(object):
  """Class for trimming and calculating gains for crosswind."""

  def __init__(self, system_params, sim_params):
    """Constructs a ControlDesign object from parameters."""

    self._system_params = system_params

    motor_model = dynamics.PureForceMomentMotorModel(
        system_params['rotors'], system_params['wing']['center_of_mass_pos'])

    # The elastic spring constant of the tether is k = E * A / L.
    # Catenary effect can also soften this spring constant; we apply a
    # correction factor in order to match in the linear models the tether
    # frequency measured in flight tests (b/69970696).
    # In the end, the controller must be stable over a range of spring
    # constants.
    spring_const = (
        system_params['tether']['tensile_stiffness'] /
        system_params['tether']['length']) * 0.75

    # TODO: What's the proper way to incorporate the
    # effective mass of the tether here.  This isn't critical to the
    # lateral dynamics, but it does play into the longitudinal
    # dynamics.
    tether_model = dynamics.SimpleSpringTetherForceModel(
        spring_const, system_params)

    self._wing = dynamics.Wing(
        system_params, sim_params, dynamics.SwigAeroModel(),
        motor_model, tether_model)

    self._crosswind_frame = CrosswindFrame()

    # NOTE: It is very important that the trim flap
    # deflections are away from the flap limits encoded in the
    # configuration system.  Otherwise, the wing model will not
    # properly calculate the control derivative.
    self._flap_offsets = np.matrix([
        [0.0] for _ in range(control_types.kNumFlaps)])
    self._flap_offsets[control_types.kFlapA1, 0] = -0.1
    self._flap_offsets[control_types.kFlapA2, 0] = -0.1
    self._flap_offsets[control_types.kFlapA4, 0] = 0.0
    self._flap_offsets[control_types.kFlapA5, 0] = 0.0
    self._flap_offsets[control_types.kFlapA7, 0] = -0.1
    self._flap_offsets[control_types.kFlapA8, 0] = -0.1
    self._flap_offsets[control_types.kFlapEle, 0] = 0.05
    self._flap_offsets[control_types.kFlapRud, 0] = 0.0

  def CalcTrim(self, kite_speed, wind_speed, alpha_cmd=0.0,
               circle_radius=150.0):
    """Finds trim conditions for the wing.

    First, finds an approximate trim state by using the z-position to
    zero z-acceleration, thrust to zero x-acceleration, roll to set
    the y-acceleration to the expect centripetal acceleration, and
    finally pitch to produce the command angle-of-attack.

    Once the wing is in this approximate trim state, we use thrust,
    ailerons, elevator, rudder, z-position, and roll angle to
    simultaneously trim all the angular and translational
    accelerations.

    Args:
      kite_speed: Nominal forward velocity [m/s] of the wing.
      wind_speed: Wind speed [m/s].
      alpha_cmd: Nominal angle-of-attack [rad].
      circle_radius: Nominal circular path radius [m].

    Returns:
      Tuple (state, inputs) where state is a CrosswindState and inputs is
      a dynamics.WingInputs.
    """
    # Build initial guess for trim state.
    wind_g = np.matrix([[0.0], [0.0], [-wind_speed]])
    tether_length = self._system_params['tether']['length']

    wing_pos_g_z = -np.sqrt(tether_length**2.0 - circle_radius**2.0)

    omega_b = np.matrix([[0.0], [0.0], [-kite_speed / circle_radius]])
    wing_pos_g = np.matrix([[0.0], [circle_radius], [wing_pos_g_z]])
    wing_vel_g = np.matrix([[kite_speed], [0.0], [0.0]])
    vec3_zero = np.matrix([[0.0], [0.0], [0.0]])

    state_0 = CrosswindState(omega_b=copy.copy(omega_b),
                             eulers_g=vec3_zero,
                             wing_vel_g=wing_vel_g,
                             wing_pos_g=wing_pos_g)

    # Build initial guess for trim inputs.
    inputs_0 = CrosswindInputs(thrust=np.matrix([[-1000.0]]),
                               motor_moment=copy.copy(vec3_zero),
                               aileron=np.matrix([[0.0]]),
                               elevator=np.matrix([[0.0]]),
                               rudder=np.matrix([[0.0]]),
                               wind_g=copy.copy(wind_g))
    wing_inputs_0 = inputs_0.ToWingInputs(self._flap_offsets)

    # Trim z position to produce zero z acceleration.
    def _TrimZPosition(x):
      trim_state = copy.deepcopy(state_0)
      trim_state.wing_pos_g[2, 0] = x[0]
      wing_state = self._crosswind_frame.StateToWingState(trim_state)
      state_dot = self._crosswind_frame.StateTangentFromWingStateTangent(
          wing_state, self._wing.CalcDeriv(wing_state, wing_inputs_0))
      return [state_dot.dwing_vel_g[2, 0]]

    # Trim thrust to produce zero x acceleration.
    def _TrimThrust(x):
      trim_inputs = copy.deepcopy(inputs_0)
      trim_inputs.thrust[0, 0] = x[0]
      wing_state = self._crosswind_frame.StateToWingState(state_0)
      wing_inputs = trim_inputs.ToWingInputs(self._flap_offsets)
      state_dot = self._crosswind_frame.StateTangentFromWingStateTangent(
          wing_state, self._wing.CalcDeriv(wing_state, wing_inputs))
      return [state_dot.dwing_vel_g[0, 0]]

    # Trim roll to produce the expected centripetal acceleration.
    def _TrimRoll(x):
      trim_state = copy.deepcopy(state_0)
      trim_state.eulers_g[0, 0] = x[0]
      wing_state = self._crosswind_frame.StateToWingState(trim_state)
      state_dot = self._crosswind_frame.StateTangentFromWingStateTangent(
          wing_state, self._wing.CalcDeriv(wing_state, wing_inputs_0))
      return [state_dot.dwing_vel_g[1, 0] + kite_speed**2 / circle_radius]

    # Trim pitch to produce the command angle-of-attack.
    def _TrimPitch(x):
      trim_state = copy.deepcopy(state_0)
      trim_state.eulers_g[1, 0] = x[0]
      wing_state = self._crosswind_frame.StateToWingState(trim_state)
      _, alpha, _ = wing_state.CalcAerodynamicAngles(wing_inputs_0.wind_g)
      return [alpha - alpha_cmd]

    # Use the z-position and roll states in the final trim
    # calculation.
    def _GetTrimState(x):
      trim_state = copy.deepcopy(state_0)
      trim_state.wing_pos_g[2, 0] += x[0]
      trim_state.eulers_g[0, 0] += x[1]
      return trim_state

    def _GetTrimInputs(x):
      """Calculates a trim inputs vector."""
      trim_inputs = copy.deepcopy(inputs_0)
      trim_inputs.thrust[0, 0] += x[2]
      trim_inputs.aileron[0, 0] = x[3]
      trim_inputs.elevator[0, 0] = x[4]
      trim_inputs.rudder[0, 0] = x[5]
      return trim_inputs

    def _TrimFunction(x):
      """Wrapper function for trimming the wing."""
      wing_state = self._crosswind_frame.StateToWingState(_GetTrimState(x))
      wing_inputs = _GetTrimInputs(x).ToWingInputs(self._flap_offsets)
      state_dot = self._crosswind_frame.StateTangentFromWingStateTangent(
          wing_state, self._wing.CalcDeriv(wing_state, wing_inputs))
      return [state_dot.domega_b[0, 0],
              state_dot.domega_b[1, 0],
              state_dot.domega_b[2, 0],
              state_dot.dwing_vel_g[0, 0],
              state_dot.dwing_vel_g[1, 0] + kite_speed**2 / circle_radius,
              state_dot.dwing_vel_g[2, 0]]

    # Perform a few single parameter trims to get the initial solution
    # closer to the final solution.
    state_0.wing_pos_g[2, 0] = optimize.fsolve(_TrimZPosition,
                                               state_0.wing_pos_g[2, 0])
    state_0.eulers_g[0, 0] = optimize.fsolve(_TrimRoll, state_0.eulers_g[0, 0])
    state_0.eulers_g[1, 0] = optimize.fsolve(_TrimPitch, state_0.eulers_g[1, 0])
    inputs_0.thrust[0, 0] = optimize.fsolve(_TrimThrust, inputs_0.thrust[0, 0])

    # Trim the wing to have zero acceleration and zero angular
    # acceleration using thrust, flap deflections, z position, and
    # roll.
    x, info, _, _ = optimize.fsolve(_TrimFunction, np.zeros((6, 1)),
                                    full_output=True)
    assert np.linalg.norm(info['fvec']) < 1e-6

    return _GetTrimState(x), _GetTrimInputs(x)

  def GetFlapOffsets(self, inputs):
    wing_inputs = inputs.ToWingInputs(self._flap_offsets)
    return [
        wing_inputs.flaps[control_types.kFlapA1][0, 0],
        wing_inputs.flaps[control_types.kFlapA2][0, 0],
        wing_inputs.flaps[control_types.kFlapA4][0, 0],
        wing_inputs.flaps[control_types.kFlapA5][0, 0],
        wing_inputs.flaps[control_types.kFlapA7][0, 0],
        wing_inputs.flaps[control_types.kFlapA8][0, 0],
        wing_inputs.flaps[control_types.kFlapEle][0, 0],
        wing_inputs.flaps[control_types.kFlapRud][0, 0]
    ]

  def PrintTrim(self, state, inputs):
    """Prints information relevant to a trimmed state.

    Args:
      state: CrosswindState structure.
      inputs: dynamics.WingInputs structure.
    """
    wing_state = self._crosswind_frame.StateToWingState(state)
    wing_inputs = inputs.ToWingInputs(self._flap_offsets)
    state_dot = self._crosswind_frame.StateTangentFromWingStateTangent(
        wing_state, self._wing.CalcDeriv(wing_state, wing_inputs))
    v_rel, alpha, beta = wing_state.CalcAerodynamicAngles(wing_inputs.wind_g)
    tether_force_g = self._wing.CalcTetherForceG(wing_state, wing_inputs)
    values = [[
        ('Wing pos. xg [m]', state.wing_pos_g[0]),
        ('Wing pos. yg [m]', state.wing_pos_g[1]),
        ('Wing pos. zg [m]', state.wing_pos_g[2])
    ], [
        ('Wing vel. xg [m/s]', state.wing_vel_g[0]),
        ('Wing vel. yg [m/s]', state.wing_vel_g[1]),
        ('Wing vel. zg [m/s]', state.wing_vel_g[2])
    ], [
        ('Roll [deg]', (180.0 / np.pi) * state.eulers_g[0]),
        ('Pitch [deg]', (180.0 / np.pi) * state.eulers_g[1]),
        ('Yaw [deg]', (180.0 / np.pi) * state.eulers_g[2])
    ], [
        ('P [rad/s]', state.omega_b[0]),
        ('Q [rad/s]', state.omega_b[1]),
        ('R [rad/s]', state.omega_b[2])
    ], [
        ('Thrust [N]', inputs.thrust),
    ], [
        ('A1 [deg]',
         (180.0 / np.pi) * wing_inputs.flaps[control_types.kFlapA1]),
        ('A2 [deg]',
         (180.0 / np.pi) * wing_inputs.flaps[control_types.kFlapA2]),
        ('A4 [deg]',
         (180.0 / np.pi) * wing_inputs.flaps[control_types.kFlapA4]),
        ('A5 [deg]',
         (180.0 / np.pi) * wing_inputs.flaps[control_types.kFlapA5]),
        ('A7 [deg]',
         (180.0 / np.pi) * wing_inputs.flaps[control_types.kFlapA7]),
        ('A8 [deg]',
         (180.0 / np.pi) * wing_inputs.flaps[control_types.kFlapA8]),
        ('Ele. [deg]',
         (180.0 / np.pi) * wing_inputs.flaps[control_types.kFlapEle]),
        ('Rud. [deg]',
         (180.0 / np.pi) * wing_inputs.flaps[control_types.kFlapRud])
    ], [
        ('Vrel [m/s]', v_rel),
        ('Alpha [deg]', (180.0 / np.pi) * alpha),
        ('Beta [deg]', (180.0 / np.pi) * beta)
    ], [
        ('Tether Force xg [N]', tether_force_g[0]),
        ('Tether Force yg [N]', tether_force_g[1]),
        ('Tether Force zg [N]', tether_force_g[2]),
    ], [
        ('Pdot [rad/s^2]', state_dot.domega_b[0]),
        ('Qdot [rad/s^2]', state_dot.domega_b[1]),
        ('Rdot [rad/s^2]', state_dot.domega_b[2])
    ], [
        ('A_g X [m/s^2]', state_dot.dwing_vel_g[0]),
        ('A_g Y [m/s^2]', state_dot.dwing_vel_g[1]),
        ('A_g Z [m/s^2]', state_dot.dwing_vel_g[2])
    ]]

    for line_values in values:
      for (name, value) in line_values:
        print '%20s: %10.3f' % (name, value)
    print ''

  def PrintStability(self, *args):
    """Prints the poles and the stability of the crosswind dynamic systems."""

    def _RoundedPolesToString(poles):
      """Rounds a list of complex poles and outputs as a list of strings."""
      return ['%.2f + %.2fi' %(np.real(elem), np.imag(elem)) for elem in poles]

    all_poles = []
    f_stable = []
    for system in args:
      sys_poles = np.linalg.eig(system.A)[0]
      f_stable.append('Unstable' if np.max(np.real(sys_poles)) > 0
                      else 'Stable')
      all_poles.append(_RoundedPolesToString(sys_poles))

    all_poles = list(itertools.izip_longest(
        all_poles[0], all_poles[1], all_poles[2], all_poles[3], all_poles[4],
        all_poles[5], fillvalue=''))

    print '|%s|' % ('-'*101)
    print '|%50s|%50s|' % ('Open-loop poles', 'Closed-loop poles')
    print '|%s|%s|' % ('-'*50, '-'*50)
    print '|%16s|%16s|%16s|%16s|%16s|%16s|' %('longitudinal', 'lateral', 'full',
                                              'longitudinal', 'lateral', 'full')
    print '|%s|%s|%s|%s|%s|%s|' % ('-'*16, '-'*16, '-'*16,
                                   '-'*16, '-'*16, '-'*16)
    for n in range(0, len(all_poles)):
      print '|%16s|%16s|%16s|%16s|%16s|%16s|' %(all_poles[n][0],
                                                all_poles[n][1],
                                                all_poles[n][2],
                                                all_poles[n][3],
                                                all_poles[n][4],
                                                all_poles[n][5])
    print '|%s|%s|%s|%s|%s|%s|' % ('-'*16, '-'*16, '-'*16,
                                   '-'*16, '-'*16, '-'*16)
    print '|%16s|%16s|%16s|%16s|%16s|%16s|' %(f_stable[0],
                                              f_stable[1],
                                              f_stable[2],
                                              f_stable[3],
                                              f_stable[4],
                                              f_stable[5])
    print '|%s|\n\n' % ('-'*101)

  def LinearizeSystem(self, trim_state, trim_inputs):
    """Calculates linearized system matrices about operating point.

    Args:
      trim_state: CrosswindState about which to linearize.
      trim_inputs: dynamics.WingInputs inputs about which to linearize.

    Returns:
      Control module state-space system containing the linearized
      system A, B, C, D matrices.
    """
    def _CalcDeriv(state, inputs):
      """Calculates state derivative vector from state and inputs vectors."""
      wing_state = self._crosswind_frame.StateToWingState(state)
      wing_inputs = inputs.ToWingInputs(self._flap_offsets)
      return self._crosswind_frame.StateTangentFromWingStateTangent(
          wing_state, self._wing.CalcDeriv(wing_state, wing_inputs)).ToVector()

    def _CalcOutputs(state, inputs):
      """Calculates output vector from state and inputs vectors."""
      wing_state = self._crosswind_frame.StateToWingState(state)
      _, alpha, beta = wing_state.CalcAerodynamicAngles(inputs.wind_g)
      _, tether_roll, _ = self._wing.CalcTetherTensionRollPitch(wing_state,
                                                                inputs)
      p = wing_state.omega_b[0, 0]
      q = wing_state.omega_b[1, 0]
      r = wing_state.omega_b[2, 0]
      return np.matrix([[tether_roll], [beta], [p], [r], [alpha], [q]])

    # Calculate step sizes for linearization.
    #
    # TODO: All step sizes are currently 1e-6, as was their original
    # default. Variable step sizes were introduced to address a failure in
    # trans_in_test.py; see b/36783475#comment19. Consider rescaling all step
    # sizes.
    state_step_sizes = trim_state.Tangent.StepVector(
        {'domega_b': 1e-6, 'deulers_g': 1e-6,
         'dwing_vel_g': 1e-6, 'dwing_pos_g': 1e-6})
    input_step_sizes = trim_inputs.StepVector(
        {'thrust': 1e-6, 'motor_moment': 1e-6, 'aileron': 1e-6,
         'elevator': 1e-6, 'rudder': 1e-6, 'wind_g': 1e-6})

    # Calculate linearized model.
    A, B = dynamics.CalcLinearization(_CalcDeriv, trim_state, trim_inputs,
                                      state_step_sizes, input_step_sizes)
    C, D = dynamics.CalcLinearization(_CalcOutputs, trim_state, trim_inputs,
                                      state_step_sizes, input_step_sizes)

    return control.ss(A, B, C, D)

  def GetServoSystem(self):
    """Builds a 2nd-order system model of the servos."""
    # Cutoff frequency [rad/s] and damping ratio [#] are based on an
    # approximate model from servo identification tests performed in
    # Feb. 2016.  See https://docs.google.com/presentation/d/
    # 1QgMiqHOHAy4YOh5jCSdqgRN6SkZlNAH_JewQ-_KfDyg/edit#slide=id.p4.
    cutoff_freq = 2.0 * np.pi * 4.0
    damping_ratio = 0.8

    # Build 2nd-order system model of servo with states:
    # x = [theta, dtheta/dt]'.
    A_servo = [[0.0, 1.0],
               [-cutoff_freq**2.0, -2.0 * damping_ratio * cutoff_freq]]
    B_servo = [[0.0],
               [cutoff_freq**2.0]]
    return control.ss(A_servo, B_servo, [[1.0, 0.0]], 0.0)

  def GetMotorSystem(self):
    """Builds a 2nd-order system model of the motors."""
    cutoff_freq = 2.0 * np.pi * 6.0
    damping_ratio = 0.707

    # Build 2nd-order system model of motor with states:
    # x = [omega, domega/dt]'.
    A_motor = [[0.0, 1.0],
               [-cutoff_freq**2.0, -2.0 * damping_ratio * cutoff_freq]]
    B_motor = [[0.0],
               [cutoff_freq**2.0]]
    return control.ss(A_motor, B_motor, [[1.0, 0.0]], 0.0)

  def GetLateralSystem(self, linearized_system):
    """Grabs the lateral subsystem of the full linearized system.

    First, do a similarity transform that converts roll to tether roll
    and y-velocity to sideslip.  Then select out only the lateral
    components of the matrices, i.e. tether roll, sideslip, roll rate,
    and yaw rate.

    Args:
      linearized_system: Control module state-space system containing
          the full linearized system A, B, C, D matrices.

    Returns:
      Control module state-space system containing the A, B, C, D
          matrices of the lateral subsystem.
    """
    state_index = CrosswindState.Tangent.GetIndices()
    inputs_index = CrosswindInputs.GetIndices()
    num_states = np.shape(linearized_system.A)[0]
    num_inputs = np.shape(linearized_system.B)[1]
    num_lateral_states = control_types.kNumCrosswindLateralStates
    num_lateral_inputs = control_types.kNumCrosswindLateralInputs

    # Transform roll to tether roll and y velocity to sideslip.
    S_transform = np.matrix(np.eye(num_states))
    S_transform[state_index.deulers_g[0], :] = (
        linearized_system.C[control_types.kCrosswindLateralStateTetherRoll, :])
    S_transform[state_index.dwing_vel_g[1], :] = (
        linearized_system.C[control_types.kCrosswindLateralStateSideslip, :])
    A_transform = S_transform * linearized_system.A * np.linalg.inv(S_transform)
    B_transform = S_transform * linearized_system.B

    # Select lateral states.
    S_lateral = np.matrix(np.zeros((num_lateral_states, num_states)))
    S_lateral[control_types.kCrosswindLateralStateTetherRoll,
              state_index.deulers_g[0]] = 1.0
    S_lateral[control_types.kCrosswindLateralStateSideslip,
              state_index.dwing_vel_g[1]] = 1.0
    S_lateral[control_types.kCrosswindLateralStateRollRate,
              state_index.domega_b[0]] = 1.0
    S_lateral[control_types.kCrosswindLateralStateYawRate,
              state_index.domega_b[2]] = 1.0

    # Combine ailerons into single delta_aileron and select lateral
    # inputs.
    T_lateral = np.matrix(np.zeros((num_inputs, num_lateral_inputs)))
    T_lateral[inputs_index.aileron,
              control_types.kCrosswindLateralInputAileron] = 1.0
    T_lateral[inputs_index.rudder,
              control_types.kCrosswindLateralInputRudder] = 1.0
    T_lateral[inputs_index.motor_moment[2],
              control_types.kCrosswindLateralInputMotorYaw] = 1.0

    A_lateral = S_lateral * A_transform * S_lateral.T
    B_lateral = S_lateral * B_transform * T_lateral

    # Setup derivatives of integrator states.  The minus sign gives us
    # the expected signs on the gains from: error = commanded - measured.
    A_lateral[control_types.kCrosswindLateralStateIntegratedTetherRoll,
              control_types.kCrosswindLateralStateTetherRoll] = -1.0
    A_lateral[control_types.kCrosswindLateralStateIntegratedSideslip,
              control_types.kCrosswindLateralStateSideslip] = -1.0

    return control.ss(A_lateral, B_lateral, np.eye(A_lateral.shape[0]),
                      np.zeros(B_lateral.shape))

  def DesignLateralController(self, lateral_system, control_params):
    """Selects a gain matrix for the lateral control system."""
    num_states = control_types.kNumCrosswindLateralStates
    num_inputs = control_types.kNumCrosswindLateralInputs

    # Approximate maximum desired state error or actuator deflection, used to
    # define LQR weights.
    state_max = np.asarray(
        control_params['crosswind']['inner']['lateral_states_max'])
    input_max = np.asarray(
        control_params['crosswind']['inner']['lateral_inputs_max'])

    # Apply Bryson's rule for cost selection.
    input_weight = 2.0
    Q = np.diag(1.0 / state_max**2.0) / num_states
    R = input_weight * np.diag(1.0 / input_max**2.0) / num_inputs

    lateral_gains, _, _ = control.lqr(lateral_system.A, lateral_system.B, Q, R)

    return lateral_gains

  def CloseLateralSystem(self, lateral_system, servo_system,
                         motor_system, lateral_gains):
    """Outputs the closed-loop lateral system."""
    actuator_system = servo_system.append(servo_system.append(motor_system))
    lateral_actuator_system = lateral_system * actuator_system
    closed_system = control.feedback(lateral_actuator_system,
                                     _GainsToSystem(lateral_gains))
    # Confirm that the closed-loop system is stable with actuator dynamics.
    poles = np.linalg.eig(closed_system.A)[0]
    assert np.max(np.real(poles)) < 0.0

    return closed_system

  def GetLongitudinalSystem(self, linearized_system):
    """Grabs the longitudinal subsystem of the full linearized system.

    Transforms the pitch angle state to an angle-of-attack state.
    Then, selects only the vertical displacement and velocity,
    angle-of-attack, and pitch rate components.  There are five states
    [z, z_dot, alpha, q, integrated_alpha] and two inputs [elevator,
    motor_pitch].

    Args:
      linearized_system: Control module state-space system containing
          the full linearized system A, B, C, D matrices.

    Returns:
      Control module state-space system containing the A, B, C, D
          matrices of the longitudinal subsystem.
    """
    state_index = CrosswindState.Tangent.GetIndices()
    inputs_index = CrosswindInputs.GetIndices()
    num_states = np.shape(linearized_system.A)[0]
    num_inputs = np.shape(linearized_system.B)[1]
    num_longitudinal_states = control_types.kNumCrosswindLongitudinalStates
    num_longitudinal_inputs = control_types.kNumCrosswindLongitudinalInputs

    # Transform pitch to angle-of-attack.
    S_transform = np.matrix(np.eye(num_states))
    S_transform[state_index.deulers_g[1], :] = linearized_system.C[4, :]
    A_transform = S_transform * linearized_system.A * np.linalg.inv(S_transform)
    B_transform = S_transform * linearized_system.B

    # Select longitudinal states.
    S_longitudinal = np.matrix(np.zeros((num_longitudinal_states, num_states)))
    S_longitudinal[control_types.kCrosswindLongitudinalStatePositionGroundZ,
                   state_index.dwing_pos_g[2]] = 1.0
    S_longitudinal[control_types.kCrosswindLongitudinalStateVelocityGroundZ,
                   state_index.dwing_vel_g[2]] = 1.0
    S_longitudinal[control_types.kCrosswindLongitudinalStateAngleOfAttack,
                   state_index.deulers_g[1]] = 1.0
    S_longitudinal[control_types.kCrosswindLongitudinalStatePitchRate,
                   state_index.domega_b[1]] = 1.0

    # Select the elevator and the motor_pitch as the control inputs.
    T_longitudinal = np.matrix(np.zeros((num_inputs, num_longitudinal_inputs)))
    T_longitudinal[inputs_index.elevator,
                   control_types.kCrosswindLongitudinalInputElevator] = 1.0
    T_longitudinal[inputs_index.motor_moment[1],
                   control_types.kCrosswindLongitudinalInputMotorPitch] = 1.0

    A_longitudinal = S_longitudinal * A_transform * S_longitudinal.T
    B_longitudinal = S_longitudinal * B_transform * T_longitudinal

    # Setup derivative of angle-of-attack integrator.
    A_longitudinal[
        control_types.kCrosswindLongitudinalStateIntegratedAngleOfAttack,
        control_types.kCrosswindLongitudinalStateAngleOfAttack] = -1.0

    return control.ss(A_longitudinal, B_longitudinal,
                      np.eye(A_longitudinal.shape[0]),
                      np.zeros(B_longitudinal.shape))

  def DesignLongitudinalController(self, longitudinal_system, control_params):
    """Selects a gain matrix for the longitudinal control system."""
    num_states = control_types.kNumCrosswindLongitudinalStates
    num_inputs = control_types.kNumCrosswindLongitudinalInputs

    # Approximate maximum desired state error or actuator deflection, used to
    # define LQR weights.
    state_max = np.asarray(
        control_params['crosswind']['inner']['longitudinal_states_max'])
    input_max = np.asarray(
        control_params['crosswind']['inner']['longitudinal_inputs_max'])

    # Apply Bryson's rule for cost selection.
    input_weight = 1.0
    Q = np.diag(1.0 / state_max**2.0) / num_states
    R = input_weight * np.diag(1.0 / input_max**2.0) / num_inputs

    longitudinal_gains, _, _ = control.lqr(longitudinal_system.A,
                                           longitudinal_system.B, Q, R)

    return longitudinal_gains

  def CloseLongitudinalSystem(self, longitudinal_system, servo_system,
                              motor_system, longitudinal_gains):
    """Outputs the closed-loop longitudinal system."""

    # The state errors for PositionGroundZ and VelocityGroundZ are unused in
    # makani/control/crosswind/crosswind_inner.c. Set the gains associated
    # to these states to zero before checking the closed-loop stability of
    # the longitudinal system.
    num_inputs = control_types.kNumCrosswindLongitudinalInputs
    reduced_longitudinal_gains = copy.copy(longitudinal_gains)
    reduced_longitudinal_gains[
        :, control_types.kCrosswindLongitudinalStatePositionGroundZ] = (
            np.zeros(num_inputs,))
    reduced_longitudinal_gains[
        :, control_types.kCrosswindLongitudinalStateVelocityGroundZ] = (
            np.zeros(num_inputs,))

    # Build the longitudinal closed-loop system.
    actuator_system = servo_system.append(motor_system)
    longitudinal_actuator_system = longitudinal_system * actuator_system
    closed_system = control.feedback(longitudinal_actuator_system,
                                     _GainsToSystem(reduced_longitudinal_gains))

    # Confirm that the closed-loop system is stable with actuator dynamics.
    poles = np.linalg.eig(closed_system.A)[0]
    assert np.max(np.real(poles)) < 0.0

    return closed_system

  def GetFullClosedLoopSystem(self, open_crosswind_system,
                              servo_system, motor_system,
                              longitudinal_gains, lateral_gains):
    """Outputs the full crosswind closed-loop system.

    Args:
      open_crosswind_system: Open-loop full system in state-space form.
          12 states: omega_b, eulers_g, wing_vel_g, wing_pos_g
          10 inputs: thrust, motor_moment, aileron, elevator, rudder, wind_g
          6 outputs: tether_roll, beta, p, r, alpha, q]
      servo_system: 2nd order servo model in state-space form.
      motor_system: 2nd-order motor model in state-space form.
      longitudinal_gains: [2 x 5] Numpy array of longitudinal control gains.
      lateral_gains: [3 x 6] Numpy array of lateral control gains.

    Returns:
      Control module state-space system of the full closed-loop system.
          25 states: 10 actuator states,
                     5 CrosswindLongitudinalStates,
                     6 CrosswindLateralStates,
                     PositionGroundX,
                     VelocityGroundX,
                     PositionGroundY,
                     Yaw.
          5 inputs: Elevator, MotorPitch, Aileron, Rudder, MotorYaw.
          15 outputs: 5 CrosswindLongitudinalStates,
                      6 CrosswindLateralStates,
                      PositionGroundX,
                      VelocityGroundX,
                      PositionGroundY,
                      Yaw.
    """

    def _AddState(system, position):
      """Augments a state-space model by adding rows and columns of zeros.

      Args:
        system: State-space model.
        position: Integer indicating the position where row/column are added.

      Returns:
        An augmented space-space model.
      """

      system.A = np.insert(system.A, position, 0.0, axis=0)
      system.A = np.insert(system.A, position, 0.0, axis=1)
      system.B = np.insert(system.B, position, 0.0, axis=0)
      system.C = np.insert(system.C, position, 0.0, axis=1)

      return system

    state_index = CrosswindState.Tangent.GetIndices()
    output_index = CrosswindOutputs.GetIndices()
    num_crosswind_states = CrosswindState.Tangent.GetDim()
    num_longitudinal_inputs = control_types.kNumCrosswindLongitudinalInputs
    num_lateral_inputs = control_types.kNumCrosswindLateralInputs

    # Transform pitch angle to angle-of-attack.
    S_transform = np.matrix(np.eye(num_crosswind_states))
    S_transform[state_index.deulers_g[1], :] = (
        open_crosswind_system.C[output_index.angle_of_attack, :])
    A_transformed = (
        S_transform * open_crosswind_system.A * np.linalg.inv(S_transform))
    B_transformed = S_transform * open_crosswind_system.B
    C_transformed = open_crosswind_system.C * np.linalg.inv(S_transform)
    D_transformed = open_crosswind_system.D

    # Transform roll to tether roll and y velocity to angle-of-sideslip.
    S_transform = np.matrix(np.eye(num_crosswind_states))
    S_transform[state_index.deulers_g[0], :] = (
        open_crosswind_system.C[output_index.tether_roll, :])
    S_transform[state_index.dwing_vel_g[1], :] = (
        open_crosswind_system.C[output_index.sideslip, :])
    A_transformed = S_transform * A_transformed * np.linalg.inv(S_transform)
    B_transformed = S_transform * B_transformed
    C_transformed *= np.linalg.inv(S_transform)

    sys_transformed = control.ss(A_transformed, B_transformed,
                                 C_transformed, D_transformed)

    # Add three integral states.
    # Integrated angle-of-attack (transformed from pitch CrosswindState).
    sys_augmented = _AddState(sys_transformed, np.shape(sys_transformed.A)[0])
    sys_augmented.A[-1, state_index.deulers_g[1]] = -1
    # Integrated tether roll (transformed from roll CrosswindState).
    sys_augmented = _AddState(sys_augmented, np.shape(sys_augmented.A)[0])
    sys_augmented.A[-1, state_index.deulers_g[0]] = -1
    # Integrated sideslip (transformed from VelocityGroundY CrosswindState).
    sys_augmented = _AddState(sys_augmented, np.shape(sys_augmented.A)[0])
    sys_augmented.A[-1, state_index.dwing_vel_g[1]] = -1

    # Re-order the states: group longitudinal states, lateral states and other
    # states.
    sys_reordered = copy.deepcopy(sys_augmented)
    S_reorder = np.zeros(np.shape(sys_augmented.A))
    augmented_crosswind_states = (
        ['RollRate', 'PitchRate', 'YawRate',
         'TetherRoll', 'AngleOfAttack', 'Yaw',
         'VelocityGroundX', 'Sideslip', 'VelocityGroundZ',
         'PositionGroundX', 'PositionGroundY', 'PositionGroundZ',
         'IntegratedAngleOfAttack', 'IntegratedTetherRoll',
         'IntegratedSideslip'])
    reordered_states = (
        ['PositionGroundZ', 'VelocityGroundZ', 'AngleOfAttack', 'PitchRate',
         'IntegratedAngleOfAttack',
         'TetherRoll', 'Sideslip', 'RollRate', 'YawRate',
         'IntegratedTetherRoll', 'IntegratedSideslip',
         'PositionGroundX', 'VelocityGroundX', 'PositionGroundY', 'Yaw'])
    for i in range(np.size(augmented_crosswind_states)):
      S_reorder[i, augmented_crosswind_states.index(reordered_states[i])] = 1
    sys_reordered.A = S_reorder * sys_augmented.A * np.linalg.inv(S_reorder)
    sys_reordered.B = S_reorder * sys_augmented.B
    sys_reordered.C = sys_augmented.C * np.linalg.inv(S_reorder)

    # Re-order the inputs. Remove inactive inputs.
    sys_temp = copy.deepcopy(sys_reordered)
    sys_reordered.B = np.matrix(
        np.zeros((np.shape(sys_reordered.B)[0],
                  num_longitudinal_inputs + num_lateral_inputs)))
    sys_reordered.D = np.matrix(
        np.zeros((np.shape(sys_reordered.D)[0],
                  num_longitudinal_inputs + num_lateral_inputs)))
    crosswind_inputs = (
        ['Thrust', 'MotorRoll', 'MotorPitch', 'MotorYaw',
         'Aileron', 'Elevator', 'Rudder',
         'WindVelocityGroundX', 'WindVelocityGroundY', 'WindVelocityGroundZ'])
    for i in range(num_longitudinal_inputs):
      idx = crosswind_inputs.index(
          c_helpers.EnumHelper(
              'CrosswindLongitudinalInput', control_types).ShortName(i))
      sys_reordered.B[:, i] = sys_temp.B[:, idx]
      sys_reordered.D[:, i] = sys_temp.D[:, idx]
    for i in range(num_lateral_inputs):
      idx = crosswind_inputs.index(
          c_helpers.EnumHelper(
              'CrosswindLateralInput', control_types).ShortName(i))
      sys_reordered.B[:, num_longitudinal_inputs + i] = sys_temp.B[:, idx]
      sys_reordered.D[:, num_longitudinal_inputs + i] = sys_temp.D[:, idx]

    # Replace the 6 outputs of the open-loop system by 15 outputs equal to the
    # states of the closed-loop system.
    sys_reordered.C = np.eye(sys_reordered.A.shape[0])
    sys_reordered.D = np.zeros(sys_reordered.B.shape)

    # Add actuator dynamics.
    actuator_system = servo_system.append(motor_system.append(
        servo_system.append(servo_system.append(motor_system))))
    crosswind_system = control.ss(sys_reordered.A, sys_reordered.B,
                                  sys_reordered.C, sys_reordered.D)
    crosswind_actuator_system = control.series(actuator_system,
                                               crosswind_system)

    # Create a state-space model for the longitudinal and lateral gains.
    # The state errors for PositionGroundZ and VelocityGroundZ are unused in
    # makani/control/crosswind/crosswind_inner.c. Set the gains associated
    # to these states to zero.
    reduced_longitudinal_gains = copy.copy(longitudinal_gains)
    reduced_longitudinal_gains[
        :, control_types.kCrosswindLongitudinalStatePositionGroundZ] = (
            np.zeros(num_longitudinal_inputs,))
    reduced_longitudinal_gains[
        :, control_types.kCrosswindLongitudinalStateVelocityGroundZ] = (
            np.zeros(num_longitudinal_inputs,))

    sys_gains = _GainsToSystem(reduced_longitudinal_gains).append(
        _GainsToSystem(lateral_gains))

    # Augment the gains system to account for the 4 uncontrolled states:
    # PositionGroundX, VelocityGroundX, PositionGroundY, Yaw.
    sys_gains.B = np.concatenate((sys_gains.B,
                                  np.zeros((np.shape(sys_gains.B)[0], 4))),
                                 axis=1)
    sys_gains.D = np.concatenate((sys_gains.D,
                                  np.zeros((np.shape(sys_gains.D)[0], 4))),
                                 axis=1)
    sys_gains = control.ss(sys_gains.A, sys_gains.B, sys_gains.C, sys_gains.D)

    # Close the loop.
    sys_closed = control.feedback(crosswind_actuator_system, sys_gains)

    return sys_closed


def _GainsToSystem(gains):
  return control.ss(0, np.zeros((1, np.shape(gains)[1])),
                    np.zeros((np.shape(gains)[0], 1)), gains)


def _StateSpaceToDict(state_space):
  return {
      'A': state_space.A,
      'B': state_space.B,
      'C': state_space.C,
      'D': state_space.D
  }


def _IsValidWingSerial(serial, model):
  """Test if the serial name belongs to the wing model."""
  if model == 'oktoberkite':
    return (system_types.WingSerialToModel(serial) ==
            system_types.kWingModelOktoberKite)
  elif model == 'm600':
    return (system_types.WingSerialToModel(serial) ==
            system_types.kWingModelYm600)
  else:
    assert False, 'Invalid wing model "%s".' % model


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n' % e
    sys.exit(1)

  if FLAGS.wing_serial_enum is None:
    wing_serials = range(system_types.kNumWingSerials)
  else:
    wing_serials = [FLAGS.wing_serial_enum]
    if not _IsValidWingSerial(FLAGS.wing_serial_enum, FLAGS.wing_model):
      assert False, ('Wing model "%s" does not have serial number %d (%s).' %
                     (FLAGS.wing_model, FLAGS.wing_serial_enum,
                      _WING_SERIAL_HELPER.ShortName(FLAGS.wing_serial_enum)))

  controllers = []
  for wing_serial in wing_serials:
    if not _IsValidWingSerial(wing_serial, FLAGS.wing_model):
      continue

    wing_serial_name = _WING_SERIAL_HELPER.Name(wing_serial)
    mconfig.WING_MODEL = FLAGS.wing_model

    wing_serial_is_active = mconfig.MakeParams(
        'common.wing_serial_status',
        overrides={'wing_serial': wing_serial},
        override_method='derived')

    if not wing_serial_is_active:
      print('Serial %s of model %s is not active in '
            'common/wing_serial_status.py' %
            (wing_serial_name, FLAGS.wing_model))
      continue

    if FLAGS.overrides:
      overrides = overrides_util.PreprocessOverrides(
          json.loads(FLAGS.overrides))
    else:
      overrides = {}

    overrides['system'] = {'wing_serial': wing_serial}
    params = mconfig.MakeParams('common.all_params',
                                overrides=overrides,
                                override_method='derived')
    crosswind = ControlDesign(params['system'], params['sim'])

    # Build set of run cases.
    if FLAGS.wing_model == 'm600':
      alpha_cmd = 0.07
      radius_trim = 150.0

    # Oktoberkite has a much lower wing incidence model.
    # See b/285483212.
    elif FLAGS.wing_model == 'oktoberkite':
      alpha_cmd = -0.08725
      radius_trim = 90.0

    data = dict({
        'params': params,
        'cases': [
            {'kite_speed': 30.0, 'wind_speed': 5.0, 'alpha_cmd': alpha_cmd},
            {'kite_speed': 60.0, 'wind_speed': 10.0, 'alpha_cmd': alpha_cmd},
            {'kite_speed': 90.0, 'wind_speed': 15.0, 'alpha_cmd': alpha_cmd}
        ]
    })

    # Calculate linearized models and controllers for each run case.
    for case in data['cases']:
      state, inputs = crosswind.CalcTrim(case['kite_speed'],
                                         case['wind_speed'],
                                         case['alpha_cmd'], radius_trim)
      open_full_system = crosswind.LinearizeSystem(state, inputs)
      open_lateral_system = crosswind.GetLateralSystem(open_full_system)
      open_longitudinal_system = crosswind.GetLongitudinalSystem(
          open_full_system)
      servo_system = crosswind.GetServoSystem()
      motor_system = crosswind.GetMotorSystem()

      lateral_gains = crosswind.DesignLateralController(open_lateral_system,
                                                        params['control'])
      closed_lateral_system = crosswind.CloseLateralSystem(
          open_lateral_system, servo_system, motor_system, lateral_gains)

      longitudinal_gains = crosswind.DesignLongitudinalController(
          open_longitudinal_system, params['control'])
      closed_longitudinal_system = crosswind.CloseLongitudinalSystem(
          open_longitudinal_system, servo_system, motor_system,
          longitudinal_gains)

      closed_full_system = crosswind.GetFullClosedLoopSystem(
          open_full_system, servo_system, motor_system, longitudinal_gains,
          lateral_gains)

      if FLAGS.display_trim:
        crosswind.PrintTrim(state, inputs)

      if FLAGS.display_stability:
        crosswind.PrintStability(open_longitudinal_system,
                                 open_lateral_system, open_full_system,
                                 closed_longitudinal_system,
                                 closed_lateral_system, closed_full_system)

      # Store information for exporting.
      case['trim_inputs'] = inputs._asdict()  # pylint: disable=protected-access
      case['trim_state'] = state._asdict()  # pylint: disable=protected-access
      case['trim_flaps'] = crosswind.GetFlapOffsets(inputs)
      case['open_full_system'] = _StateSpaceToDict(open_full_system)
      case['open_lateral_system'] = _StateSpaceToDict(open_lateral_system)
      case['open_longitudinal_system'] = _StateSpaceToDict(
          open_longitudinal_system)
      case['closed_lateral_system'] = _StateSpaceToDict(closed_lateral_system)
      case['closed_longitudinal_system'] = _StateSpaceToDict(
          closed_longitudinal_system)
      case['closed_full_system'] = _StateSpaceToDict(closed_full_system)
      case['servo_system'] = _StateSpaceToDict(servo_system)
      case['motor_system'] = _StateSpaceToDict(motor_system)
      case['lateral_gains'] = lateral_gains
      case['longitudinal_gains'] = longitudinal_gains

      # Select the submatrix corresponding to {aileron, elevator,
      # rudder} --> {p, q, r}_dot.
      state_index = CrosswindState.Tangent.GetIndices()
      inputs_index = CrosswindInputs.GetIndices()
      num_states = CrosswindState.Tangent.GetDim()
      num_inputs = CrosswindInputs.GetDim()
      S = np.matrix(np.zeros((3, num_states)))
      T = np.matrix(np.zeros((num_inputs, 3)))
      S[0, state_index.domega_b[0]] = 1
      S[1, state_index.domega_b[1]] = 1
      S[2, state_index.domega_b[2]] = 1
      T[inputs_index.aileron, 0] = 1
      T[inputs_index.elevator, 1] = 1
      T[inputs_index.rudder, 2] = 1
      case['B_flaps_to_pqr'] = S * case['open_full_system']['B'] * T

    # Write controllers to configuration file.
    controllers.append(collections.OrderedDict([
        ('wing_serial', _WING_SERIAL_HELPER.Name(wing_serial)),
        ('airspeed_table', [case['kite_speed'] for case in data['cases']]),

        # The low airspeed flap trim is used by default in the crosswind
        # controller.  The flap trim does not change significantly with
        # airspeed, and the integrators can handle the difference.
        ('flap_offsets', np.round(
            data['cases'][0]['trim_flaps'], 3).tolist()),
        ('longitudinal_gains_min_airspeed', np.round(
            data['cases'][0]['longitudinal_gains'], 3).tolist()),
        ('longitudinal_gains_nominal_airspeed', np.round(
            data['cases'][1]['longitudinal_gains'], 3).tolist()),
        ('longitudinal_gains_max_airspeed', np.round(
            data['cases'][2]['longitudinal_gains'], 3).tolist()),
        ('lateral_gains_min_airspeed', np.round(
            data['cases'][0]['lateral_gains'], 3).tolist()),
        ('lateral_gains_nominal_airspeed', np.round(
            data['cases'][1]['lateral_gains'], 3).tolist()),
        ('lateral_gains_max_airspeed', np.round(
            data['cases'][2]['lateral_gains'], 3).tolist()),

        # Subset of the output matrix (flaps to pqr).
        ('B_flaps_to_pqr_min_airspeed', np.round(
            data['cases'][0]['B_flaps_to_pqr'], 3).tolist()),
    ]))

  write_controllers.WriteControllers(
      'analysis/control/crosswind.py',
      os.path.join(makani.HOME,
                   'config/%s/control/crosswind_controllers.py' %
                   FLAGS.wing_model),
      controllers)

  # Tranform the list of controllers into a dictionary of controllers.
  all_controllers = {}
  for c in controllers:
    all_controllers[c['wing_serial']] = c

  # Write controllers to JSON.
  if FLAGS.export_json_filename:
    write_controllers.WriteControllersToJson(FLAGS.export_json_filename,
                                             all_controllers)

  # Export linearized models and controllers to MATLAB.
  if FLAGS.export_matlab_filename:
    io.savemat(FLAGS.export_matlab_filename, all_controllers,
               long_field_names=True)


if __name__ == '__main__':
  main(sys.argv)

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

"""Trans-in analysis module."""

from __future__ import absolute_import

import collections
import copy
import os
import sys

import control
import gflags
import makani
import makani.analysis.control
from makani.analysis.control import dynamics
from makani.analysis.control import geometry
from makani.analysis.control import systems
from makani.analysis.control import type_util
from makani.analysis.control import write_controllers
from makani.config import mconfig
from makani.control import control_types
from makani.control import system_types
from makani.lib.python import c_helpers
from makani.lib.python import wing_flag
from makani.sim.physics import physics
import numpy as np
from scipy import optimize

wing_flag.AppeaseLintWhenImportingFlagOnly()
makani.SetRunfilesDirFromBinaryPath()

# pylint doesn't like capital letters in variable names, in contrast
# to control systems conventions.
# pylint: disable=invalid-name

gflags.DEFINE_integer('wing_serial_enum', None,
                      'Enum of the WingSerial to be processed')

FLAGS = gflags.FLAGS


_WING_SERIAL_HELPER = c_helpers.EnumHelper('WingSerial', system_types)


def _DesignSparseController(A, B, Q, R, sparsity_pattern):
  """Minimize the LQR cost with a sparsity constraint.

  Args:
    A: nx-by-nx matrix.
    B: nx-by-nu matrix.
    Q: nx-by-nx positive definite matrix.
    R: nu-by-nu positive definite matrix.
    sparsity_pattern: nu-by-nx matrix.  Positive elements indicate
        gains which are allowed to be non-zero.

  Returns:
    A gain matrix K which stabilizes (A, B) and locally minimizes the
    LQR cost function.  If no such matrix is found an assert is thrown.
  """
  nx, nu = B.shape
  indices = np.ravel_multi_index(np.argwhere(sparsity_pattern > 0.0).T,
                                 (nu, nx))
  def _GainMatrix(x):
    """Populate the gain matrix with its non-zero entries."""
    K = np.zeros((nu * nx,))
    K[indices] = x
    K = np.reshape(K, (nu, nx))
    return K

  def _QuadraticCost(K):
    """Calculate the LQR equivalent cost."""

    # If \dot x(t) = A x(t) + B u(t) is stabilized by u = - K x(t) then
    # for any x(0),
    #
    #   \int_0^\infty ||x(t)||^2 + ||u(t)||^2 = x(0)^T P x(0)
    #
    # where P*(A - B*K) + (A - B*K)' * P = Q + K'*R*K.
    #
    # The variable L is used to ensure K'*R*K is symmetric.
    L = np.dot(K.T, np.linalg.cholesky(R))
    P = control.lyap((A - np.dot(B, K)).T, Q + np.dot(L, L.T))

    try:
      np.linalg.cholesky(P)
    except np.linalg.LinAlgError:
      # P is not positive definite if and only if K is not stabilizing.
      return float('inf')

    # Using trace here places an even weighting on all possible
    # initial conditions.
    return np.trace(P)

  def _CostFunction(x):
    return _QuadraticCost(_GainMatrix(x))

  K, _, _ = control.lqr(A, B, Q, R)
  x_0 = np.reshape(K, (nu * nx,))[indices]
  sol = optimize.minimize(_CostFunction, x_0, method='SLSQP')
  assert sol.success

  K = _GainMatrix(sol.x)
  poles = np.linalg.eig(A - B * K)[0]
  assert np.max(np.real(poles)) < 0.0

  return K


class TransInState(type_util.MakeStateClass(
    'TransInState', [('omega_b', range(0, 3)),
                     ('dcm_ti2b', range(3, 6)),
                     ('wing_vel_ti', range(6, 9)),
                     ('wing_pos_ti', range(9, 12))])):
  """Class representing the state of the wing.

  Attributes:
    omega_b: Body angular rates.
    dcm_ti2b: Trans-in coordinates to body rotation DCM.  Increments
        in the DCM are represented by an Euler vector.
    wing_vel_ti: Velocity of the wing in trans-in coordinates.
    wing_pos_ti: Position of the wing in trans-in coordinates.
  """

  def Increment(self, tangent, step=1.0):
    """Return a state evolved from this state along a tangent direction.

    Args:
      tangent: A WingState.Tangent along which to move.
      step: A scaling of how far to move.

    Returns:
      A new WingState.
    """
    return TransInState(
        omega_b=self.omega_b + step * tangent.domega_b,
        dcm_ti2b=geometry.AxisToDcm(step * tangent.ddcm_ti2b) * self.dcm_ti2b,
        wing_vel_ti=self.wing_vel_ti + step * tangent.dwing_vel_ti,
        wing_pos_ti=self.wing_pos_ti + step * tangent.dwing_pos_ti)

  def Difference(self, other_state):
    """Inverse operation of Increment with a step size of 1.0."""
    return TransInState.Tangent(
        domega_b=other_state.omega_b - self.omega_b,
        ddcm_ti2b=geometry.DcmToAxis(other_state.dcm_ti2b * self.dcm_ti2b.T),
        dwing_vel_ti=other_state.wing_vel_ti - self.wing_vel_ti,
        dwing_pos_ti=other_state.wing_pos_ti - self.wing_pos_ti)


class TransInInputs(type_util.MakeNamedVectorClass(
    'TransInInputs', [('thrust', range(0, 1)),
                      ('motor_moment', range(1, 4)),
                      ('aileron', range(4, 5)),
                      ('flap', range(5, 6)),
                      ('elevator', range(6, 7)),
                      ('rudder', range(7, 8)),
                      ('wind_g', range(8, 11))])):
  """Structure trans-in inputs to the wing.

  Attributes:
    thrust: Motor thrust [N] (1-by-1 np.matrix).
    motor_moment: Motor moments [N-m] (3-by-1 np.matrix).
    aileron: Aileron deflection [rad].
    flap: Flap deflection [rad].
    elevator: Elevator deflection [rad].
    rudder: Rudder deflection [rad].
    wind_g: Wind speed [m/s] in ground coordinates (3-by-1 np.matrix).
  """

  @type_util.RequireMatrixArguments(None, (control_types.kNumFlaps, 1), None)
  def ToWingInputs(self, flap_offsets, midboard_flap_ratio):
    flaps = copy.copy(flap_offsets)
    flaps[control_types.kFlapA1] += -self.aileron
    flaps[control_types.kFlapA2] += (
        -self.aileron + midboard_flap_ratio * self.flap)
    flaps[control_types.kFlapA4] += self.flap
    flaps[control_types.kFlapA5] += self.flap
    flaps[control_types.kFlapA7] += (
        self.aileron + midboard_flap_ratio * self.flap)
    flaps[control_types.kFlapA8] += self.aileron
    flaps[control_types.kFlapEle] += self.elevator
    flaps[control_types.kFlapRud] += self.rudder
    return dynamics.WingInputs(thrust=copy.copy(self.thrust),
                               motor_moment=copy.copy(self.motor_moment),
                               flaps=flaps,
                               wind_g=copy.copy(self.wind_g))


class TransInFrame(object):
  """Representation of the transformation from WingState to TransInState."""

  def __init__(self, dcm_g2ti, ti_origin_g):
    """Constructor.

    Args:
      dcm_g2ti: 3-by-3 np.matrix storing the DCM representing the rotation from
          the g-coordinates to the TI-coordinates.
      ti_origin_g: 3-by-1 np.matrix storing the position [m] of the
          TI-coordinate origin resolved in g coordinates.
    """
    self._dcm_g2ti = copy.copy(dcm_g2ti)
    self._ti_origin_g = copy.copy(ti_origin_g)

  @type_util.RequireMatrixArguments(None, (3, 1))
  def RotateG2Ti(self, vec_g):
    return self._dcm_g2ti * vec_g

  def StateTangentFromWingStateTangent(self, unused_wing_state, tangent):
    """Convert a WingState.Tangent to a TransInState.Tangent."""
    return TransInState.Tangent(
        domega_b=tangent.domega_b,
        ddcm_ti2b=tangent.ddcm_g2b,
        dwing_vel_ti=self._dcm_g2ti * tangent.dwing_vel_g,
        dwing_pos_ti=self._dcm_g2ti * tangent.dwing_pos_g)

  def StateToWingState(self, state):
    """Convert a TransInState to a WingState."""
    return dynamics.WingState(
        omega_b=state.omega_b,
        dcm_g2b=state.dcm_ti2b * self._dcm_g2ti,
        wing_pos_g=self._ti_origin_g + self._dcm_g2ti.T * state.wing_pos_ti,
        wing_vel_g=self._dcm_g2ti.T * state.wing_vel_ti)


class ControlDesign(object):
  """Class for trimming and calculating gains for trans-in."""

  def __init__(self, system_params, control_params, sim_params):
    """Constructs a ControlDesign object from parameters."""
    self._tether_length = system_params['tether']['length']

    # Determine the path parameters.
    ti_origin_g = np.matrix([[0.0], [0.0], [0.0]])

    # We currently assume no wind in the trans-in script, so the trans-in start
    # azimuth should not matter. A starting azimuth should be specified if we
    # want to account for wind speed in this script.
    # TODO: Use playbook entry to calculate accel start azimuth.
    dcm_g2ti = geometry.AngleToDcm(0.0, 0.0, 0.0)
    self._trans_in_frame = TransInFrame(dcm_g2ti, ti_origin_g)

    self._wing_area = system_params['wing']['A']
    self._wing_span = system_params['wing']['b']
    self._wing_chord = system_params['wing']['c']
    self._air_density = system_params['phys']['rho']

    rotor_databases = [
        physics.RotorDatabase(physics.GetRotorDatabase(
            sim_params['rotor_sim']['database_names'][i]['name']))
        for i in range(control_types.kNumMotors)
    ]
    self._motor_model = dynamics.MotorMixerMotorModel(
        rotor_databases, self._air_density,
        control_params['trans_in']['output']['thrust_moment_weights'],
        system_params['rotors'], control_params['rotor_control'])

    self._thrust_cmd = control_params['trans_in']['longitudinal']['thrust_cmd']

    if system_params['gs_model'] == system_types.kGroundStationModelTopHat:
      gsg_pos_g = (np.array(system_params['perch']['winch_drum_origin_p'])
                   + np.array(system_params['perch']['gsg_pos_wd']))
    elif system_params['gs_model'] == system_types.kGroundStationModelGSv2:
      gs02_params = system_params['ground_station']['gs02']
      # We assume the platform frame is aligned with the g-frame, i.e. the
      # platform azimuth is zero.
      gsg_pos_g = (np.array(gs02_params['drum_origin_p'])
                   + np.array(gs02_params['gsg_pos_drum']))
    else:
      assert False, 'Invalid GS model.'

    tether_model = dynamics.CatenaryTetherForceModel(
        system_params['tether'], gsg_pos_g, system_params['wing']['bridle_rad'],
        system_params['phys']['g'], system_params['phys']['rho'])

    self._wing = dynamics.Wing(
        system_params, sim_params, dynamics.SwigAeroModel(),
        self._motor_model, tether_model)

    self._wing_weight = system_params['wing']['m'] * system_params['phys']['g']
    self._motor_yaw_lever_arm = np.abs(
        system_params['rotors'][control_types.kMotorSbo]['pos'][1]
        + system_params['rotors'][control_types.kMotorSbi]['pos'][1]) / 2.0

    flap_offsets = np.deg2rad(np.array([
        -11.5, -11.5, -11.5, -11.5, -11.5, -11.5, 0.0, 0.0
    ]))
    assert len(flap_offsets) == 8

    self._midboard_flap_ratio = control_params['trans_in']['attitude'][
        'midboard_flap_ratio']
    self._flap_offsets = np.matrix(np.zeros((control_types.kNumFlaps, 1)))
    self._flap_offsets[control_types.kFlapA1] = flap_offsets[0]
    self._flap_offsets[control_types.kFlapA2] = flap_offsets[1]
    self._flap_offsets[control_types.kFlapA4] = flap_offsets[2]
    self._flap_offsets[control_types.kFlapA5] = flap_offsets[3]
    self._flap_offsets[control_types.kFlapA7] = flap_offsets[4]
    self._flap_offsets[control_types.kFlapA8] = flap_offsets[5]
    self._flap_offsets[control_types.kFlapEle] = flap_offsets[6]
    self._flap_offsets[control_types.kFlapRud] = flap_offsets[7]

  def StateToWingState(self, state):
    return self._trans_in_frame.StateToWingState(state)

  def InputsToWingInputs(self, inputs):
    return inputs.ToWingInputs(self._flap_offsets, self._midboard_flap_ratio)

  def CalcTrim(self, wind_speed, aero_climb_angle, on_tether,
               angle_of_attack=None):
    """Find trim conditions for the wing.

    Determines control and attitude trim.  The table below provides a
    rough idea of the expected trim relationships.

        Trim Input  | Trim Output
        ------------+----------------------
        aileron     | roll moment
        elevator    | pitch moment
        rudder      | yaw moment
        velocity    | x-body acceleration
        yaw         | y-body acceleration
        pitch       | z-body acceleration
        climb angle | aerodynamic climb angle, or angle of attack if given.

    Thrust level is determined by the limits imposed by MixRotors.

    Args:
      wind_speed: Speed [m/s] of the wind.
      aero_climb_angle: Aerodynamic climb angle [rad].
      on_tether: True if the kite should be modeled with high tether tension.
      angle_of_attack: Trim angle-of-attack [rad].  If not None, angle-of-attack
          is trimmed instead of climb angle.

    Returns:
      A tuple (state, inputs) where state is a TransInState and inputs is a
      TransInInputs.
    """
    elevation_angle_ti = np.pi / 6.0
    climb_angle_0 = aero_climb_angle
    wing_vel_ti_0 = 20.0 * np.matrix([[np.cos(climb_angle_0)],
                                      [0.0], [-np.sin(climb_angle_0)]])
    def _GetState(omega_b=None, yaw=0.0, pitch=0.0, roll=0.0,
                  dwing_vel_ti_x=0.0, dwing_vel_ti_z=0.0):
      """Produce a TransInState with default values."""
      if omega_b is None:
        omega_b = np.matrix(np.zeros((3, 1)))

      dcm_ti2b = geometry.AngleToDcm(yaw, pitch, roll)

      wing_vel_ti = wing_vel_ti_0 + [[dwing_vel_ti_x], [0.0], [dwing_vel_ti_z]]

      if on_tether:
        radial_pos_ti = self._tether_length + 4.0
      else:
        radial_pos_ti = self._tether_length - 5.0

      wing_pos_ti = radial_pos_ti * np.matrix([[-np.cos(elevation_angle_ti)],
                                               [0.0],
                                               [-np.sin(elevation_angle_ti)]])

      return TransInState(omega_b=omega_b, dcm_ti2b=dcm_ti2b,
                          wing_vel_ti=wing_vel_ti, wing_pos_ti=wing_pos_ti)

    def _GetInputs(aileron=0.0, elevator=0.0, rudder=0.0):
      return TransInInputs(
          thrust=np.matrix([[self._thrust_cmd]]),
          motor_moment=np.matrix(np.zeros((3, 1))),
          aileron=np.matrix([[aileron]]),
          flap=np.matrix([[0.0]]),
          elevator=np.matrix([[elevator]]),
          rudder=np.matrix([[rudder]]),
          wind_g=np.matrix([[-wind_speed], [0.0], [0.0]]))

    def _GetOutputs(state, inputs):
      """Calculate outputs."""
      wing_state = self.StateToWingState(state)
      wing_inputs = self.InputsToWingInputs(inputs)

      v_rel_ti = (
          state.wing_vel_ti -
          self._trans_in_frame.RotateG2Ti(wing_inputs.wind_g))
      v_rel, alpha, _ = wing_state.CalcAerodynamicAngles(wing_inputs.wind_g)
      gamma_aero = np.arcsin(-v_rel_ti[2, 0] / v_rel)

      state_dot = self._trans_in_frame.StateTangentFromWingStateTangent(
          wing_state, self._wing.CalcDeriv(wing_state, wing_inputs))

      return gamma_aero, alpha, state_dot

    def _AttitudeTrimFunction(x):
      """Use pitch and yaw to set angle-of-attack and angle-of-sideslip."""
      state = _GetState(yaw=x[1], pitch=x[0])
      inputs = _GetInputs()
      wing_state = self.StateToWingState(state)
      _, alpha, beta = wing_state.CalcAerodynamicAngles(inputs.wind_g)

      return [
          alpha - (0.0 if angle_of_attack is None else angle_of_attack),
          beta
      ]

    def _AirspeedTrimFunction(x):
      """Adjust the AOA and airspeed for longitudinal force balance."""
      state = _GetState(pitch=x[0], dwing_vel_ti_x=x[1], dwing_vel_ti_z=x[2])
      inputs = _GetInputs()
      gamma_aero, _, state_dot = _GetOutputs(state, inputs)

      return [
          state_dot.dwing_vel_ti[0, 0],
          state_dot.dwing_vel_ti[2, 0],
          gamma_aero - aero_climb_angle
      ]

    def _FlapTrimFunction(x):
      """Use aileron, elevator and rudder to zero the angular accelerations."""
      state = _GetState(yaw=x_attitude[1], pitch=x_attitude[0],
                        dwing_vel_ti_x=x_airspeed[1, 0],
                        dwing_vel_ti_z=x_airspeed[2, 0])
      inputs = _GetInputs(aileron=x[0], elevator=x[1], rudder=x[2])
      _, _, state_dot = _GetOutputs(state, inputs)

      return [
          state_dot.domega_b[0, 0],
          state_dot.domega_b[1, 0],
          state_dot.domega_b[2, 0]
      ]

    def _GetStateAndInputs(x):
      """Returns the trim state given the trim input variables."""
      state = _GetState(
          roll=x[0], pitch=x[1], yaw=x[2],
          dwing_vel_ti_x=x[6], dwing_vel_ti_z=x[7])
      inputs = _GetInputs(aileron=x[3], elevator=x[4], rudder=x[5])

      return state, inputs

    def _TrimFunction(x):
      """Wrapper function for trimming the wing."""
      state, inputs = _GetStateAndInputs(x)
      gamma_aero, alpha, state_dot = _GetOutputs(state, inputs)

      residuals = [
          gamma_aero - aero_climb_angle,
          state_dot.dwing_vel_ti[0, 0],
          state_dot.dwing_vel_ti[1, 0],
          state_dot.dwing_vel_ti[2, 0],
          state_dot.domega_b[0, 0],
          state_dot.domega_b[1, 0],
          state_dot.domega_b[2, 0]
      ]

      if angle_of_attack is not None:
        residuals[0] = alpha - angle_of_attack

      return residuals

    def _CheckTrimResidual(x):
      """Make sure the trim residuals aren't too large."""
      # In some cases, for reasons that are not well-understood, it isn't
      # possible to reach a perfect equilibrium. Consequently, we require only
      # that the state the minimizer has reached is a reasonable approximation
      # of an equilibrium. The bounds here are heuristic.
      residual_bounds = [
          ('climb angle' if angle_of_attack is None else 'alpha',
           np.deg2rad(0.05)),
          ('x-acceleration', 2e-4),
          ('y-acceleration', 2e-4),
          ('z-acceleration', 2e-4),
          ('dp/dt', 2e-4),
          ('dq/dt', 2.1e-4),
          ('dr/dt', 2e-4),
      ]

      residual = _TrimFunction(x)
      for value, (label, bound) in zip(residual, residual_bounds):
        assert abs(value) < bound, (
            'Trim residual "%s" is %g; magnitude exceeds bound of %g.'
            % (label, value, bound))

    x_attitude = np.matrix(optimize.fsolve(
        _AttitudeTrimFunction, np.zeros((2, 1)))).T

    x_airspeed = np.matrix(optimize.fsolve(
        _AirspeedTrimFunction, [x_attitude[0, 0], 0.0, 0.0])).T

    x_attitude[0, 0] = x_airspeed[0, 0]

    x_flaps = np.matrix(optimize.fsolve(
        _FlapTrimFunction, np.zeros((3, 1)))).T

    # Minimize the trim function while preferring small roll angles.
    objective = lambda x: np.linalg.norm(_TrimFunction(x)) + x[0]**2

    x0 = np.vstack((0.0, x_attitude, x_flaps, x_airspeed[1:, :]))
    bounds = [
        np.deg2rad([-5.0, 5.0]),  # Roll
        np.deg2rad([40.0, 60.0]),  # Pitch
        np.deg2rad([-8.0, 8.0]),  # Yaw,
        np.deg2rad([-8.5, 8.5]),  # Aileron
        np.deg2rad([-10.0, 15.0]),  # Elevator
        np.deg2rad([-22.0, 22.0]),  # Rudder
        (0.0, 20.0),  # x-velocity offset
        (-20.0, 0.0),  # z-velocity offset
    ]

    result = optimize.minimize(objective, x0, method='SLSQP', bounds=bounds,
                               options={'maxiter': 200})
    assert result.success, 'Failed to converge: %s' % result
    _CheckTrimResidual(result.x)

    state, inputs = _GetStateAndInputs(result.x)

    wing_state = self.StateToWingState(state)
    wing_inputs = self.InputsToWingInputs(inputs)
    _, alpha, beta = wing_state.CalcAerodynamicAngles(wing_inputs.wind_g)

    # Apply a final check to the angle-of-attack and angle-of-sideslip.
    assert alpha > -0.15 and alpha < 0.05 and beta > -0.1 and beta < 0.1, (
        'alpha=%g, beta=%g' % (alpha, beta))

    return state, inputs

  def CalcLiftAndDragCoeffs(self, state, inputs):
    """Calculate the lift and drag coefficients."""

    wing_inputs = self.InputsToWingInputs(inputs)
    wing_state = self.StateToWingState(state)

    v_rel, alpha, beta = wing_state.CalcAerodynamicAngles(wing_inputs.wind_g)
    # Fixing total thrust coefficient to 0.0 for this application.
    thrust_coeff = 0.0
    _, cf_b, _ = self._wing.CalcAeroForceMomentPos(
        v_rel, alpha, beta, wing_state.omega_b, wing_inputs.flaps, thrust_coeff)

    c_cf_b = physics.Vec3()
    c_cf_b.x = cf_b[0, 0]
    c_cf_b.y = cf_b[1, 0]
    c_cf_b.z = cf_b[2, 0]
    c_cf_w = physics.Vec3()
    physics.RotBToW(c_cf_b.this, alpha, beta, c_cf_w.this)

    return -c_cf_w.z, -c_cf_w.x

  def PrintTrim(self, state, inputs):
    """Print information relevant to a trimmed state.

    Args:
      state: TransInState structure.
      inputs: dynamics.WingInputs structure.
    """
    wing_inputs = self.InputsToWingInputs(inputs)
    wing_state = self.StateToWingState(state)
    trans_in_state_dot = self._trans_in_frame.StateTangentFromWingStateTangent(
        wing_state, self._wing.CalcDeriv(wing_state, wing_inputs))

    (v_rel, alpha, beta) = wing_state.CalcAerodynamicAngles(inputs.wind_g)
    climb_angle_g = np.arctan2(-wing_state.wing_vel_g[2, 0],
                               wing_state.wing_vel_g[0, 0])
    tether_force_b = (wing_state.dcm_g2b
                      * self._wing.CalcTetherForceG(wing_state, inputs))

    yaw, pitch, roll = geometry.DcmToAngle(state.dcm_ti2b)

    CL, CD = self.CalcLiftAndDragCoeffs(state, inputs)

    rotor_speeds = self._motor_model.CalcRotorSpeeds(
        v_rel, wing_state.omega_b, wing_inputs.thrust, inputs.motor_moment)

    motor_force_moment_pos = self._motor_model.CalcMotorForceMomentPos(
        v_rel, alpha, beta, wing_state.omega_b, wing_inputs.thrust,
        inputs.motor_moment)

    values = [[
        ('Roll [deg]', (180.0 / np.pi) * roll),
        ('Pitch [deg]', (180.0 / np.pi) * pitch),
        ('Yaw [deg]', (180.0 / np.pi) * yaw)
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
        ('Rotor %d Speed [rad/s]' % i, rotor_speeds[i])
        for i in range(control_types.kNumMotors)
    ], [
        ('Motor Thrust [kN]', motor_force_moment_pos.force[0, 0] / 1000.0),
        ('Motor Roll [kN-m]', motor_force_moment_pos.moment[0, 0] / 1000.0),
        ('Motor Pitch [kN-m]', motor_force_moment_pos.moment[1, 0] / 1000.0),
        ('Motor Yaw [kN-m]', motor_force_moment_pos.moment[2, 0] / 1000.0),
    ], [
        ('Vrel [m/s]', v_rel),
        ('Alpha [deg]', (180.0 / np.pi) * alpha),
        ('Beta [deg]', (180.0 / np.pi) * beta),
        ('CL [#]', CL),
        ('CD [#]', CD),
        ('Climb Angle [deg]', (180.0 / np.pi) * climb_angle_g)
    ], [
        ('Tether Force Xb [N]', tether_force_b[0]),
        ('Tether Force Yb [N]', tether_force_b[1]),
        ('Tether Force Zb [N]', tether_force_b[2]),
    ], [
        ('Pdot [rad/s^2]', trans_in_state_dot.domega_b[0]),
        ('Qdot [rad/s^2]', trans_in_state_dot.domega_b[1]),
        ('Rdot [rad/s^2]', trans_in_state_dot.domega_b[2])
    ], [
        ('A_ti X [m/s^2]', trans_in_state_dot.dwing_vel_ti[0]),
        ('A_ti Y [m/s^2]', trans_in_state_dot.dwing_vel_ti[1]),
        ('A_ti Z [m/s^2]', trans_in_state_dot.dwing_vel_ti[2])
    ]]

    for line_values in values:
      for (name, value) in line_values:
        print '%22s: %10.3f' % (name, value)

  def GetLinearizedModel(self, trim_state, trim_inputs):
    """Extracts a linearized model of the whole wing in TI coordinates.

    In addition to calculating a linearization of the wing state-space model,
    the wing position and velocity are rotated into trans-in coordinates.

    Args:
      trim_state: TransInState to linearize about.
      trim_inputs: dynamics.WingInputs to linearize about.

    Returns:
      An LTI model describing the wing with position and velocity rotated into
      trans-in coordinates.
    """
    def _CalcDeriv(state, inputs):
      wing_state = self.StateToWingState(state)
      wing_inputs = self.InputsToWingInputs(inputs)
      return self._trans_in_frame.StateTangentFromWingStateTangent(
          wing_state, self._wing.CalcDeriv(wing_state, wing_inputs)).ToVector()

    def _CalcOutputs(state, inputs):
      wing_state = self.StateToWingState(state)
      wing_inputs = self.InputsToWingInputs(inputs)
      (v, alpha, beta) = wing_state.CalcAerodynamicAngles(wing_inputs.wind_g)
      tether_force_g = self._wing.CalcTetherForceG(wing_state, wing_inputs)
      tether_force_b = wing_state.dcm_g2b * tether_force_g

      return np.vstack((np.matrix([[v], [alpha], [beta]]),
                        tether_force_b))

    # Calculate step sizes for linearization.
    #
    # TODO: The step size for all fields used to be 1e-6. Only the
    # motor moment inputs are currently different in order to work around a test
    # failure; see b/36783475#comment19. Consider rescaling other step sizes as
    # well.
    state_step_sizes = trim_state.Tangent.StepVector(
        {'domega_b': 1e-6, 'ddcm_ti2b': 1e-6,
         'dwing_vel_ti': 1e-6, 'dwing_pos_ti': 1e-6})
    input_step_sizes = trim_inputs.StepVector(
        {'thrust': 1e-6, 'motor_moment': 20.0, 'aileron': 1e-6,
         'flap': 1e-6, 'elevator': 1e-6, 'rudder': 1e-6, 'wind_g': 1e-6})

    # Calculate linearized model.
    (A, B) = dynamics.CalcLinearization(_CalcDeriv, trim_state, trim_inputs,
                                        state_step_sizes, input_step_sizes)
    (C, D) = dynamics.CalcLinearization(_CalcOutputs, trim_state, trim_inputs,
                                        state_step_sizes, input_step_sizes)

    output_names = [
        'airspeed', 'angle_of_attack', 'angle_of_sideslip',
        'tether_force_b_x', 'tether_force_b_y', 'tether_force_b_z'
    ]

    input_names = [None for _ in range(TransInInputs.GetDim())]
    input_indices = TransInInputs.GetIndices()

    input_names[input_indices.thrust[0]] = 'thrust'
    input_names[input_indices.motor_moment[0]] = 'motor_roll'
    input_names[input_indices.motor_moment[1]] = 'motor_pitch'
    input_names[input_indices.motor_moment[2]] = 'motor_yaw'
    input_names[input_indices.aileron[0]] = 'delta_aileron'
    input_names[input_indices.flap[0]] = 'delta_flap'
    input_names[input_indices.elevator[0]] = 'delta_elevator'
    input_names[input_indices.rudder[0]] = 'delta_rudder'
    input_names[input_indices.wind_g[0]] = 'wind_g_x'
    input_names[input_indices.wind_g[1]] = 'wind_g_y'
    input_names[input_indices.wind_g[2]] = 'wind_g_z'

    state_names = [None for _ in range(TransInState.Tangent.GetDim())]
    state_indices = TransInState.Tangent.GetIndices()

    state_names[state_indices.domega_b[0]] = 'roll_rate'
    state_names[state_indices.domega_b[1]] = 'pitch_rate'
    state_names[state_indices.domega_b[2]] = 'yaw_rate'

    state_names[state_indices.ddcm_ti2b[0]] = 'phi_b_x'
    state_names[state_indices.ddcm_ti2b[1]] = 'phi_b_y'
    state_names[state_indices.ddcm_ti2b[2]] = 'phi_b_z'

    state_names[state_indices.dwing_pos_ti[0]] = 'pos_x'
    state_names[state_indices.dwing_pos_ti[1]] = 'pos_y'
    state_names[state_indices.dwing_pos_ti[2]] = 'pos_z'

    state_names[state_indices.dwing_vel_ti[0]] = 'vel_x'
    state_names[state_indices.dwing_vel_ti[1]] = 'vel_y'
    state_names[state_indices.dwing_vel_ti[2]] = 'vel_z'

    output_names += state_names
    C = np.vstack((C, np.eye(len(state_names))))
    D = np.vstack((D, np.zeros((len(state_names), len(input_names)))))

    return systems.System(A, B, C, D, 0.0,
                          systems.SignalList(state_names),
                          systems.SignalList(input_names),
                          systems.SignalList(output_names))

  def _GetLateralAttitudeSystem(self, system):
    """Sub-select a system modeling the lateral attitude dynamics."""
    output_names = ['angle_of_sideslip']
    A, B, C, D, Ts = system[output_names, :].GetStateSpaceModel()

    # Replace vel_y with angle_of_sideslip.
    S = np.eye(system.nx)
    aos_index = system.states.GetIndices(['vel_y'])[0]
    S[aos_index, :] = C[0, :]
    S_inv = np.linalg.inv(S)
    A = S * A * S_inv
    B = S * B
    C = C * S_inv  # pylint: disable=g-no-augmented-assignment

    # Add the roll error integrator.
    A = np.hstack((np.vstack((A, np.zeros((1, A.shape[0])))),
                   np.zeros((A.shape[0] + 1, 1))))
    A[-1, system.states.GetIndices(['phi_b_x'])] = 1.0
    B = np.vstack((B, np.zeros((1, B.shape[1]))))
    C = np.hstack((C, np.zeros((C.shape[0], 1))))

    states = ['angle_of_sideslip' if state_name == 'vel_y' else state_name
              for state_name in system.states.names]
    states.append('int_phi_b_x')

    lat_sys = systems.System(A, B, C, D, Ts, states, system.inputs,
                             output_names)
    state_names = [
        None for _ in range(control_types.kNumTransInLateralStates)
    ]
    state_names[control_types.kTransInLateralStateRoll] = 'phi_b_x'
    state_names[control_types.kTransInLateralStateYaw] = 'phi_b_z'
    state_names[control_types.kTransInLateralStateRollRate] = 'roll_rate'
    state_names[control_types.kTransInLateralStateYawRate] = 'yaw_rate'
    state_names[control_types.kTransInLateralStateIntRoll] = 'int_phi_b_x'
    state_names[control_types.kTransInLateralStateAngleOfSideslip] = (
        'angle_of_sideslip')

    input_names = [None for _ in range(control_types.kNumTransInLateralInputs)]
    input_names[control_types.kTransInLateralInputAileron] = 'delta_aileron'
    input_names[control_types.kTransInLateralInputRudder] = 'delta_rudder'
    input_names[control_types.kTransInLateralInputMotorYaw] = 'motor_yaw'

    return lat_sys.ReduceStates(state_names)[:, input_names]

  def DesignLateralAttitudeController(self, system, input_weight,
                                      Q=None, R=None):
    """Selects a gain matrix for the attitude control."""
    lat_sys = self._GetLateralAttitudeSystem(system)

    # Apply Bryson's rule for cost selection if none specified.
    if Q is None:
      state_max = np.zeros((control_types.kNumTransInLateralStates,))
      state_max[control_types.kTransInLateralStateRoll] = 0.2
      state_max[control_types.kTransInLateralStateYaw] = 0.2
      state_max[control_types.kTransInLateralStateRollRate] = 0.2
      state_max[control_types.kTransInLateralStateYawRate] = 0.1
      state_max[control_types.kTransInLateralStateIntRoll] = 0.05
      state_max[control_types.kTransInLateralStateAngleOfSideslip] = 0.1
      Q = np.diag(1.0 / state_max**2.0)

    if R is None:
      input_max = np.zeros((control_types.kNumTransInLateralInputs,))
      input_max[control_types.kTransInLateralInputAileron] = 0.125
      input_max[control_types.kTransInLateralInputRudder] = 0.2
      input_max[control_types.kTransInLateralInputMotorYaw] = (
          0.25 * self._wing_weight * self._motor_yaw_lever_arm)
      R = np.diag((input_weight / input_max)**2.0)

    A, B, _, _, _ = lat_sys.GetStateSpaceModel()

    sparsity_pattern = np.ones((lat_sys.nu, lat_sys.nx))
    sparsity_pattern[:, control_types.kTransInLateralStateAngleOfSideslip] = 0.0
    sparsity_pattern[control_types.kTransInLateralInputMotorYaw,
                     control_types.kTransInLateralStateRoll] = 0.0
    sparsity_pattern[control_types.kTransInLateralInputMotorYaw,
                     control_types.kTransInLateralStateRollRate] = 0.0
    sparsity_pattern[control_types.kTransInLateralInputMotorYaw,
                     control_types.kTransInLateralStateIntRoll] = 0.0

    K = _DesignSparseController(A, B, Q, R, sparsity_pattern)

    return K

  def _GetLongitudinalAttitudeSystem(self, system):
    """Sub-select a system modeling the longitudinal attitude dynamics."""
    # Add the angle-of-attack integrator.
    outputs = ['angle_of_attack']
    A, B, C, D, _ = system[outputs, :].GetStateSpaceModel()

    A = np.hstack((np.vstack((A, C)), np.zeros((A.shape[0] + 1, 1))))
    B = np.vstack((B, np.zeros((1, B.shape[1]))))
    C = np.hstack((C, np.zeros((C.shape[0], 1))))

    long_sys = systems.System(
        A, B, C, D, 0.0,
        system.states + systems.SignalList(['int_angle_of_attack']),
        system.inputs, outputs)

    state_names = [
        None for _ in range(control_types.kNumTransInLongitudinalStates)
    ]
    state_names[control_types.kTransInLongitudinalStatePitch] = 'phi_b_y'
    state_names[control_types.kTransInLongitudinalStatePitchRate] = 'pitch_rate'
    state_names[control_types.kTransInLongitudinalStateIntAngleOfAttack] = (
        'int_angle_of_attack')

    input_names = [
        None for _ in range(control_types.kNumTransInLongitudinalInputs)
    ]
    input_names[control_types.kTransInLongitudinalInputMotorPitch] = (
        'motor_pitch')
    input_names[control_types.kTransInLongitudinalInputElevator] = (
        'delta_elevator')

    return long_sys.ReduceStates(state_names)[:, input_names]

  def DesignLongitudinalAttitudeController(self, system, input_weight):
    """Selects a gain matrix for the longitudinal control system."""
    state_max = np.zeros((control_types.kNumTransInLongitudinalStates,))
    state_max[control_types.kTransInLongitudinalStatePitch] = 0.2
    state_max[control_types.kTransInLongitudinalStatePitchRate] = 0.25
    state_max[control_types.kTransInLongitudinalStateIntAngleOfAttack] = 0.3

    input_max = np.zeros((control_types.kNumTransInLongitudinalInputs,))
    input_max[control_types.kTransInLongitudinalInputMotorPitch] = 1250.0
    input_max[control_types.kTransInLongitudinalInputElevator] = 0.16

    # Apply Bryson's rule for cost selection.
    Q = np.diag(1.0 / state_max**2.0)
    R = np.diag((input_weight / input_max)**2.0)

    long_sys = self._GetLongitudinalAttitudeSystem(system)
    A, B, _, _, _ = long_sys.GetStateSpaceModel()
    K, _, _ = control.lqr(A, B, Q, R)

    return K

  def CalcLiftCoeffPerDeltaFlap(self, state, inputs_0):
    """Calculate the change in CL per change in delta_flap."""
    h = 0.01
    inputs = copy.deepcopy(inputs_0)
    inputs.flap[0, 0] += h
    CL_p, _ = self.CalcLiftAndDragCoeffs(state, inputs)

    inputs = copy.deepcopy(inputs_0)
    inputs.flap[0, 0] -= h
    CL_n, _ = self.CalcLiftAndDragCoeffs(state, inputs)

    return (CL_p - CL_n) / (2.0 * h)

  def CalcElevatorFeedforward(self, wind_speed, aero_climb_angle):
    """Calculate a linear fit from elevator angle to trim angle-of-attack."""

    def GetDeltaEleAndAlpha(alpha_in):
      state, inputs = self.CalcTrim(
          wind_speed, aero_climb_angle, True, angle_of_attack=alpha_in)
      wing_state = self.StateToWingState(state)
      wing_inputs = self.InputsToWingInputs(inputs)

      delta_ele = inputs.elevator[0, 0]
      _, alpha_out, _ = wing_state.CalcAerodynamicAngles(wing_inputs.wind_g)
      return delta_ele, alpha_out

    # The dominant effect on the fit is which section of the (piecewise linear)
    # aero model we hit. The sample points shouldn't span any corners.
    #
    # The input alphas are currently chosen to meet two criteria:
    #   1. In the -5 deg <= alpha <= 0 deg segment of the aero model.
    #   2. Below -3.5 degrees, above which the trim calculation has trouble
    #      converging with the Gen4 rotor model. (This issue is not
    #      well-understood.)
    # Since the trim is approximate, the alphas used in the finite difference
    # are from the output of CalcTrim.
    delta_ele_0, alpha_0 = GetDeltaEleAndAlpha(np.deg2rad(-4.0))
    delta_ele_1, alpha_1 = GetDeltaEleAndAlpha(np.deg2rad(-3.5))

    ddelta_elevator_dalpha = (delta_ele_1 - delta_ele_0) / (alpha_1 - alpha_0)
    delta_elevator_alpha_zero = delta_ele_0 - ddelta_elevator_dalpha * alpha_0

    return delta_elevator_alpha_zero, ddelta_elevator_dalpha


def CheckAeroSimParams(all_params):
  """Check that aero sim params are properly configured for this script."""

  aero_sim_params = all_params['sim']['aero_sim']
  dbs = aero_sim_params['small_deflection_databases']

  # Make sure that:
  #   - merge_databases is False
  #   - The zero angular rate database is in use and is the only small
  #     deflection database being used in config/m600/sim/aero_sim.py.)
  # See https://goo.gl/QAYQf4.
  ok = (not aero_sim_params['merge_databases']
        and dbs[0]['name'] == 'm600/m600_aswing_baseline_zero_angular_rate.json'
        and all([not db['name'] for db in dbs[1:]]))

  assert ok, ('Modify aero_sim.py to meet the criteria in this function, '
              'then rebuild and re-run this script. Then revert those '
              'changes to aero_sim.py before pushing to Gerrit.')


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

    mconfig.WING_MODEL = FLAGS.wing_model

    wing_serial_is_active = mconfig.MakeParams(
        'common.wing_serial_status', overrides={'wing_serial': wing_serial},
        override_method='derived')
    if not wing_serial_is_active:
      continue

    print '\nWing serial: %s\n' % (_WING_SERIAL_HELPER.Name(wing_serial))
    overrides = {'system': {'wing_serial': wing_serial}}
    all_params = mconfig.MakeParams('common.all_params', overrides=overrides,
                                    override_method='derived')
    CheckAeroSimParams(all_params)

    trans_in = ControlDesign(all_params['system'],
                             all_params['control'],
                             all_params['sim'])

    # TODO: Consider the impact of nonzero wind speed, with updates
    # to the tether model to include wind-related drag.
    wind_speed = 0.0
    aero_climb_angle = np.deg2rad(50.0)

    # The low tension trim is used to set the trim roll, yaw, beta, and
    # flap positions.
    (state, inputs) = trans_in.CalcTrim(wind_speed, aero_climb_angle, False)
    print 'Low tension trim:'
    trans_in.PrintTrim(state, inputs)
    wing_inputs = trans_in.InputsToWingInputs(inputs)
    wing_state = trans_in.StateToWingState(state)

    # Calculate lateral trim variables.
    _, _, beta = wing_state.CalcAerodynamicAngles(wing_inputs.wind_g)
    angle_of_sideslip_cmd = beta
    yaw_ti_cmd, _, roll_ti_cmd = geometry.DcmToAngle(state.dcm_ti2b)

    # Calculate flap offsets.
    flap_offsets = copy.deepcopy(wing_inputs.flaps)
    # The elevator trim is handled separately by
    # delta_elevator_alpha_zero and ddelta_elevator_dalpha.
    flap_offsets[control_types.kFlapEle] = 0.0

    # Calculate dCL_dflap for the longitudinal controller.
    dCL_dflap = trans_in.CalcLiftCoeffPerDeltaFlap(state, inputs)

    # Calculate the low tension gains.
    system = trans_in.GetLinearizedModel(state, inputs)
    lat_gains_low_tension = trans_in.DesignLateralAttitudeController(
        system, 1.5)
    long_gains_low_tension = trans_in.DesignLongitudinalAttitudeController(
        system, 1.0)

    # Calculate the high tension gains.
    (state, inputs) = trans_in.CalcTrim(wind_speed, aero_climb_angle, True)
    print '\nHigh tension trim:'
    trans_in.PrintTrim(state, inputs)

    system = trans_in.GetLinearizedModel(state, inputs)
    lat_gains_high_tension = trans_in.DesignLateralAttitudeController(
        system, 1.5)
    long_gains_high_tension = trans_in.DesignLongitudinalAttitudeController(
        system, 1.0)

    # Calculate the pitch forward gains.
    lat_gains_pitch_forward = copy.deepcopy(lat_gains_low_tension)
    lat_gains_pitch_forward[control_types.kTransInLateralInputAileron,
                            control_types.kTransInLateralStateRoll] = 0.0
    lat_gains_pitch_forward[control_types.kTransInLateralInputAileron,
                            control_types.kTransInLateralStateYaw] = 0.0
    lat_gains_pitch_forward[control_types.kTransInLateralInputRudder,
                            control_types.kTransInLateralStateRoll] = 0.0
    lat_gains_pitch_forward[control_types.kTransInLateralInputRudder,
                            control_types.kTransInLateralStateYaw] = 0.0

    # Calculate the linear model of alpha to elevator deflection.
    (delta_elevator_alpha_zero,
     ddelta_elevator_dalpha) = trans_in.CalcElevatorFeedforward(
         wind_speed, aero_climb_angle)

    controllers.append(collections.OrderedDict([
        ('wing_serial', _WING_SERIAL_HELPER.Name(wing_serial)),
        ('dCL_dflap', np.round(dCL_dflap, 2)),
        ('angle_of_sideslip_cmd', np.round(angle_of_sideslip_cmd, 4)),
        ('roll_ti_cmd', np.round(roll_ti_cmd, 4)),
        ('yaw_ti_cmd', np.round(yaw_ti_cmd, 4)),
        ('delta_elevator_alpha_zero', np.round(delta_elevator_alpha_zero, 2)),
        ('ddelta_elevator_dalpha', np.round(ddelta_elevator_dalpha, 2)),
        ('lat_gains_pitch_forward',
         np.round(lat_gains_pitch_forward, 3).tolist()),
        ('lat_gains_low_tension',
         np.round(lat_gains_low_tension, 3).tolist()),
        ('lat_gains_high_tension',
         np.round(lat_gains_high_tension, 3).tolist()),
        ('long_gains_low_tension',
         np.round(long_gains_low_tension, 3).tolist()),
        ('long_gains_high_tension',
         np.round(long_gains_high_tension, 3).tolist()),
        ('flap_offsets', [np.round(flap[0, 0], 4) for flap in flap_offsets])
    ]))

  write_controllers.WriteControllers(
      'analysis/control/trans_in.py',
      os.path.join(makani.HOME, 'config/m600/control/trans_in_controllers.py'),
      controllers)


if __name__ == '__main__':
  main(sys.argv)

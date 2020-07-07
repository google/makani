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

"""Functions for working with simplified aerodynamic models."""

import copy
import ctypes
import json
import os
import pprint

import makani
from makani.analysis.aero import apparent_wind_util
from makani.analysis.aero import load_database
from makani.analysis.control import optimize
from makani.avionics.network import aio_labels
from makani.control import simple_aero
from makani.lib.python import lru_cache
from makani.sim.physics import physics
import numpy as np
import scipy.interpolate

# pylint doesn't like capital letters in variable names, in contrast to physics
# conventions.
# pylint: disable=invalid-name


class SimpleRotorModelParamsFormatException(Exception):
  pass


class Vec3FormatException(Exception):
  pass


def _DictToSimpleRotorModelParams(simple_rotor_model):
  """Convert a dictionary into a simple_aero.SimpleRotorModelParams."""
  simple_rotor_model_fields = set([
      name for (name, _) in getattr(simple_aero.SimpleRotorModelParams,
                                    '_fields_')
  ])
  # Test that the fields of the structure have not changed.
  assert (simple_rotor_model_fields
          == {'thrust_coeffs', 'J_neutral', 'J_max', 'D', 'D4'})
  if (set(simple_rotor_model.keys()) != simple_rotor_model_fields
      or (len(simple_rotor_model['thrust_coeffs'])
          != simple_aero.NUM_SIMPLE_ROTOR_MODEL_COEFFS)):
    raise SimpleRotorModelParamsFormatException(simple_rotor_model)

  c_model = simple_aero.SimpleRotorModelParams()
  for i in range(simple_aero.NUM_SIMPLE_ROTOR_MODEL_COEFFS):
    c_model.thrust_coeffs[i] = simple_rotor_model['thrust_coeffs'][i]
  c_model.J_neutral = simple_rotor_model['J_neutral']
  c_model.J_max = simple_rotor_model['J_max']
  c_model.D = simple_rotor_model['D']
  c_model.D4 = c_model.D**4.0

  return c_model


def _ArrayToVec3(vec3):
  """Convert an array of three doubles into a simple_aero.Vec3."""
  if len(vec3) != 3:
    raise Vec3FormatException(vec3)
  c_vec3 = simple_aero.Vec3()
  c_vec3.x = vec3[0]
  c_vec3.y = vec3[1]
  c_vec3.z = vec3[2]
  return c_vec3


def CalcLocalAirspeed(airspeed, local_pressure_coeff, pos, pqr):
  c_pos = _ArrayToVec3(pos)
  c_pqr = _ArrayToVec3(pqr)
  return simple_aero.CalcLocalAirspeed(airspeed, local_pressure_coeff,
                                       ctypes.pointer(c_pos),
                                       ctypes.pointer(c_pqr))


def ThrustToPower(thrust, v_freestream, simple_rotor_model):
  c_model = _DictToSimpleRotorModelParams(simple_rotor_model)
  return simple_aero.ThrustToPower(thrust, v_freestream,
                                   ctypes.pointer(c_model))


def _CheckRelativeTolerance(expected, actual, rel_tol):
  return np.all(np.abs(np.array(expected) - np.array(actual))
                <= rel_tol * np.abs(expected))


def CalcAdvanceRatio(model, airspeed, omega):
  """Calculate the advance ratio.

  Args:
    model: Dict describing a SimpleRotorModel.
    airspeed: Airspeed [m/s].
    omega: Rotor angular rate [rad/s].

  Returns:
    The advance ratio.
  """
  return (2.0 * np.pi * airspeed) / (omega * model['D'])


def CalcThrustCoeff(model, J):
  """Calculates the thrust coefficient from an advance ratio.

  The simple propeller model is based around:

    T = air_density * (omega / 2.0 / pi)^2 * D^4 * k_T(J)

  where

    k_T(J) = a1*(J - J_neutral)
             + a2*(J - J_neutral)^2
             + a3*(J - J_neutral)^3

  J_neutral: zero thrust J
  J_max:     thrust/power drop off rapidly above here

  The thrust_coeffs are determined by fitting a polynomial to the
  non-dimensional thrust coefficient as a function of J - J_neutral.

  Static thrust tests on the propellers consistently show lower values
  of the thrust coefficient than XROTOR predicts.  Our best theory for
  this is that XROTOR doesn't work well when the flow field doesn't
  look like a stream tube (i.e. for situations where the propeller is
  eating its own wake.)  To account for this lower value, we multiply
  the thrust coefficients so that it properly predicts static thrust.
  This may cause the simple propeller models to under-predict drag at
  high advance ratios.

  Args:
    model: Simple rotor model dictionary.  This contains a list of
        thrust coefficients, the neutral and maximum advance ratio,
        and the propeller diameter.
    J: Advance ratio of the propeller: V / (f * D).

  Returns:
    The thrust coefficient defined by k_T(J) above.
  """
  dJ = J - model['J_neutral']
  return np.polyval(model['thrust_coeffs'], dJ) * dJ


def _CalcCrossingIndex(table, axis=0, unique=True):
  """Calculates the approximate zero crossing for each row or column of a table.

  Args:
    table: a nx-by-ny array.
    axis: Axis along which to search for a zero crossing.
    unique: Whether to raise a ValueError when there are multiple crossings.

  Returns:
    If axis=0, returns an array of shape (ny,) containing floating
    point values between [0,nx] indicating the approximate point where
    the zero crossing occurs or None.

  Raises:
    ValueError: if multiple zero crossings are detected and unique is true.
  """
  sign_changes = np.diff(np.sign(table), axis=axis)
  # Transitions such as [-1.0, 0.0, 1.0] register two changes in sign.
  # We find such double steps and exclude them.
  double_edge = np.argwhere(np.delete(sign_changes, 0, axis=axis) ==
                            np.delete(sign_changes, -1, axis=axis))
  double_edge[:, axis] += 1
  sign_changes[double_edge[:, 0], double_edge[:, 1]] = 0.0

  crossing_indices = np.argwhere(sign_changes)

  other_axis = 1 - axis
  result = [None for _ in range(table.shape[other_axis])]
  for crossing_index in crossing_indices:
    if result[crossing_index[other_axis]] is not None:
      if unique:
        raise ValueError('Multiple crossings.')
    else:
      lhs_value = table[crossing_index[0], crossing_index[1]]
      if lhs_value == 0.0:
        index = crossing_index[axis]
      else:
        rhs_index = copy.copy(crossing_index)
        rhs_index[axis] += 1
        rhs_value = table[rhs_index[0], rhs_index[1]]
        index = crossing_index[axis] - (lhs_value / (rhs_value - lhs_value))

      result[crossing_index[other_axis]] = index
  return result


def CalcTorqueLimits(voltage, omega, params):
  """Calculate the maximum and minimum torque a motor can supply.

  Args:
    voltage: Voltage [V] at the motor.
    omega: Rotor speed [rad/s].
    params: Motor parameters.

  Returns:
    A tuple of torques (min_motoring_torque, max_motoring_torque).  These
    torques are positive when motoring.
  """
  motor_params = physics.MotorParams()
  motor_params.modulation_limit = params['modulation_limit']
  motor_params.phase_current_cmd_limit = params['phase_current_cmd_limit']
  motor_params.iq_cmd_lower_limit = params['iq_cmd_lower_limit']
  motor_params.iq_cmd_upper_limit = params['iq_cmd_upper_limit']
  motor_params.Ld = params['Ld']
  motor_params.Lq = params['Lq']
  motor_params.Rs = params['Rs']
  motor_params.flux_linkage = params['flux_linkage']
  motor_params.num_pole_pairs = params['num_pole_pairs']

  return physics.CalcTorqueLimits(voltage, omega, motor_params)


# See GetRotorModel, below.
_rotor_model_cache = lru_cache.LruCache()


def GetRotorModel(air_density, rotor_database_path, motor_params,
                  nominal_voltage, tip_speed_limit,
                  advance_ratio_stall_margin):
  """Returns a RotorModel from a module-level cache.

  This is the first of a two-level caching strategy for
  RotorModel.CalcMaxThrusts, which is one of the most expensive calls in the
  config system.

  In practice, we only create rotor models with a few different sets of input
  parameters. Furthermore, CalcMaxThrusts is only evaluated with a few different
  inputs. Here, we cache RotorModels as they are created, and each RotorModel
  contains a cache of CalcMaxThrusts outputs.

  Note that using floats directly in cache keys works fine in practice because
  the inputs are never perturbed.

  Args:
    air_density: See RotorModel.
    rotor_database_path: See RotorModel.
    motor_params: See RotorModel.
    nominal_voltage: See RotorModel.
    tip_speed_limit: See RotorModel.
    advance_ratio_stall_margin: See RotorModel.
  """

  # To form a hashable and determistic cache key, we JSONify motor_params with
  # sorted keys. For sake of conservatism, we ensure that it is a flat
  # dictionary of scalars. More complex structures ought to be determistically
  # serialized as well with sort_keys=True, but this assertion should be
  # scrutinized if we come to rely on it.
  for value in motor_params.values():
    assert isinstance(value, (int, float))
  key = (air_density, rotor_database_path,
         json.dumps(motor_params, sort_keys=True),
         nominal_voltage, tip_speed_limit, advance_ratio_stall_margin)

  if key not in _rotor_model_cache:
    _rotor_model_cache[key] = RotorModel(
        air_density, rotor_database_path, motor_params, nominal_voltage,
        tip_speed_limit, advance_ratio_stall_margin)
  return _rotor_model_cache[key]


class RotorModel(object):
  """Wrapper describing a rotor model in combination with power train limits.

  Attributes:
    num_v_freestreams: Number [#] of freestream velocity point on grid.
    num_omegas: Number [#] of rotor velocities on grid.
    omegas: Array of shape (num_omegas, 1) containing rotor speeds [rad/s].
    v_freestreams: Array of shape (1, num_v_freestreams) containing freestream
        velocities [m/s].
    thrust: Array of (num_omegas, num_v_freestreams) thrusts [N].
    torque: Array of (num_omegas, num_v_freestreams) torques [N-m].
    power: Array of (num_omegas, num_v_freestreams) powers [W].
    advance_ratios: Array of (num_omegas, num_v_freestreams) advance ratios [#].
    motor_params: Motor parameters.
    tip_speed_limit: Maximum tip speed [m/s] to use in fits.
    advance_ratio_stall_margin: Amount [#] to reduce the maximum
        allowed advance ratio from the stall advance ratio.
    stall_advance_ratios: Array of (num_v_freestreams, 1) advance ratios that
        achieve minimum thrust.
    max_advance_ratio: Maximum advance ratio [#] based on stall_advance_ratios
        and the prescribed margins.
    keep: Array of (num_omegas, num_v_freestreams) boolean values indicating
        points on the grid which satisfy power, torque and tip speed limits.
  """

  def __init__(self, air_density, rotor_database_path, motor_params,
               nominal_voltage, tip_speed_limit,
               advance_ratio_stall_margin=0.0):
    """Constructor.

    Args:
      air_density: Air density [kg/m^3] used for calculating torque and power
          limits.
      rotor_database_path: Path to a rotor database file.
      motor_params: Motor parameters.
      nominal_voltage: Operating nominal voltage [V].
      tip_speed_limit: Maximum tip speed [m/s] used in fits.
      advance_ratio_stall_margin: Amount [#] to reduce the maximum
          allowed advance ratio from the stall advance ratio.
    """
    # Load fields from the database.
    self._air_density = air_density
    self._rotor_database_path = rotor_database_path
    data = load_database.LoadPropDatabase(
        os.path.join(makani.HOME, 'database', rotor_database_path))
    self._D = data['diameter']
    self._D4 = self._D**4
    self._num_v_freestreams = data['num_v_freestreams']
    self._num_omegas = data['num_omegas']
    self._omegas = np.reshape(data['omegas'], (self._num_omegas, 1))
    self._v_freestreams = np.reshape(data['v_freestreams'],
                                     (1, self._num_v_freestreams))
    self._thrust_coeffs = np.array(data['thrust_coeffs'])
    self._power_coeffs = np.array(data['power_coeffs'])

    assert self._thrust_coeffs.shape == (self._num_omegas,
                                         self._num_v_freestreams)
    assert self._power_coeffs.shape == (self._num_omegas,
                                        self._num_v_freestreams)

    self._thrust = (air_density * (self._omegas / 2.0 / np.pi)**2.0
                    * self._D**4.0 * self._thrust_coeffs)
    self._power = (air_density * (self._omegas / 2.0 / np.pi)**3.0
                   * self._D**5.0 * self._power_coeffs)
    self._torque = self._power / self._omegas

    self._advance_ratios = (2.0 * np.pi * self._v_freestreams) / (
        self._omegas * self._D)

    self._motor_params = motor_params

    self._tip_speed_limit = tip_speed_limit

    # Calculate the stall advance ratio for each freestream velocity.
    self._stall_advance_ratios = np.zeros((self._num_v_freestreams,))
    for j in range(self._num_v_freestreams):
      # Select the highest rotor speed at which increasing speed
      # decreases thrust.
      decreasing = np.argwhere(self._thrust[1:, j] < self._thrust[:-1, j])
      if decreasing.size == 0:
        i_stall = 0
      else:
        i_stall = decreasing[-1]
      self._stall_advance_ratios[j] = self._advance_ratios[i_stall, j]

    self._max_advance_ratio = np.median(
        self._stall_advance_ratios - advance_ratio_stall_margin)

    # We loosen the torque and power limits by 10% to require our
    # SimpleRotorModel to fit well over a larger range.
    min_torque, _, max_torque, _ = self._CalcTorqueLimits(nominal_voltage)
    self._keep = np.logical_and.reduce((
        self._advance_ratios < self._stall_advance_ratios,
        self._torque > 1.1 * min_torque,
        self._torque < 1.1 * max_torque,
        (self._omegas * self._D / 2.0 < self._tip_speed_limit) * np.ones(
            self._v_freestreams.shape)
    ))

    # Inner layer of CalcMaxThrusts cache. See docstring of GetRotorModel.
    self._max_thrust_cache = lru_cache.LruCache()

  def _CalcTorqueLimits(self, voltage):
    """Calculate torque limits for each rotor speed in the database."""
    min_torque = np.zeros(self._omegas.shape)
    min_torque_constraint = [None for _ in range(self._num_omegas)]
    max_torque = np.zeros(self._omegas.shape)
    max_torque_constraint = [None for _ in range(self._num_omegas)]
    for i in range(self._num_omegas):
      (motoring_lower_limit, lower_constraint,
       motoring_upper_limit, upper_constraint) = CalcTorqueLimits(
           voltage, self._omegas[i, 0], self._motor_params)

      min_torque[i, 0] = -motoring_upper_limit
      min_torque_constraint[i] = upper_constraint

      max_torque[i, 0] = -motoring_lower_limit
      max_torque_constraint[i] = lower_constraint

    return min_torque, min_torque_constraint, max_torque, max_torque_constraint

  def CalcMaxThrusts(self, voltage, freestream_velocities):
    """Calculates maximum possible thrust at given freestream velocities.

    Args:
      voltage: Motor voltage [V].
      freestream_velocities: Airspeeds [m/s] at which to determine the
          maximum thrusts.

    Returns:
      A tuple containing a list of maximum thrusts for each rotor and
      a list of strings describing the particular constraint, torque or
      power, that limited the maximum thrust.
    """
    # Cache the outputs of _CalcMaxThrusts.
    #
    # Tuplify freestream_velocities so it becomes hashable. In practice, it is
    # generally a single element.
    key = (voltage, tuple(f for f in freestream_velocities))
    if key not in self._max_thrust_cache:
      self._max_thrust_cache[key] = self._CalcMaxThrusts(voltage,
                                                         freestream_velocities)
    return self._max_thrust_cache[key]

  def _CalcMaxThrusts(self, voltage, freestream_velocities):
    """Implementation of CalcMaxThrusts."""
    limit_strings = {
        physics.kSimMotorLimitNone: 'none',
        physics.kSimMotorLimitGroundPower: 'ground_power',
        physics.kSimMotorLimitPhaseCurrent: 'phase_current',
        physics.kSimMotorLimitPower: 'power'
    }
    # Calculate the angular rate floating point index that results in
    # the limiting power and torque.
    min_torque, min_torque_constraint, _, _ = self._CalcTorqueLimits(voltage)
    torque_crossing_index = _CalcCrossingIndex(
        self._torque - min_torque)

    # Create a list of maximum thrusts, based on the power and torque
    # limits, for each freestream velocity in the rotor table.  Record
    # the limiting constraint.
    max_thrusts = [None for _ in range(self._num_v_freestreams)]
    thrust_constraints = ['none' for _ in range(self._num_v_freestreams)]
    for i in range(self._num_v_freestreams):
      index = self._num_omegas - 1
      constraint = physics.kSimMotorLimitNone
      if torque_crossing_index[i] and torque_crossing_index[i] <= index:
        index = torque_crossing_index[i]
        constraint = min_torque_constraint[int(np.round(index))]

      max_thrusts[i] = scipy.interpolate.interp1d(range(self._num_omegas),
                                                  self._thrust[:, i])(index)
      thrust_constraints[i] = limit_strings[constraint]

    # Make interpolation functions for the maximum thrust and the
    # index of the airspeed in the rotor database.
    max_thrust = scipy.interpolate.interp1d(np.squeeze(self._v_freestreams),
                                            max_thrusts)
    v_freestreams_ind = scipy.interpolate.interp1d(
        np.squeeze(self._v_freestreams), range(len(thrust_constraints)))

    # Coerce freestream velocities to be above the minimum in the
    # database.
    freestream_velocities = copy.copy(freestream_velocities)
    if freestream_velocities[0] <= 0.0:
      freestream_velocities[0] = self._v_freestreams[0, 0]

    return ([float(max_thrust(v)) for v in freestream_velocities],
            [thrust_constraints[int(v_freestreams_ind(v))]
             for v in freestream_velocities])

  def CalcStaticTorquePerThrust(self):
    """Calculate the approximate sensitivity of torque to change in thrust."""
    if self._v_freestreams[0, 0] > 1.0:
      raise ValueError('Database freestream velocities do not extend '
                       'to zero.')
    dtorques = np.diff(self._torque[self._keep[:, 0], 0])
    dthrusts = np.diff(self._thrust[self._keep[:, 0], 0])
    if np.any(dtorques > 0.0) or np.any(dthrusts < 0.0):
      raise ValueError('Static torque and/or thrust are not monotone.')
    return np.median(dtorques / dthrusts)

  def CalcSimpleRotorModel(self):
    """Calculates a polynomial thrust and torque coefficient model.

    See CalcThrustCoeff for the model definition.

    Returns:
      A dictionary describing a SimpleRotorModel parameters.

    Raises:
      ValueError: if too many rotor speeds lack a neutral advance ratio.
    """
    # Use linear interpolation to approximate the advance ratio
    # corresponding to zero thrust for each rotor angular rate [rad/s].
    crossing_indices = _CalcCrossingIndex(self._thrust, axis=1)
    neutral_advance_ratios = [
        scipy.interpolate.interp1d(range(self._num_v_freestreams),
                                   self._advance_ratios[i, :].T)(index)
        for i, index in enumerate(crossing_indices) if index is not None
    ]
    if len(neutral_advance_ratios) < 0.9 * self._num_omegas:
      raise ValueError('Too many rotor speeds lack a neutral advance ratio.')

    # Pick one value to represent this crossing point.
    neutral_advance_ratio = np.median(neutral_advance_ratios)

    # Prepare data for model fit.
    indices = self._keep.ravel()
    dJ = (self._advance_ratios - neutral_advance_ratio).ravel()
    A = np.transpose([dJ[indices]**3, dJ[indices]**2, dJ[indices]])
    k_thrust = self._thrust_coeffs.ravel()

    return {
        # The value of 'rcond' in the following is simply the
        # default value from an older version of numpy.
        'thrust_coeffs': np.linalg.lstsq(A, k_thrust[indices],
                                         rcond=-1)[0].tolist(),
        'J_neutral': neutral_advance_ratio,
        'J_max': self._max_advance_ratio,
        'D': self._D,
        'D4': self._D4
    }

  def CalcThrustToPowerFit(self, v_freestream):
    """Generate a fit of the thrust-to-power relationship for a rotor.

    Args:
      v_freestream: Freestream velocity [m/s] at which to generate the fit.

    Returns:
      A tuple (min_thrust, max_thrust, thrust_to_power).  thrust_to_power is
      a function which returns the power [W] associated with a given thrust [N]
      that must lie in the interval [min_thrust, max_thrust].
    """
    if v_freestream == 0.0:
      omega_min = 0.0
      v_freestream = self._v_freestreams[0, 0]
    else:
      omega_min = (2.0 * np.pi * v_freestream) / (
          self._D * self._max_advance_ratio)

    ind = np.squeeze(np.argwhere(self._omegas >= omega_min))[:, 0]

    # Interpolate the thrust and power curves for the desired freestream
    # velocity.
    power = [
        scipy.interpolate.interp1d(self._v_freestreams[0, :].T,
                                   self._power[idx, :].T)(v_freestream)
        for idx in ind
    ]
    thrust = [
        scipy.interpolate.interp1d(self._v_freestreams[0, :].T,
                                   self._thrust[idx, :].T)(v_freestream)
        for idx in ind
    ]

    return thrust[0], thrust[-1], scipy.interpolate.interp1d(thrust, power)

  def CheckSimpleRotorModel(self, model, abs_thrust_tol, rel_thrust_tol):
    """Compares a SimpleRotorModel to a rotor database.

    See makani.analysis.control.simple_aero.CalcThrustCoeff for the
    model definition.  The thrusts must either match within an
    absolute tolerance (given by abs_thrust_tol) *or*
    within a relative tolerance (given by rel_thrust_tol).

    Args:
      model: Dictionary describing a SimpleRotorModelParams.
      abs_thrust_tol: Absolute tolerance [N] for thrust error.
      rel_thrust_tol: Relative tolerance [N] for thrust error.

    Returns:
      True if model and this database approximately agree.

    """
    for i, omega in enumerate(self._omegas):
      for j, v_freestream in enumerate(self._v_freestreams.T):
        if not self._keep[i, j]:
          continue

        database_thrust = self._thrust[i, j]
        J = CalcAdvanceRatio(model, v_freestream, omega)

        # Test thrust value.
        model_thrust = (
            self._air_density * (omega / 2.0 / np.pi)**2.0
            * self._D**4.0 * CalcThrustCoeff(model, J))
        if (np.abs(database_thrust - model_thrust)
            > np.max((rel_thrust_tol * np.abs(database_thrust),
                      abs_thrust_tol))):
          print ('Thrust model for %s does not match (%g vs. %g) at omega = %g,'
                 ' v_freestream = %g' % (self._rotor_database_path,
                                         database_thrust, model_thrust, omega,
                                         v_freestream))
          return False

    return True


def CalcVoltageThrustLimit(v_source, num_faulted_blocks, R_tether,
                           motor_efficiency, rotor_models, v_freestream):
  """Calculate the maximum motor thrusts determined by voltage sag.

  This function computes the per-motor maximum thrusts taking into
  account the sag in voltage at the kite during high power draw and the voltage
  sharing during stack faults. Each motor is assumed to be operating at its
  limit.

  Args:
    v_source: Ground voltage [V].
    num_faulted_blocks: How many blocks are experiencing a stack fault [int].
    R_tether: Tether resistance [Ohm] (set to 0 if using voltage compensation).
    motor_efficiency: Motor efficiency [#].
    rotor_models: Array of kNumMotors rotor models.
    v_freestream: Freestream velocity [m/s].

  Returns:
    An array of kNumMotors maximum thrusts.
  """
  thrust_to_powers = [
      r.CalcThrustToPowerFit(v_freestream)[2] for r in rotor_models
  ]

  # Use binary search to solve: kite_voltage(max_thrust(V / num_blocks)) = V.
  num_blocks = aio_labels.kNumMotors / 2 - num_faulted_blocks
  lower_bound = 0.0
  upper_bound = v_source / num_blocks
  while upper_bound - lower_bound > v_source / 1e3:
    voltage = (upper_bound + lower_bound) / 2.0
    max_thrusts = [
        r.CalcMaxThrusts(voltage, [v_freestream])[0][0] for r in rotor_models]
    aero_powers = [
        thrust_to_power(max_thrust)
        for thrust_to_power, max_thrust in zip(thrust_to_powers, max_thrusts)
    ]
    # Note that power is negative motoring.  The electrical power is greater
    # than the mechanical power when motoring and less when generating.
    elec_power = np.sum([
        power * (1.0 / motor_efficiency if power < 0.0 else motor_efficiency)
        for power in aero_powers
    ])
    # Model the drop in kite voltage due to the tether resistance.
    v_kite = v_source / 2.0 + (
        (v_source / 2.0)**2.0 + R_tether * elec_power)**0.5
    motor_voltage = v_kite / num_blocks
    if voltage > motor_voltage:
      upper_bound = voltage
    else:
      lower_bound = voltage

  return max_thrusts


def CalcMaxTotalThrustForFreestreamVelocity(
    min_aero_power, v_freestream, rotor_models, thrust_moment_matrix,
    thrusts_matrix, threshold=1e3):
  """Calculates motor thrusts which balance torques and hit a power limit.

  Args:
    min_aero_power: Power [W] limit (negative is thrusting).
    v_freestream: Freestream velocity [m/s].
    rotor_models: Array of aio_labels.kNumMotors RotorModel objects.
    thrust_moment_matrix: A 4-by-d np.matrix mapping the d degrees of freedom
        available for thrust commands to [thrust [N]; roll [N-m]; pitch [N-m];
        yaw [N-m]].
    thrusts_matrix: A kNumMotors-by-d np.matrix mapping the d degrees of freedom
        available for thrust commands to the individual motor thrusts.
    threshold: A power threshold [W] tested to design when the given thrusts
        are close enough to drawing min_aero_power.

  Returns:
    The maximum total thrust that can be achieved while (1) maintaining zero
    pitch and yaw moment, (2) respecting the total power limit (min_aero_power),
    and (3) respecting each individual rotor thrust limit (implied by the extent
    of the underlying rotor tables).
  """
  assert len(rotor_models) == aio_labels.kNumMotors

  min_thrusts = np.matrix(np.zeros((aio_labels.kNumMotors, 1)))
  max_thrusts = np.matrix(np.zeros((aio_labels.kNumMotors, 1)))
  thrust_to_powers = [None for _ in range(aio_labels.kNumMotors)]
  for i, rotor_model in enumerate(rotor_models):
    (min_thrusts[i], max_thrusts[i],
     thrust_to_powers[i]) = rotor_model.CalcThrustToPowerFit(v_freestream)

  def _CalcPower(x_candidate):
    """Calculate the power used by a given candidate command."""
    thrusts = thrusts_matrix * x_candidate
    return np.sum([
        thrust_to_powers[i](thrusts[i, 0])
        for i in range(aio_labels.kNumMotors)
    ])

  # First, find the minimum and maximum thrust while balancing
  # the pitch and yaw moments.
  num_var = thrusts_matrix.shape[1]
  # These two matrices represent the constraint that each
  # motor thrust obey the given minimum and maximum.
  G = np.matrix(np.vstack((thrusts_matrix, -thrusts_matrix)))
  h = np.matrix(np.vstack((max_thrusts, -min_thrusts)))
  # These matrices are used to constrain the total pitch and yaw
  # moments to be zero.
  A_pitch_yaw = thrust_moment_matrix[2:4, :]
  b_pitch_yaw = np.matrix(np.zeros((2, 1)))

  # Solve a linear program to find the minimum total thrust which
  # satisfies the constraints.
  x_lower_bound = optimize.SolveQp(np.matrix(np.zeros((num_var, num_var))),
                                   thrust_moment_matrix[0, :].T,
                                   G, h, A_pitch_yaw, b_pitch_yaw)
  thrust_lower_bound = thrust_moment_matrix[0, :] * x_lower_bound
  thrust_lower_bound_power = _CalcPower(x_lower_bound)

  # Solve a linear program to find the maximum total thrust which
  # satisfies the constraints.
  x_upper_bound = optimize.SolveQp(np.matrix(np.zeros((num_var, num_var))),
                                   -thrust_moment_matrix[0, :].T,
                                   G, h, A_pitch_yaw, b_pitch_yaw)
  thrust_upper_bound = thrust_moment_matrix[0, :] * x_upper_bound
  thrust_upper_bound_power = _CalcPower(x_upper_bound)

  # Test whether we hit the power constraint. If not, we've found the
  # achievable maximum total thrust.
  if thrust_upper_bound_power >= min_aero_power:
    return float(thrust_moment_matrix[0, :] * x_upper_bound)

  # Perform binary search on total thrust to hit the power constraint.
  # We use equality constraints to hold the pitch and yaw moments to
  # zero and vary the total thrust.  Additionally, we minimize the
  # squared norm of the solution to emulate the use of pseudo-inverses
  # in the quadratic program solver used by MixRotors.
  A_thrust_pitch_moment = np.matrix(np.vstack((
      thrust_moment_matrix[0, :],
      thrust_moment_matrix[2, :],
      thrust_moment_matrix[3, :],
  )))
  b_thrust_pitch_moment = np.matrix(np.zeros((3, 1)))

  while thrust_lower_bound_power - thrust_upper_bound_power > threshold:
    assert thrust_upper_bound_power < min_aero_power
    assert min_aero_power < thrust_lower_bound_power

    candidate_thrust = (thrust_upper_bound + thrust_lower_bound) / 2.0
    b_thrust_pitch_moment[0] = candidate_thrust

    x_candidate = optimize.SolveQp(np.matrix(np.eye(num_var)),
                                   np.matrix(np.zeros((num_var, 1))),
                                   G, h, A_thrust_pitch_moment,
                                   b_thrust_pitch_moment)

    # Calculate the power draw for this thrust, and continue the binary search.
    power = _CalcPower(x_candidate)

    if _CalcPower(x_candidate) < min_aero_power:
      thrust_upper_bound = thrust_moment_matrix[0, :] * x_candidate
      thrust_upper_bound_power = power
    else:
      thrust_lower_bound = thrust_moment_matrix[0, :] * x_candidate
      thrust_lower_bound_power = power
      # We keep track of the candidate that achieves lower power as that
      # is preferred for the final solution.
      x_lower_bound = x_candidate

  return float(thrust_moment_matrix[0, :] * x_lower_bound)


def _CalcSimpleAeroModel(aero_database, crosswind_trimmed=True,
                         flap_offset=None, CL_0_offset=0.0):
  """Computes a simple aero model using the specified 'aero_database'.

  Computes a simple aerodynamic model using a low incidence database about the
  nominal elevator trim and dimensionless body-rates.
  The following corrections are taken into account:
    - Non-linear flap effectiveness as defined in config/m600/sim/aero_sim.py.
    - Rotor wake induced correction to tail surface derivatives.
  The following corrections are not taken into account:
    - Aero coefficient offsets (except for CL_0_offset) and scale factors
      defined in config/m600/sim/aero_sim.py.

  Args:
    aero_database: Path to a .json aerodynamics file.
    crosswind_trimmed: Whether to use the crosswind elevator trim
        and body rates.
    flap_offset: Common offset to apply to all flaps. Only valid if
        crosswind_trimmed == False.
    CL_0_offset: Offset to apply to 'CL_0' returned by this function.

  Returns:
    True if simple_aero_model agrees with the low incidence database.
  """
  if crosswind_trimmed:
    assert flap_offset is None, 'flap_offset not valid with crosswind trim.'
  else:
    assert flap_offset is not None, (
        'flap_offset must be specified if not using crosswind trim.')

  avl_db = load_database.AvlDatabase(os.path.join(makani.HOME, 'database',
                                                  aero_database))

  def GetFlapsAndCOmega(alpha, beta):
    """Gets flap offsets and dimensionless body rates for aero calcs."""
    if crosswind_trimmed:
      flaps, c_omega = avl_db.GetCrosswindFlapsAndCOmega(alpha, beta)
    else:
      ailerons = [simple_aero.kFlapA1, simple_aero.kFlapA2,
                  simple_aero.kFlapA4, simple_aero.kFlapA5,
                  simple_aero.kFlapA7, simple_aero.kFlapA8]
      flaps = np.array([flap_offset if flap in ailerons else 0.0
                        for flap in range(simple_aero.kNumFlaps)])
      c_omega = np.zeros(3)

    return flaps, c_omega

  def _CalcCLCDCY(alpha, beta):
    """Calculate CL, CD, and CY at a given AOA and AOS."""
    flaps, c_omega = GetFlapsAndCOmega(alpha, beta)
    # Rotor thrust coefficient set to 0.0 to not alter CL and CY from the
    # aero database.
    thrust_coeff = 0.0
    (CF, _) = avl_db.CalcFMCoeff(alpha, beta, flaps, c_omega, thrust_coeff)

    # Rotate the force coefficients into the wind frame to calculate
    # CL and CD.
    dcm_w2b = apparent_wind_util.CalcDcmWToB(alpha, beta)

    # The diagonal matrix here accounts for the left-handed "wind
    # tunnel" frame in which CL and CD are computed (positive being up
    # and aft, respectively, rather than down and forward, as in the
    # body frame and wind frame).
    CF_wind = np.matmul(np.diag([-1, 1, -1]), np.matmul(dcm_w2b.T, CF))
    CD = CF_wind[0, 0]
    CL = CF_wind[2, 0]

    # We retain side-force coefficient CY in the body frame.
    # (Confusingly, the wind-frame side-force coefficient is also
    # called CY.)
    CY = CF[1, 0]

    return (CL, CD, CY)

  def _CalcDCLDFlap(flap_label):
    """Calculate dCL_dflap for a given flap at alpha=0, beta=0."""

    h = 0.01  # Step size [rad] for finite difference.
    flaps, c_omega = GetFlapsAndCOmega(0.0, 0.0)
    flaps[flap_label] -= h
    CF_lower, _ = avl_db.CalcFMCoeff(0.0, 0.0, flaps, c_omega, 0.0)
    flaps[flap_label] += 2.0 * h
    CF_upper, _ = avl_db.CalcFMCoeff(0.0, 0.0, flaps, c_omega, 0.0)

    # CF is computed with alpha==0, so there's no need to convert to wind
    # coordinates.
    return -float((CF_upper[2] - CF_lower[2])) / (2.0 * h)

  (CL_0, CD_0, CY_0) = _CalcCLCDCY(0.0, 0.0)

  # Approximate derivatives by central difference.
  h = 0.01  # Step size [rad] for finite difference.
  (CL_alphap, _, _) = _CalcCLCDCY(h, 0.0)
  (CL_alpham, _, _) = _CalcCLCDCY(-h, 0.0)
  dCL_dalpha = (CL_alphap - CL_alpham) / (2.0 * h)

  # NOTE(b/110886499): It seems that there may be issues associated
  # with interpolating the body-frame aero coefficients to determine
  # dCL/dalpha here.  To address this, consider fitting a polynomial
  # over a wider range of alpha; or expose the CD values from the
  # underlying aero database to avoid unneeded coordinate transforms.
  # TODO: Ben suggests modeling the drag coefficient as
  # quadratic in alpha or CL.
  (_, CD_alphap, _) = _CalcCLCDCY(h, 0.0)
  (_, CD_alpham, _) = _CalcCLCDCY(-h, 0.0)
  dCD_dalpha = (CD_alphap - CD_alpham) / (2.0 * h)

  (_, _, CY_betap) = _CalcCLCDCY(0.0, h)
  (_, _, CY_betam) = _CalcCLCDCY(0.0, -h)
  dCY_dbeta = (CY_betap - CY_betam) / (2.0 * h)

  return {
      'CL_0': CL_0 + CL_0_offset,
      'CD_0': CD_0,
      'dCL_dalpha': dCL_dalpha,
      'dCD_dalpha': dCD_dalpha,
      'base_flaps': GetFlapsAndCOmega(0.0, 0.0)[0].tolist(),
      'dCL_dflap': [_CalcDCLDFlap(flap)
                    for flap in range(simple_aero.kNumFlaps)],
      'CY_0': CY_0,
      'dCY_dbeta': dCY_dbeta,
  }


def CheckSimpleAeroModel(aero_database, simple_aero_model, rel_tol,
                         crosswind_trimmed=False, flap_offset=None,
                         CL_0_offset=0.0):
  """Compares a simple aero model to the default AVL database.

  Compares the M600 AVL database about the nominal elevator trim and
  dimensionless body-rates to a simple aero model's coefficients.  No
  added drag or other modifications are applied.

  Args:
    aero_database: Path to a .json aerodynamics file.
    simple_aero_model: A dictionary with keys 'CL_0', 'dCL_dalpha', 'CY_0' and
        'dCY_dbeta', describing a simplified aerodynamics model.
    rel_tol: Relative tolerance for the coefficient comparison.
    crosswind_trimmed: True if the crosswind nominal body rates
        and surface deflections should be used.
    flap_offset: Offset to apply to all ailerons. Only allowed if
        crosswind_trimmed is False.
    CL_0_offset: Offset to apply to 'CL_0'.
  """
  database_simple_aero_model = _CalcSimpleAeroModel(
      aero_database, crosswind_trimmed=crosswind_trimmed,
      flap_offset=flap_offset, CL_0_offset=CL_0_offset)

  bad_fields = []
  for k in database_simple_aero_model:
    if not _CheckRelativeTolerance(database_simple_aero_model[k],
                                   simple_aero_model[k], rel_tol):
      bad_fields.append(k)

  if bad_fields:
    print '\n'.join([
        'The simplified model derived from %s,' % aero_database,
        pprint.pformat(database_simple_aero_model, indent=2),
        'disagrees with the encoded values. If updating %s,' % aero_database,
        'then the reference values should be changed for:',
        '    ' + ', '.join(bad_fields)])

    assert False

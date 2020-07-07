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

"""Provides a Python interface to AVL .dat files."""

import json
from makani.control import system_types
from makani.sim.physics import physics
import numpy


def _FileToArray(filename):
  array = []
  with open(filename, 'r') as f:
    for line in f:
      array += [float(x) for x in line.rstrip().split(' ')]
  return numpy.array(array)


class _FileReader(object):

  def __init__(self, filename):
    self._index = 0
    self._array = _FileToArray(filename)

  def Read(self, count):
    array_slice = self._array[self._index:(self._index + count)]
    self._index += count
    return array_slice


class DimensionException(Exception):
  """Raised by databases when an argument has an invalid dimension."""
  pass


class UnknownFormatException(Exception):
  """Raised when we could not determine the database format."""
  pass


def LoadPropDatabase(filename):
  """Loads a propellor database from a .json file."""
  with open(filename, 'r') as f:
    prop_data = json.loads(f.read())
  return prop_data


class DvlDatabase(object):
  """Loads and queries an DVL database from a .json file."""

  def __init__(self, filename):
    self._swig_dvl = physics.DvlAeroDatabase(filename)

  def CalcFMCoeff(self, alpha, beta, flaps, c_omega, thrust_coeff):
    """Calculates the body force and moment coefficients from an DVL database.

    This function should mirror the functionality of Aero::CalcDvlAeroCFM in the
    sim.

    Args:
      alpha: Angle of attack [rad].
      beta: Side-slip angle [rad].
      flaps: Array of kNumFlaps-by-1 control surface deflections [rad].
      c_omega: Array of 3-by-1 dimensionless body rates [#].
      thrust_coeff: Double of thrust coefficient [#], wind turbine notation.

    Returns:
      A tuple (cf, cm) containing the force and moment coefficients.

    Raises:
      DimensionException: The argument flaps has an incorrect shape.
    """
    if flaps.shape not in ((system_types.kNumFlaps,),
                           (system_types.kNumFlaps, 1)):
      raise DimensionException('Only %d flaps supported.' % self._num_flaps)
    if c_omega.shape not in ((3,), (3, 1)):
      raise DimensionException('c_omega must be a 3 vector.')

    flaps_vec = physics.VecWrapper(system_types.kNumFlaps)
    for i in range(system_types.kNumFlaps):
      flaps_vec.SetValue(i, flaps[i])

    c_omega_vec3 = physics.Vec3()
    c_omega_vec3.x = c_omega[0]
    c_omega_vec3.y = c_omega[1]
    c_omega_vec3.z = c_omega[2]

    force_moment = physics.ForceMoment()
    self._swig_dvl.CalcForceMomentCoeff(alpha, beta, c_omega_vec3.this,
                                        flaps_vec.GetVec(), force_moment.this,
                                        thrust_coeff)
    cf = numpy.array([[force_moment.force.x,
                       force_moment.force.y,
                       force_moment.force.z]]).T
    cm = numpy.array([[force_moment.moment.x,
                       force_moment.moment.y,
                       force_moment.moment.z]]).T

    return (cf, cm)


class AvlDatabase(object):
  """Loads and queries an AVL database from a .json file."""

  def __init__(self, filename):
    self._swig_avl = physics.AvlAeroDatabase(filename,
                                             physics.GetAeroSimParams())
    if self._swig_avl.num_flaps() != system_types.kNumFlaps:
      raise DimensionException('Only %d flaps supported.'
                               % system_types.kNumFlaps)

  def GetCrosswindFlapsAndCOmega(self, alpha, beta):
    flaps = numpy.zeros(system_types.kNumFlaps)
    flaps[system_types.kFlapEle] = self._swig_avl.GetNominalElevatorDeflection(
        alpha, beta)
    c_omega = self._swig_avl.omega_hat_0()
    return flaps, numpy.array([c_omega.x, c_omega.y, c_omega.z])

  # pylint: disable=unused-argument
  def CalcFMCoeff(self, alpha, beta, flaps, c_omega, thrust_coeff):
    """Calculates the body force and moment coefficients from an AVL database.

    This function should mirror the functionality of Aero::CalcAvlAeroCFM in the
    sim.

    Args:
      alpha: Angle of attack [rad].
      beta: Side-slip angle [rad].
      flaps: Array of kNumFlaps-by-1 control surface deflections [rad].
      c_omega: Array of 3-by-1 dimensionless body rates [#].
      thrust_coeff: Double of thrust coefficient [#], wind turbine notation.

    Returns:
      A tuple (cf, cm) containing the force and moment coefficients.

    Raises:
      DimensionException: The argument flaps has an incorrect shape.
    """
    num_flaps = system_types.kNumFlaps
    if flaps.shape not in ((num_flaps,), (num_flaps, 1)):
      raise DimensionException('Only %d flaps supported.' % num_flaps)
    if c_omega.shape not in ((3,), (3, 1)):
      raise DimensionException('c_omega must be a 3 vector.')

    flaps_vec = physics.VecWrapper(num_flaps)
    for i in range(num_flaps):
      flaps_vec.SetValue(i, flaps[i])

    c_omega_vec3 = physics.Vec3()
    c_omega_vec3.x = c_omega[0]
    c_omega_vec3.y = c_omega[1]
    c_omega_vec3.z = c_omega[2]

    force_moment = physics.ForceMoment()
    self._swig_avl.CalcForceMomentCoeff(alpha, beta, c_omega_vec3.this,
                                        flaps_vec.GetVec(), force_moment.this,
                                        thrust_coeff)
    cf = numpy.array([[force_moment.force.x,
                       force_moment.force.y,
                       force_moment.force.z]]).T
    cm = numpy.array([[force_moment.moment.x,
                       force_moment.moment.y,
                       force_moment.moment.z]]).T

    return (cf, cm)


class AeroDatabase(object):
  """Loads and queries an aero database from a .json file."""

  def __init__(self, filename):
    if self._IsAvlDatabase(filename):
      self.format = 'avl'
      self._database = AvlDatabase(filename)
    elif self._IsDvlDatabase(filename):
      self.format = 'dvl'
      self._database = DvlDatabase(filename)
    else:
      raise UnknownFormatException('%s does not have a known database format'
                                   % filename)

  def _IsAvlDatabase(self, filename):
    with open(filename) as f:
      database = json.load(f)
      try:
        return isinstance(database['num_deltas'], int)
      except KeyError:
        return False

  def _IsDvlDatabase(self, filename):
    with open(filename) as f:
      database = json.load(f)
      try:
        if 'num_delta1s' in database:
          return isinstance(database['num_delta1s'], int)
        else:
          return isinstance(database['num_deltas'], list)
      except KeyError:
        return False

  def CalcFMCoeff(self, alpha, beta, flaps, c_omega, thrust_coeff):
    """Calculates body force and moment coefficients from aero database.

    This function should mirror the functionality of Aero::CalcDvlAeroCFM in the
    sim.

    Args:
      alpha: Angle of attack [rad].
      beta: Side-slip angle [rad].
      flaps: Array of kNumFlaps-by-1 control surface deflections [rad].
      c_omega: Array of 3-by-1 dimensionless body rates [#].
      thrust_coeff: Double of thrust coefficient [#], wind turbine notation.

    Returns:
      A tuple (cf, cm) containing the force and moment coefficients.

    Raises:
      DimensionException: The argument flaps has an incorrect shape.
    """
    return self._database.CalcFMCoeff(alpha, beta, flaps, c_omega, thrust_coeff)

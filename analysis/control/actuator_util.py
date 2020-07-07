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

"""Functions for working with actuator models."""

import ctypes

from makani.control import actuator_util
from makani.lib.python import ctype_util
from makani.lib.python.autogen import autogen_util
import numpy as np


_thrust_moment_keys = ['thrust', 'moment']


def _PythonToCtype(data, c_type):
  """Populate a ctypes data type with a Python structure."""
  if c_type is actuator_util.Vec3:
    # Handle Vec3.
    assert len(data) == 3
    c_data = c_type()
    c_data.x = data[0]
    c_data.y = data[1]
    c_data.z = data[2]
    return c_data
  elif hasattr(c_type, '_length_'):
    # Handle arrays.
    length = getattr(c_type, '_length_')
    assert len(data) == length

    c_data = c_type()
    for i in range(length):
      c_data[i] = _PythonToCtype(data[i], getattr(c_type, '_type_'))

  elif hasattr(c_type, '_fields_'):
    # Handle structures.
    fields = autogen_util.GetCFields(c_type)
    assert set(data.keys()) == {field for field, _ in fields}

    c_data = c_type()
    for field, field_type in fields:
      setattr(c_data, field, _PythonToCtype(data[field], field_type))

  else:
    c_data = c_type(data)

  return c_data


def _ThrustMomentToArray(thrust_moment):
  """Convert a ThrustMoment dictionary into an array of thrust and moments.

  Args:
    thrust_moment: A ThrustMoment dictionary to be converted.

  Returns:
    A 4-by-1 numpy.matrix version of thrust_moment in array form.
  """
  assert thrust_moment.keys() == _thrust_moment_keys

  return np.matrix([thrust_moment['thrust'],
                    thrust_moment['moment'][0],
                    thrust_moment['moment'][1],
                    thrust_moment['moment'][2]])


def _ArrayToThrustMoment(array):
  """Convert a 4-by-1 array into a ThrustMoment dictionary.

  Args:
    array: A 4-by-1 numpy.matrix to be converted.

  Returns:
    A ThrustMoment dictionary.
  """
  assert np.size(array) == 4

  return {'thrust': array[0],
          'moment': [array[1], array[2], array[3]]}


def _AddThrustMoment(thrust_moment_0, thrust_moment_1):
  """Add two ThrustMoment dictionaries."""

  assert thrust_moment_0.keys() == _thrust_moment_keys
  assert thrust_moment_1.keys() == _thrust_moment_keys

  thrust_moment = {}
  for k in _thrust_moment_keys:
    thrust_moment[k] = (np.asarray(thrust_moment_0[k])
                        + np.asarray(thrust_moment_1[k]))

  return thrust_moment


def MixRotors(thrust_moment, weights,
              v_app, pqr, stacking_state,
              hover_flight_mode, air_density,
              rotor_params,
              rotor_control_params):
  """Wrapper around MixRotors function.

  See MixRotors in control/actuator_util.c.

  Args:
    thrust_moment: Dict with keys 'thrust', whose value is a float, and
        'moment', whose value is an array of three floats.
    weights: Dict with keys 'thrust', whose value is a float, and
        'moment', whose value is an array of three floats.
    v_app: Float storing the airspeed [m/s].
    pqr: Array of 3 floats representing the body rates [rad/s].
    stacking_state: Integer (see the enum StackingState).
    hover_flight_mode: Bool indicating if we are in a hover flight mode.
    air_density: Float storing the air density [kg/m^3].
    rotor_params: Array of kNumMotors dicts storing the contents of RotorParams
        structures.
    rotor_control_params: Dict storing the contents of the RotorControlParams
        structure.

  Returns:
    An 8-by-1 np.matrix containing the rotor speeds [rad/s].
  """
  assert len(rotor_params) == actuator_util.kNumMotors
  c_rotor_params = [
      _PythonToCtype(r, actuator_util.RotorParams) for r in rotor_params
  ]
  c_rotor_params_pointers = (
      ctypes.POINTER(actuator_util.RotorParams) * len(rotor_params))()
  for i, c_r in enumerate(c_rotor_params):
    c_rotor_params_pointers[i] = ctypes.pointer(c_r)

  c_rotors = (ctypes.c_double * actuator_util.kNumMotors)()
  c_available_thrust_moment = actuator_util.ThrustMoment()

  c_v_app_locals = (ctypes.c_double * actuator_util.kNumMotors)()

  actuator_util.MixRotors(
      ctypes.pointer(_PythonToCtype(thrust_moment, actuator_util.ThrustMoment)),
      ctypes.pointer(_PythonToCtype(weights, actuator_util.ThrustMoment)),
      v_app,
      ctypes.pointer(_PythonToCtype(pqr, actuator_util.Vec3)),
      stacking_state,
      hover_flight_mode,
      air_density,
      ctype_util.SizelessArray(c_rotor_params_pointers),
      ctypes.pointer(_PythonToCtype(rotor_control_params,
                                    actuator_util.RotorControlParams)),
      c_rotors,
      ctypes.pointer(c_available_thrust_moment),
      c_v_app_locals)

  return np.matrix([[c_rotors[i]] for i in range(actuator_util.kNumMotors)])


def LinearizeMixRotors(thrust_moment, params, h=1e-6):
  """Calculate a Jacobian matrix for the MixRotors function.

  Produces a linearized model:

    MixRotors(thrust_moment + delta_thrust_moment)
      ~ MixRotors(thrust_moment) + A * delta_thrust_moment

  Args:
    thrust_moment: A ThrustMoment dictionary around which to linearize.
    params: A parameters structure from mconfig.
    h: Step-size used in finite difference.

  Returns:
    A numpy.matrix of Jacobian values of units rad/s/N and rad/s/(N-m).
  """
  num_inputs = 4
  num_outputs = len(params['system']['rotors'])
  dfdu = np.matrix(np.zeros((num_outputs, num_inputs)))

  for i in range(num_inputs):
    e = np.zeros(num_inputs)
    e[i] = h
    delta_thrust_moment = _ArrayToThrustMoment(e)

    dfdu[:, i] = (
        MixRotors(
            _AddThrustMoment(thrust_moment, delta_thrust_moment),
            params['control']['hover']['output']['weights'],
            0.0,
            [0.0, 0.0, 0.0],
            actuator_util.kStackingStateNormal,
            True,
            params['system']['phys']['rho'],
            params['system']['rotors'],
            params['control']['rotor_control'])
        - MixRotors(
            thrust_moment,
            params['control']['hover']['output']['weights'],
            0.0,
            [0.0, 0.0, 0.0],
            actuator_util.kStackingStateNormal,
            True,
            params['system']['phys']['rho'],
            params['system']['rotors'],
            params['control']['rotor_control'])) / (2.0 * h)

  return dfdu

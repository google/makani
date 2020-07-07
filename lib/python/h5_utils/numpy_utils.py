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

"""Numpy utilities."""

import numpy


class NumpyIndexError(Exception):
  """An exception raised when an index cannot be found in a log."""

  def __init__(self, root, record, message=''):
    Exception.__init__(self, root, record, message)
    self.root = root
    self.record = record
    self.message = message


def IndexAsDict(data, record=None):
  """Indexes a Numpy struct as a nested dictionary of plain Numpy arrays.

  Args:
    data: A Numpy struct.
    record: List of strings, each string is an index to the next level of
                the compound Numpy struct. (default: None)

  Raises:
    NumpyIndexError: If the record index cannot be found.

  Returns:
    The value at the specified location. This can be a plain Numpy array
        or a nested dictionary of plain Numpy arrays if the value
        corresponds to a compound Numpy struct.
  """

  if record is None:
    record = []
  # This is a workaround of an h5py bug dealing with zero length data sets.
  if numpy.prod(data.shape) == 0:
    return _BuildEmptyDict(_DeepSelect(data.dtype, record), data.shape)
  else:
    data = _DeepSelect(data, record)
    if data.dtype.names is None:
      return data
    else:
      return {name: IndexAsDict(data[name]) for name in data.dtype.names}


def DictSkeleton(data, record=None, depth=None):
  """Returns a skeleton of the compound Numpy struct.

  Args:
    data: A Numpy struct.
    record: The record to retrive the skeleton. A record is referred to as
        a list of strings, each string is an index to the next level of
        the compound Numpy struct. (default: None)
    depth: The number of levels to probe. If None, it probes all paths.
        (default: None)

  Raises:
    NumpyIndexError: If the record index cannot be found.

  Returns:
    A dictionary representing the structural skeleton. At the leaves are
    tuples describing the shape of the multi-dimensional array, or {} if
    it stops probing due to the depth limit.
  """
  if depth is not None:
    if depth <= 0:
      if data.dtype.names is None:
        return data.shape
      else:
        # Indicate that it can be further expanded.
        return {}
    depth -= 1
  if record is None:
    record = []
  # This is a workaround of an h5py bug dealing with zero length data sets.
  if numpy.prod(data.shape) == 0:
    return _BuildEmptyDict(_DeepSelect(data.dtype, record), data.shape, True)
  else:
    data = _DeepSelect(data, record)
    if data.dtype.names is None:
      return data.shape
    else:
      return {name: DictSkeleton(data[name], depth=depth) for name
              in data.dtype.names}


def _DeepSelect(t, record):
  """Private function for indexing into a dict of dicts of dicts."""
  for i in range(len(record)):
    try:
      t = t[record[i]]
    except (KeyError, ValueError), e:
      raise NumpyIndexError(None, record[:i+1], e.message)
  return t


def _BuildEmptyDict(t, shape, is_skeleton=False):
  """Private function for constructing an empty dict based on a Numpy dtype."""
  if t.names is None:
    if is_skeleton:
      return shape
    else:
      return numpy.zeros(shape)
  else:
    return {name: _BuildEmptyDict(t[name], shape) for name in t.names}


def NumpyToJson(data):
  """Regularize Numpy data so that they are JSON-serializable."""

  if isinstance(data, numpy.ndarray):
    if not data.dtype.names:
      return data.tolist()
    else:
      return {key: NumpyToJson(data[key]) for key in data.dtype.names}
  elif isinstance(data, numpy.integer):
    return int(data)
  elif isinstance(data, numpy.inexact):
    return float(data)
  elif isinstance(data, dict):
    return {key: NumpyToJson(value) for key, value in data.iteritems()}
  elif isinstance(data, (tuple, set, list)):
    return type(data)(NumpyToJson(value) for value in data)
  elif isinstance(data, (int, float, str, long, unicode)) or data is None:
    return data
  else:
    raise TypeError('Invalid NumPy data type "%s" for JSON conversion.' %
                    type(data))


def CartesianToPolar(x, y):
  """Convert cartesian coordinates to polar coordinates."""
  rho = numpy.sqrt(x**2 + y**2)
  phi = numpy.arctan2(y, x)
  return rho, phi


def PolarToCartesian(rho, phi):
  """Convert polar coordinates to cartesian coordinates."""
  x = rho * numpy.cos(phi)
  y = rho * numpy.sin(phi)
  return x, y


def Vec3ToArray(vec3):
  """Returns a view of a Vec3 timeseries as timeseries of length-3 arrays."""
  return vec3.view(numpy.dtype(('>f8', 3)))


def Vec3Norm(vec3):
  """Returns a timeseries norm of a Vec3 timeseries."""
  return numpy.linalg.norm(Vec3ToArray(vec3), axis=1)


def Mat3Vec3Mult(mat3, vec3):
  """Performs elementwise Mat3-Vec3 multiplication."""
  return numpy.einsum('lij,lj->li', mat3['d'], Vec3ToArray(vec3))


def Mat3TransVec3Mult(mat3, vec3):
  """Performs elementwise Mat3.transpose-Vec3 multiplication."""
  return numpy.einsum('lij,li->lj', mat3['d'], Vec3ToArray(vec3))


def Wrap(values, lower_bound, upper_bound):
  """Wrap-around values within lower (inclusive) and upper (exclusive) bounds.

  Example: Wrap(np.array([-200, 60, 270]), -180, 180) = np.array([160, 60, -90])

  Args:
    values: The input numpy array to be wrapped.
    lower_bound: The lower bound of the wrap-around range.
    upper_bound: The upper bound of the wrap-around range.

  Returns:
    A new numpy array with values wrapped around.
  """
  return (values - lower_bound) % (upper_bound - lower_bound) + lower_bound

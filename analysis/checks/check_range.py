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

"""Classes to check values within certain ranges."""

import numbers
import numpy


class BaseRange(object):
  """A base class for all the range types."""

  def __contains__(self, value):
    """Check whether a value or an entire array falls within the range.

    Args:
      value: The value to check. It can be a scalar or a NumPy array.

    Returns:
      True if the scalar falls within the range, or all elements of a NumPy
      array fall within the range.
    """
    raise NotImplementedError

  def Select(self, ndarray):
    """Return a bit mask for array elements falling within the range.

    Args:
      ndarray: A NumPy array of values.

    Returns:
      A Boolean array telling whether each element falls within the range.
    """
    raise NotImplementedError

  def __str__(self):
    raise NotImplementedError

  def RawArgs(self):
    """Returns basic Python objects to reconstruct an equivalent BaseRange."""
    raise NotImplementedError


class Interval(BaseRange):
  """An interval range to check for inclusion and select elements."""

  def __init__(self, bounds, inclusiveness=(True, True)):
    """Specify the interval bounds.

    Args:
      bounds: The pair of lower and upper bounds to check for inclusion.
          If any bound is None, then it means unbounded.

      inclusiveness: A list/tuple of two Boolean variables specifying the
          inclusiveness of the bounds. True means inclusive, False otherwise.
          Only applicable if the value has lower/upper bounds.
    """

    assert self.AreValidBounds(bounds)
    self._lower, self._upper = bounds
    self._inclusiveness = inclusiveness

  def RawArgs(self):
    return [[self._lower, self._upper]]

  def __str__(self):
    lower = '-inf' if self._lower is None else self._lower
    lower_closure = '[' if self._inclusiveness[0] else '('
    upper = 'inf' if self._upper is None else self._upper
    upper_closure = ']' if self._inclusiveness[1] else ')'
    return '%s%s, %s%s' % (lower_closure, lower, upper, upper_closure)

  @classmethod
  def AreValidBounds(cls, pair):
    return (isinstance(pair, (list, tuple)) and len(pair) == 2 and
            (pair[0] is None or pair[1] is None or pair[0] < pair[1]))

  def _CheckUpperBound(self, value):
    if self._inclusiveness[1]:
      return value <= self._upper
    else:
      return value < self._upper

  def _CheckLowerBound(self, value):
    if self._inclusiveness[0]:
      return value >= self._lower
    else:
      return value > self._lower

  def __contains__(self, value):
    """Check whether a value or an entire array falls within the range."""

    if isinstance(value, numpy.ndarray):
      if (self._lower is not None
          and not self._CheckLowerBound(value).all()):
        return False

      if (self._upper is not None
          and not self._CheckUpperBound(value).all()):
        return False

      return True

    else:
      if self._lower is not None and not self._CheckLowerBound(value):
        return False

      if self._upper is not None and not self._CheckUpperBound(value):
        return False

      return True

  def Select(self, ndarray):
    """Return a bit mask for array elements falling within the range."""

    falls_within = numpy.ones(ndarray.shape, dtype=bool)
    if self._lower is not None:
      falls_within &= self._CheckLowerBound(ndarray)
    if self._upper is not None:
      falls_within &= self._CheckUpperBound(ndarray)
    return falls_within


class Singleton(BaseRange):
  """A singleton range to check for equality."""

  def __init__(self, value):
    assert value is not None
    self._value = value

  def __contains__(self, value):
    """Check whether a value or an entire array equals the singleton."""

    if isinstance(value, numpy.ndarray):
      return self.Select(value).all()
    else:
      return value == self._value

  def __str__(self):
    return str(self._value)

  def RawArgs(self):
    return [self._value]

  def Select(self, ndarray):
    """Return a bit mask for array elements falling within the range."""

    return ndarray == self._value


class Container(BaseRange):
  """A container range to check for inclusion."""

  def __init__(self, container):
    assert isinstance(container, (dict, set, list, tuple))
    self._container = set(
        container.keys() if isinstance(container, dict) else container)
    self._numerics = []
    self._nonnumerics = []
    for v in self._container:
      if isinstance(v, numbers.Number):
        self._numerics.append(v)
      else:
        self._nonnumerics.append(v)
    self._numerics = numpy.array(self._numerics)
    self._nonnumerics = numpy.array(self._nonnumerics)

  def RawArgs(self):
    """The list of a single set reconstructs an equivalent RangeChecker."""
    return [self._container]

  def __str__(self):
    keys = (self._container.keys()
            if isinstance(self._container, dict) else self._container)
    return '{' + str(sorted(keys))[1:-1] + '}'

  def __contains__(self, value):
    """Check whether a value or an entire array falls within the range."""
    if isinstance(value, numpy.ndarray):
      return self.Select(value).all()
    else:
      return value in self._container

  def Select(self, ndarray):
    """Return a bit mask for array elements falling within the range."""
    shape = ndarray.shape
    if len(shape) != 1:
      ndarray = numpy.flatten(ndarray)

    mask = (numpy.in1d(ndarray, self._numerics) |
            numpy.in1d(ndarray, self._nonnumerics))

    if len(shape) != 1:
      mask = mask.reshape(shape)

    return mask


class AllInclusiveRange(BaseRange):
  """A range that always passes the check."""

  def __contains__(self, value):
    """Check whether a value or an entire array falls within the range."""
    return True

  def Select(self, ndarray):
    """Return a bit mask for array elements falling within the range."""
    return numpy.ones(ndarray.shape, dtype=bool)

  def __str__(self):
    return 'any'

  def RawArgs(self):
    return [[None, None]]


class AllExclusiveRange(BaseRange):
  """A range that always fails the check."""

  def __contains__(self, value):
    """Check whether a value or an entire array falls out of the range."""
    return False

  def Select(self, ndarray):
    """Return a bit mask for array elements falling out of the range."""
    return numpy.zeros(ndarray.shape, dtype=bool)

  def __str__(self):
    return 'none'

  def RawArgs(self):
    return [None]


class RangeChecker(BaseRange):
  """A generic class to check whether a value falls within several ranges."""

  def __init__(self, ranges):
    """Initialize a RangeChecker object.

    Args:
      ranges: A list of items each define a range to check. An item
          can be a CheckRange object or something that can be used to create a
          CheckRange object. Example items include:
          A scalar (int, float, string, bool, etc),
          A list of two numerical values, for inclusive ranges,
          A collection (set/dict/list/tuple) of scalars.

    Raises:
      TypeError: Raised if the input is not valid.
    """

    if not isinstance(ranges, (list, set)):
      raise TypeError('Invalid type to create BaseRange objects.')

    self._ranges = []
    for value in ranges:
      if isinstance(value, (Interval, Singleton, Container)):
        self._ranges.append(value)
      elif Interval.AreValidBounds(value):
        self._ranges.append(Interval(value))
      elif isinstance(value, (int, float, long, str, unicode, bool)):
        self._ranges.append(Singleton(value))
      elif value == [None, None]:
        self._ranges = [AllInclusiveRange()]
        break
      elif value is None:
        self._ranges = [AllExclusiveRange()]
        break
      elif isinstance(value, (dict, set, list, tuple)):
        self._ranges.append(Container(value))
      else:
        raise TypeError('Invalid type to create BaseRange objects.')

  def __str__(self):
    return '[' + ', '.join(str(r) for r in self._ranges) + ']'

  def RawArgs(self):
    raw_args = []
    for value in self._ranges:
      raw_args += value.RawArgs()
    return raw_args

  def __contains__(self, value):
    """Check whether a value is included in any ranges."""

    if isinstance(value, numpy.ndarray):
      return self.Select(value).all()
    else:
      for one_range in self._ranges:
        if value in one_range:
          return True
      return False

  def Select(self, ndarray):
    """Return a bit mask for array elements falling in any of the ranges."""

    if not self._ranges:
      # With an empty list of ranges, nothing is selected.
      return numpy.zeros(ndarray.shape, dtype=bool)
    else:
      selection = numpy.zeros(ndarray.shape, dtype=bool)
      for one_range in self._ranges:
        selection |= one_range.Select(ndarray)
      return selection


def BuildRanges(ranges):
  """Build BaseRange objects from raw Python objects."""
  if isinstance(ranges, BaseRange):
    return ranges
  else:
    return RangeChecker(ranges)

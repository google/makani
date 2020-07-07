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

"""Helper functions for additional type-checking."""

import collections

import numpy as np


class MatrixShapeOrTypeException(Exception):
  """Exception indicating a non-matrix argument."""
  pass


def CheckIsMatrix(arg, shape=None):
  return isinstance(arg, np.matrix) and (shape is None or arg.shape == shape)


def RequireMatrixArguments(*shapes):
  """A decorator that ensures arguments are np.matrix objects of given shapes.

  Args:
    *shapes: A list whose elements are either None or two element tuples (n, m).
        There should be one element per argument to the decorated function.  The
        non-None arguments will be required to be n-by-m numpy.matrix objects.

  Returns:
    Decorator.
  """
  def _CheckArguments(f):
    assert len(shapes) == f.func_code.co_argcount
    def _Wrapped(*args, **kwargs):
      for (arg, shape) in zip(args, shapes):
        if shape is not None and not CheckIsMatrix(arg, shape=shape):
          raise MatrixShapeOrTypeException(shape)
      return f(*args, **kwargs)
    return _Wrapped
  return _CheckArguments


def MakeNamedVectorClass(name, field_indices):
  """Generate a class for handling vectors with named sub-components.

  Returns a class which extends collections.namedtuple so that each
  element is a "slice" of a dim-by-1 np.matrix.  The field_indices
  argument is a list of pairs.  The first entry of each pair is the
  field name, the second entry is a list of vector indices, e.g.:

  FooClass = MakeNamedVectorClass('Foo', [('r', [0]), ('i', [1, 2, 3])])

  foo_instance = Foo(r=np.matrix[[1.0]], i=np.matrix[[2.0], [3.0], [4.0]])

  Here, the total dimension is 4, and foo_instance.ToVector() will be
  np.matrix([[0.0], [1.0], [2.0], [3.0]]).

  Args:
    name: Name to give the class.
    field_indices: List of tuples defining the class as above.

  Returns:
    Named vector class defined as above.
  """

  keys = [key for (key, _) in field_indices]
  indices = [index for (_, index) in field_indices]
  all_indices = []
  for index in indices:
    all_indices += index
  dim = len(all_indices)
  assert set(all_indices) == set(range(dim))

  tuple_type = collections.namedtuple(name + 'Repr', keys)

  class NamedVector(tuple_type):
    """Class representing a dim-by-1 np.matrix with named slices."""

    def __init__(self, *args, **kwargs):
      indices_dict = {key: index for (key, index) in field_indices}
      for (key, value) in kwargs.iteritems():
        if not CheckIsMatrix(value, shape=(len(indices_dict[key]), 1)):
          raise MatrixShapeOrTypeException((key, value))
      super(NamedVector, self).__init__(*args, **kwargs)

    def ToVector(self):
      """Return the dim-by-1 np.matrix combining the named component vectors."""
      vector = np.matrix(np.zeros((dim, 1)))
      for i, index in enumerate(indices):
        vector[index] = self[i]
      return vector

    @classmethod
    @RequireMatrixArguments(None, (dim, 1))
    def FromVector(cls, vector):
      """Inverse of ToVector()."""
      values = [None for _ in keys]
      for i, index in enumerate(indices):
        values[i] = vector[index]
      return cls(*values)

    @classmethod
    def GetIndices(cls):
      """Get a namedtuple whose elements are the component indices."""
      return tuple_type(*indices)

    @classmethod
    def GetDim(cls):
      return dim

    @classmethod
    def StepVector(cls, step_sizes):
      """Maps a {field_name: step_size} dict to a vector of step sizes."""
      step_vector = np.matrix(np.zeros((cls.GetDim(), 1)))
      indices = cls.GetIndices()
      for field_name, size in step_sizes.iteritems():
        step_vector[getattr(indices, field_name), 0] = size
      assert (step_vector > 0.0).all
      return step_vector

  return NamedVector


def MakeStateClass(name, field_indices):
  """Creates a class for representing system state.

  Generates a class for representing the state of a system where some
  components of the state lie on manifolds such as SO(3).  This
  involves constructing two classes.  The first is a class that
  behaves like a namedtuple with each entry being a component of the
  state.  The second class behaves like a NamedVector and represents a
  tangent vector for this space.  The user must make a subclass of
  this StateClass returned by this method to handle moving states
  along tangent directions and recovering tangent directions from
  pairs of states.  An example is given below:

  class AttitudeState(MakeStateClass(
      'AttitudeState, [('omega', range(0, 3)),
                       ('dcm_g2b', range(3, 6))])):
    def Increment(self, tangent, step=1.0):
      ...

    def Decrement(self, other_state):
      ...

  state = AttitudeState(omega=np.matrix(np.zeros((3, 1))),
                        dcm_g2b=np.matrix(np.eye(3)))

  tangent = AttitudeState.Tangent(domega=np.matrix([[1.0], [2.0], [3.0]]),
                                  ddcm_g2b=np.matrix([[4.0], [5.0], [6.0]]))

  # This is equivalent to np.matrix([[1.0], [2.0], [3.0], [4.0], [5.0], [6.0]]).
  tangent.ToVector()

  The structure of the state is given by field_indices which is a list
  of pairs (field_name, tangent_indices).  The string field_name gives
  a name to this component of the state.

  The Tangent class is a NamedVector with fields named 'd' +
  field_name which are stored in the tangent_indices components of the
  vector.

  Args:
    name: Name of the class to create.
    field_indices: List of pairs (field_name, tangent_indices) describing
        the structure of the class to create.

  Returns:
    A new class as described above.
  """
  keys = [key for (key, _) in field_indices]

  class StateClass(collections.namedtuple(name, keys)):
    """Class representing the state of a system."""

    Tangent = MakeNamedVectorClass(  # pylint: disable=invalid-name
        name + 'Tangent',
        [('d' + key, value) for (key, value) in field_indices])

    def Increment(self, tangent, step=1.0):
      raise NotImplementedError

    def Difference(self, other_state):
      raise NotImplementedError

  return StateClass


def MakeFlatStateClass(name, field_indices):
  """Creates a class for representing system state in R^n.

  Generates a class for representing the state of a system where
  the Tangent vectors can be defined by element-wise addition
  and subtraction of the states.

  Args:
    name: See MakeStateClass.
    field_indices: See MakeStateClass.

  Returns:
    A new class as described above.
  """

  class FlatStateClass(MakeStateClass(name, field_indices)):
    """StateClass representing a state in R^n."""

    def Increment(self, tangent, step=1.0):
      assert isinstance(tangent, FlatStateClass.Tangent)

      return FlatStateClass(
          *[value + step * tangent_value
            for (value, tangent_value) in zip(self, tangent)])

    def Difference(self, other_state):
      return FlatStateClass.Tangent(
          *[other_value - value
            for (other_value, value) in zip(other_state, self)])

  return FlatStateClass

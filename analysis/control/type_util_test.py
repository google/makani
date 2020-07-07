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

"""Tests for makani.analysis.control.type_util."""

import unittest

from makani.analysis.control import type_util
import numpy as np


_ExampleVector = type_util.MakeNamedVectorClass(  # pylint: disable=invalid-name
    'ExampleVector', [('bar', range(3, 6)),
                      ('foo', range(0, 3)),
                      ('baz', range(6, 9))])

_FlatExample = type_util.MakeFlatStateClass(  # pylint: disable=invalid-name
    'ExampleFlatPoint', [('bar', range(1, 4)),
                         ('foo', range(0, 1)),
                         ('baz', range(4, 7))])


class TypeUtilTest(unittest.TestCase):

  def testGetIndices(self):
    """Simple test of NamedVector.GetIndices()."""
    indices = _ExampleVector.GetIndices()
    all_indices = np.concatenate((indices.foo, indices.bar, indices.baz))
    self.assertEqual(set(all_indices),
                     set(range(_ExampleVector.GetDim())))

  def testStateToVector(self):
    """Simple test of NamedVector.ToVector and NamedVector.FromVector."""
    foo = np.matrix([[1.0], [2.0], [3.0]])
    bar = np.matrix([[4.0], [5.0], [6.0]])
    baz = np.matrix([[7.0], [8.0], [9.0]])
    example_vector = _ExampleVector(foo=foo, bar=bar, baz=baz).ToVector()
    self.assertEqual((_ExampleVector.GetDim(), 1), example_vector.shape)
    example = _ExampleVector.FromVector(example_vector)
    for i in range(3):
      self.assertEqual(foo[i, 0], example.foo[i, 0])
      self.assertEqual(bar[i, 0], example.bar[i, 0])
      self.assertEqual(baz[i, 0], example.baz[i, 0])

  def testFlatState(self):
    """Simple test of NamedVector.ToVector and NamedVector.FromVector."""
    foo = np.matrix([[1.0], [2.0], [3.0]])
    bar = np.matrix([[4.0], [5.0], [6.0]])
    baz = np.matrix([[7.0], [8.0], [9.0]])
    zero = np.matrix(np.zeros((3, 1)))
    zero_point = _FlatExample(foo=zero, bar=zero, baz=zero)
    example_point = _FlatExample(foo=foo, bar=bar, baz=baz)
    tangent = zero_point.Difference(example_point)
    self.assertEqual(foo[0, 0], tangent.dfoo[0, 0])
    for i in range(3):
      self.assertEqual(bar[i, 0], tangent.dbar[i, 0])
      self.assertEqual(baz[i, 0], tangent.dbaz[i, 0])


if __name__ == '__main__':
  unittest.main()

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

import os
import tempfile
import unittest

from makani.lib.python import test_util
from makani.lib.python.h5_utils import numpy_utils
import numpy


class TestNumpyUtils(unittest.TestCase):

  def setUp(self):
    data = [('spire', '250um', [(0, 1.89e6, 0.0), (1, 2e6, 1e-2)]),
            ('spire', '350', [(0, 1.89e6, 0.0), (2, 2.02e6, 3.8e-2)])
           ]
    self._table = numpy.array(data, dtype=[('instrument', '|S32'),
                                           ('filter', '|S64'),
                                           ('response', [('linenumber', 'i'),
                                                         ('wavelength', 'f'),
                                                         ('throughput', 'f')],
                                            (2,))
                                          ])

  def testIndexAsDictDefault(self):
    np_dict = numpy_utils.IndexAsDict(self._table)
    expected = {'instrument', 'filter', 'response'}
    self.assertEqual(set(np_dict.keys()), expected)
    check_list = ['instrument', 'filter']
    for item in check_list:
      self.assertTrue(numpy.array_equal(self._table[item],
                                        np_dict[item]))

    expected = {'wavelength', 'throughput', 'linenumber'}
    self.assertEqual(set(np_dict['response'].keys()), expected)
    for item in expected:
      self.assertTrue(numpy.array_equal(self._table['response'][item],
                                        np_dict['response'][item]))

  def testIndexAsDict(self):
    index = 'filter'
    np_dict = numpy_utils.IndexAsDict(self._table, [index])
    self.assertTrue(numpy.array_equal(self._table[index],
                                      np_dict))

    index = ['response']
    np_dict = numpy_utils.IndexAsDict(self._table, index)
    expected = {'wavelength', 'throughput', 'linenumber'}
    self.assertEqual(set(np_dict.keys()), expected)
    for item in expected:
      self.assertTrue(numpy.array_equal(self._table['response'][item],
                                        np_dict[item]))

    index = ['response', 'wavelength']
    np_dict = numpy_utils.IndexAsDict(self._table, index)
    reference = self._table
    for idx in index:
      reference = reference[idx]
    self.assertTrue(numpy.array_equal(reference, np_dict))

  def testBadIndex(self):
    with self.assertRaises(numpy_utils.NumpyIndexError):
      index = ['bad_index']
      numpy_utils.IndexAsDict(self._table, index)

    with self.assertRaises(numpy_utils.NumpyIndexError):
      index = ['response', 'bad_index']
      numpy_utils.IndexAsDict(self._table, index)

  def testDictSkeleton(self):
    response_skeleton = {
        'linenumber': (2, 2),
        'wavelength': (2, 2),
        'throughput': (2, 2),
    }

    table_skeleton = {
        'instrument': (2,),
        'filter': (2,),
        'response': response_skeleton,
    }

    skeleton = numpy_utils.DictSkeleton(self._table, ['response', 'linenumber'])
    self.assertEqual(skeleton, (2, 2))

    skeleton = numpy_utils.DictSkeleton(self._table)
    self.assertEqual(skeleton, table_skeleton)

    table_skeleton['response'] = {}
    skeleton = numpy_utils.DictSkeleton(self._table, depth=1)
    self.assertEqual(skeleton, table_skeleton)

  def testH5MathHelpers(self):
    num_samples = 10

    # A fake log file is used strictly to get appropriate dtypes for the Vec3
    # and Mat3 timeseries.
    with tempfile.NamedTemporaryFile(suffix='.h5', delete=False) as tmp:
      file_name = tmp.name
    log_file = test_util.CreateSampleHDF5File(file_name, 1)
    state_est = (log_file['messages']['kAioNodeControllerA']
                 ['kMessageTypeControlTelemetry']['message']['state_est'])
    vec3 = numpy.zeros(num_samples, dtype=state_est['Xg'].dtype)
    mat3 = numpy.zeros(num_samples, dtype=state_est['dcm_g2b'].dtype)
    log_file.close()
    os.remove(file_name)

    # Populate vec3 and mat3 with random data.
    for axis in ('x', 'y', 'z'):
      vec3[axis] = numpy.random.rand(num_samples)
    mat3['d'] = numpy.random.rand(num_samples, 3, 3)

    # Test Vec3ToArray. Values of `vec3` and `array` should be bitwise
    # identical, so equality is appropriate.
    array = numpy_utils.Vec3ToArray(vec3)
    for i, axis in enumerate(('x', 'y', 'z')):
      self.assertTrue((array[:, i] == vec3[axis]).all())

    # Test Vec3Norm.
    norm1 = (vec3['x']**2.0 + vec3['y']**2.0 + vec3['z']**2.0)**0.5
    norm2 = numpy_utils.Vec3Norm(vec3)
    self.assertTrue((numpy.abs(norm1 - norm2) < 1e-15).all())

    # Test Mat3Vec3Mult.
    mult = numpy_utils.Mat3Vec3Mult(mat3, vec3)
    for i in range(num_samples):
      b_expected = numpy.dot(
          mat3['d'][i],
          numpy.array([vec3['x'][i], vec3['y'][i], vec3['z'][i]]))
      self.assertTrue((numpy.abs(mult[i, :] - b_expected) < 1e-15).all())

    # Test Mat3TransVec3Mult.
    trans_mult = numpy_utils.Mat3TransVec3Mult(mat3, vec3)
    for i in range(num_samples):
      b_expected = numpy.dot(
          mat3['d'][i].T,
          numpy.array([vec3['x'][i], vec3['y'][i], vec3['z'][i]]))
      self.assertTrue((numpy.abs(trans_mult[i, :] - b_expected) < 1e-15).all())

  def testWrap(self):
    lower = -numpy.pi
    upper = numpy.pi
    period = upper - lower

    values = numpy.array([-1.2, 0.0, 1.2])

    # No need to wrap around.
    self.assertTrue(
        (numpy.abs(
            numpy_utils.Wrap(values, lower, upper) - values) < 1e-6).all())

    # Wrap-around values that are below the lower bound.
    self.assertTrue(
        (numpy.abs(
            numpy_utils.Wrap(values - period * 10, lower, upper)
            - values) < 1e-6).all())

    # Wrap-around values that are more than the upper bound.
    self.assertTrue(
        (numpy.abs(
            numpy_utils.Wrap(values + period * 10, lower, upper)
            - values) < 1e-6).all())

    # Lower bounds are inclusive and upper bounds are exclusive.
    self.assertTrue(
        (numpy.abs(
            numpy_utils.Wrap(numpy.array([lower, upper]), lower, upper)
            - numpy.array([lower, lower])) < 1e-6).all())


if __name__ == '__main__':
  unittest.main()

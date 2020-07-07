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

import copy
import tempfile
import unittest

from makani.gs.monitor2.apps.receiver import test_util
from makani.lib.python import ctype_util
from makani.lib.python import struct_tree
from makani.lib.python.h5_utils import h5_io
from makani.lib.python.h5_utils import h5_io_test
import numpy as np


class TestStructTreeIndex(unittest.TestCase):

  def setUp(self):
    self.array = np.array([(0.0, 20), (1.0, 40), (2.0, 80), (3.0, 160)],
                          dtype=[('x', '<f8'), ('y', '<i8')])

    self._data = {
        'numbers': [
            {1: ['apple', 'antenna', 'alps']},
            {2: ['banana', 'bus', 'bound']},
            {3: ['car', 'cat', 'circuit']},
            {4: ['dance', 'deer', 'direction']},
        ],
        'strings': {
            'a': ['apple', 'antenna', 'alps'],
            'b': ['banana', 'bus', 'bound'],
            'c': ['car', 'cat', 'circuit'],
            'd': ['dance', 'deer', 'direction'],
        },
        'deep': {
            'x': {
                'y': {
                    'z': 0
                }
            }
        },
        'numpy': self.array
    }
    self._struct_tree = struct_tree.StructTree(self._data)
    self._ndarray = np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8]])
    self._numpy_tree = struct_tree.StructTree({'array': self._ndarray})
    self._ctype_tree = test_util.SynthesizeMessages('MotorStatus', 0)
    self.assertIn('strings.a', self._struct_tree)
    self.assertNotIn('strings.z', self._struct_tree)
    self.assertNotIn('shallow.a', self._struct_tree)

  def testCache(self):
    tree = struct_tree.StructTree(self._data, readonly=True)
    self.assertEqual(tree['strings.a'], ['apple', 'antenna', 'alps'])
    self._data['strings']['a'] = ['potatoes', 'tomatoes']
    # Data has changed, but StructTree should still return cached data.
    self.assertEqual(tree['strings.a'], ['apple', 'antenna', 'alps'])
    # If we re-create a tree, then it should return the new data.
    tree = struct_tree.StructTree(self._data, readonly=True)
    self.assertEqual(tree['strings.a'], ['potatoes', 'tomatoes'])

  def testSubtree(self):
    self.assertEqual(
        self._struct_tree.Subtree('strings.c').Data(
            convert_to_basic_types=True),
        ['car', 'cat', 'circuit'])
    self.assertEqual(self._struct_tree.Subtree('deep.x')['y.z'], 0)

    self._struct_tree = struct_tree.StructTree(self._data)
    with tempfile.NamedTemporaryFile() as temp_hdf5:
      data = copy.copy(self._data)
      # Lists of dicts are not valid HDF5 objects. Remove them.
      del data['numbers']
      h5_io.H5Dump(temp_hdf5.name, data)
      h5_tree = struct_tree.StructTree(temp_hdf5.name)
      self.assertEqual(
          h5_tree.Subtree('strings.c').Data(convert_to_basic_types=True),
          ['car', 'cat', 'circuit'])
      # Dataset[()] is h5py's way to get the value of a scalar dataset.
      self.assertEqual(h5_tree.Subtree('deep.x')['y.z'][()], 0)

  def testSkeleton(self):
    skeleton = {
        'numbers': [
            {1: []},
            {2: []},
            {3: []},
            {4: []},
        ],
        'strings': {
            'a': [],
            'b': [],
            'c': [],
            'd': [],
        },
        'deep': {'x': {'y': {'z': None}}},
        'numpy': {'y': (4,), 'x': (4,)},
    }
    self.assertEqual(self._struct_tree.Skeleton(), skeleton)

  def testOneLayer(self):
    self.assertEqual(self._struct_tree['numbers'], self._data['numbers'])

  def testMultiLayer(self):
    self.assertEqual(self._struct_tree['strings.a'], ['apple', 'antenna',
                                                      'alps'])
    self.assertEqual(self._struct_tree['strings', 'a'], ['apple', 'antenna',
                                                         'alps'])
    self.assertEqual(self._struct_tree['deep.x.y.z'], 0)
    self.assertEqual(self._struct_tree['deep', 'x', 'y', 'z'], 0)

  def testMultiDict(self):
    self.assertEqual(self._struct_tree['strings[:][0]'], {
        'a': 'apple',
        'b': 'banana',
        'c': 'car',
        'd': 'dance',
    })

  def testMultiLayerIndex(self):
    self.assertEqual(self._struct_tree['strings.a[1]'], 'antenna')

  def testMultiLayerSublist(self):
    self.assertEqual(self._struct_tree['strings.a[0:3:2]'], ['apple', 'alps'])
    self.assertEqual(self._struct_tree['strings.a[ 0 : 3 : 2 ]'],
                     ['apple', 'alps'])

  def testMultiIndex(self):
    self.assertEqual(self._struct_tree['strings[{a, b}]'],
                     {
                         'a': ['apple', 'antenna', 'alps'],
                         'b': ['banana', 'bus', 'bound'],
                     })
    self.assertEqual(self._struct_tree['strings[{a, b}][0:3:2]'],
                     {
                         'a': ['apple', 'alps'],
                         'b': ['banana', 'bound'],
                     })
    self.assertEqual(self._struct_tree['strings[ {a, b} ][1]'],
                     {
                         'a': 'antenna',
                         'b': 'bus',
                     })

  def testNumpy(self):
    self.assertTrue(np.array_equal(self._struct_tree['numpy']['x'],
                                   np.array([0.0, 1.0, 2.0, 3.0])))
    self.assertTrue(np.array_equal(self._struct_tree['numpy']['y'][1:3],
                                   np.array([40, 80])))
    self.assertTrue(np.array_equal(self._numpy_tree['array[1]'],
                                   np.array([3, 4, 5])))
    self.assertTrue(np.array_equal(self._numpy_tree['array[1:3]'],
                                   np.array([[3, 4, 5], [6, 7, 8]])))
    self.assertIn('array', self._numpy_tree)
    self.assertNotIn('not_exist', self._numpy_tree)
    self.assertEqual(self._numpy_tree['array[1:3][0]'], [3, 6])
    self.assertEqual(self._numpy_tree['array[1, 2]'], 5)
    self.assertEqual(self._numpy_tree['array[1][2]'], 5)

  def testCtype(self):
    self.assertEqual(self._ctype_tree['MotorStatus.MotorPbi.omega'], 0.0)
    self.assertEqual(
        self._ctype_tree['MotorStatus.MotorPbi.motor_mon.ina219_populated'], 0)
    self.assertEqual(self._ctype_tree['MotorStatus.MotorPbi.temps[1:3]'],
                     [0.0, 0.0])
    self.assertNotIn('MotorStatus.MotorPbi.not_exist_field', self._ctype_tree)
    self.assertEqual(
        ctype_util.CTypeToPython(
            self._ctype_tree['MotorStatus.MotorPbi.motor_mon.flags']),
        {'status': 0, 'warning': 0, 'error': 0})

    skeleton = self._ctype_tree.Skeleton('MotorStatus.MotorPbi', 1)
    self.assertEqual(skeleton['temps'], [])
    self.assertEqual(skeleton['omega'], None)

    skeleton = self._ctype_tree.Skeleton('MotorStatus.MotorPbi.motor_mon', 1)
    self.assertEqual(skeleton['ina219_populated'], None)
    self.assertEqual(skeleton['ina219_data'], [])
    self.assertEqual(skeleton['flags'], {})

  def testFailures(self):

    with self.assertRaises(KeyError):
      self._struct_tree['invalid']  # pylint: disable=pointless-statement

    with self.assertRaises(KeyError):
      self._struct_tree['strings[a, b]']  # pylint: disable=pointless-statement

    with self.assertRaises(KeyError):
      self._struct_tree['strings{a, b}']  # pylint: disable=pointless-statement

    with self.assertRaises(KeyError):
      self._struct_tree['strings.a{1,3}']  # pylint: disable=pointless-statement

    with self.assertRaises(KeyError):
      self._struct_tree['strings[a][x]']  # pylint: disable=pointless-statement

    with self.assertRaises(KeyError):
      self._numpy_tree['array[1][:][0]']  # pylint: disable=pointless-statement

    with self.assertRaises(KeyError):
      self._ctype_tree[  # pylint: disable=pointless-statement
          'MotorStatus.BadMotor']

  def testEmpty(self):
    self.assertTrue(self._ctype_tree)
    self.assertTrue(self._numpy_tree)
    self.assertTrue(self._struct_tree)
    self.assertFalse(struct_tree.StructTree({}))

  def testDictToD3Tree(self):
    data = {
        'deep': {'x': {'y': None}},
        'format': {'array': (4,), 'list': (4,)},
        'numbers': [
            {'pi': []},
            {'sigma': []},
        ],
    }

    expected = {
        'path': '',
        'leaf': False,
        'name': 'root',
        'children': [
            {
                'path': 'numbers',
                'leaf': False,
                'name': 'numbers',
                'children': [
                    {
                        'path': 'numbers.0',
                        'leaf': False,
                        'name': 0,
                        'children': [
                            {
                                'path': 'numbers.0.pi',
                                'leaf': False,
                                'name': 'pi'
                            }
                        ]
                    },
                    {
                        'path': 'numbers.1',
                        'leaf': False,
                        'name': 1,
                        'children': [
                            {
                                'path': 'numbers.1.sigma',
                                'leaf': False,
                                'name': 'sigma'
                            }
                        ]
                    }
                ]
            },
            {
                'path': 'deep',
                'leaf': False,
                'name': 'deep',
                'children': [
                    {
                        'path': 'deep.x',
                        'leaf': False,
                        'name': 'x',
                        'children': [
                            {
                                'path': 'deep.x.y',
                                'leaf': True,
                                'name': 'y'
                            }
                        ]
                    }
                ]
            },
            {
                'path': 'format',
                'leaf': False,
                'name': 'format',
                'children': [
                    {
                        'path': 'format.array[:]',
                        'leaf': True,
                        'name': 'array[4]'
                    },
                    {
                        'path': 'format.list[:]',
                        'leaf': True,
                        'name': 'list[4]'
                    }
                ]
            }
        ]
    }

    self.assertEqual(struct_tree.DictToD3Tree(data, 'root'), expected)


class TestStructTreeForHDF5(h5_io_test.TestWithComplexHDF5):

  def testH5Extract(self):
    hdf5_tree = struct_tree.StructTree(self.tempdata_fp.name)

    self.assertEqual(3.4, hdf5_tree.Index('numbers.floats[1]'))
    self.assertTrue(np.array_equal([1, 2, 3], hdf5_tree.Index('dataset[:]')))

    self.assertEqual({
        'wavelength': np.float32(2e6),
        'throughput': np.float32(1.6),
        'linenumber': np.float32(1),
    }, hdf5_tree.Index('numbers.table.response[:][0, 1]'))

    expected = {
        'wavelength': np.array([1.89e6, 2e6]),
        'throughput': np.array([0., 1.6]),
        'linenumber': np.array([0, 1])
    }
    data = hdf5_tree.Index('numbers.table.response[:][0]')
    self.assertEqual(set(data.keys()), set(expected.keys()))
    for key in data.keys():
      np.testing.assert_array_almost_equal(data[key], expected[key])

  def testH5Skeleton(self):
    hdf5_tree = struct_tree.StructTree(self.tempdata_fp.name)
    table_skeleton = {
        'instrument': (2,),
        'filter': (2,),
        'response': {
            'linenumber': (2, 2),
            'wavelength': (2, 2),
            'throughput': (2, 2),
        }
    }
    self.assertEqual(hdf5_tree.Skeleton(['numbers', 'table']), table_skeleton)

    flattened_table_skeleton = [
        ('instrument', (2,)),
        ('filter', (2,)),
        ('response.linenumber', (2, 2)),
        ('response.wavelength', (2, 2)),
        ('response.throughput', (2, 2)),
    ]
    self.assertEqual(set(hdf5_tree.ListSkeleton(['numbers', 'table'])),
                     set(flattened_table_skeleton))

    expected_skeleton = {
        'dataset': (3,),
        'empty_group': None,  # An empty group's skeleton is None.
        'numbers': {
            'integers': (3,),
            'floats': (3,),
            'nans': None,
            'table': table_skeleton,
        }
    }
    self.assertEqual(hdf5_tree.Skeleton(), expected_skeleton)

    expected_flattened_skeleton = [
        ('dataset', (3,)),
        ('empty_group', None),  # An empty group's skeleton is None.
        ('numbers.integers', (3,)),
        ('numbers.floats', (3,)),
        ('numbers.nans', None),
    ] + [
        ('numbers.table.%s' % k, v) for k, v in flattened_table_skeleton
    ]

    self.assertEqual(set(hdf5_tree.ListSkeleton()),
                     set(expected_flattened_skeleton))

    expected_skeleton['numbers']['table'] = {}
    self.assertEqual(hdf5_tree.Skeleton(depth=2), expected_skeleton)


if __name__ == '__main__':
  unittest.main()

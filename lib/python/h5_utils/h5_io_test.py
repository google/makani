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

import tempfile
import unittest

from makani.lib.python.h5_utils import h5_io
from makani.lib.python.h5_utils import numpy_utils
import numpy


class TestWithComplexHDF5(unittest.TestCase):

  def setUp(self):
    data = [('spire', '250um', [(0, 1.89e6, 0.0), (1, 2e6, 1.6)]),
            ('spire', '350', [(0, 1.89e6, 0.0), (2, 2.02e6, 3.8)])
           ]
    self.table = numpy.array(data, dtype=[('instrument', '|S32'),
                                          ('filter', '|S64'),
                                          ('response', [('linenumber', 'i'),
                                                        ('wavelength', 'f'),
                                                        ('throughput', 'f')],
                                           (2,))
                                         ])

    self.integers = numpy.array([1, 2, 3])
    self.floats = numpy.array([1.2, 3.4, 5.6])
    self.datasets = {
        'dataset': self.integers,
        'empty_group': {},  # Empty group.
        'numbers': {
            'integers': self.integers,
            'floats': self.floats,
            'nans': None,  # Empty group.
            'table': self.table,
        }
    }
    self.tempdata_fp = tempfile.NamedTemporaryFile()
    h5_io.H5Dump(self.tempdata_fp.name, self.datasets)

  def tearDown(self):
    self.tempdata_fp.close()


class TestH5IO(TestWithComplexHDF5):

  def testH5ListAll(self):
    catalog = h5_io.H5List(self.tempdata_fp.name, list_all=True)
    expected = {'/', u'/dataset', '/empty_group', '/numbers',
                '/numbers/floats', '/numbers/integers', '/numbers/nans',
                '/numbers/table'}
    self.assertEqual(set(catalog), expected)

  def testH5ListNonExist(self):
    with self.assertRaises(h5_io.H5IndexError):
      h5_io.H5List(self.tempdata_fp.name, 'nonexist_name', list_all=True)

  def testH5ListDataset(self):
    catalog = h5_io.H5List(self.tempdata_fp.name, list_all=False)
    expected = {
        '/dataset',
        '/numbers/floats',
        '/numbers/integers',
        '/numbers/table',
    }
    self.assertEqual(set(catalog), expected)

  def testH5ListNode(self):
    catalog = h5_io.H5List(self.tempdata_fp.name,
                           '/numbers',
                           list_all=False)
    expected = {
        '/numbers/floats',
        '/numbers/integers',
        '/numbers/table',
    }
    self.assertEqual(set(catalog), expected)

  def testH5ReadDataset(self):
    with self.assertRaises(h5_io.H5IndexError):
      data = h5_io.H5ReadDataset(self.tempdata_fp.name, '/nonexist')

    check_list = [
        ('/dataset', self.integers),
        ('/numbers/integers', self.integers),
        ('/numbers/floats', self.floats),
        ('/numbers/table', self.table),
    ]
    for node, reference in check_list:
      data = h5_io.H5ReadDataset(self.tempdata_fp.name, node)
      self.assertTrue(numpy.array_equal(data, reference))

  def testH5LoadDatasets(self):
    with self.assertRaises(h5_io.H5IndexError):
      data = h5_io.H5LoadDatasets(self.tempdata_fp.name, '/nonexist')

    data = h5_io.H5LoadDatasets(self.tempdata_fp.name, '/')
    expected = {
        '/dataset',
        '/numbers/floats',
        '/numbers/integers',
        '/numbers/table',
    }
    self.assertEqual(set(data.keys()), expected)

    check_list = [
        ('/dataset', self.integers),
        ('/numbers/integers', self.integers),
        ('/numbers/floats', self.floats),
        ('/numbers/table', self.table),
    ]
    for node, reference in check_list:
      self.assertTrue(numpy.array_equal(data[node], reference))

  def testH5Load(self):
    with self.assertRaises(h5_io.H5IndexError):
      data = h5_io.H5Load(self.tempdata_fp.name, '/nonexist')

    data = h5_io.H5Load(self.tempdata_fp.name, '/')
    expected = {
        'dataset',
        'empty_group',
        'numbers',
    }
    self.assertEqual(set(data.keys()), expected)
    self.assertEqual(data['empty_group'], {})

    expected = {
        'floats',
        'integers',
        'nans',
        'table',
    }
    self.assertEqual(set(data['numbers'].keys()), expected)

    self.assertTrue(numpy.array_equal(data['dataset'], self.integers))
    self.assertTrue(numpy.array_equal(data['numbers']['integers'],
                                      self.integers))
    self.assertTrue(numpy.array_equal(data['numbers']['floats'], self.floats))
    self.assertTrue(numpy.array_equal(data['numbers']['table'], self.table))
    # '/numbers/nans' is None and is converted to an empty group.
    self.assertEqual(data['numbers']['nans'], {})

  def _CheckTableCorrectness(self, np_dict):
    expected = {'instrument', 'filter', 'response'}
    self.assertEqual(set(np_dict.keys()), expected)
    check_list = ['instrument', 'filter']
    for item in check_list:
      self.assertTrue(numpy.array_equal(self.table[item],
                                        np_dict[item]))

    expected = {'wavelength', 'throughput', 'linenumber'}
    self.assertEqual(set(np_dict['response'].keys()), expected)
    for item in expected:
      self.assertTrue(numpy.array_equal(self.table['response'][item],
                                        np_dict['response'][item]))

  def testH5FileToDictBadIndex(self):
    with self.assertRaises(h5_io.H5IndexError):
      h5_io.H5FileToDict(self.tempdata_fp.name, '/nonexist')

    with self.assertRaises(numpy_utils.NumpyIndexError):
      h5_io.H5FileToDict(self.tempdata_fp.name, '/numbers/table', ['nonexist'])

    with self.assertRaises(h5_io.H5IndexError):
      h5_io.H5FileToDict(self.tempdata_fp.name, '/numbers', ['nonexist'])

  def testH5FileToDict(self):
    data = h5_io.H5FileToDict(self.tempdata_fp.name, '/')
    np_dict = data['numbers']['table']
    self._CheckTableCorrectness(np_dict)

    data = h5_io.H5FileToDict(self.tempdata_fp.name, '/numbers/table')
    self._CheckTableCorrectness(data)

  def testH5FileToDictDeep(self):
    data = h5_io.H5FileToDict(self.tempdata_fp.name, '/numbers/table',
                              ['response', 'wavelength'])
    self.assertTrue(numpy.array_equal(data,
                                      self.table['response']['wavelength']))


if __name__ == '__main__':
  unittest.main()

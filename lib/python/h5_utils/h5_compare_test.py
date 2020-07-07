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

"""Tests for the log module."""

import unittest

from makani.lib.python.h5_utils import h5_compare as h5c
import numpy


class CompareLogsTest(unittest.TestCase):

  def setUp(self):
    self.one = numpy.array([[1]])
    self.two = numpy.array([[2]])
    self.one_two = numpy.array([[1], [2]])
    self.d1 = {
        'c': {
            'a': self.one,
            'b': self.two
        },
        'd': self.one_two
    }
    self.d2 = {
        'c': {
            'b': self.two
        }
    }

  def testKeyMissing(self):
    report = h5c.CompareLogs(self.d1, self.d2)
    self.assertEqual(report.keys(), ['c', 'd'])
    self.assertEqual(report['c'].keys(), ['a'])
    self.assertNotEqual('', h5c.GenerateReportString(report))

  def testValueMismatch(self):
    report = h5c.CompareLogs({'a': self.one}, {'a': self.two})
    self.assertEqual(report.keys(), ['a'])
    self.assertNotEqual('', h5c.GenerateReportString(report))

  def testShapeMismatch(self):
    report = h5c.CompareLogs({'a': self.one}, {'a': self.one_two})
    self.assertEqual(report.keys(), ['a'])
    self.assertNotEqual('', h5c.GenerateReportString(report))

    # Test the 90% cutoff for truncation.
    report = h5c.CompareLogs({'a': numpy.array([[i] for i in range(18)])},
                             {'a': numpy.array([[i] for i in range(20)])})
    self.assertEqual(report.keys(), ['a'])
    self.assertNotEqual('', h5c.GenerateReportString(report))

    report = h5c.CompareLogs({'a': numpy.array([[i] for i in range(19)])},
                             {'a': numpy.array([[i] for i in range(20)])},
                             truncate=True)
    self.assertIsNone(report)
    self.assertEqual('', h5c.GenerateReportString(report))

    report = h5c.CompareLogs({'a': self.one}, {'a': self.two},
                             truncate=True)
    self.assertIsNotNone(report)
    self.assertNotEqual('', h5c.GenerateReportString(report))

  def testTypeMismatch(self):
    report = h5c.CompareLogs({'a': self.one},
                             {'a': self.one.astype(float)})
    self.assertEqual(report.keys(), ['a'])
    self.assertNotEqual('', h5c.GenerateReportString(report))


if __name__ == '__main__':
  unittest.main()

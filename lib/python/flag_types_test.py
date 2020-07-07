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

"""Tests for makani.flag_types."""

import unittest

import gflags
from makani.lib.python import flag_types
import numpy


class LinspaceFlagTest(unittest.TestCase):

  def setUp(self):
    self.flags = gflags.FlagValues()

  def Parse(self, cmd_line_arg):
    self.flags(['foo_binary', cmd_line_arg])

  def testParsing(self):
    flag_types.DEFINE_linspace('test_linspace', None, 'help_string',
                               flag_values=self.flags)

    self.Parse('--test_linspace=1.1, 3.3, 20')
    self.assertTrue(numpy.all(numpy.linspace(1.1, 3.3, 20)
                              == self.flags.test_linspace))

    with self.assertRaisesRegexp(gflags.IllegalFlagValue,
                                 'Wrong number of components.*'):
      self.Parse('--test_linspace=1.1, 3.3')

    with self.assertRaisesRegexp(gflags.IllegalFlagValue, 'Bad value.*'):
      self.Parse('--test_linspace=1.1, z, 20')

  def testNonempty(self):
    flag_types.DEFINE_linspace('nonempty_linspace', None, 'help string',
                               nonempty=True, flag_values=self.flags)

    with self.assertRaisesRegexp(gflags.IllegalFlagValue, '.*nonempty.*'):
      self.Parse('--nonempty_linspace=1.0, 2.0, 0')

  def testIncreasing(self):
    flag_types.DEFINE_linspace('increasing_linspace', None, 'help string',
                               increasing=True, flag_values=self.flags)

    # It's OK to have 0 or 1 elements.
    self.Parse('--increasing_linspace=1.0, 2.0, 0')
    self.Parse('--increasing_linspace=1.0, 2.0, 1')

    with self.assertRaisesRegexp(gflags.IllegalFlagValue, '.*increasing.*'):
      self.Parse('--increasing_linspace=1.0, -2.0, 10')


if __name__ == '__main__':
  unittest.main()

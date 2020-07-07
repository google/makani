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

"""Tests for makani.test_util."""

import logging
import unittest

import gflags
from makani.lib.python import test_util
import mock

gflags.DEFINE_string('makani_test_util_test_string', None,
                     'Used for FlagSaverTest.')

gflags.DEFINE_list('makani_test_util_test_list', None,
                   'Used for FlagSaverTest.')

FLAGS = gflags.FLAGS


class LogDisablerTest(unittest.TestCase):

  def testLogDisabler(self):
    # logging.root.handle is "a thing that gets called whenver a message is
    # actually logged."  There might be a more appropriate method to mock, but
    # the obvious ones (e.g. logging.root.log) don't do the trick.
    with mock.patch.object(logging.root, 'handle') as mock_handle:
      with test_util.LogDisabler(logging.INFO):
        logging.info('foo')
        logging.warning('foo')
        logging.error('foo')
      self.assertEqual(2, mock_handle.call_count)

    with mock.patch.object(logging.root, 'handle') as mock_handle:
      with test_util.LogDisabler(logging.WARNING):
        logging.info('foo')
        logging.warning('foo')
        logging.error('foo')
      self.assertEqual(1, mock_handle.call_count)

    with mock.patch.object(logging.root, 'handle') as mock_handle:
      with test_util.LogDisabler(logging.ERROR):
        logging.info('foo')
        logging.warning('foo')
        logging.error('foo')
      self.assertEqual(0, mock_handle.call_count)

  def testDisableWarnings(self):
    with mock.patch.object(logging.root, 'handle') as mock_handle:
      with test_util.DisableWarnings():
        logging.warning('foo')
        logging.error('foo')
      self.assertEqual(1, mock_handle.call_count)

  def testDecorator(self):
    @test_util.LogDisabler(logging.WARNING)
    def LogSomeStuff():
      logging.warning('foo')
      logging.error('foo')

    with mock.patch.object(logging.root, 'handle') as mock_handle:
      LogSomeStuff()
      self.assertEqual(1, mock_handle.call_count)

  @test_util.LogDisabler(logging.WARNING)
  def testDecoratedTestMethod(self):
    with mock.patch.object(logging.root, 'handle') as mock_handle:
      logging.warning('foo')
      logging.error('foo')
      self.assertEqual(1, mock_handle.call_count)


class FlagSaverTest(unittest.TestCase):

  def testStringFlag(self):
    FLAGS.makani_test_util_test_string = 'foo'
    with test_util.FlagValueSaver():
      FLAGS.makani_test_util_test_string = 'bar'
      self.assertEqual('bar', FLAGS.makani_test_util_test_string)

    self.assertEqual('foo', FLAGS.makani_test_util_test_string)

  def testListFlag(self):
    FLAGS.makani_test_util_test_list = ['a', 'b']
    with test_util.FlagValueSaver():
      FLAGS.makani_test_util_test_list = ['c', 'd']
      self.assertEqual(['c', 'd'], FLAGS.makani_test_util_test_list)
    self.assertEqual(['a', 'b'], FLAGS.makani_test_util_test_list)

  def testDecorator(self):
    @test_util.FlagValueSaver()
    def SetFlagToBar():
      FLAGS.makani_test_util_test_string = 'bar'
      self.assertEqual('bar', FLAGS.makani_test_util_test_string)

    FLAGS.makani_test_util_test_string = 'foo'
    SetFlagToBar()
    self.assertEqual('foo', FLAGS.makani_test_util_test_string)


if __name__ == '__main__':
  unittest.main()

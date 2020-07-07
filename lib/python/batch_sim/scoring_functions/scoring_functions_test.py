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

"""Tests for scoring_functions/__init__.py."""

import unittest

from makani.lib.python.batch_sim import scoring_functions


class SingleSidedTest(scoring_functions.SingleSidedLimitScoringFunction):

  def GetValue(self, output):
    return output


class DoubleSidedTest(scoring_functions.DoubleSidedLimitScoringFunction):

  def GetValue(self, output):
    return output


class ScoringFunctionsTest(unittest.TestCase):

  def testSingleSidedLimits(self):
    with self.assertRaises(scoring_functions.LimitsException):
      SingleSidedTest('Foo', 'smoot', 1.0, 1.0, severity=1)

    upper_limit = SingleSidedTest('Foo', 'smoot', 0.5, 2.0, severity=1)
    self.assertLess(1.0, upper_limit.GetScore(2.01))
    self.assertGreater(1.0, upper_limit.GetScore(1.0))
    self.assertLess(0.0, upper_limit.GetScore(1.0))
    self.assertGreater(0.0, upper_limit.GetScore(0.49))

    self.assertEqual(None, upper_limit.GetFailureCount(None))
    with self.assertRaises(NotImplementedError):
      upper_limit.GetFailureCount({'data1': [1], 'data2': [2]})
    self.assertEqual(0, upper_limit.GetFailureCount({'data': [0.0, 0.5, 1.0]}))
    self.assertEqual(2, upper_limit.GetFailureCount(
        {'data': [0.0, 0.5, 1.0, 2.5, 3.0]}))

    self.assertEqual(None, upper_limit.GetIndexOfFirstFailure(None))
    with self.assertRaises(NotImplementedError):
      upper_limit.GetIndexOfFirstFailure({'data1': [1], 'data2': [2]})
    self.assertEqual(-1, upper_limit.GetIndexOfFirstFailure(
        {'data': [0.0, 0.5, 1.0]}))
    self.assertEqual(0, upper_limit.GetIndexOfFirstFailure(
        {'data': [2.5, 0.0, 3.0]}))
    self.assertEqual(2, upper_limit.GetIndexOfFirstFailure(
        {'data': [0.0, 0.5, 3.0]}))

    lower_limit = SingleSidedTest('Foo', 'smoot', 2.0, 0.5, severity=1)
    self.assertGreater(0.0, lower_limit.GetScore(2.01))
    self.assertGreater(1.0, lower_limit.GetScore(1.0))
    self.assertLess(0.0, lower_limit.GetScore(1.0))
    self.assertLess(1.0, lower_limit.GetScore(0.49))

    self.assertEqual(None, lower_limit.GetFailureCount(None))
    with self.assertRaises(NotImplementedError):
      lower_limit.GetFailureCount({'data1': [1], 'data2': [2]})
    self.assertEqual(0, lower_limit.GetFailureCount({'data': [1.0, 1.5, 2.0]}))
    self.assertEqual(4, lower_limit.GetFailureCount(
        {'data': [0.0, 0.1, 0.2, 0.5, 1.0]}))

    self.assertEqual(None, lower_limit.GetIndexOfFirstFailure(None))
    with self.assertRaises(NotImplementedError):
      lower_limit.GetIndexOfFirstFailure({'data1': [1], 'data2': [2]})
    self.assertEqual(-1, lower_limit.GetIndexOfFirstFailure(
        {'data': [1.0, 2.0, 3.0]}))
    self.assertEqual(0, lower_limit.GetIndexOfFirstFailure(
        {'data': [0.0, 3.0, 0.2]}))
    self.assertEqual(2, lower_limit.GetIndexOfFirstFailure(
        {'data': [2.0, 1.0, 0.0]}))

  def testDoubleSidedLimits(self):
    with self.assertRaises(scoring_functions.LimitsException):
      DoubleSidedTest('Foo', 'smoot', -0.5, -1.0, 0.5, 1.0, severity=1)

    with self.assertRaises(scoring_functions.LimitsException):
      DoubleSidedTest('Foo', 'smoot', -1.0, 0.5, -0.5, 1.0, severity=1)

    with self.assertRaises(scoring_functions.LimitsException):
      DoubleSidedTest('Foo', 'smoot', -1.0, -0.5, 1.0, 0.5, severity=1)

    limit = DoubleSidedTest('Foo', 'smoot', -1.0, -0.5, 0.5, 1.0, severity=1)
    self.assertLess(1.0, limit.GetScore(-1.01))
    self.assertGreater(0.0, limit.GetScore(-0.49))
    self.assertGreater(0.0, limit.GetScore(0.49))
    self.assertLess(1.0, limit.GetScore(1.01))

    self.assertEqual(None, limit.GetFailureCount(None))
    with self.assertRaises(NotImplementedError):
      limit.GetFailureCount({'data1': [1], 'data2': [2]})
    self.assertEqual(0, limit.GetFailureCount({'data': [-0.5, 0.0, 0.5]}))
    self.assertEqual(4, limit.GetFailureCount(
        {'data': [-1.5, -1.0, 0.0, 1.0, 1.5]}))

    self.assertEqual(None, limit.GetIndexOfFirstFailure(None))
    with self.assertRaises(NotImplementedError):
      limit.GetIndexOfFirstFailure({'data1': [1], 'data2': [2]})
    self.assertEqual(-1, limit.GetIndexOfFirstFailure(
        {'data': [-0.5, 0.0, 0.5]}))
    self.assertEqual(0, limit.GetIndexOfFirstFailure(
        {'data': [-2.0, -1.0, 0.0, 2.0]}))
    self.assertEqual(1, limit.GetIndexOfFirstFailure(
        {'data': [-0.5, 2.0, 1.0, 2.0]}))


if __name__ == '__main__':
  unittest.main()

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

"""Tests for scoring_functions/aero.py."""

import unittest

import aero
import numpy as np


class MainWingAlphaScoringFunctionTest(unittest.TestCase):

  def setUp(self):
    super(MainWingAlphaScoringFunctionTest, self).setUp()
    self.lower_bad_limit = -4.0
    self.lower_good_limit = -2.0
    self.upper_good_limit = 2.0
    self.upper_bad_limit = 4.0
    self.severity = 3
    self.scoring_function = aero.MainWingAlphaScoringFunction(
        self.lower_bad_limit, self.lower_good_limit, self.upper_good_limit,
        self.upper_bad_limit, self.severity)

  def testGetFailureCount(self):
    # There are no failures happening in both alphas_max and alphas_min at the
    # same time.
    upper_fail = self.upper_bad_limit + 1.0
    lower_fail = self.lower_bad_limit - 1.0
    timeseries = {
        'alphas_max': np.array([upper_fail, upper_fail, 3.0, 2.0, 1.0, 0.0]),
        'alphas_min': np.array([0.0, -1.0, -2.0, -3.0, lower_fail, lower_fail])
    }
    self.assertEqual(4, self.scoring_function.GetFailureCount(timeseries))

  def testGetFailureCount_SameTimeFailure(self):
    # The failure happens in both alphas_max and alphas_min at the same time,
    # therefore it should only count once.
    upper_fail = self.upper_bad_limit + 1.0
    lower_fail = self.lower_bad_limit - 1.0
    timeseries = {
        'alphas_max': np.array([upper_fail, 3.0, 2.0, 1.0]),
        'alphas_min': np.array([lower_fail, -3.0, -2.0, -1.0])
    }
    self.assertEqual(1, self.scoring_function.GetFailureCount(timeseries))

  def testGetFailureCount_NoFailure(self):
    timeseries = {
        # Values are below upper bad limit.
        'alphas_max': np.array([1.0, 2.0, 3.0]),
        # Values are above lower bad limit.
        'alphas_min': np.array([-1.0, -2.0, -3.0])
    }
    self.assertEqual(0, self.scoring_function.GetFailureCount(timeseries))

  def testGetFailureCount_None(self):
    self.assertIsNone(self.scoring_function.GetFailureCount(None))

  def testGetFailureCount_CrossLimits(self):
    # Test when alphas_max violate lower limit and alphas_min violate upper
    # limit at the same time. This will cause an assert exception.
    timeseries = {
        # -5.0 is below the lower bad limit, but in alphas_max.
        'alphas_max': np.array([-5.0, 1.0, 2.0, 3.0]),
        # 10.0 is above the upper bad limit, but in alphas_min.
        'alphas_min': np.array([10.0, -1.0, -2.0, -3.0])
    }
    with self.assertRaises(AssertionError):
      self.scoring_function.GetFailureCount(timeseries)

  def testGetIndexOfFirstFailure(self):
    # There are no failures happening in both alphas_max and alphas_min at the
    # same time.
    timeseries = {
        # 6.0 and 7.0 are above the upper bad limit.
        'alphas_max': np.array([0.5, 6.0, 7.0, 3.0, 2.0, 1.0]),
        # -6.0 and -7.0 are below the lower bad limit.
        'alphas_min': np.array([0.0, -1.0, -2.0, -3.0, -6.0, -7.0])
    }
    self.assertEqual(1, self.scoring_function.GetIndexOfFirstFailure(
        timeseries))

  def testGetIndexOfFirstFailure_SameTimeFailure(self):
    # The failure happens in both alphas_max and alphas_min at the same time,
    # therefore it should only count once.
    upper_fail = self.upper_bad_limit + 1.0
    lower_fail = self.lower_bad_limit - 1.0
    timeseries = {
        'alphas_max': np.array([3.0, upper_fail, 2.0, 1.0]),
        'alphas_min': np.array([-3.0, lower_fail, -2.0, -1.0])
    }
    self.assertEqual(1, self.scoring_function.GetIndexOfFirstFailure(
        timeseries))

  def testGetIndexOfFirstFailure_NoFailure(self):
    timeseries = {
        # Values are below upper bad limit.
        'alphas_max': np.array([1.0, 2.0, 3.0]),
        # Values are above lower bad limit.
        'alphas_min': np.array([-1.0, -2.0, -3.0])
    }
    self.assertEqual(-1, self.scoring_function.GetIndexOfFirstFailure(
        timeseries))

  def testGetIndexOfFirstFailure_None(self):
    self.assertIsNone(self.scoring_function.GetIndexOfFirstFailure(None))

  def testGetIndexOfFirstFailure_CrossLimits(self):
    # Test when alphas_max violate lower limit and alphas_min violate upper
    # limit at the same time. This will cause an assert exception.
    upper_fail = self.upper_bad_limit + 1.0
    lower_fail = self.lower_bad_limit - 1.0
    timeseries = {
        'alphas_max': np.array([lower_fail, 1.0, 2.0, 3.0]),
        'alphas_min': np.array([upper_fail, -1.0, -2.0, -3.0])
    }
    with self.assertRaises(AssertionError):
      self.scoring_function.GetIndexOfFirstFailure(timeseries)


if __name__ == '__main__':
  unittest.main()

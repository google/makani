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

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import unittest

from makani.analysis.util.simdata_analysis import bootstrap
from makani.analysis.util.simdata_analysis.statistic import Statistic
import numpy as np


class TestStatistic(unittest.TestCase):

  def test_init(self):
    data = np.array([1, 2, 3, 4, np.nan, 6, 7, 8, 9, 10]).astype(float)
    bs = bootstrap.bootstrap(data, 10, 5)
    stat = Statistic(bs, np.mean)
    self.assertAlmostEqual(stat.mean, 5.5, delta=2.)

  def test_stat(self):
    data = np.array(range(100)).astype(float)
    bs = bootstrap.bootstrap(data, 100, 1000)
    stat = Statistic(bs, np.mean)
    self.assertEqual(stat.resampled_stat.shape, (1000,))

    data[50] = np.nan
    bs = bootstrap.bootstrap(data, 100, 1000)
    stat = Statistic(bs, np.mean)
    self.assertEqual(stat.resampled_stat.shape, (1000,))

  def test_mean(self):
    data = np.array(range(100)).astype(float)
    bs = bootstrap.bootstrap(data, 100, 1000)
    stat = Statistic(bs, np.mean)
    self.assertAlmostEqual(stat.mean, np.mean(range(100)), delta=2.)


if __name__ == '__main__':
  unittest.main()

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
from makani.analysis.util.simdata_analysis.bootstrap import bootstrap
import numpy as np


class TestBootstrap(unittest.TestCase):

  def test_bootstrap(self):
    data = np.array([1, 2, 3, 4, np.nan, 6, 7, 8, 9, 10])
    bs = bootstrap(data, 10, 5)
    self.assertEqual(bs.shape, (5, 10))

    data = range(101)
    bs = bootstrap(data, 200, 100)
    self.assertEqual(bs.shape, (100, 200))


if __name__ == '__main__':
  unittest.main()

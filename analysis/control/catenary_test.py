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

"""Tests for makani.analysis.control.trans_in."""

import unittest

from makani.analysis.control import catenary
import numpy as np


class CatenaryTest(unittest.TestCase):

  def testCorrectLength(self):
    for horizontal in np.linspace(-5.0, 5.0, 20):
      for vertical in np.linspace(-5.0, 5.0, 20):
        n = 400
        tether_fractions = np.linspace(0.0, 1.0, n)
        x = np.zeros(tether_fractions.shape)
        y = np.zeros(tether_fractions.shape)
        for i, tether_fraction in enumerate(tether_fractions):
          (x[i], y[i]) = catenary.DimensionlessCatenaryPoint(
              horizontal, vertical, tether_fraction)
        if vertical >= 0.0:
          (x_bot, y_bot) = catenary.DimensionlessCatenaryBottom(
              horizontal, vertical)
          idx = np.argmin(y)
          self.assertAlmostEqual(y_bot, y[idx], places=2)
          self.assertAlmostEqual(x_bot, x[idx], places=2)

        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.hypot(dx, dy)
        # Test length.
        self.assertAlmostEqual(1.0, np.sum(ds), places=5)
        # Test governing differential equation.
        slope = dy / dx
        slope_0 = (tether_fractions[0:-1] - 1.0 + vertical) / horizontal
        self.assertAlmostEqual(0.0, np.max(np.abs(slope - slope_0)), places=2)

  def testFindTensions(self):
    for theta in np.linspace(-np.pi, np.pi):
      for r in np.linspace(0.0, 0.99):
        x_des = r * np.cos(theta)
        y_des = r * np.sin(theta)
        (horizontal, vertical) = catenary.DimensionlessTensionsFromPoint(x_des,
                                                                         y_des)
        (x, y) = catenary.DimensionlessCatenaryPoint(horizontal, vertical)
        self.assertAlmostEqual(x_des, x, places=3)
        self.assertAlmostEqual(y_des, y, places=3)


if __name__ == '__main__':
  unittest.main()

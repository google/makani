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

"""Tests for makani.analysis.aero.apparent_wind."""

import unittest

from makani.analysis.aero import apparent_wind_util
import numpy as np
from numpy import cos
from numpy import sin


class ApparentWindTest(unittest.TestCase):

  def testApparentWindCartToSph(self):
    # Compare to results from C code.
    apparent_winds = np.array([[0.0, 0.0, 0.0],
                               [-1.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0],
                               [0.0, 0.0, -1.0]])
    airspeeds, alphas, betas = (
        apparent_wind_util.ApparentWindCartToSph(apparent_winds))
    self.assertAlmostEqual(airspeeds[0], 0.0, delta=1e-9)
    self.assertAlmostEqual(alphas[0], -np.pi, delta=1e-9)
    self.assertAlmostEqual(betas[0], 0.0, delta=1e-9)
    self.assertAlmostEqual(airspeeds[1], 1.0, delta=1e-9)
    self.assertAlmostEqual(alphas[1], 0.0, delta=1e-9)
    self.assertAlmostEqual(betas[1], 0.0, delta=1e-9)
    self.assertAlmostEqual(airspeeds[2], 1.0, delta=1e-9)
    self.assertAlmostEqual(alphas[2], -np.pi, delta=1e-9)
    self.assertAlmostEqual(betas[2], np.pi / 2.0, delta=1e-9)
    self.assertAlmostEqual(airspeeds[3], 1.0, delta=1e-9)
    self.assertAlmostEqual(alphas[3], np.pi / 2.0, delta=1e-9)
    self.assertAlmostEqual(betas[3], 0.0, delta=1e-9)

  def testApparentWindSphToCart(self):
    # Compare to results from C code.
    airspeeds = np.array([0.0, 1.0, 1.0, 1.0])
    alphas = np.array([0.0, 0.0, 0.1, 0.0])
    betas = np.array([0.0, 0.0, 0.0, 0.1])
    apparent_winds = (
        apparent_wind_util.ApparentWindSphToCart(airspeeds, alphas, betas))
    self.assertAlmostEqual(apparent_winds[0][0], 0.0)
    self.assertAlmostEqual(apparent_winds[0][1], 0.0)
    self.assertAlmostEqual(apparent_winds[0][2], 0.0)
    self.assertAlmostEqual(apparent_winds[1][0], -1.0)
    self.assertAlmostEqual(apparent_winds[1][1], 0.0)
    self.assertAlmostEqual(apparent_winds[1][2], 0.0)
    self.assertAlmostEqual(apparent_winds[2][0], -0.995004, delta=1e-6)
    self.assertAlmostEqual(apparent_winds[2][1], 0.0)
    self.assertAlmostEqual(apparent_winds[2][2], -0.099833, delta=1e-6)
    self.assertAlmostEqual(apparent_winds[3][0], -0.995004, delta=1e-6)
    self.assertAlmostEqual(apparent_winds[3][1], -0.099833, delta=1e-6)
    self.assertAlmostEqual(apparent_winds[3][2], 0.0)

  def testCalcDcmWToB(self):
    for alpha in np.arange(-np.pi, np.pi, 0.1):
      for beta in np.arange(-np.pi, np.pi, 0.1):
        # See Stevens and Lewis, Aircraft Control and Simulation, pg. 63
        # equation (2.3-2); and the corresponding test in aero_frame_test.cc
        dcm_b2w = np.matrix(
            [[cos(alpha) * cos(beta), sin(beta), sin(alpha) * cos(beta)],
             [-cos(alpha) * sin(beta), cos(beta), -sin(alpha) * sin(beta)],
             [-sin(alpha), 0.0, cos(alpha)]])
        dcm_w2b = apparent_wind_util.CalcDcmWToB(alpha, beta)
        for i in range(3):
          for j in range(3):
            self.assertAlmostEqual(dcm_b2w[j, i], dcm_w2b[i, j])

if __name__ == '__main__':
  unittest.main()

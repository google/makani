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

"""Tests for makani.analysis.aero.airfoil."""

import os
import unittest

from makani.analysis.aero import airfoil
import numpy


class AirfoilTest(unittest.TestCase):

  def testGetCoeffs(self):
    f8_airfoil = airfoil.Airfoil(
        os.path.join(os.path.dirname(__file__), 'hover_model/airfoils/f8.json'),
        stall_angles_deg=[-180.0, 180.0])
    mixed_airfoil = airfoil.Airfoil(
        os.path.join(os.path.dirname(__file__), 'hover_model/airfoils/f8.json'),
        stall_angles_deg=[-10.0, 10.0])
    high_aoa_airfoil = airfoil.Airfoil(
        os.path.join(os.path.dirname(__file__),
                     'airfoils/naca0012_high_aoa.json'),
        stall_angles_deg=[-180.0, 180.0])

    # Check that the mixed airfoil is equal to the normal airfoil at
    # low angles-of-attack.
    for angle in numpy.pi / 180.0 * numpy.arange(-10.0, 10.0, 20):
      for i in range(3):
        self.assertAlmostEqual(f8_airfoil.GetCoeffs(0.0)[i],
                               mixed_airfoil.GetCoeffs(0.0)[i], delta=1e-3)

    # Check that the mixed airfoil is equal to the high
    # angle-of-attack airfoil at high angles-of-attack.
    for angle in numpy.pi / 180.0 * numpy.arange(20.0, 180.0, 100):
      for i in range(3):
        self.assertAlmostEqual(high_aoa_airfoil.GetCoeffs(angle)[i],
                               mixed_airfoil.GetCoeffs(angle)[i],
                               delta=1e-3)
        self.assertAlmostEqual(high_aoa_airfoil.GetCoeffs(-angle)[i],
                               mixed_airfoil.GetCoeffs(-angle)[i],
                               delta=1e-3)


if __name__ == '__main__':
  unittest.main()

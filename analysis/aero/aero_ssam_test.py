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

"""Tests for the SSAM model."""

import unittest

from makani.analysis.aero import aero_ssam
import numpy as np


class SSAMTest(unittest.TestCase):

  def ZeroDegreeAlphaTests(self):
    # Tests should produce alpha = 0 degrees.
    correct_answer = 0.0

    # Test the alpha beta calculation.
    time = np.array([1., 2., 3., 4., 5.])
    ones = np.ones(np.shape(time))
    zeros = np.zeros(np.shape(time))
    # First test.
    apparent_wind = np.array([-10.0 * ones, zeros, zeros]).transpose()
    omega_b = np.array([zeros, zeros, zeros]).transpose()
    r_test = np.repeat(np.expand_dims(np.asarray([0.0, 0.0, 0.0]),
                                      axis=1).transpose(), np.shape(omega_b)[0],
                       axis=0)
    alpha_test, _ = aero_ssam._ComputeRelativeAlphaBeta(omega_b, r_test,
                                                        apparent_wind)
    assert self.assertAlmostEqual(alpha_test, correct_answer, delta=1e-3)

    # Second test.
    apparent_wind = np.array([-10.0 * ones, zeros, zeros]).transpose()
    omega_b = np.array([zeros, zeros, -1.0 * ones]).transpose()
    r_test = np.repeat(np.expand_dims(np.asarray([0.0, 10.0, 0.0]),
                                      axis=1).transpose(),
                       np.shape(omega_b)[0], axis=0)
    alpha_test, _ = aero_ssam._ComputeRelativeAlphaBeta(omega_b, r_test,
                                                        apparent_wind)
    assert self.assertAlmostEqual(alpha_test, correct_answer, delta=1e-3)

  def Alpha45DegreeTests(self):
    # Tests should produce alpha = 45 degrees.
    correct_answer = 45.0

    # Test the alpha beta calculation.
    time = np.array([1., 2., 3., 4., 5.])
    ones = np.ones(np.shape(time))
    zeros = np.zeros(np.shape(time))
    # First test.
    apparent_wind = np.array([-10.0 * ones, zeros, -10.0 * ones]).transpose()
    omega_b = np.array([zeros, zeros, zeros]).transpose()
    r_test = np.repeat(np.expand_dims(np.asarray([0.0, 0.0, 0.0]),
                                      axis=1).transpose(), np.shape(omega_b)[0],
                       axis=0)
    alpha_test, _ = aero_ssam._ComputeRelativeAlphaBeta(omega_b, r_test,
                                                        apparent_wind)
    assert self.assertAlmostEqual(alpha_test, correct_answer, delta=1e-3)

    # Second test.
    apparent_wind = np.array([-10.0 * ones, zeros, zeros]).transpose()
    omega_b = np.array([-1.0 * ones, zeros, zeros]).transpose()
    r_test = np.repeat(np.expand_dims(np.asarray([0.0, -10.0, 0.0]),
                                      axis=1).transpose(),
                       np.shape(omega_b)[0], axis=0)
    alpha_test, _ = aero_ssam._ComputeRelativeAlphaBeta(omega_b, r_test,
                                                        apparent_wind)
    assert self.assertAlmostEqual(alpha_test, correct_answer, delta=1e-3)

  def Alpha90DegreeTests(self):
    # Tests should produce alpha = 90 degrees.
    correct_answer = 90.0

    # Test the alpha beta calculation.
    time = np.array([1., 2., 3., 4., 5.])
    ones = np.ones(np.shape(time))
    zeros = np.zeros(np.shape(time))
    # First test: no body rates.
    apparent_wind = np.array([zeros, zeros, -10.0 * ones]).transpose()
    omega_b = np.array([zeros, zeros, zeros]).transpose()
    r_test = np.repeat(np.expand_dims(np.asarray([0.0, 0.0, 0.0]),
                                      axis=1).transpose(), np.shape(omega_b)[0],
                       axis=0)
    alpha_test, _ = aero_ssam._ComputeRelativeAlphaBeta(omega_b, r_test,
                                                        apparent_wind)
    assert self.assertAlmostEqual(alpha_test, correct_answer, delta=1e-3)

    # Second test: body rate to cause 90 degrees.
    apparent_wind = np.array([zeros, zeros, zeros]).transpose()
    omega_b = np.array([-1.0 * ones, zeros, zeros]).transpose()
    r_test = np.repeat(np.expand_dims(np.asarray([0.0, -10.0, 0.0]),
                                      axis=1).transpose(),
                       np.shape(omega_b)[0], axis=0)
    alpha_test, _ = aero_ssam._ComputeRelativeAlphaBeta(omega_b, r_test,
                                                        apparent_wind)
    assert self.assertAlmostEqual(alpha_test, correct_answer, delta=1e-3)

  def Beta45DegreeTests(self):
    # Tests should produce beta = 45 degrees.
    correct_answer = 45.0

    # Test the alpha beta calculation.
    time = np.array([1., 2., 3., 4., 5.])
    ones = np.ones(np.shape(time))
    zeros = np.zeros(np.shape(time))
    # First test.
    apparent_wind = np.array([-10.0 * ones, -10.0 * ones, zeros]).transpose()
    omega_b = np.array([zeros, zeros, zeros]).transpose()
    r_test = np.repeat(np.expand_dims(np.asarray([0.0, 0.0, 0.0]),
                                      axis=1).transpose(), np.shape(omega_b)[0],
                       axis=0)
    _, beta_test = aero_ssam._ComputeRelativeAlphaBeta(omega_b, r_test,
                                                       apparent_wind)
    assert self.assertAlmostEqual(beta_test, correct_answer, delta=1e-3)

    # Second test.
    apparent_wind = np.array([-10.0 * ones, zeros, zeros]).transpose()
    omega_b = np.array([zeros, zeros, 1.0 * ones]).transpose()
    r_test = np.repeat(np.expand_dims(np.asarray([10.0, 0.0, 0.0]),
                                      axis=1).transpose(), np.shape(omega_b)[0],
                       axis=0)
    _, beta_test = aero_ssam._ComputeRelativeAlphaBeta(omega_b, r_test,
                                                       apparent_wind)
    assert self.assertAlmostEqual(beta_test, correct_answer, delta=1e-3)

  def Beta90DegreeTests(self):
    # Tests should produce beta = 45 degrees.
    correct_answer = 90.0

    # Test the alpha beta calculation.
    time = np.array([1., 2., 3., 4., 5.])
    ones = np.ones(np.shape(time))
    zeros = np.zeros(np.shape(time))
    # First test: No body rate.
    apparent_wind = np.array([zeros, -10.0 * ones, zeros]).transpose()
    omega_b = np.array([zeros, zeros, zeros]).transpose()
    r_test = np.repeat(np.expand_dims(np.asarray([0.0, 0.0, 0.0]),
                                      axis=1).transpose(), np.shape(omega_b)[0],
                       axis=0)
    _, beta_test = aero_ssam._ComputeRelativeAlphaBeta(omega_b, r_test,
                                                       apparent_wind)
    assert self.assertAlmostEqual(beta_test, correct_answer, delta=1e-3)

    # Second test: Body rates cause 90 degree sideslip.
    apparent_wind = np.array([zeros, zeros, zeros]).transpose()
    omega_b = np.array([zeros, zeros, 1.0 * ones]).transpose()
    r_test = np.repeat(np.expand_dims(np.asarray([10.0, 0.0, 0.0]),
                                      axis=1).transpose(), np.shape(omega_b)[0],
                       axis=0)
    _, beta_test = aero_ssam._ComputeRelativeAlphaBeta(omega_b, r_test,
                                                       apparent_wind)
    assert self.assertAlmostEqual(beta_test, correct_answer, delta=1e-3)

if __name__ == '__main__':
  unittest.main()

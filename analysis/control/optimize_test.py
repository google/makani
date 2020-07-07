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

"""Tests for makani.analysis.control.optimize."""

import unittest

from makani.analysis.control import optimize
import numpy as np

# pylint doesn't like capital letters in variable names, in contrast
# to numerical optimization conventions.
# pylint: disable=invalid-name


class OptimizeTest(unittest.TestCase):

  def testSolveQp(self):
    P = np.matrix(np.eye(2))
    q = np.matrix([[1.0], [-2.0]])
    G = np.matrix(np.zeros((0, 2)))
    h = np.matrix(np.zeros((0, 1)))
    A = np.matrix(np.zeros((0, 2)))
    b = np.matrix(np.zeros((0, 1)))

    x = optimize.SolveQp(P, q, G, h, A, b)
    self.assertAlmostEqual(-1.0, x[0, 0])
    self.assertAlmostEqual(2.0, x[1, 0])

    # Check argument testing.
    with self.assertRaises(optimize.ArgumentException):
      x = optimize.SolveQp(-P, q, G, h, A, b)

    with self.assertRaises(optimize.ArgumentException):
      Q = np.matrix(P)
      Q[0, 1] += 0.1  # Argument should be symmetric.
      x = optimize.SolveQp(Q, q, G, h, A, b)

    A = np.matrix([[0.0, 1.0]])
    b = np.matrix([[1.0]])
    x = optimize.SolveQp(P, q, G, h, A, b)
    self.assertAlmostEqual(-1.0, x[0, 0])
    self.assertAlmostEqual(1.0, x[1, 0])

    G = np.matrix([[-1.0, 0.0]])
    h = np.matrix([[-1.0]])
    x = optimize.SolveQp(P, q, G, h, A, b)
    self.assertAlmostEqual(1.0, x[0, 0])
    self.assertAlmostEqual(1.0, x[1, 0])

    # Give impossible tolerances to test error handling.
    with self.assertRaises(optimize.BadSolutionException):
      x = optimize.SolveQp(P, q, G, h, A, b, ineq_tolerance=-1.0)

    with self.assertRaises(optimize.BadSolutionException):
      x = optimize.SolveQp(P, q, G, h, A, b, eq_tolerance=-1.0)

    with self.assertRaises(optimize.NoSolutionException):
      # This raises a domain error in the solver.
      G = np.matrix([[0.0, 1.0]])
      h = np.matrix([[0.0]])
      x = optimize.SolveQp(P, q, G, h, A, b)

    with self.assertRaises(optimize.NoSolutionException):
      # This causes the solver to terminate without an optimal solution.
      G = np.matrix([[0.0, 1.0]])
      h = np.matrix([[0.99]])
      x = optimize.SolveQp(P, q, G, h, A, b)

    # Check the functions testing of argument sizes.
    args = [P, q, G, h, A, b]
    for i in range(len(args)):
      bad_args = [arg.tolist() if k == i else arg
                  for k, arg in enumerate(args)]
      with self.assertRaises(optimize.ArgumentException):
        optimize.SolveQp(*bad_args)
      bad_args = [arg[1:, :] if k == i else arg
                  for k, arg in enumerate(args)]
      with self.assertRaises(optimize.ArgumentException):
        optimize.SolveQp(*bad_args)


if __name__ == '__main__':
  unittest.main()

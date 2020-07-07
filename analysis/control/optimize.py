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

"""Wrappers around third-party optimization libraries."""


import cvxopt
import cvxopt.solvers
from makani.analysis.control import type_util
import numpy as np
from numpy import linalg

# pylint doesn't like capital letters in variable names, in contrast
# to numerical optimization conventions.
# pylint: disable=invalid-name


class ArgumentException(Exception):
  pass


class BadSolutionException(Exception):
  pass


class NoSolutionException(Exception):
  pass


def SolveQp(P, q, G, h, A, b, ineq_tolerance=1e-6, eq_tolerance=1e-6):
  """Solve a quadratic program (QP).

  Solves a QP of the form:

    min. 0.5 * x^T * P x + q^T * x

    subj. to  (G * x)[i] <= h[i] for i in len(x)
               A * x = b

  Arguments:
    P: Quadratic term of the cost (n-by-n positive semidefinite np.matrix).
    q: Linear term of the cost (n-by-1 np.matrix).
    G: Linear term of inequality constraints (k-by-n np.matrix).
    h: constant term of the inequality constraints (k-by-1 np.matrix).
    A: Linear term of the equations (l-by-n np.matrix).
    b: Constant term of the equations (l-by-1 np.matrix).
    ineq_tolerance: Required tolerance for inequality constraints.
    eq_tolerance: Required tolerance for equality constraints.

  Raises:
    ArgumentException: If the arguments are not of the right type or shape.
    NoSolutionException: If the solver reports any errors.
    BadSolutionException: If the solver returns a solution that does not respect
        constraints conditions.

  Returns:
    A n-by-1 np.matrix containing the (not necessarily unique) optimal solution.
  """
  if (not isinstance(q, np.matrix) or not isinstance(h, np.matrix)
      or not isinstance(b, np.matrix)):
    raise ArgumentException()
  num_var = q.shape[0]
  num_ineq = h.shape[0]
  num_eq = b.shape[0]

  if (not type_util.CheckIsMatrix(P, shape=(num_var, num_var))
      or not np.all(P.A1 == P.T.A1)
      or not type_util.CheckIsMatrix(q, shape=(num_var, 1))
      or not type_util.CheckIsMatrix(G, shape=(num_ineq, num_var))
      or not type_util.CheckIsMatrix(h, shape=(num_ineq, 1))
      or not type_util.CheckIsMatrix(A, shape=(num_eq, num_var))
      or not type_util.CheckIsMatrix(b, shape=(num_eq, 1))):
    raise ArgumentException(num_var, num_ineq, num_eq)

  eig, _ = linalg.eigh(P)
  if not np.all(eig >= -1e-6):
    raise ArgumentException('P is not positive definite')

  old_options = cvxopt.solvers.options
  cvxopt.solvers.options['show_progress'] = False
  try:
    result = cvxopt.solvers.qp(cvxopt.matrix(P),
                               cvxopt.matrix(q),
                               cvxopt.matrix(G),
                               cvxopt.matrix(h),
                               cvxopt.matrix(A),
                               cvxopt.matrix(b))
  except Exception as e:
    raise NoSolutionException(e)

  if result['status'] != 'optimal':
    raise NoSolutionException(result)

  x = np.matrix(result['x'])

  # Check primal feasibility.
  if np.any(G * x - h >= ineq_tolerance):
    raise BadSolutionException('Inequalities mismatch.')

  if np.any(np.abs(b - A * x) >= eq_tolerance):
    raise BadSolutionException('Equalities mismatch.')

  # TODO: Test optimality.

  cvxopt.solvers.options = old_options

  return np.matrix(result['x'])

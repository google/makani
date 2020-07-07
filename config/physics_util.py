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

"""Simple physics calculations used in the configuration scripts."""

import numpy as np

# pylint doesn't like capital letters in variable names, in contrast to physics
# conventions.
# pylint: disable=invalid-name


def ConvertDampingRatioToCoeff(spring_const, mass, damping_ratio):
  """Calculate damping coefficient.

  Calculates the damping coefficient depending on spring
  constant, mass, and the damping ratio.

  Args:
    spring_const: Spring constant of the mass-spring system.
    mass: Mass of the mass-spring system.
    damping_ratio: Damping ratio of a second order system.

  Returns:
    A damping coefficient for the mass-spring system that achieves the
    given damping ratio.
  """
  return 2.0 * damping_ratio * np.sqrt(mass * spring_const)


def CalcTotalMomentOfInertia(I1, I2, m1, m2, r1, l1):
  """Calculates the total moment-of-inertia of a 2-linkage system.

  Args:
    I1: The moment-of-inertia [kg-m^2] of the first linkage.
    I2: The moment-of-inertia [kg-m^2] of the second linkage.
    m1: The mass [kg] of the first linkage.
    m2: The mass [kg] of the second linkage.
    r1: The distance [m] between the center-of-rotation and the
        center-of-mass of the first linkage.
    l1: The distance [m] between the center-of-rotation of the first
        and second linkages.

  Returns:
    The total moment-of-inertia.
  """
  return I1 + I2 + m1 * r1**2 + m2 * l1**2


def CalcTwoLinkageDynamics(I_tot, I2, b1, b2):
  """Calculates the dynamics of a 2-link planar revolute manipulator.

  Takes the simplified two-link planar manipulator in homogeneous
  coordinates and finds the state-space representation:

    Dq q'' + Cq q' = tau
    q''= -inv_Dq Cq q' + inv_Dq tau
    q'' = A q' + B tau

  Note that this code makes the simplifying assumption that
  centrifugal and Coriolis forces are zero, which is true as long as
  the winch drum CG lies on the winch drum center-of-rotation.  That
  assumption produces a constant A-matrix.

  Args:
    I_tot: The total moment-of-inertia [kg-m^2] of the system.
    I2: The moment-of-inertia [kg-m^2] of the second linkage.
    b1: The damping coefficient [N-m/(rad/s)] of the first linkage.
    b2: The damping coefficient [N-m/(rad/s)] of the second linkage.

  Returns:
    A tuple of the A and B matrices of the dynamic system.
  """
  Dq = np.array([[I_tot, I2], [I2, I2]])
  Cq = np.array([[b1, 0.0], [0.0, b2]])

  inv_Dq = np.linalg.inv(Dq)
  A = np.dot(-inv_Dq, Cq)
  B = inv_Dq
  return A, B

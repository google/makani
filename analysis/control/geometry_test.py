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

"""Tests for makani.analysis.control.geometry."""

import unittest

from makani.analysis.control import geometry
from makani.lib.python import c_helpers
from makani.sim import sim_types

import numpy as np
import scipy.linalg

_rotation_order_helper = c_helpers.EnumHelper(
    'RotationOrder', sim_types, prefix='kRotationOrder')


class GeometryTest(unittest.TestCase):

  def testCrossMatrix(self):
    for _ in range(22):
      x = np.matrix([np.random.randn(1) for _ in range(3)])
      y = np.matrix([np.random.randn(1) for _ in range(3)])
      cross_in = np.cross(np.transpose(x), np.transpose(y))
      cross_out = geometry.CrossMatrix(x) * y
      for i in range(3):
        self.assertEqual(cross_in[0, i], cross_out[i, 0])

  def testAngleToDcmRecovery(self):
    for a1 in ['x', 'y', 'z']:
      for a3 in ['x', 'y', 'z']:
        for a2 in set(['x', 'y', 'z']) - set([a1, a3]):
          order = ''.join([a1, a2, a3])
          eulers = np.random.rand(3)
          dcm = geometry.AngleToDcm(eulers[2], eulers[1], eulers[0], order)
          (r1, r2, r3) = geometry.DcmToAngle(dcm, order)
          self.assertAlmostEqual(eulers[2], r1)
          self.assertAlmostEqual(eulers[1], r2)
          self.assertAlmostEqual(eulers[0], r3)

  def testAxisToDcm(self):
    v = np.matrix(np.random.rand(3, 1))
    dcm = np.matrix(scipy.linalg.expm(geometry.CrossMatrix(-v)))
    nearly_eye = dcm.T * geometry.AxisToDcm(v)
    for i in range(3):
      for j in range(3):
        self.assertAlmostEqual(1.0 if i == j else 0.0,
                               nearly_eye[i, j])

  def testDcmToAxis(self):
    v = np.matrix(np.random.rand(3, 1))
    dcm = np.matrix(scipy.linalg.expm(geometry.CrossMatrix(-v)))
    nearly_v = geometry.DcmToAxis(dcm)
    for i in range(3):
      self.assertAlmostEqual(v[i, 0], nearly_v[i, 0])

  def testQuatToDcmTrivial(self):
    v = np.matrix(np.random.rand(3, 1))
    dcm = np.matrix(scipy.linalg.expm(geometry.CrossMatrix(-v)))

    angle = np.linalg.norm(v)
    v /= angle
    q = np.matrix([
        [np.cos(angle / 2.0)],
        [np.sin(angle / 2.0) * v[0]],
        [np.sin(angle / 2.0) * v[1]],
        [np.sin(angle / 2.0) * v[2]]
    ])

    q_from_dcm = geometry.DcmToQuat(dcm)
    self.assertAlmostEqual(1.0, ((np.transpose(q_from_dcm) * q)[0, 0])**2)

    dcm_from_q = geometry.QuatToDcm(q)
    for i in range(3):
      for j in range(3):
        self.assertAlmostEqual(dcm[i, j], dcm_from_q[i, j])

  def testAngleToDcmTrival(self):
    for a1 in ['x', 'y', 'z']:
      for a2 in ['x', 'y', 'z']:
        if a1 == a2:
          continue
        order = ''.join([a1, a2, a1])
        flip_order = ''.join([a2, a1, a2])
        r = np.random.rand(1)
        self.assertTrue(np.all(geometry.AngleToDcm(r, 0.0, 0.0, order)
                               == geometry.AngleToDcm(0.0, 0.0, r, order)))
        self.assertTrue(np.all(geometry.AngleToDcm(0.0, r, 0.0, flip_order)
                               == geometry.AngleToDcm(0.0, 0.0, r, order)))

  def testGetAngleDerivatives(self):
    for _ in range(22):
      eulers_zyx = np.matrix([np.random.rand(1) for _ in range(3)])
      pqr = np.matrix([np.random.randn(1) for _ in range(3)])
      dcm = geometry.AngleToDcm(eulers_zyx[2], eulers_zyx[1], eulers_zyx[0])

      h = 1e-8
      dcm_p = scipy.linalg.expm(- h * geometry.CrossMatrix(pqr)) * dcm
      (r3, r2, r1) = geometry.DcmToAngle(dcm_p)
      eulers_dot_approx = ([[r1], [r2], [r3]] - eulers_zyx)/h
      eulers_dot = geometry.GetAngleDerivative(eulers_zyx, pqr)
      for i in range(3):
        self.assertAlmostEqual(eulers_dot_approx[i, 0],
                               eulers_dot[i, 0], places=6)

  def testVelocitiesToAerodynamicAngles(self):
    v_rel_in = 5.0
    alpha_in = 0.1
    beta_in = -0.05
    v_app_b = v_rel_in * np.matrix([[np.cos(alpha_in) * np.cos(beta_in)],
                                    [np.sin(beta_in)],
                                    [np.sin(alpha_in) * np.cos(beta_in)]])
    v_zero = np.matrix([[0.0], [0.0], [0.0]])
    for _ in range(22):
      dcm_g2b = geometry.AngleToDcm(np.random.rand(1),
                                    np.random.rand(1),
                                    np.random.rand(1))
      v_app_g = np.transpose(dcm_g2b) * v_app_b
      (v_rel, alpha, beta) = geometry.VelocitiesToAerodynamicAngles(dcm_g2b,
                                                                    v_app_g,
                                                                    v_zero)
      self.assertAlmostEqual(v_rel, v_rel_in)
      self.assertAlmostEqual(alpha, alpha_in)
      self.assertAlmostEqual(beta, beta_in)

      (v_rel, alpha, beta) = geometry.VelocitiesToAerodynamicAngles(dcm_g2b,
                                                                    v_zero,
                                                                    -v_app_g)
      self.assertAlmostEqual(v_rel, v_rel_in)
      self.assertAlmostEqual(alpha, alpha_in)
      self.assertAlmostEqual(beta, beta_in)

  def testTetherForceCartToSph(self):
    # Test no deflection.
    tension, tether_roll, tether_pitch = geometry.TetherForceCartToSph(
        np.matrix([[0.0], [0.0], [1.0]]))
    self.assertAlmostEqual(tension, 1.0)
    self.assertAlmostEqual(tether_roll, 0.0)
    self.assertAlmostEqual(tether_pitch, 0.0)

    # Pure pitch deflection.
    tension, tether_roll, tether_pitch = geometry.TetherForceCartToSph(
        np.matrix([[1.0], [0.0], [0.0]]))
    self.assertAlmostEqual(tension, 1.0)
    self.assertAlmostEqual(tether_roll, 0.0)
    self.assertAlmostEqual(tether_pitch, np.pi / 2.0)

    # Pure roll deflection.
    tension, tether_roll, tether_pitch = geometry.TetherForceCartToSph(
        np.matrix([[0.0], [1.0], [0.0]]))
    self.assertAlmostEqual(tension, 1.0)
    self.assertAlmostEqual(tether_roll, -np.pi / 2.0)
    self.assertAlmostEqual(tether_pitch, 0.0)


if __name__ == '__main__':
  unittest.main()

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

"""Tests for makani.analysis.control.dynamics."""

import unittest

from makani.analysis.control import dynamics
from makani.analysis.control import geometry
from makani.config import mconfig
from makani.control import system_types
import numpy as np


class VacuumAeroModel(object):
  """Dummy aerodynamics model that applies no forces or moments."""

  # pylint: disable=unused-argument
  def CalcFMCoeff(self, alpha, beta, reynolds_number, flaps, omega_hat,
                  thrust_coeff):

    return (np.matrix(np.zeros((3, 1))),
            np.matrix(np.zeros((3, 1))))


def _GetRandomVec(n):
  """Generate a random vector of a given length."""
  return np.matrix([[r] for r in np.random.rand(n)])


class WingStateTest(unittest.TestCase):

  def testIncrementDecrement(self):
    axis = _GetRandomVec(3)
    dcm = geometry.AxisToDcm(axis)
    state_a = dynamics.WingState(omega_b=_GetRandomVec(3),
                                 dcm_g2b=np.eye(3),
                                 wing_vel_g=_GetRandomVec(3),
                                 wing_pos_g=_GetRandomVec(3))
    state_b = dynamics.WingState(omega_b=_GetRandomVec(3),
                                 dcm_g2b=dcm,
                                 wing_vel_g=_GetRandomVec(3),
                                 wing_pos_g=_GetRandomVec(3))
    tangent = state_a.Difference(state_b)
    state_c = state_a.Increment(tangent)
    for i in range(3):
      self.assertAlmostEqual(state_b.omega_b[i, 0], state_c.omega_b[i, 0])
      for j in range(3):
        self.assertAlmostEqual(state_b.dcm_g2b[i, j],
                               state_c.dcm_g2b[i, j])
      self.assertAlmostEqual(state_b.wing_vel_g[i, 0],
                             state_c.wing_vel_g[i, 0])
      self.assertAlmostEqual(state_b.wing_pos_g[i, 0],
                             state_c.wing_pos_g[i, 0])
      self.assertAlmostEqual(state_b.omega_b[i, 0] - state_a.omega_b[i, 0],
                             tangent.domega_b[i, 0])
      self.assertAlmostEqual(axis[i, 0], tangent.ddcm_g2b[i, 0])
      self.assertAlmostEqual(
          state_b.wing_vel_g[i, 0] - state_a.wing_vel_g[i, 0],
          tangent.dwing_vel_g[i, 0])
      self.assertAlmostEqual(
          state_b.wing_pos_g[i, 0] - state_a.wing_pos_g[i, 0],
          tangent.dwing_pos_g[i, 0])


class WingNoAeroTest(unittest.TestCase):
  """Tests which make use of a Wing model without aerodynamics."""

  def setUp(self):
    all_params = mconfig.MakeParams('common.all_params')
    self._sim_params = all_params['sim']
    self._system_params = all_params['system']
    self._dcm_g2control = geometry.AngleToDcm(0.1, 0.2, 0.3)
    motor_model = dynamics.PureForceMomentMotorModel(
        self._system_params['rotors'],
        self._system_params['wing']['center_of_mass_pos'])

    self._wing = dynamics.Wing(
        self._system_params, self._sim_params, VacuumAeroModel(), motor_model,
        dynamics.ConstantTetherForceModel(np.matrix(np.zeros((3, 1)))))

  def testCalcGravityForceMomentPos(self):
    """Test that gravity does not apply a moment."""
    eulers = _GetRandomVec(3)
    dcm_g2b = geometry.AngleToDcm(eulers[2], eulers[1], eulers[0])
    force_moment_pos = self._wing._CalcGravityForceMomentPos(dcm_g2b)
    force_moment = self._wing._BodyForceMomentPosToComForceMoment(
        [force_moment_pos])
    self.assertAlmostEqual(self._wing._wing_mass
                           * self._system_params['phys']['g'],
                           np.linalg.norm(force_moment.force))
    self.assertAlmostEqual(0.0, force_moment.moment[0])
    self.assertAlmostEqual(0.0, force_moment.moment[1])
    self.assertAlmostEqual(0.0, force_moment.moment[2])
    self.assertTrue(isinstance(force_moment.force, np.matrix))
    self.assertTrue(isinstance(force_moment.moment, np.matrix))

  def testCalcMotorForceMomentPos(self):
    """Test the motor forces and torque."""
    thrust = np.matrix([[1000.0]])
    moment = np.matrix([[2000.0], [3000.0], [4000.0]])
    force_moment_pos = self._wing._CalcMotorForceMomentPos(
        0.0, 0.0, 0.0, np.matrix(np.zeros((3, 1))), thrust, moment)
    force_moment = self._wing._BodyForceMomentPosToComForceMoment(
        [force_moment_pos])
    self.assertAlmostEqual(thrust[0, 0],
                           np.linalg.norm(force_moment.force))
    rotor_axis = np.matrix(self._system_params['rotors'][0]['axis']).T
    roll_moment = rotor_axis.T * force_moment.moment
    yaw_moment = np.hypot(
        force_moment.moment[0, 0] - roll_moment[0, 0] * rotor_axis[0, 0],
        force_moment.moment[2, 0] - roll_moment[0, 0] * rotor_axis[2, 0])
    self.assertAlmostEqual(moment[0, 0], roll_moment)
    self.assertAlmostEqual(moment[1, 0], force_moment.moment[1])
    self.assertAlmostEqual(moment[2, 0], yaw_moment)
    self.assertTrue(isinstance(force_moment.force, np.matrix))
    self.assertTrue(isinstance(force_moment.moment, np.matrix))

  def testConservation(self):
    """Test that energy and angular momentum are conserved."""
    np.random.seed(22)
    for _ in range(1000):
      eulers = _GetRandomVec(3)
      state = dynamics.WingState(
          omega_b=_GetRandomVec(3),
          dcm_g2b=geometry.AngleToDcm(eulers[2], eulers[1], eulers[0]),
          wing_pos_g=_GetRandomVec(3),
          wing_vel_g=_GetRandomVec(3))

      inputs = dynamics.WingInputs(
          thrust=np.matrix([[0.0]]),
          motor_moment=np.matrix([[0.0] for _ in range(3)]),
          flaps=_GetRandomVec(system_types.kNumFlaps),
          wind_g=_GetRandomVec(3))

      state_dot = self._wing.CalcDeriv(state, inputs)

      # Step-size for finite-difference comparisons.
      h = 1e-4

      # Check that the kinematics are correct.
      state_pre = state.Increment(state_dot, step=-h)
      state_post = state.Increment(state_dot, step=h)
      dcm_g2b_diff = (state_post.dcm_g2b - state_pre.dcm_g2b) / (2.0 * h)
      dcm_g2b_dot = -geometry.CrossMatrix(state.omega_b) * state.dcm_g2b
      for i in range(3):
        for j in range(3):
          self.assertAlmostEqual(dcm_g2b_dot[i, j],
                                 dcm_g2b_diff[i, j])

      for i in range(3):
        self.assertEqual(state.wing_vel_g[i, 0], state_dot.dwing_pos_g[i, 0])

      # Check conservation of angular momentum (the only force here is gravity).
      angular_momentum_dot = (
          (geometry.CrossMatrix(state.omega_b)
           * self._wing._wing_inertia_matrix
           * state.omega_b)
          + self._wing._wing_inertia_matrix * state_dot.domega_b)

      for i in range(3):
        self.assertAlmostEqual(0.0, angular_momentum_dot[i])

      # Check that energy is conserved.
      energies = [
          self._wing.CalcEnergy(state_pre),
          self._wing.CalcEnergy(state_post)
      ]
      energy_dot = (energies[1] - energies[0]) / (2.0 * h)
      self.assertLess(np.abs(energy_dot),
                      np.fmax(1e-6, h * np.abs(energies[0])))


if __name__ == '__main__':
  unittest.main()

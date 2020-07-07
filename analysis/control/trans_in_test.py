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

from makani.analysis.control import geometry
from makani.analysis.control import trans_in
from makani.config import mconfig
from makani.control import control_types
from makani.sim.physics import physics
import numpy as np

# pylint doesn't like capital letters in variable names, in contrast
# to control systems conventions.
# pylint: disable=invalid-name

# TODO(b/36783475): This hacks around a test failure when the CL
# offset was raised in go/makanicl/21564. The offset used here
# corresponds to the value that was used to generate the current
# trans-in trim and gains.
physics.OverrideCL0(-0.22)


def _GetRandomVec(n):
  """Generate a random vector of a given length."""
  return np.matrix([[r] for r in np.random.rand(n)])


class TransInTest(unittest.TestCase):

  def setUp(self):
    all_params = mconfig.MakeParams('common.all_params')
    self._trans_in = trans_in.ControlDesign(all_params['system'],
                                            all_params['control'],
                                            all_params['sim'])
    self._trans_in_frame = self._trans_in._trans_in_frame
    (self._trim_state, self._trim_inputs) = self._trans_in.CalcTrim(
        5.0, np.deg2rad(50.0), False)

  def testTransInState(self):
    for i in range(100):
      state_a = trans_in.TransInState(
          omega_b=_GetRandomVec(3),
          dcm_ti2b=geometry.AxisToDcm(_GetRandomVec(3)),
          wing_vel_ti=_GetRandomVec(3),
          wing_pos_ti=_GetRandomVec(3))
      state_b = trans_in.TransInState(
          omega_b=_GetRandomVec(3),
          dcm_ti2b=geometry.AxisToDcm(_GetRandomVec(3)),
          wing_vel_ti=_GetRandomVec(3),
          wing_pos_ti=_GetRandomVec(3))
      tangent = state_a.Difference(state_b)
      state_c = state_a.Increment(tangent)
      for i in range(3):
        self.assertAlmostEqual(state_b.omega_b[i], state_c.omega_b[i])
        for j in range(3):
          self.assertAlmostEqual(state_b.dcm_ti2b[i, j],
                                 state_c.dcm_ti2b[i, j])
        self.assertAlmostEqual(state_b.wing_vel_ti[i], state_c.wing_vel_ti[i])
        self.assertAlmostEqual(state_b.wing_pos_ti[i], state_c.wing_pos_ti[i])

      wing_state_a = self._trans_in_frame.StateToWingState(state_a)
      wing_state_b = self._trans_in_frame.StateToWingState(state_b)

      wing_state_tangent = wing_state_a.Difference(wing_state_b)
      tangent_cmp = self._trans_in_frame.StateTangentFromWingStateTangent(
          wing_state_a, wing_state_tangent)

      for i in range(3):
        self.assertAlmostEqual(tangent.domega_b[i],
                               tangent_cmp.domega_b[i])
        # The increment in Eulers will not be equal for large steps.
        self.assertAlmostEqual(tangent.dwing_vel_ti[i],
                               tangent_cmp.dwing_vel_ti[i])
        self.assertAlmostEqual(tangent.dwing_pos_ti[i],
                               tangent_cmp.dwing_pos_ti[i])

      h = 1e-6
      state_b = state_a.Increment(tangent, step=h)
      wing_state_b = self._trans_in_frame.StateToWingState(state_b)
      tangent_cmp = self._trans_in_frame.StateTangentFromWingStateTangent(
          wing_state_a, wing_state_a.Difference(wing_state_b))

      for i in range(3):
        self.assertAlmostEqual(tangent.domega_b[i],
                               tangent_cmp.domega_b[i]/h, places=5)
        self.assertAlmostEqual(tangent.ddcm_ti2b[i],
                               tangent_cmp.ddcm_ti2b[i]/h, places=5)
        self.assertAlmostEqual(tangent.dwing_vel_ti[i],
                               tangent_cmp.dwing_vel_ti[i]/h, places=5)
        self.assertAlmostEqual(tangent.dwing_pos_ti[i],
                               tangent_cmp.dwing_pos_ti[i]/h, places=5)

  def testCalcTrim(self):
    """Applies simple checks to CalcTrim."""
    climb_angle = np.deg2rad(50.0)
    (state, inputs) = self._trans_in.CalcTrim(5.0, climb_angle, False)
    wing_state = self._trans_in_frame.StateToWingState(state)
    wing_inputs = inputs.ToWingInputs(self._trans_in._flap_offsets,
                                      self._trans_in._midboard_flap_ratio)
    state_dot = self._trans_in_frame.StateTangentFromWingStateTangent(
        wing_state, self._trans_in._wing.CalcDeriv(wing_state, wing_inputs))
    v_rel, _, _ = wing_state.CalcAerodynamicAngles(wing_inputs.wind_g)

    v_rel_ti = (
        state.wing_vel_ti - self._trans_in_frame.RotateG2Ti(inputs.wind_g))
    v_rel = np.linalg.norm(v_rel_ti)
    gamma_aero = np.arcsin(-v_rel_ti[2, 0] / v_rel)

    self.assertAlmostEqual(climb_angle, gamma_aero, delta=np.deg2rad(0.01))
    self.assertAlmostEqual(0.0, state_dot.domega_b[0, 0], 3)
    self.assertAlmostEqual(0.0, state_dot.domega_b[1, 0], 3)
    self.assertAlmostEqual(0.0, state_dot.domega_b[2, 0], 3)
    self.assertAlmostEqual(0.0, state_dot.dwing_vel_ti[0, 0], 3)
    self.assertAlmostEqual(0.0, state_dot.dwing_vel_ti[1, 0], 3)
    self.assertAlmostEqual(0.0, state_dot.dwing_vel_ti[2, 0], 3)

  def testGetLateralAttitudeSystem(self):
    """Applies sign, sparsity, and magnitude checks to the lateral model."""
    sys = self._trans_in.GetLinearizedModel(self._trim_state, self._trim_inputs)
    lat_sys = self._trans_in._GetLateralAttitudeSystem(sys)
    A, B, _, _, _ = lat_sys.GetStateSpaceModel()

    # Check positive damping.
    self.assertGreater(
        0.0, A[control_types.kTransInLateralStateRollRate,
               control_types.kTransInLateralStateRollRate])
    self.assertGreater(
        0.0, A[control_types.kTransInLateralStateYawRate,
               control_types.kTransInLateralStateYawRate])

    # Check the motor yaw sign and relative magnitude.
    self.assertLess(
        0.0, B[control_types.kTransInLateralStateYawRate,
               control_types.kTransInLateralInputMotorYaw])
    self.assertGreater(
        np.abs(B[control_types.kTransInLateralStateYawRate,
                 control_types.kTransInLateralInputMotorYaw]),
        5.0 * np.abs(B[control_types.kTransInLateralStateRollRate,
                       control_types.kTransInLateralInputMotorYaw]))

    # Check the ailerons sign and relative magnitude.
    self.assertGreater(
        0.0, B[control_types.kTransInLateralStateRollRate,
               control_types.kTransInLateralInputAileron])
    self.assertGreater(
        np.abs(B[control_types.kTransInLateralStateRollRate,
                 control_types.kTransInLateralInputAileron]),
        5.0 * np.abs(B[control_types.kTransInLateralStateYawRate,
                       control_types.kTransInLateralInputAileron]))

    # Check the rudder sign and relative magnitude.
    self.assertGreater(
        0.0, B[control_types.kTransInLateralStateYawRate,
               control_types.kTransInLateralInputRudder])
    self.assertGreater(
        np.abs(B[control_types.kTransInLateralStateYawRate,
                 control_types.kTransInLateralInputRudder]),
        5.0 * np.abs(B[control_types.kTransInLateralStateRollRate,
                       control_types.kTransInLateralInputRudder]))

    # Lastly we check some sparsity constraints in the kinematics.
    for i in range(control_types.kNumTransInLateralStates):
      if i not in [control_types.kTransInLateralStateRollRate,
                   control_types.kTransInLateralStateYawRate]:
        self.assertEqual(
            0.0, A[control_types.kTransInLateralStateRoll, i])
        self.assertEqual(
            0.0, A[control_types.kTransInLateralStateYaw, i])

    for i in [control_types.kTransInLateralStateRoll,
              control_types.kTransInLateralStateYaw]:
      for j in range(control_types.kNumTransInLateralInputs):
        self.assertEqual(0.0, B[i, j])

  def testGetLongitudinalAttitudeSystem(self):
    """Applies checks to the longitudinal model."""
    sys = self._trans_in.GetLinearizedModel(self._trim_state, self._trim_inputs)
    long_sys = self._trans_in._GetLongitudinalAttitudeSystem(sys)
    A, B, _, _, _ = long_sys.GetStateSpaceModel()

    # Check for positive damping.
    self.assertGreater(
        0.0, A[control_types.kTransInLongitudinalStatePitchRate,
               control_types.kTransInLongitudinalStatePitchRate])

    for i in range(control_types.kNumTransInLongitudinalStates):
      if i == control_types.kTransInLongitudinalStatePitchRate:
        self.assertGreater(1e-2, np.abs(
            1.0 - A[control_types.kTransInLongitudinalStatePitch, i]))
      else:
        self.assertEqual(
            0.0, A[control_types.kTransInLongitudinalStatePitch, i])

    for i in range(control_types.kNumTransInLongitudinalInputs):
      self.assertEqual(0.0, B[control_types.kTransInLongitudinalStatePitch, i])


if __name__ == '__main__':
  unittest.main()

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

"""Tests for makani.analysis.control.actuator_util."""

import unittest

from makani.analysis.control import actuator_util
from makani.config import mconfig
from makani.control import control_types
import numpy as np


class ActuatorUtilTest(unittest.TestCase):

  def testMixRotors(self):
    all_params = mconfig.MakeParams('common.all_params')

    rotors_0 = actuator_util.MixRotors(
        {'thrust': 1e4, 'moment': [0.0, 0.0, 0.0]},
        {'thrust': 1e-3, 'moment': [1e-4, 1.0, 1.0]},
        0.0, [0.0, 0.0, 0.0], control_types.kStackingStateNormal, False,
        all_params['system']['phys']['rho'],
        all_params['system']['rotors'], all_params['control']['rotor_control'])
    self.assertTrue(isinstance(rotors_0, np.matrix))
    self.assertEqual(control_types.kNumMotors, rotors_0.shape[0])
    self.assertEqual(1, rotors_0.shape[1])

    rotors_10 = actuator_util.MixRotors(
        {'thrust': 1e4, 'moment': [0.0, 0.0, 0.0]},
        {'thrust': 1e-3, 'moment': [1e-4, 1.0, 1.0]},
        10.0, [0.0, 0.0, 0.0], control_types.kStackingStateNormal, False,
        all_params['system']['phys']['rho'],
        all_params['system']['rotors'], all_params['control']['rotor_control'])
    for i in range(rotors_10.shape[0]):
      self.assertGreaterEqual(rotors_10[i, 0], rotors_0[i, 0])

  def testLinearizeMixRotors(self):
    params = mconfig.MakeParams('common.all_params')

    a = actuator_util.LinearizeMixRotors(
        {'thrust': 1e4, 'moment': [0.0, 0.0, 0.0]}, params)

    # A pure thrust command results in almost equal rotor speed increase for
    # all the rotors.
    self.assertTrue(all(np.abs(a[:, 0]/a[0, 0] - 1.0) < 5e-2))

    # A pure pitch moment command results in almost equal rotor speed increase
    # for the top rotors, of opposite sign as the rotor speed increase of the
    # lower rotors.
    self.assertTrue(all(
        np.abs(a[:, 2]/a[0, 2] -
               np.matrix([1, 1, 1, 1, -1, -1, -1, -1]).T < 5e-2)))

  def testAddThrustMoment(self):
    thrust_moment = actuator_util._AddThrustMoment(
        {'thrust': 1e4, 'moment': [0.0, 0.0, 0.0]},
        {'thrust': 1e4, 'moment': [1e2, 2e2, 3e2]})

    self.assertEqual(thrust_moment['thrust'], 2e4)
    self.assertEqual(thrust_moment['moment'][0], 1e2)
    self.assertEqual(thrust_moment['moment'][1], 2e2)
    self.assertEqual(thrust_moment['moment'][2], 3e2)


if __name__ == '__main__':
  unittest.main()

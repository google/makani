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

"""Tests for makani.analysis.control.simple_aero."""

import unittest

from makani.analysis.control import simple_aero
from makani.control import control_types
import numpy as np


class SimpleAeroTest(unittest.TestCase):

  def setUp(self):
    self._example_rotor_model = {
        'thrust_coeffs': [1.72e-2, -9.28e-2, -2.54e-1],
        'J_neutral': 8.37e-1,
        'J_max': 1.37,
        'D': 1.1,
        'D4': 1.1 ** 4.0
    }

  def testDictToSimpleRotorModelParams(self):
    thrust_coeffs = [
        np.random.rand(1)
        for _ in range(control_types.NUM_SIMPLE_ROTOR_MODEL_COEFFS)
    ]
    # Intentionally leave out thrust_coeffs to trigger an assert.
    model = {
        'J_neutral': np.random.rand(1),
        'J_max': np.random.rand(1),
        'D': np.random.rand(1)
    }
    model['D4'] = model['D'] ** 4.0

    with self.assertRaises(simple_aero.SimpleRotorModelParamsFormatException):
      c_model = simple_aero._DictToSimpleRotorModelParams(model)

    # Add in thrust coeffs, but with the wrong length.
    model['thrust_coeffs'] = thrust_coeffs[1:]
    with self.assertRaises(simple_aero.SimpleRotorModelParamsFormatException):
      c_model = simple_aero._DictToSimpleRotorModelParams(model)

    model['thrust_coeffs'] = thrust_coeffs
    c_model = simple_aero._DictToSimpleRotorModelParams(model)

    for i in range(control_types.NUM_SIMPLE_ROTOR_MODEL_COEFFS):
      self.assertEqual(c_model.thrust_coeffs[i], model['thrust_coeffs'][i])
    self.assertEqual(model['J_neutral'], c_model.J_neutral)
    self.assertEqual(model['J_max'], c_model.J_max)
    self.assertEqual(model['D'], c_model.D)

  def testArrayToVec3(self):
    vec3 = np.random.rand(3)
    c_vec3 = simple_aero._ArrayToVec3(vec3)
    self.assertEqual(vec3[0], c_vec3.x)
    self.assertEqual(vec3[1], c_vec3.y)
    self.assertEqual(vec3[2], c_vec3.z)

  def testCalcLocalAirspeed(self):
    zero_vec3 = (0.0, 0.0, 0.0)
    # Test basic points.
    self.assertEqual(0.0, simple_aero.CalcLocalAirspeed(1.0, 1.0,
                                                        zero_vec3, zero_vec3))
    self.assertEqual(1.0, simple_aero.CalcLocalAirspeed(1.0, 0.0,
                                                        zero_vec3, zero_vec3))
    self.assertEqual(4.0, simple_aero.CalcLocalAirspeed(2.0, -3.0,
                                                        zero_vec3, zero_vec3))

    # Test argument order for position and body rates.
    self.assertEqual(1.0, simple_aero.CalcLocalAirspeed(
        0.0, 0.0, (0.0, -1.0, 0.0), (0.0, 0.0, 1.0)))

  def testCalcCrossingIndex(self):
    # Test basic operation.
    test_data = np.array([[-2.0, 2.0], [0.0, 0.0], [2.0, -2.0]])

    indices = simple_aero._CalcCrossingIndex(test_data, axis=1)
    self.assertEqual(0.5, indices[0])
    self.assertEqual(None, indices[1])
    self.assertEqual(0.5, indices[2])

    indices = simple_aero._CalcCrossingIndex(test_data, axis=0)
    self.assertEqual(1.0, indices[0])
    self.assertEqual(1.0, indices[1])


if __name__ == '__main__':
  unittest.main()

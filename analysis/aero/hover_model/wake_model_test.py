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

"""Tests for makani.analysis.aero.hover_model.wake_model."""

import copy
import unittest

from makani.analysis.aero.hover_model import wake_model
import numpy as np


class WakeModelTest(unittest.TestCase):

  def testSumWakeVelocityIncrementsAtPoint(self):
    # Check that the single rotor case is identical to the plain wake
    # model.
    single_rotor_params = {
        'rotors': [
            {
                'pos': [1.613, 3.639, 1.597],
                'thrust': 1600.0 * 9.81 / 8.0 * 0.8,
                'radius': 1.0
            }
        ],
        'rotor_pitch': 0.0,
        'phys': {
            'rho': 1.2
        }
    }

    point_b = copy.copy(single_rotor_params['rotors'][0]['pos'])
    point_b[0] -= 1.0
    vel_incr_b = wake_model.SumWakeVelocityIncrementsAtPoint(
        point_b, np.array([[0.0, 0.0, 0.0]]), single_rotor_params)
    vel_incr = wake_model._CalcWakeVelocityIncrement(
        1.0, 0.0, single_rotor_params['rotors'][0]['thrust'], [0.0, 0.0, 0.0],
        single_rotor_params['rotors'][0]['radius'],
        single_rotor_params['phys']['rho'])
    self.assertAlmostEqual(vel_incr_b[0][0], -vel_incr, delta=1e-3)

    multiple_rotor_params = {
        'rotors': [
            {
                'pos': [1.613, 3.639, 1.597],
                'thrust': 1500.0 * 9.81 / 8.0 * 0.8,
                'radius': 1.03
            },
            {
                'pos': [1.613, 1.213, 1.597],
                'thrust': 1500.0 * 9.81 / 8.0 * 0.8,
                'radius': 1.03
            },
            {
                'pos': [1.613, -1.213, 1.597],
                'thrust': 1500.0 * 9.81 / 8.0 * 0.8,
                'radius': 1.03
            },
            {
                'pos': [1.613, -3.639, 1.597],
                'thrust': 1500.0 * 9.81 / 8.0 * 0.8,
                'radius': 1.03
            },
            {
                'pos': [1.960, -3.639, -1.216],
                'thrust': 1500.0 * 9.81 / 8.0 * 1.2,
                'radius': 1.03
            },
            {
                'pos': [1.960, -1.213, -1.216],
                'thrust': 1500.0 * 9.81 / 8.0 * 1.2,
                'radius': 1.03
            },
            {
                'pos': [1.960, 1.213, -1.216],
                'thrust': 1500.0 * 9.81 / 8.0 * 1.2,
                'radius': 1.03
            },
            {
                'pos': [1.960, 3.639, -1.216],
                'thrust': 1500.0 * 9.81 / 8.0 * 1.2,
                'radius': 1.03
            }
        ],
        'rotor_pitch': np.deg2rad(-3.0),
        'phys': {
            'rho': 1.2,
        }
    }

    # Check that we get the same results when we vectorize point_b,
    # apparent_wind_b, both, or neither.
    point_b = np.array([-5.0, 0.0, -3.0])
    apparent_wind_b = np.array([-10.0, 1.0, -1.0])
    vel_incr = wake_model.SumWakeVelocityIncrementsAtPoint(
        point_b, apparent_wind_b, multiple_rotor_params)
    vel_incr_test = wake_model.SumWakeVelocityIncrementsAtPoint(
        np.array([point_b, point_b]), apparent_wind_b, multiple_rotor_params)
    for i in range(2):
      self.assertEqual(vel_incr_test[i][0], vel_incr[0])
      self.assertEqual(vel_incr_test[i][1], vel_incr[1])
      self.assertEqual(vel_incr_test[i][2], vel_incr[2])
    vel_incr_test = wake_model.SumWakeVelocityIncrementsAtPoint(
        point_b, np.array([apparent_wind_b, apparent_wind_b]),
        multiple_rotor_params)
    for i in range(2):
      self.assertEqual(vel_incr_test[i][0], vel_incr[0])
      self.assertEqual(vel_incr_test[i][1], vel_incr[1])
      self.assertEqual(vel_incr_test[i][2], vel_incr[2])
    vel_incr_test = wake_model.SumWakeVelocityIncrementsAtPoint(
        np.array([point_b, point_b]),
        np.array([apparent_wind_b, apparent_wind_b]), multiple_rotor_params)
    for i in range(2):
      self.assertEqual(vel_incr_test[i][0], vel_incr[0])
      self.assertEqual(vel_incr_test[i][1], vel_incr[1])
      self.assertEqual(vel_incr_test[i][2], vel_incr[2])

  def testCalcWakePath(self):
    # The wake path should go straight down if there is zero apparent
    # wind.
    wake_path_r = wake_model._CalcWakePath(100.0, np.array([[0.0, 0.0, 0.0]]),
                                           1.0, 1.2)
    for x in -np.linspace(0.1, 5.0, 10):
      for axis in range(2):
        self.assertAlmostEqual(wake_path_r[0](x)[axis], 0.0)

    # The wake path should always be tangent to the local airflow.
    thrust = 100.0
    wind = 10.0
    wake_path_r = wake_model._CalcWakePath(thrust,
                                           np.array([[0.0, 0.0, -wind]]),
                                           1.0, 1.2)
    dx = 0.01
    for x in -np.linspace(0.1, 8.0, 10):
      dz = wake_path_r[0](x + dx)[1] - wake_path_r[0](x)[1]
      vel_incr = wake_model._CalcWakeVelocityIncrement(-x, 0.0, thrust,
                                                       [0.0, 0.0, 0.0], 1.0,
                                                       1.2)
      self.assertAlmostEqual(np.arctan2(dz, dx),
                             np.arctan2(wind, vel_incr), delta=1e-2)
      self.assertAlmostEqual(wake_path_r[0](x)[0], 0.0, delta=1e-3)

  def testCalcWakeVelocityIncrementAtPoint(self):
    # Check that the x velocity is the same as that calculated with
    # the _CalcWakeVelocityIncrement function.
    thrust = 100.0
    apparent_wind_r = [0.0, 0.0, 0.0]
    wake_path_r = wake_model._CalcWakePath(thrust, np.array([apparent_wind_r]),
                                           1.0, 1.2)
    for x_jet in np.linspace(0.1, 5.0, 10):
      vel_incr = wake_model._CalcWakeVelocityIncrement(x_jet, 0.0, thrust,
                                                       apparent_wind_r, 1.0,
                                                       1.2)
      vel_incr_r = wake_model._CalcWakeVelocityIncrementAtPoint(
          [-x_jet, 0.0, 0.0], wake_path_r, thrust, np.array([apparent_wind_r]),
          1.0, 1.2)
      self.assertAlmostEqual(vel_incr_r[0][0], -vel_incr, delta=1e-3)
      self.assertAlmostEqual(vel_incr_r[0][1], 0.0, delta=1e-3)
      self.assertAlmostEqual(vel_incr_r[0][2], 0.0, delta=1e-3)

  def testCalcWakeVelocityIncrement(self):
    thrust = 100.0
    rotor_radius = 1.0
    rotor_area = np.pi * rotor_radius**2.0
    air_density = 1.2

    # Check that the wake velocity increment obeys basic momentum
    # theory approximations.  Note that, even though we evaluate this
    # function at a small axial position, we use the far downstream
    # velocity because this wake model deals with this difference by
    # contracting the radius near the source.
    self.assertAlmostEqual(
        wake_model._CalcWakeVelocityIncrement(0.01, 0.0, thrust,
                                              [0.0, 0.0, 0.0], rotor_radius,
                                              air_density),
        2.0 * np.sqrt(thrust / (2.0 * air_density * rotor_area)), delta=1e-3)

    freestream_velocity = 10.0
    inflow_factor = -0.5 + 0.5 * np.sqrt(1.0 + 2.0 * thrust
                                         / (air_density * rotor_area
                                            * freestream_velocity**2.0))
    self.assertAlmostEqual(
        wake_model._CalcWakeVelocityIncrement(0.01, 0.0, thrust,
                                              [-freestream_velocity, 0.0, 0.0],
                                              rotor_radius, air_density),
        2.0 * inflow_factor * freestream_velocity, delta=1e-3)

    # Check that the wake increment is zero outside the contracted
    # radius (radius / sqrt(2)) when there is no inflow velocity.
    self.assertAlmostEqual(
        wake_model._CalcWakeVelocityIncrement(0.01, 0.71, thrust,
                                              [0.0, 0.0, 0.0], rotor_radius,
                                              air_density), 0.0, delta=1e-3)


if __name__ == '__main__':
  unittest.main()

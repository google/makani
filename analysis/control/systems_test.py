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

"""Tests for makani.analysis.systems.py."""

import unittest

from makani.analysis.control import systems
import numpy as np


# pylint doesn't like capital letters in variable names, in contrast
# to control systems conventions.
# pylint: disable=invalid-name


class SignalListTest(unittest.TestCase):

  def testBasic(self):
    # Test constructor.
    signals_one = systems.SignalList(['Foo', 'Bar', 'Baz'])
    signals_two = systems.SignalList(['Quux', 'Quuux', 'Baz'])

    with self.assertRaises(systems.SignalListInvalidArgumentException):
      systems.SignalList(['Foo', 'Foo'])

    # Test __len__.
    self.assertEqual(3, len(signals_one))
    self.assertEqual(3, len(signals_two))

    # Test __getitem__.
    subset_one = signals_one[['Bar', 'Foo']]
    self.assertEqual('Bar', subset_one.names[0])
    self.assertEqual('Foo', subset_one.names[1])

    subset_two = signals_one[['Foo', 2]]
    self.assertEqual('Foo', subset_two.names[0])
    self.assertEqual('Baz', subset_two.names[1])

    with self.assertRaises(systems.SignalListInvalidArgumentException):
      _ = signals_one['Foo']

    # Test GetName.
    self.assertEqual('Foo', signals_one.names[0])
    self.assertEqual('Bar', signals_one.names[1])
    self.assertEqual('Baz', signals_one.names[2])

    # TODO: Test GetIndices.


class SystemTest(unittest.TestCase):

  def testConstructor(self):
    A = np.matrix([[0.0, 1.0, 0.0],
                   [0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0]])
    B = np.matrix([[0.0], [1.0], [0.0]])
    C = np.matrix(np.hstack((np.eye(2), np.zeros((2, 1)))))
    D = np.matrix([[4.0], [3.0]])

    state_list = systems.SignalList(['pos', 'vel', 'int'])
    input_list = systems.SignalList(['accel'])
    output_list = systems.SignalList(['pos', 'vel'])
    Ts = 0.01
    sys = systems.System(
        A, B, C, D, Ts, state_list, input_list, output_list)
    self.assertEqual(3, sys.nx)
    self.assertEqual(1, sys.nu)
    self.assertEqual(2, sys.ny)
    self.assertEqual(Ts, sys.Ts)

    # Test passing an empty A matrix.
    with self.assertRaises(systems.SystemBadDimensionException):
      sys = systems.System(
          [], B, C, D, 0.0, state_list, input_list, output_list)

    with self.assertRaises(systems.SystemBadDimensionException):
      sys = systems.System(
          A, [], C, D, 0.0, state_list, input_list, output_list)

    with self.assertRaises(systems.SystemBadDimensionException):
      sys = systems.System(
          A, B, [], D, 0.0, state_list, input_list, output_list)

    with self.assertRaises(systems.SystemBadDimensionException):
      sys = systems.System(
          [], [], C, D, 0.0, state_list, input_list, output_list)

    with self.assertRaises(systems.SystemBadDimensionException):
      sys = systems.System(
          A, [], [], D, 0.0, state_list, input_list, output_list)

    with self.assertRaises(systems.SystemBadDimensionException):
      sys = systems.System(
          [], B, [], D, 0.0, state_list, input_list, output_list)

    sys = systems.System(
        [], [], [], D, 0.0, systems.SignalList([]), input_list, output_list)
    self.assertEqual(0, sys.nx)
    self.assertEqual(1, sys.nu)
    self.assertEqual(2, sys.ny)
    self.assertEqual(D[0, 0], sys._D[0, 0])

    sys = systems.System(
        A, B, C, [], 0.0, state_list, input_list, output_list)
    self.assertEqual(3, sys.nx)
    self.assertEqual(1, sys.nu)
    self.assertEqual(2, sys.ny)
    self.assertEqual(0.0, sys._D[0, 0])

  def testGetItem(self):
    nx = 10
    nu = 5
    ny = 3
    states = systems.SignalList(['s' + str(i) for i in range(nx)])
    inputs = systems.SignalList(['i' + str(i) for i in range(nu)])
    outputs = systems.SignalList(['o' + str(i) for i in range(ny)])
    sys = systems.System(np.random.randn(nx, nx), np.random.randn(nx, nu),
                         np.random.randn(ny, nx), np.random.randn(ny, nu),
                         0.0, states, inputs, outputs)
    A, B, C, D, Ts = sys.GetStateSpaceModel()
    for _ in range(10):
      input_perm = np.random.permutation(nu)
      output_perm = np.random.permutation(ny)
      for sub_nu in range(1, nu):
        for sub_ny in range(1, ny):
          input_indices = input_perm[0:sub_nu]
          output_indices = output_perm[0:sub_ny]
          input_names = [inputs.names[i] for i in input_indices]
          output_names = [outputs.names[i] for i in output_indices]
          sub = sys[output_names, input_names]
          sub_A, sub_B, sub_C, sub_D, _ = sub.GetStateSpaceModel()
          self.assertTrue((sub_A == A).all())
          for i, j in np.ndindex((nx, sub_nu)):
            self.assertEqual(B[i, input_indices[j]], sub_B[i, j])

          for i, j in np.ndindex((nx, sub_ny)):
            self.assertEqual(C[output_indices[j], i], sub_C[j, i])

          for i, j in np.ndindex((sub_ny, sub_nu)):
            self.assertEqual(D[output_indices[i], input_indices[j]],
                             sub_D[i, j])

          sub = sys[output_names, :]
          sub_A, sub_B, sub_C, sub_D, sub_Ts = sub.GetStateSpaceModel()
          self.assertTrue((sub_A == A).all())
          self.assertTrue((sub_B == B).all())
          self.assertTrue(sub_Ts == Ts)
          for i, j in np.ndindex((nx, sub_ny)):
            self.assertEqual(C[output_indices[j], i], sub_C[j, i])

          for i, j in np.ndindex((sub_ny, nu)):
            self.assertEqual(D[output_indices[i], j], sub_D[i, j])

          sub = sys[:, input_names]
          sub_A, sub_B, sub_C, sub_D, sub_Ts = sub.GetStateSpaceModel()
          self.assertTrue((sub_A == A).all())
          self.assertTrue((sub_C == C).all())
          self.assertTrue(sub_Ts == Ts)
          for i, j in np.ndindex((nx, sub_nu)):
            self.assertEqual(B[i, input_indices[j]], sub_B[i, j])

          for i, j in np.ndindex((ny, sub_nu)):
            self.assertEqual(D[i, input_indices[j]], sub_D[i, j])

          sub = sys[:, :]
          sub_A, sub_B, sub_C, sub_D, sub_Ts = sub.GetStateSpaceModel()
          self.assertTrue((sub_A == A).all())
          self.assertTrue((sub_B == B).all())
          self.assertTrue((sub_C == C).all())
          self.assertTrue((sub_D == D).all())
          self.assertTrue(sub_Ts == Ts)

  def testTruncate(self):
    A = np.matrix([[1.0, 2.0, 3.0, 4.0],
                   [5.0, 6.0, 7.0, 8.0],
                   [9.0, 10.0, 11.0, 12.0],
                   [13.0, 14.0, 15.0, 16.0]])
    B = np.matrix([[17.0], [18.0], [19.0], [20.0]])
    C = np.matrix(np.hstack((np.eye(2), np.zeros((2, 2)))))
    D = np.matrix([[4.0], [3.0]])
    Ts = 0.01

    state_list = systems.SignalList(['s1', 's2', 's3', 's4'])
    input_list = systems.SignalList(['i'])
    output_list = systems.SignalList(['o1', 'o2'])

    sys = systems.System(A, B, C, D, Ts, state_list, input_list, output_list)
    sys_trunc = sys.ReduceStates(['s2', 's4'])
    A_red, B_red, C_red, D_red, Ts_red = sys_trunc.GetStateSpaceModel()

    self.assertEqual(2, sys_trunc.nx)
    self.assertEqual('s2', sys_trunc.states.names[0])
    self.assertEqual('s4', sys_trunc.states.names[1])

    self.assertEqual(sys.nu, sys_trunc.nu)
    for i in range(sys.nu):
      self.assertEqual(sys.inputs.names[i], sys_trunc.inputs.names[i])

    self.assertEqual(sys.ny, sys_trunc.ny)
    for i in range(sys.ny):
      self.assertEqual(sys.outputs.names[i], sys_trunc.outputs.names[i])

    self.assertEqual(6.0, A_red[0, 0])
    self.assertEqual(8.0, A_red[0, 1])
    self.assertEqual(14.0, A_red[1, 0])
    self.assertEqual(16.0, A_red[1, 1])

    self.assertEqual(18.0, B_red[0, 0])
    self.assertEqual(20.0, B_red[1, 0])

    self.assertEqual(0.0, C_red[0, 0])
    self.assertEqual(0.0, C_red[0, 1])
    self.assertEqual(1.0, C_red[1, 0])
    self.assertEqual(0.0, C_red[1, 1])

    self.assertTrue(np.all(D - D_red == 0.0))
    self.assertEqual(Ts, Ts_red)

if __name__ == '__main__':
  unittest.main()

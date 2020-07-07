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

import unittest

from makani.lib.bazel.swig_test import array
from makani.lib.bazel.swig_test import lab_c
from makani.lib.bazel.swig_test import lab_cc
import numpy


class SwigCTest(unittest.TestCase):

  def testPercent(self):
    self.assertEqual(lab_c.Percent(60, 80), 75.0)

  def testMissRate(self):
    self.assertEqual(lab_c.MissRate(60, 80), 25.0)

  def testWholesalePrice(self):
    produce = lab_c.WholesalePrice(0.6, 80)
    self.assertEqual(produce.total, 48.0)
    self.assertEqual(produce.fruit, 0)
    self.assertFalse(lab_c.CvtGetControllerCommandMessage(0, None, None, None))

    price = lab_c.copy_double_pointer(0.6)
    self.assertAlmostEqual(lab_c.TotalPrice(3, price), 1.8)
    lab_c.delete_double_pointer(price)

    # Enums are wrapped as integers.
    fruits = numpy.array([0, 1, 2], dtype=numpy.int32)
    self.assertEqual(array.MixedPrice(fruits), 0.6)
    buf = 'This is a wonderful day!'
    self.assertEqual(array.PrintBuffer(-1, buf), len(buf))
    self.assertEqual(array.PrintBuffer(-1, ''), -1)


class SwigCCTest(unittest.TestCase):

  def testPercent(self):
    self.assertEqual(lab_cc.PortionA(6, 2), 75.0)

  def testMissRate(self):
    self.assertEqual(lab_cc.PortionB(6, 2), 25.0)

  def testWholesalePrice(self):
    produce = lab_cc.WholesalePrice(0.2, 90)
    self.assertEqual(produce.total, 18.0)
    self.assertEqual(produce.fruit, 1)
    self.assertFalse(lab_cc.CvtGetControllerCommandMessage(0, None, None, None))

    price = lab_cc.copy_double_pointer(0.6)
    self.assertAlmostEqual(lab_cc.TotalPrice(3, price), 1.8)
    lab_cc.delete_double_pointer(price)


if __name__ == '__main__':
  unittest.main()

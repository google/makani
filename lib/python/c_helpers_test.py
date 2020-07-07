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

"""Tests for makani.c_helpers."""

import ctypes
import unittest

from makani.avionics.common import pack_avionics_messages
from makani.common.c_math import filter as c_math_filter
from makani.common.c_math import util as c_math_util
from makani.lib.python import c_helpers


class Pack(unittest.TestCase):

  def testPackAndUnpack(self):
    before = pack_avionics_messages.MotorStatusMessage()
    before.bus_voltage = 3.14
    before.omega = 5.16
    packed = c_helpers.Pack(before)
    self.assertIsNotNone(packed)

    after = c_helpers.Unpack(packed, type(before))
    self.assertIsNotNone(after)
    self.assertEqual(after.bus_voltage, before.bus_voltage)
    self.assertEqual(after.omega, before.omega)

  def testUnpackBuffer(self):
    packed = (ctypes.c_uint8
              * pack_avionics_messages.PACK_MOTORSTATUSMESSAGE_SIZE)()
    self.assertIsNotNone(
        c_helpers.Unpack(packed, pack_avionics_messages.MotorStatusMessage))

  def testFailedPack(self):
    self.assertIsNone(c_helpers.Pack(3))
    self.assertIsNone(c_helpers.Pack('baboon'))

  def testFailedUnpack(self):
    # Buffer too small.
    packed = (ctypes.c_uint8
              * (pack_avionics_messages.PACK_MOTORSTATUSMESSAGE_SIZE - 1))()
    with self.assertRaises(c_helpers.UnpackError):
      c_helpers.Unpack(packed, pack_avionics_messages.MotorStatusMessage)

    # Buffer too large.
    packed = (ctypes.c_uint8
              * (pack_avionics_messages.PACK_MOTORSTATUSMESSAGE_SIZE + 1))()
    with self.assertRaises(c_helpers.UnpackError):
      c_helpers.Unpack(packed, pack_avionics_messages.MotorStatusMessage)

    # Type not unpackable.
    self.assertIsNone(
        c_helpers.Unpack(ctypes.create_string_buffer(10), unittest.TestCase))


ENUM_VALUES = {'kTestEnumValue%d' % i: i for i in range(20)}
FULL_ENUM = ENUM_VALUES.copy()
FULL_ENUM.update({'kTestEnumForceSigned': -1,
                  'kNumTestEnums': len(ENUM_VALUES)})


class FakeModule(object):
  """For use with EnumHelperTest."""

  def __init__(self, enum_dict):
    for name, value in enum_dict.iteritems():
      setattr(self, name, value)
    self.other_junk = 'Some other junk we should never see again'


class EnumHelperTest(unittest.TestCase):

  def setUp(self):
    module = FakeModule(FULL_ENUM)
    self.helper = c_helpers.EnumHelper('TestEnum', module)

  def testConversions(self):
    for name, value in ENUM_VALUES.iteritems():
      short_name = name.lstrip('kTestEnum')

      self.assertEqual(value, self.helper.Value(name))
      self.assertEqual(value, self.helper.Value(short_name))
      self.assertEqual(name, self.helper.Name(short_name))
      self.assertEqual(name, self.helper.Name(value))
      self.assertEqual(short_name, self.helper.ShortName(name))
      self.assertEqual(short_name, self.helper.ShortName(value))

  def testIteration(self):
    enum = {name: value for name, value in self.helper}
    self.assertEqual(enum, ENUM_VALUES)

  def testLen(self):
    self.assertEqual(len(self.helper), len(ENUM_VALUES))

  def testExceptions(self):
    with self.assertRaises(c_helpers.EnumError):
      self.helper.Value('Burrito')
    with self.assertRaises(c_helpers.EnumError):
      self.helper.Value('kTestEnumBurrito')
    with self.assertRaises(c_helpers.EnumError):
      self.helper.Name(len(ENUM_VALUES))
    with self.assertRaises(c_helpers.EnumError):
      self.helper.Name('Burrito')
    with self.assertRaises(c_helpers.EnumError):
      self.helper.ShortName('kTestEnumBurrito')
    with self.assertRaises(c_helpers.EnumError):
      self.helper.ShortName(len(ENUM_VALUES))

  def testShortNames(self):
    short_names = self.helper.ShortNames()
    self.assertEqual(len(short_names), 20)
    for i in range(20):
      self.assertEqual(short_names[i], self.helper.ShortName(i))

  def testValues(self):
    self.assertEqual(self.helper.Values(), range(20))

  def testManualPrefix(self):
    # The type name of the test enum is completely artifical. Here, we claim it
    # has a type name that disagrees with the prefix in order to test the prefix
    # argument.
    helper = c_helpers.EnumHelper('FakeTypeName', FakeModule(FULL_ENUM),
                                  prefix='kTestEnum')
    for name, value in ENUM_VALUES.iteritems():
      self.assertEqual(value, helper.Value(name))

  def testBadLength(self):
    bad_enum = FULL_ENUM.copy()
    bad_enum['kNumTestEnums'] = len(ENUM_VALUES) + 1
    module = FakeModule(bad_enum)
    with self.assertRaises(AssertionError):
      c_helpers.EnumHelper('TestEnum', module)

  def testNameCollision(self):
    bad_enum = {'kCollisionAValue1': 1,
                'kCollisionBValue1': 1}
    module = FakeModule(bad_enum)
    with self.assertRaises(AssertionError) as e:
      c_helpers.EnumHelper('Collision', module)
    self.assertIn('kCollisionAValue1', e.exception.message)
    self.assertIn('kCollisionBValue1', e.exception.message)


class CamelToSnakeTest(unittest.TestCase):

  def setUp(self):
    self._cases = [
        ('CamelToSnake', 'camel_to_snake'),
        ('HDF5', 'hdf5'),
        ('LEADINGCaps', 'leading_caps'),
        ('CamelToHDF5', 'camel_to_hdf5'),
        ('HTMLToSomethingElse', 'html_to_something_else'),
        ('HDF5ToSomethingElse', 'hdf5_to_something_else'),
        ('HDF5HTML', 'hdf5_html'),
    ]

  def testBasicCases(self):
    for c in self._cases:
      self.assertEqual(c_helpers.CamelToSnake(c[0]), c[1])


class CopyToEquivalentTypeTest(unittest.TestCase):

  def testSuccess(self):
    src = c_math_filter.Vec3()
    src.x, src.y, src.z = 1.0, 2.0, 3.0
    dest = c_helpers.CopyToEquivalentType(src, c_math_util.Vec3)
    self.assertIsInstance(dest, c_math_util.Vec3)
    self.assertEqual((src.x, src.y, src.z),
                     (dest.x, dest.y, dest.z))

  def testFailIfTypesArentEquivalent(self):
    src = c_math_util.Vec2()
    src.x, src.y = 1.0, 2.0
    with self.assertRaises(AssertionError):
      c_helpers.CopyToEquivalentType(src, c_math_util.Vec3)

  def testArray(self):
    src = (c_math_filter.Vec3 * 2)()
    src[0].x, src[0].y, src[0].z = 1.0, 2.0, 3.0
    src[0].x, src[0].y, src[0].z = 4.0, 5.0, 6.0
    dest_type = c_math_util.Vec3 * 2
    dest = c_helpers.CopyToEquivalentType(src, dest_type)
    self.assertEqual(type(dest), dest_type)
    for i in range(len(src)):
      self.assertEqual((src[i].x, src[i].y, src[i].z),
                       (dest[i].x, dest[i].y, dest[i].z))

  def testArrayErrors(self):
    with self.assertRaises(TypeError):
      c_helpers.CopyToEquivalentType(c_math_filter.Vec3 * 2,
                                     c_math_filter.Vec3)

    with self.assertRaises(TypeError):
      c_helpers.CopyToEquivalentType(c_math_filter.Vec3,
                                     c_math_filter.Vec3 * 2)

if __name__ == '__main__':
  unittest.main()

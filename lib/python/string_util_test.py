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

"""Tests for string_util."""

import unittest

from makani.lib.python import string_util


class StringUtilTest(unittest.TestCase):

  def testCamelToSnake(self):
    self.assertEqual(string_util.CamelToSnake('AppleBee'), 'apple_bee')
    self.assertEqual(string_util.CamelToSnake('AA12BB34'), 'aa12_bb34')
    self.assertEqual(string_util.CamelToSnake('Aa12Bb34'), 'aa12_bb34')
    self.assertEqual(string_util.CamelToSnake('orange', False), 'orange')
    self.assertEqual(string_util.CamelToSnake('orangeFarm', False),
                     'orange_farm')
    self.assertEqual(string_util.CamelToSnake('Orange_Farm', False),
                     'orange_farm')
    with self.assertRaises(AssertionError):
      string_util.CamelToSnake('orange')
    with self.assertRaises(AssertionError):
      string_util.CamelToSnake('orangeFarm')
    with self.assertRaises(AssertionError):
      string_util.CamelToSnake('Orange_Farm')

  def testSnakeToCamel(self):
    self.assertEqual(string_util.SnakeToCamel('apple_bee'), 'AppleBee')
    self.assertEqual(string_util.SnakeToCamel('aa12_bb34'), 'Aa12Bb34')
    with self.assertRaises(AssertionError):
      string_util.SnakeToCamel('Orange')
    with self.assertRaises(AssertionError):
      string_util.SnakeToCamel('orangeFarm')
    with self.assertRaises(AssertionError):
      string_util.SnakeToCamel('Orange_Farm')
    self.assertEqual(string_util.SnakeToCamel('Orange', False), 'Orange')
    self.assertEqual(string_util.SnakeToCamel('orangeFarm', False),
                     'OrangeFarm')
    self.assertEqual(string_util.SnakeToCamel('Orange_Farm', False),
                     'OrangeFarm')
    self.assertEqual(string_util.SnakeToCamel('AB12cd', False), 'AB12cd')

  def testSnakeCase(self):
    self.assertTrue(string_util.IsSnakeCase('abc123'))
    self.assertTrue(string_util.IsSnakeCase('abc_123'))
    self.assertTrue(string_util.IsSnakeCase('a1_b2'))
    self.assertTrue(string_util.IsSnakeCase('_a1_b2_'))

    self.assertFalse(string_util.IsSnakeCase('ABC123'))
    self.assertFalse(string_util.IsSnakeCase('Abc123'))
    self.assertFalse(string_util.IsSnakeCase('ABC_123'))
    self.assertFalse(string_util.IsSnakeCase('_A1_B2_'))

  def testCamelCase(self):
    self.assertTrue(string_util.IsCamelCase('AbcDef'))
    self.assertTrue(string_util.IsCamelCase('ABC123'))
    self.assertTrue(string_util.IsCamelCase('Abc123'))
    self.assertTrue(string_util.IsCamelCase('ABcd12'))
    self.assertTrue(string_util.IsCamelCase('AB12cd'))
    self.assertTrue(string_util.IsCamelCase('AB12Cd'))

    self.assertFalse(string_util.IsCamelCase('abc123'))
    self.assertFalse(string_util.IsCamelCase('abc_123'))
    self.assertFalse(string_util.IsCamelCase('a1_b2'))
    self.assertFalse(string_util.IsCamelCase('_a1_b2_'))
    self.assertFalse(string_util.IsCamelCase('ABC_123'))
    self.assertFalse(string_util.IsCamelCase('_A1_B2_'))


if __name__ == '__main__':
  unittest.main()

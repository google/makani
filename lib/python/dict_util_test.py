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

"""Tests for dict_util."""

import collections
import copy
import unittest

from makani.lib.python import dict_util


class DictUtilTest(unittest.TestCase):

  def testGetByPath(self):
    my_dict = {
        'fruit': {
            'apple': {
                'GoldenDelicious': 1,
                'Fuji': 2,
                'Gala': 3},
            'peach': {
                'SummerPearl': 4,
                'BlushingStar': 5}},
        'pi': 3.1415,
        'foo': {
            'bar': {
                'baz': 137}}}

    self.assertEqual(dict_util.GetByPath(my_dict, []), my_dict)
    self.assertEqual(dict_util.GetByPath(my_dict, ['fruit']), my_dict['fruit'])
    self.assertEqual(dict_util.GetByPath(my_dict, ['fruit', 'apple']),
                     my_dict['fruit']['apple'])
    self.assertEqual(dict_util.GetByPath(my_dict,
                                         ['fruit', 'peach', 'SummerPearl']),
                     my_dict['fruit']['peach']['SummerPearl'])
    self.assertEqual(dict_util.GetByPath(my_dict, ['pi']),
                     my_dict['pi'])
    self.assertEqual(dict_util.GetByPath(my_dict, ['foo', 'bar', 'baz']),
                     my_dict['foo']['bar']['baz'])

    with self.assertRaises(KeyError) as cm:
      dict_util.GetByPath(my_dict, ['fruit', 'apple', 'BlushingStar'])
    self.assertEqual(cm.exception.message,
                     'Can\'t find field fruit.apple.BlushingStar.')

  def testOrderDict(self):
    sd1 = collections.OrderedDict([('b', 22), ('a', 23)])
    sd2 = collections.OrderedDict([('d', 22), ('c', 23)])
    d = collections.OrderedDict([('b', 1), ('a', [sd1, sd2])])
    o = dict_util.OrderDict(d)
    self.assertEqual('a', o.keys()[0])
    self.assertEqual('b', o.keys()[1])
    self.assertEqual('a', o['a'][0].keys()[0])
    self.assertEqual('b', o['a'][0].keys()[1])
    self.assertEqual('c', o['a'][1].keys()[0])
    self.assertEqual('d', o['a'][1].keys()[1])

  def testMergeNestedDicts(self):
    overrides = {
        'sim': {
            'foo': 22,
            'bar': 23
        }
    }
    overrides_copy = copy.deepcopy(overrides)

    with self.assertRaises(dict_util.DictMergeError):
      dict_util.MergeNestedDicts(overrides, {'sim': 24})
    with self.assertRaises(dict_util.DictMergeError):
      dict_util.MergeNestedDicts({'sim': 24}, overrides)
    self.assertEqual(overrides_copy, overrides)

    with self.assertRaises(dict_util.DictMergeError):
      dict_util.MergeNestedDicts(overrides, {'sim': {'foo': 24}})
    with self.assertRaises(dict_util.DictMergeError):
      dict_util.MergeNestedDicts({'sim': {'foo': 24}}, overrides)
    self.assertEqual(overrides_copy, overrides)

    expected_overrides = {
        'sim': {
            'foo': 22,
            'bar': 23,
            'quux': 24
        }
    }

    updated_overrides = dict_util.MergeNestedDicts(overrides,
                                                   {'sim': {'quux': 24}})
    self.assertEqual(expected_overrides, updated_overrides)

  def testMustConsumeAllDictEntries(self):
    d = {'red': 137,
         'green': 22,
         'blue': 42}

    # Test for false negatives.
    with self.assertRaises(dict_util.UnreadKeysError):
      with dict_util.MustConsumeAllDictEntries(d) as x:
        self.assertEqual(x['red'], 137)
        self.assertEqual(x['green'], 22)

    # Test for false positives.
    with dict_util.MustConsumeAllDictEntries(d) as y:
      self.assertEqual(y.GetUnreadKeys(), set(y.keys()))
      self.assertEqual(y['red'], 137)
      self.assertEqual(y.GetUnreadKeys(), {'blue', 'green'})
      self.assertEqual(y['green'], 22)
      self.assertEqual(y['green'], 22)
      self.assertEqual(y.GetUnreadKeys(), {'blue'})
      self.assertEqual(y['blue'], 42)
      self.assertEqual(y.GetUnreadKeys(), set())

if __name__ == '__main__':
  unittest.main()

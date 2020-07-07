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

import copy
import unittest

from makani.lib.python.trackers import json_obj


class A(json_obj.JsonObj):

  def __init__(self):
    self.x = 63
    self.y = 'go'
    self.ref = B()


class B(json_obj.JsonObj):

  def __init__(self):
    self.z = [1, 2, 3]
    self.t = {'apple': 'fruit', 'palm': 'tree'}


class TestJsonObj(unittest.TestCase):

  def setUp(self):
    self.a = A()
    self.a_json = {
        'x': 63,
        'y': 'go',
        'py/object': '__main__.A',
        'ref': {
            'z': [1, 2, 3],
            't': {'apple': 'fruit', 'palm': 'tree'},
            'py/object': '__main__.B',
        }
    }

  def testObjDumps(self):
    a_json = self.a.Dumps()
    json_dict = copy.deepcopy(self.a_json)
    self.assertEqual(a_json, json_dict)

    a_json = self.a.Dumps()
    self.assertEqual(a_json, self.a_json)

  def testObjLoads(self):
    # Test loading directly from a json object.
    a_obj = json_obj.Loads(self.a_json)
    self.assertEqual(a_obj, self.a)

  def testUtilities(self):
    self.assertTrue(json_obj.HasKey(self.a, 'x'))
    self.assertFalse(json_obj.HasKey(self.a, 'j'))
    self.assertTrue(json_obj.HasKey(self.a_json, 'x'))
    self.assertFalse(json_obj.HasKey(self.a_json, 'j'))

    self.assertEqual(json_obj.GetAttr(self.a, 'x'), 63)
    self.assertEqual(json_obj.GetAttr(self.a_json, 'x'), 63)

    self.assertEqual(json_obj.GetAttrs(self.a), {'x', 'y', 'ref'})
    self.assertEqual(json_obj.GetAttrs(self.a_json), {'x', 'y', 'ref',
                                                      'py/object'})


if __name__ == '__main__':
  unittest.main()

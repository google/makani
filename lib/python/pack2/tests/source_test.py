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

from makani.lib.python.pack2 import parser


class TestSource(unittest.TestCase):

  def testSource(self):
    with open('lib/python/pack2/tests/test_param.p2', 'r') as f:
      source = f.read()

    p = parser.Parser()
    metadata = p.Parse(source)
    test_param = metadata.type_map['TestConfigParams']

    p2 = parser.Parser()
    metadata2 = p2.Parse(test_param.Source())
    test_param2 = metadata2.type_map['TestConfigParams']

    self.assertEqual(str(test_param), str(test_param2))

if __name__ == '__main__':
  unittest.main()

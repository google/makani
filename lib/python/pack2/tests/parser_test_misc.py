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

import textwrap
import unittest

from  makani.lib.python.pack2 import parser


class TestParserMisc(unittest.TestCase):

  def testString(self):
    text = textwrap.dedent("""\
        // test_param
        param test_param {
          uint8 thing_a;
          string[5] thing_b;
          float32 thing_c;
        }
        """)

    expected = textwrap.dedent("""\
        param test_param {
          uint8 thing_a;  // offset: 0
          string[5] thing_b;  // offset: 1
          float32 thing_c;  // offset: 8
        }
        """)

    p = parser.Parser()
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testComment(self):
    text = textwrap.dedent("""\
        // test_param
        param test_param {
          uint8 thing_a;
          // uint32 thing_b;
          float32 thing_c;
        }
        """)

    expected = textwrap.dedent("""\
        param test_param {
          uint8 thing_a;  // offset: 0
          float32 thing_c;  // offset: 4
        }
        """)

    p = parser.Parser()
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testBadKeyword(self):
    # Ensure a ParseError is raised on an invalid keyword.
    text = textwrap.dedent("""\
        garbage test_struct {
          uint8 thing_a;
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

if __name__ == '__main__':
  unittest.main()

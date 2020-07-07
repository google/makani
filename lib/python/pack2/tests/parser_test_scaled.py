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


class TestParserScaled(unittest.TestCase):

  def testScaled(self):
    text = textwrap.dedent("""\
        scaled8 test_scaled8 {
          offset = 0.0,
          scale = -1.0,
        }

        scaled16 test_scaled16 {
          offset = 1.2e1,
          scale = 0.1e-3,
        }

        scaled32 test_scaled32 {
          offset = -13,
          scale = -0.1e30,
        }

        scaled32 test_scaled32_2 {
          offset = -0,
          scale = -1,
        }

        param test_param {
          uint8 thing_a;
          float32 thing_b;

          test_scaled8 thing_c;
          test_scaled16 thing_d;
          test_scaled32 thing_e;
          test_scaled32_2 thing_f;
        }
        """)

    expected = textwrap.dedent("""\
        param test_param {
          uint8 thing_a;  // offset: 0
          float32 thing_b;  // offset: 4
          scaled8 test_scaled8 {
            offset = 0,
            scale = -1,
          } thing_c;  // offset: 8
          scaled16 test_scaled16 {
            offset = 12,
            scale = 0.0001,
          } thing_d;  // offset: 10
          scaled32 test_scaled32 {
            offset = -13,
            scale = -1e+29,
          } thing_e;  // offset: 12
          scaled32 test_scaled32_2 {
            offset = 0,
            scale = -1,
          } thing_f;  // offset: 16
        }
        """)
    p = parser.Parser()
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testDupScaledType(self):
    # Ensure a SyntaxError is raised when two scaled type have the same name.
    text = textwrap.dedent("""\
        scaled8 test_scaled8 {
          offset = -0,
          scale = -1,
        }

        scaled8 test_scaled8 {
          offset = -0,
          scale = -1,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testDupScaledProperty(self):
    # Ensure a SyntaxError is raised when a scaled type has duplicate
    # properties.
    text = textwrap.dedent("""\
        scaled8 test_scaled8 {
          offset = -0,
          scale = -1,
          scale = -1,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testNoScaledOffset(self):
    # Ensure a SyntaxError is raised when a scaled type has no offset property.
    text = textwrap.dedent("""\
        scaled8 test_scaled8 {
          scale = -1,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testNoScaledScale(self):
    # Ensure a SyntaxError is raised when a scaled type has no scale property.
    text = textwrap.dedent("""\
        scaled8 test_scaled8 {
          offset = -1,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testUnnownScaledProperty(self):
    # Ensure a SyntaxError is raised when a scaled type has an unknown property.
    text = textwrap.dedent("""\
        scaled8 test_scaled8 {
          offset = -0,
          scale = -1,
          abc = -1,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testScaledNotFloat(self):
    # Ensure a SyntaxError is raised when a scaled property is not a float.
    text = textwrap.dedent("""\
        scaled8 test_scaled8 {
          offset = -0,
          scale = abc,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)
if __name__ == '__main__':
  unittest.main()

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


class TestParserEnum(unittest.TestCase):

  def testEnum(self):
    text = textwrap.dedent("""\
        enum8 test_enum8 {
          VALNEG = -1,
          VAL0 = 0,
          VAL1 = 1,
          VAL2 = 0b10,
          VAL_MAX = 0x7f,
        }

        enum16 test_enum16 {
          VALNEG = -1,
          VAL0 = 0,
          VAL1 = 1,
          VAL2 = 0b10,
          VALMAX = 0x7fff,
        }

        enum32 test_enum32 {
          VALNEG = -1,
          VAL0 = 0,
          VAL1 = 1,
          VAL2 = 0b10,
          VAL_MAX = 0x7fffffff,
        }

        param test_param {
          uint8 thing_a;
          float32 thing_b;

          test_enum8 thing_c;
          test_enum16 thing_d;
          test_enum32 thing_e;
        }
        """)

    expected = textwrap.dedent("""\
        param test_param {
          uint8 thing_a;  // offset: 0
          float32 thing_b;  // offset: 4
          enum8 test_enum8 {
            VALNEG = -1,
            VAL0 = 0,
            VAL1 = 1,
            VAL2 = 2,
            VAL_MAX = 127,
          } thing_c;  // offset: 8
          enum16 test_enum16 {
            VALNEG = -1,
            VAL0 = 0,
            VAL1 = 1,
            VAL2 = 2,
            VALMAX = 32767,
          } thing_d;  // offset: 10
          enum32 test_enum32 {
            VALNEG = -1,
            VAL0 = 0,
            VAL1 = 1,
            VAL2 = 2,
            VAL_MAX = 2147483647,
          } thing_e;  // offset: 12
        }
        """)
    p = parser.Parser()
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testDupEnumType(self):
    # Ensure a SyntaxError is raised when two enums have the same name.
    text = textwrap.dedent("""\
        enum8 test_enum8 {
          VAL0 = 0,
          VAL1 = 1,
        }

        enum8 test_enum8 {
          VAL0 = 0,
          VAL1 = 1,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testDupEnumLabel(self):
    # Ensure a SyntaxError is raised when an enum contains duplicate labels.
    text = textwrap.dedent("""\
        enum8 test_enum8 {
          VAL0 = 0,
          VAL0 = 1,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testDupEnumValue(self):
    # Ensure a SyntaxError is raised when an enum contains duplicate values.
    text = textwrap.dedent("""\
        enum8 test_enum8 {
          VAL0 = 0,
          VAL1 = 0,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def _TestEnumBadValue(self, width, value):
    text = textwrap.dedent("""\
        enum{width} test_enum {{
          VAL0 = {value},
        }}
        """).format(width=width, value=value)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def _TestEnumRanges(self, width):
    self._TestEnumBadValue(width, 2**(width - 1))
    self._TestEnumBadValue(width, -(2**(width - 1) + 1))

  def testEnum8Ranges(self):
    self._TestEnumRanges(8)

  def testEnum16Ranges(self):
    self._TestEnumRanges(16)

  def testEnum32Ranges(self):
    self._TestEnumRanges(32)

if __name__ == '__main__':
  unittest.main()

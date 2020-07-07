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


class TestParserBitfield(unittest.TestCase):

  def testBitfield(self):
    text = textwrap.dedent("""\
        bitfield8 test_bf8 {
          0: FLAG0,
          1: FLAG1,
          7: FLAG_MAX,
        }

        bitfield16 test_bf16 {
          0: FLAG0,
          1: FLAG1,
          15: FLAG_MAX,
        }

        bitfield32 test_bf32 {
          0: FLAG0,
          1: FLAG1,
          31: FLAG_MAX,
        }

        param test_param {
          uint8 thing_a;
          float32 thing_b;

          test_bf8 thing_c;
          test_bf16 thing_d;
          test_bf32 thing_e;
        }
        """)

    expected = textwrap.dedent("""\
        param test_param {
          uint8 thing_a;  // offset: 0
          float32 thing_b;  // offset: 4
          bitfield8 test_bf8 {
            0: FLAG0,
            1: FLAG1,
            7: FLAG_MAX,
          } thing_c;  // offset: 8
          bitfield16 test_bf16 {
            0: FLAG0,
            1: FLAG1,
            15: FLAG_MAX,
          } thing_d;  // offset: 10
          bitfield32 test_bf32 {
            0: FLAG0,
            1: FLAG1,
            31: FLAG_MAX,
          } thing_e;  // offset: 12
        }
        """)
    p = parser.Parser()
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testDupBitmapType(self):
    # Ensure a SyntaxError is raised when two bitfields have the same name.
    text = textwrap.dedent("""\
        bitfield8 test_bitfield8 {
          0: FLAG0,
          1: FLAG1,
        }

        bitfield16 test_bitfield8 {
          0: FLAG0,
          1: FLAG1,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testDupBitmapLabel(self):
    # Ensure a SyntaxError is raised when a bitmap contains duplicate labels.
    text = textwrap.dedent("""\
        bitfield8 test_bitfield8 {
          0: FLAG0,
          1: FLAG0,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testDupBitmapValue(self):
    # Ensure a SyntaxError is raised when a bitmap contains duplicate values.
    text = textwrap.dedent("""\
        bitfield8 test_bitfield8 {
          0: FLAG0,
          0: FLAG1,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testBitmapNegValue(self):
    # Ensure a SyntaxError is raised when a bitmap contains a negative value.
    text = textwrap.dedent("""\
        bitfield8 test_bitfield8 {
          0: FLAG0,
          -1: FLAG1,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testBitmap8Overflow(self):
    # Ensure a SyntaxError is raised when a bitmap8 contains a value that is out
    # of range of its width.
    text = textwrap.dedent("""\
        bitfield8 test_bitfield8 {
          0: FLAG0,
          8: FLAG8,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testBitmap16Overflow(self):
    # Ensure a SyntaxError is raised when a bitmap16 contains a value that is
    # out of range of its width.
    text = textwrap.dedent("""\
        bitfield16 test_bitfield16 {
          0: FLAG0,
          16: FLAG8,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testBitmap32Overflow(self):
    # Ensure a SyntaxError is raised when a bitmap32 contains a value that is
    # out of range of its width.
    text = textwrap.dedent("""\
        bitfield32 test_bitfield32 {
          0: FLAG0,
          32: FLAG8,
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

if __name__ == '__main__':
  unittest.main()

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


class TestParserStruct(unittest.TestCase):

  def testSimpleHeader(self):
    text = textwrap.dedent("""\
        header ParamHeader {
          uint32 param_format_version;
          int32 unused;
          int32 data_length;
          uint32 checksum;
          uint32 version_number;
        }
        """)

    expected = textwrap.dedent("""\
        header ParamHeader {
          uint32 param_format_version;  // offset: 0
          int32 unused;  // offset: 4
          int32 data_length;  // offset: 8
          uint32 checksum;  // offset: 12
          uint32 version_number;  // offset: 16
        }
        """)

    p = parser.Parser()
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testSimpleParam(self):
    text = textwrap.dedent("""\
        param SerialParams {
          int32 serial_number;
          date date_of_manufacture;
          string[32] part_number;
        }
        """)

    expected = textwrap.dedent("""\
        param SerialParams {
          int32 serial_number;  // offset: 0
          date date_of_manufacture;  // offset: 4
          string[32] part_number;  // offset: 8
        }
        """)

    p = parser.Parser()
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testStruct(self):
    text = textwrap.dedent("""\
        struct test_struct {
          uint8 field0;
          int32 field1;
        }

        param test_param {
          uint8 thing_a;
          float32 thing_b;

          test_struct thing_c;
          test_struct thing_d;
        }
        """)

    expected = textwrap.dedent("""\
        param test_param {
          uint8 thing_a;  // offset: 0
          float32 thing_b;  // offset: 4
          struct test_struct {
            uint8 field0;  // offset: 0
            int32 field1;  // offset: 4
          } thing_c;  // offset: 8
          struct test_struct {
            uint8 field0;  // offset: 0
            int32 field1;  // offset: 4
          } thing_d;  // offset: 16
        }
        """)
    p = parser.Parser()
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testArrayField(self):
    text = textwrap.dedent("""\
        param test_param {
          uint8 thing_a[7];
          uint32 thing_b;
        }
        """)

    expected = textwrap.dedent("""\
        param test_param {
          uint8 thing_a[7];  // offset: 0
          uint32 thing_b;  // offset: 8
        }
        """)

    p = parser.Parser()
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testDateField(self):
    text = textwrap.dedent("""\
        param test_param {
          date date;
          uint32 thing_b;
        }
        """)

    expected = textwrap.dedent("""\
        param test_param {
          date date;  // offset: 0
          uint32 thing_b;  // offset: 4
        }
        """)

    p = parser.Parser()
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testArrayFieldNegativeExtent(self):
    # Ensure that a ParseError is raised on an array with a negative length.
    text = textwrap.dedent("""\
        param test_param {
          uint8 thing_a[-7];
          uint32 thing_b;
        }
        """)

    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testDuplicateField(self):
    # Ensure that a ParseError is raised when two fields have the same name.
    text = textwrap.dedent("""\
        param test_param {
          uint8 thing_a;
          float32 thing_a;
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testDuplicateHeader(self):
    # Ensure a ParseError is raised when two headers have the same name.
    text = textwrap.dedent("""\
        header test_header {
          uint8 thing_a;
        }

        header test_header {
          uint8 thing_a;
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testDuplicateParam(self):
    # Ensure a ParseError is raised when two params have the same name.
    text = textwrap.dedent("""\
        param test_param {
          uint8 thing_a;
        }

        param test_param {
          uint8 thing_a;
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testDuplicateStruct(self):
    # Ensure a ParseError is raised when two structs have the same name.
    text = textwrap.dedent("""\
        struct test_struct {
          uint8 thing_a;
        }

        struct test_struct {
          uint8 thing_a;
        }
        """)
    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

if __name__ == '__main__':
  unittest.main()

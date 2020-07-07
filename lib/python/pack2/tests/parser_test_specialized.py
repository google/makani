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


class TestParserSpecialized(unittest.TestCase):

  def testSpecialized(self):
    text = textwrap.dedent("""\
        enum32 FooRevision {
          kFooRevisionA = 0,
          kFooRevisionB = 1,
        }

        param BaseConfigParams {
          string[32] name;
          int32 revision;
        }

        specialize(BaseConfigParams) FooConfigParams {
          FooRevision revision;
        }
        """)

    expected = textwrap.dedent("""\
        param BaseConfigParams {
          string[32] name;  // offset: 0
          int32 revision;  // offset: 32
        }
        param FooConfigParams {
          string[32] name;  // offset: 0
          enum32 FooRevision {
            kFooRevisionA = 0,
            kFooRevisionB = 1,
          } revision;  // offset: 32
        }
        """)
    p = parser.Parser()
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testParentNotDefined(self):
    text = textwrap.dedent("""\
        enum32 FooRevision {
          kFooRevisionA = 0,
          kFooRevisionB = 1,
        }

        specialize(BaseConfigParams) FooConfigParams {
          FooRevision revision;
        }
        """)

    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testEnumNotDefined(self):
    text = textwrap.dedent("""\
        param BaseConfigParams {
          string[32] name;
          int32 revision;
        }

        specialize(BaseConfigParams) FooConfigParams {
          FooRevision revision;
        }
        """)

    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testMisnamedField(self):
    text = textwrap.dedent("""\
        param BaseConfigParams {
          string[32] name;
          int32 revision;
        }

        enum32 FooRevision {
          kFooRevisionA = 0,
          kFooRevisionB = 1,
        }

        specialize(BaseConfigParams) FooConfigParams {
          FooRevision bad_revision;
        }
        """)

    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testNotSignedInt(self):
    text = textwrap.dedent("""\
        param BaseConfigParams {
          string[32] name;
          uint32 revision;
        }

        enum32 FooRevision {
          kFooRevisionA = 0,
          kFooRevisionB = 1,
        }

        specialize(BaseConfigParams) FooConfigParams {
          FooRevision revision;
        }
        """)

    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testMismatchedWidth(self):
    text = textwrap.dedent("""\
        param BaseConfigParams {
          string[32] name;
          int32 revision;
        }

        enum16 FooRevision {
          kFooRevisionA = 0,
          kFooRevisionB = 1,
        }

        specialize(BaseConfigParams) FooConfigParams {
          FooRevision revision;
        }
        """)

    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testBadParentExtent(self):
    text = textwrap.dedent("""\
        param BaseConfigParams {
          string[32] name;
          int32 revision[2];
        }

        enum32 FooRevision {
          kFooRevisionA = 0,
          kFooRevisionB = 1,
        }

        specialize(BaseConfigParams) FooConfigParams {
          FooRevision revision;
        }
        """)

    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testBadFieldExtent(self):
    text = textwrap.dedent("""\
        param BaseConfigParams {
          string[32] name;
          int32 revision;
        }

        enum32 FooRevision {
          kFooRevisionA = 0,
          kFooRevisionB = 1,
        }

        specialize(BaseConfigParams) FooConfigParams {
          FooRevision revision[2];
        }
        """)

    p = parser.Parser()
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

if __name__ == '__main__':
  unittest.main()

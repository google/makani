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


class _MockFileLoader(object):

  def __init__(self):
    self.file_map = {}

  def AddFile(self, name, contents):
    self.file_map[name] = contents

  def ReadFile(self, file_name):
    return self.file_map[file_name]


class TestParserInclude(unittest.TestCase):

  def _SimpleFailure(self, path):
    loader = _MockFileLoader()
    loader.AddFile(path, textwrap.dedent("""\
        enum8 ChildEnum {
          kValue0 = 0,
          kValue1 = 1,
        }
        """))

    text = textwrap.dedent("""\
        include "{path}";

        param ParentParam {{
          ChildEnum child_enum;
        }}
        """).format(path=path)
    p = parser.Parser(file_loader=loader)
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testInclude(self):
    loader = _MockFileLoader()
    loader.AddFile('child.p2', textwrap.dedent("""\
        enum8 ChildEnum {
          kValue0 = 0,
          kValue1 = 1,
        }
        """))

    text = textwrap.dedent("""\
        include "child.p2";

        param ParentParam {
          ChildEnum child_enum;
        }
        """)

    expected = textwrap.dedent("""\
        param ParentParam {
          enum8 ChildEnum {
            kValue0 = 0,
            kValue1 = 1,
          } child_enum;  // offset: 0
        }
        """)
    p = parser.Parser(file_loader=loader)
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testDupicateIncludeOK(self):
    loader = _MockFileLoader()
    loader.AddFile('child.p2', textwrap.dedent("""\
        enum8 ChildEnum {
          kValue0 = 0,
          kValue1 = 1,
        }
        """))

    text = textwrap.dedent("""\
        include "child.p2";
        include "child.p2";

        param ParentParam {
          ChildEnum child_enum;
        }
        """)

    expected = textwrap.dedent("""\
        param ParentParam {
          enum8 ChildEnum {
            kValue0 = 0,
            kValue1 = 1,
          } child_enum;  // offset: 0
        }
        """)
    p = parser.Parser(file_loader=loader)
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testDuplicateIndirectIncludeOK(self):
    loader = _MockFileLoader()
    loader.AddFile('child3.p2', textwrap.dedent("""\
        enum8 ChildEnum3 {
          kValue0 = 0,
          kValue1 = 1,
        }
        """))
    loader.AddFile('child2.p2', textwrap.dedent("""\
        include "child3.p2";

        enum8 ChildEnum2 {
          kValue0 = 0,
          kValue1 = 1,
        }
        """))
    loader.AddFile('child1.p2', textwrap.dedent("""\
        include "child3.p2";

        enum8 ChildEnum1 {
          kValue0 = 0,
          kValue1 = 1,
        }
        """))

    text = textwrap.dedent("""\
        include "child1.p2";
        include "child2.p2";

        param ParentParam {
          ChildEnum1 child_enum1;
          ChildEnum2 child_enum2;
          ChildEnum3 child_enum3;
        }
        """)

    expected = textwrap.dedent("""\
        param ParentParam {
          enum8 ChildEnum1 {
            kValue0 = 0,
            kValue1 = 1,
          } child_enum1;  // offset: 0
          enum8 ChildEnum2 {
            kValue0 = 0,
            kValue1 = 1,
          } child_enum2;  // offset: 1
          enum8 ChildEnum3 {
            kValue0 = 0,
            kValue1 = 1,
          } child_enum3;  // offset: 2
        }
        """)
    p = parser.Parser(file_loader=loader)
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testRecursiveIncludeOK(self):
    loader = _MockFileLoader()
    loader.AddFile('child.p2', textwrap.dedent("""\
        include "child.p2";

        enum8 ChildEnum {
          kValue0 = 0,
          kValue1 = 1,
        }
        """))

    text = textwrap.dedent("""\
        include "child.p2";

        param ParentParam {
          ChildEnum child_enum;
        }
        """)

    expected = textwrap.dedent("""\
        param ParentParam {
          enum8 ChildEnum {
            kValue0 = 0,
            kValue1 = 1,
          } child_enum;  // offset: 0
        }
        """)
    p = parser.Parser(file_loader=loader)
    metadata = p.Parse(text)
    self.assertEqual(str(metadata), expected)

  def testRedefiendIncludeType(self):
    loader = _MockFileLoader()
    loader.AddFile('child.p2', textwrap.dedent("""\
        enum8 ChildEnum {
          kValue0 = 0,
          kValue1 = 1,
        }
        """))
    loader.AddFile('child2.p2', textwrap.dedent("""\
        enum8 ChildEnum {
          kValue0 = 0,
          kValue1 = 1,
        }
        """))

    text = textwrap.dedent("""\
        include "child.p2";
        include "child2.p2";

        param ParentParam {
          ChildEnum child_enum;
        }
        """)
    p = parser.Parser(file_loader=loader)
    with self.assertRaises(parser.ParseError):
      p.Parse(text)

  def testMisnamedInclude(self):
    self._SimpleFailure('child2')

  def testNoAbsoluteInclude(self):
    self._SimpleFailure('/child2.p2')

  def testNoBackRefInclude(self):
    self._SimpleFailure('test/../child2.p2')
    self._SimpleFailure('../child2.p2')

  def testNoCurrentDirInclude(self):
    self._SimpleFailure('test/./child2.p2')
    self._SimpleFailure('./child2.p2')


if __name__ == '__main__':
  unittest.main()

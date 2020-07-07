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

from makani.lib.python.pack2 import parser


class _NullFileLoader(object):

  def ReadFile(self, _):
    return ''


class TestFormatter(unittest.TestCase):

  def testFormatter(self):
    with open('lib/python/pack2/tests/test_param_pre_fmt.p2', 'r') as f:
      pre_format = f.read()
    with open('lib/python/pack2/tests/test_param_post_fmt.p2', 'r') as f:
      post_format = f.read()

    p = parser.Parser()
    p.Parse(pre_format)
    self.assertEqual(p.GetFormattedSource(), post_format)

  def testIncludeOrder(self):
    pre_format = textwrap.dedent("""\
        include "charlie.p2";
        include "beta.p2";

        param TestParam {
          int32 test_field;
        }
        include "delta.p2";
        include "alpha.p2";
        """)
    post_format = textwrap.dedent("""\
        include "alpha.p2";
        include "beta.p2";
        include "charlie.p2";
        include "delta.p2";

        param TestParam {
          int32 test_field;
        }
        """)
    p = parser.Parser(file_loader=_NullFileLoader())
    p.Parse(pre_format)
    self.assertEqual(p.GetFormattedSource(), post_format)


if __name__ == '__main__':
  unittest.main()

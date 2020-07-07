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

from makani.lib.python import import_util


class PracticeClass(object):
  value = 0

  def __init__(self, value=1):
    self.value = value


class TestImportClass(unittest.TestCase):
  """Test utilities in the analysis module."""

  def testAttributeError(self):
    path_to_class = 'makani.lib.python.import_util_test.FakeClass'
    with self.assertRaises(AttributeError):
      import_util.ImportClass(path_to_class)

  def testTestClass(self):
    """Test that only the module is returned and not initialized."""
    path_to_class = 'makani.lib.python.import_util_test.PracticeClass'
    cls = import_util.ImportClass(path_to_class)
    self.assertEqual(cls.value, 0)

if __name__ == '__main__':
  unittest.main()

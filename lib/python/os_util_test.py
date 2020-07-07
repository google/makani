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

"""Tests for makani.os_util."""

import os
import shutil
import unittest

import gflags
from makani.lib.python import os_util
from makani.lib.python import test_util
import mock

FLAGS = gflags.FLAGS


class ChangeDirTest(unittest.TestCase):

  def testChangeDir(self):
    original_dir = os.getcwd()
    with os_util.ChangeDir('/tmp'):
      self.assertEqual(os.path.realpath('/tmp'), os.getcwd())
    self.assertEqual(original_dir, os.getcwd())


class TempDirTest(unittest.TestCase):

  def DirBasename(self, path):
    """Gets the basename of a directory, regardless of trailing separator."""
    return os.path.basename(path.rstrip(os.path.sep))

  def testDeleteDirOnExit(self):
    with os_util.TempDir() as temp_dir:
      self.assertTrue(os.path.isdir(temp_dir))
    self.assertFalse(os.path.exists(temp_dir))

  def testDeleteDirOnError(self):
    with self.assertRaises(ValueError), os_util.TempDir() as temp_dir:
      dir_exists_before_exception = os.path.isdir(temp_dir)
      raise ValueError

    self.assertTrue(dir_exists_before_exception)
    self.assertFalse(os.path.exists(temp_dir))

  def testSpecifyDirName(self):
    # Grab an unused directory name.
    with os_util.TempDir() as temp_dir:
      pass

    with os_util.TempDir(self.DirBasename(temp_dir)):
      self.assertTrue(os.path.isdir(temp_dir))
    self.assertFalse(os.path.exists(temp_dir))

  @test_util.FlagValueSaver()
  def testDeleteTempDirsFlag(self):
    FLAGS.delete_temp_dirs = False

    with mock.patch(os_util.__name__ + '.logging.info') as mock_log:
      with os_util.TempDir('foo') as temp_dir:
        self.assertTrue(os.path.isdir(temp_dir))

    self.assertTrue(os.path.isdir(temp_dir))
    self.assertTrue(mock_log.assert_any_call)
    shutil.rmtree(temp_dir)

  @test_util.FlagValueSaver()
  def testDeleteTempDirsOnErrorFlag(self):
    FLAGS.delete_temp_dirs_on_error = False

    with mock.patch(os_util.__name__ + '.logging.warning') as mock_log:
      with self.assertRaises(ValueError):
        with os_util.TempDir('foo') as temp_dir:
          self.assertTrue(os.path.isdir(temp_dir))
          raise ValueError

    self.assertTrue(os.path.isdir(temp_dir))
    self.assertTrue(mock_log.assert_any_call)
    shutil.rmtree(temp_dir)

  def testCantUseAbsolutePath(self):
    with os_util.TempDir() as temp_dir:  # temp_dir is an absolute path.
      with self.assertRaisesRegexp(os_util.OsUtilError,
                                   r'.*does not accept absolute paths.*'):
        with os_util.TempDir(temp_dir):
          pass

  def testTempDirsCollide(self):
    # The second TempDir tries to create a directory with the same name as the
    # first, resulting in an error.
    with os_util.TempDir() as temp_dir:
      temp_basename = self.DirBasename(temp_dir)
      with self.assertRaisesRegexp(os_util.OsUtilError, r'.*will delete.*'):
        with os_util.TempDir(temp_basename):
          pass

  def testIntermediateDirectoryMustExist(self):
    # If the intermediate directory exists, everything works as expected.
    with os_util.TempDir() as intermediate_path:
      intermediate_basename = self.DirBasename(intermediate_path)
      final_basename = os.path.join(intermediate_basename, 'foo')

      with os_util.TempDir(final_basename) as final_dir:
        self.assertTrue(os.path.exists(final_dir))
      self.assertFalse(os.path.exists(final_dir))

    # If the intermediate directory doesn't exist, we get an error.
    with self.assertRaisesRegexp(os_util.OsUtilError,
                                 r'.*must already exist.*'):
      with os_util.TempDir(final_basename):
        pass

  def testDirPathIsAFile(self):
    with os_util.TempDir() as temp_dir:  # Working directory for this test.
      file_path = os.path.join(temp_dir, 'foo')
      with open(file_path, 'w') as f:
        f.write('foo')

      file_relpath = os.path.join(self.DirBasename(temp_dir), 'foo')
      with self.assertRaisesRegexp(os_util.OsUtilError, r'.*not a directory.*'):
        with os_util.TempDir(file_relpath):
          pass


class CommonPrefixTest(unittest.TestCase):

  def testFoo(self):
    # Absolute, normalized paths.
    self.assertEqual('/abc', os_util.CommonPrefix(('/abc/def', '/abc/ghi')))
    self.assertEqual('/abc', os_util.CommonPrefix(('/abc/de', '/abc/def')))
    self.assertEqual('/abc',
                     os_util.CommonPrefix(('/abc/def/ghi', '/abc/jkl/ghi')))
    self.assertEqual('/abc', os_util.CommonPrefix(('/abc', '/abc')))
    self.assertEqual('/abc', os_util.CommonPrefix(('/abc/def', '/abc/ghi')))
    self.assertEqual('/abc/def',
                     os_util.CommonPrefix(('/abc/def', '/abc/def/ghi')))
    self.assertEqual('/abc',
                     os_util.CommonPrefix(('/abc/def', '/abc/ghi', '/abc/jkl')))
    self.assertEqual('/', os_util.CommonPrefix(('/abc/def', '/ghi/jkl')))

    # Path normalization.
    self.assertEqual('/abc', os_util.CommonPrefix(('/abc//def', '////abc/ghi')))

    # Relative paths.
    self.assertEqual('', os_util.CommonPrefix(('abc', 'def')))
    self.assertEqual('abc', os_util.CommonPrefix(('abc/def', 'abc/ghi')))

    # Mixing absolute and relative paths.
    self.assertEqual('', os_util.CommonPrefix(('/abc', 'abc')))


if __name__ == '__main__':
  unittest.main()

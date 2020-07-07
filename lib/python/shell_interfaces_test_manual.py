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

"""Tests for the shell_interfaces module."""

import os
import shutil
import subprocess
import tarfile
import tempfile
import unittest

import gflags
from makani.lib.python import shell_interfaces
from makani.lib.python import test_util

FLAGS = gflags.FLAGS


class ExecutorTest(unittest.TestCase):

  def setUp(self):
    self.executor = shell_interfaces.Executor()

  def testRun(self):
    self.assertEqual(0, self.executor.Run(['sleep', '0.1']).returncode)

  def testCheckRun(self):
    with self.assertRaises(shell_interfaces.ExecutorError):
      with open(os.devnull, 'w') as devnull:
        self.executor.CheckRun(['sleep', 'a'], stderr=devnull)

  def testRunInBackground(self):
    process = self.executor.RunInBackground(['sleep', '1000000'])
    self.assertIsNone(process.poll())  # Still running

    process.kill()
    self.assertLess(process.poll(), 0)

  def testKillIfRunning(self):
    process = self.executor.RunInBackground(['sleep', '1000000'])
    self.assertIsNone(process.poll())  # Still running
    self.executor.KillIfRunning(process)
    self.assertLess(process.poll(), 0)
    self.assertRaises(shell_interfaces.ExecutorError,
                      self.executor.KillIfRunning, process)

  def testKillRunningProcessesInDestructor(self):
    process1 = self.executor.RunInBackground(['sleep', '1000000'])
    process2 = self.executor.RunInBackground(['sleep', '1000000'])
    self.assertIsNone(process1.poll())
    self.assertIsNone(process2.poll())

    del self.executor
    self.assertLess(process1.poll(), 0)
    self.assertLess(process2.poll(), 0)

  def testCaptureStdout(self):
    _, stdout, stderr = self.executor.Run(['sleep', '--version'],
                                          stdout=subprocess.PIPE)
    self.assertGreater(len(stdout), 0)
    self.assertIsNone(stderr)

  def testCaptureStderr(self):
    _, stdout, stderr = self.executor.Run(['sleep', 'a'],
                                          stderr=subprocess.PIPE)
    self.assertIsNone(stdout)
    self.assertGreater(len(stderr), 0)


class PackagerTest(unittest.TestCase):

  def Path(self, filename):
    return os.path.join(self.tempdir, filename)

  def setUp(self):
    self.packager = shell_interfaces.Packager()
    self.tempdir = tempfile.mkdtemp()
    self.package_path = self.Path('package.tar.gz')

  def tearDown(self):
    shutil.rmtree(self.tempdir)

  def CreateFile(self, relpath, contents):
    path = self.Path(relpath)
    dirname = os.path.dirname(path)
    if not os.path.exists(dirname):
      os.mkdir(dirname)
    with open(path, 'w') as f:
      f.write(contents)
    return path

  def CheckFileInPackage(self, package, path_in_package, contents):
    self.assertIn(path_in_package, package.getnames())
    f = package.extractfile(path_in_package)
    self.assertEqual(contents, f.read())
    f.close()

  # This test assumes we're operating in a git repository.  It's acutally
  # creating an archive of HEAD, but that only takes O(1 ms) at time of
  # writing.
  def testCreateRepoPackage(self):
    # Restrict the archive to a few specific files.  This keeps test execuation
    # time from getting longer, and we'd only be verifying a few files anyway.
    verified_files = ['lib/python/shell_interfaces.py',
                      'lib/python/shell_interfaces_test_manual.py']

    with test_util.DisableWarnings():
      self.packager.CreateRepoPackage(self.package_path,
                                      restrict_paths=verified_files)

    with tarfile.open(self.package_path, 'r:gz') as package:
      for filename in verified_files:
        self.assertIn(filename, package.getnames())

  def testAddFile(self):
    monkey_path = self.CreateFile('monkey.txt', 'LEMUR')
    bird_path = self.CreateFile('bird.txt', 'PELICAN')

    self.packager.OpenPackage(self.package_path)
    self.packager.AddFile(monkey_path, 'monkey/lemur.txt')
    self.packager.AddFile(bird_path, 'bird/pelican.txt')
    self.packager.ClosePackage()

    with tarfile.open(self.package_path, 'r:gz') as package:
      self.CheckFileInPackage(package, 'monkey/lemur.txt', 'LEMUR')
      self.CheckFileInPackage(package, 'bird/pelican.txt', 'PELICAN')

  def testAddString(self):
    self.packager.OpenPackage(self.package_path)
    self.packager.AddString('LEMUR', 'monkey/lemur.txt')
    self.packager.AddString('PELICAN', 'bird/pelican.txt')
    self.packager.ClosePackage()

    with tarfile.open(self.package_path, 'r:gz') as package:
      self.CheckFileInPackage(package, 'monkey/lemur.txt', 'LEMUR')
      self.CheckFileInPackage(package, 'bird/pelican.txt', 'PELICAN')


class GetStdoutTest(unittest.TestCase):

  def testSuccess(self):
    self.assertEqual(shell_interfaces.GetStdout('echo foo'), 'foo\n')

  def testNonzeroReturnCode(self):
    with self.assertRaises(RuntimeError):
      shell_interfaces.GetStdout('grep -q abc def')


if __name__ == '__main__':
  unittest.main()

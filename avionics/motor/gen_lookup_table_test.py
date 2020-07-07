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

import os
import subprocess
import tempfile
import textwrap
import unittest

import makani
import numpy

SCRIPT_PATH = os.path.join(makani.HOME, 'avionics/motor/gen_lookup_table.py')


class GenLookupTableTest(unittest.TestCase):

  def testSuccess(self):
    with tempfile.NamedTemporaryFile() as omega_file:
      omega_file.write(textwrap.dedent("""
          t_step = 0.5
          t_end = 1.0
          def Cmd(t):
            return [1.0] * 2"""[1:]))
      omega_file.flush()

      popen = subprocess.Popen([SCRIPT_PATH, '--input_file', omega_file.name],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      stdout, _ = popen.communicate()

      self.assertEqual(0, popen.returncode)
      expected = numpy.array([[0.0, 1.0, 1.0],
                              [0.5, 1.0, 1.0],
                              [1.0, 1.0, 1.0]])
      with tempfile.NamedTemporaryFile() as f:
        f.write(stdout)
        f.flush()
        diff = expected - numpy.loadtxt(f.name)
      self.assertAlmostEqual(0.0, diff.max(), 14)

  def testSuccessBinary(self):
    with tempfile.NamedTemporaryFile() as omega_file:
      omega_file.write(textwrap.dedent("""
          t_step = 0.5
          t_end = 1.0
          def Cmd(t):
            return [1.0] * 2"""[1:]))
      omega_file.flush()

      with tempfile.NamedTemporaryFile() as stdout:
        popen = subprocess.Popen([SCRIPT_PATH, '--input_file', omega_file.name,
                                  '--binary'],
                                 stdout=stdout, stderr=subprocess.PIPE)
        popen.communicate()

        self.assertEqual(0, popen.returncode)
        expected = numpy.array([[0.0, 1.0, 1.0],
                                [0.5, 1.0, 1.0],
                                [1.0, 1.0, 1.0]])
        diff = expected - numpy.load(stdout.name)
      self.assertAlmostEqual(0.0, diff.max(), 14)

  def testNoFile(self):
    popen = subprocess.Popen([SCRIPT_PATH, '--input_file', 'junk'],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, _ = popen.communicate()
    self.assertNotEqual(0, popen.returncode)
    self.assertRegexpMatches(stdout,
                             '(?s).*--input_file must point to a file.*')

  def testInvalidFile(self):
    with tempfile.NamedTemporaryFile() as omega_file:
      omega_file.write('this is not valid Python')
      omega_file.flush()

      popen = subprocess.Popen([SCRIPT_PATH, '--input_file', omega_file.name],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      _, stderr = popen.communicate()

    self.assertNotEqual(0, popen.returncode)
    self.assertRegexpMatches(stderr, '(?s).*SyntaxError.*')

  def testMissingFunction(self):
    with tempfile.NamedTemporaryFile() as omega_file:
      omega_file.write(textwrap.dedent("""
          t_step = 0.5
          t_end = 1.0"""[1:]))
      omega_file.flush()

      popen = subprocess.Popen([SCRIPT_PATH, '--input_file', omega_file.name],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      _, stderr = popen.communicate()

    self.assertNotEqual(0, popen.returncode)
    self.assertRegexpMatches(stderr, r'(?s).*Missing function Cmd\(t\).*')

  def testCmdFails(self):
    with tempfile.NamedTemporaryFile() as omega_file:
      omega_file.write(textwrap.dedent("""
          t_step = 0.5
          t_end = 1.0
          def Cmd(t):
            raise ValueError('Uh-oh.')"""[1:]))
      omega_file.flush()

      popen = subprocess.Popen([SCRIPT_PATH, '--input_file', omega_file.name],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      _, stderr = popen.communicate()

    self.assertNotEqual(0, popen.returncode)
    self.assertRegexpMatches(stderr, r'(?s).*Cmd\(t\) failed.*ValueError.*')

  def testNoTimestep(self):
    with tempfile.NamedTemporaryFile() as omega_file:
      omega_file.write(textwrap.dedent("""
          t_end = 1.0
          def Cmd(t):
            return [1.0] * 2"""[1:]))
      omega_file.flush()

      popen = subprocess.Popen([SCRIPT_PATH, '--input_file', omega_file.name],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      _, stderr = popen.communicate()

    self.assertNotEqual(0, popen.returncode)
    self.assertRegexpMatches(stderr, '(?s).*t_step.*')

  def testNoEndTime(self):
    with tempfile.NamedTemporaryFile() as omega_file:
      omega_file.write(textwrap.dedent("""
          t_step = 0.1
          def Cmd(t):
            return [1.0] * 2"""[1:]))
      omega_file.flush()

      popen = subprocess.Popen([SCRIPT_PATH, '--input_file', omega_file.name],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      _, stderr = popen.communicate()

    self.assertNotEqual(0, popen.returncode)
    self.assertRegexpMatches(stderr, '(?s).*t_end.*')


if __name__ == '__main__':
  unittest.main()

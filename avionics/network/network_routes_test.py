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

"""Tests for the network_routes module."""

import os
import subprocess
import unittest

from makani.avionics.network import network_routes


# This test just checks that network_routes.py completes
# successfully with its various options.
class NetworkRoutesTest(unittest.TestCase):

  def setUp(self):
    # Set and check the script path.
    self.script_path = network_routes.__file__
    if self.script_path.endswith('.pyc'):
      self.script_path = self.script_path[:-1]
    if not os.path.isfile(self.script_path):
      raise RuntimeError('network_routes module not found at %s.' %
                         self.script_path)

    # Set and check the config path.
    self.config_path = os.path.join(os.path.dirname(self.script_path),
                                    'network.yaml')
    if not os.path.isfile(self.config_path):
      raise RuntimeError('No network config found at %s.' % self.config_path)

  def testPrintRoutes(self):
    popen = subprocess.Popen([self.script_path, '--print_routes',
                              '--network_file', self.config_path],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, _ = popen.communicate()

    self.assertEqual(popen.returncode, 0)
    self.assertRegexpMatches(stdout, '(?s).*Multicast Routes.*')

  def testWriteDotFiles(self):
    return_code = subprocess.call([self.script_path, '--write_dot_files',
                                   '--network_file', self.config_path])
    self.assertEqual(return_code, 0)
    self.assertTrue(os.path.isfile('reduced.dot'))
    self.assertTrue(os.path.isfile('full.dot'))


if __name__ == '__main__':
  unittest.main()

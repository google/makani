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

"""Tests for the network_bandwidth module."""

import json
import os
import subprocess
import unittest

from makani.avionics.network import network_bandwidth


# Verify that network_bandwidth.py completes successfully and that link
# bandwidth limits are satisfied.
class NetworkBandwidthTest(unittest.TestCase):

  def setUp(self):
    # Set and check the script path.
    self.script_path = network_bandwidth.__file__
    if self.script_path.endswith('.pyc'):
      self.script_path = self.script_path[:-1]
    if not os.path.isfile(self.script_path):
      raise RuntimeError('network_bandwidth module not found at %s.' %
                         self.script_path)

    # Set and check the config path.
    self.config_path = os.path.join(os.path.dirname(self.script_path),
                                    'network.yaml')
    if not os.path.isfile(self.config_path):
      raise RuntimeError('No network config found at %s.' % self.config_path)

  def testPrintBandwidthStats(self):
    popen = subprocess.Popen([self.script_path, '--print_bandwidth_stats',
                              '--network_file', self.config_path],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = popen.communicate()
    self.assertEqual(popen.returncode, 0, msg=stderr)
    self.assertRegexpMatches(stdout, '(?s).*Bandwidth Statistics.*')

  def testValidateBandwidthStats(self):
    popen = subprocess.Popen([self.script_path, '--validate_bandwidth_stats',
                              '--network_file', self.config_path,
                              '--ignore_node', 'controller_b',
                              '--ignore_node', 'controller_c',
                              '--ignore_location', 'test_fixture'],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    _, stderr = popen.communicate()
    self.assertEqual(popen.returncode, 0, msg=stderr)

  def testExportJson(self):
    popen = subprocess.Popen([self.script_path, '--export_json',
                              '--network_file', self.config_path],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = popen.communicate()
    self.assertEqual(popen.returncode, 0, msg=stderr)
    # Verify that the output is valid json.
    json.loads(stdout)


if __name__ == '__main__':
  unittest.main()

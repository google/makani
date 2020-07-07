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

"""Tests for the network_util module that require the full network.yaml."""

import os
import unittest

import makani
from makani.avionics.network import network_config
from makani.avionics.network import network_util


class NetworkYamlTest(unittest.TestCase):

  def setUp(self):
    filename = os.path.join(makani.HOME, 'avionics/network/network.yaml')
    self._network_config = network_config.NetworkConfig(filename)

  def testCheckForLoopRoutes(self):
    config = self._network_config
    message_types = config.all_messages
    path_finder = network_util.PathFinder(config.GetSwitches(), message_types)
    for message in message_types:
      graph = network_util.MessageGraph(path_finder, message)
      visitor = network_util.MessageGraphVisitor()
      graph.VisitSenders(visitor, message.all_senders)

  def testCheckForUnintendedRecipients(self):
    config = self._network_config
    message_types = config.all_messages
    path_finder = network_util.PathFinder(config.GetSwitches(), message_types)
    for message in message_types:
      graph = network_util.MessageGraph(path_finder, message)
      network_util.CheckForUnintendedRecipients(graph)


if __name__ == '__main__':
  unittest.main()

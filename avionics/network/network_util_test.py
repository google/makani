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

"""Tests for the network_util module."""

import os
import unittest

import makani
from makani.avionics.network import network_config
from makani.avionics.network import network_util


class NetworkUtilTest(unittest.TestCase):

  def setUp(self):
    filename = os.path.join(makani.HOME, 'avionics/network/test.yaml')
    self._network_config = network_config.NetworkConfig(filename)

  def _GetSegmentStats(self, message_name):
    switches = self._network_config.GetSwitches()
    message_types = self._network_config.all_messages
    path_finder = network_util.PathFinder(switches, message_types)
    message = self._network_config.GetAioMessageType(message_name)
    graph = network_util.MessageGraph(path_finder, message)
    return network_util.GetSegmentStats(graph, message.all_senders)

  def _GetPeak(self, stats, in_tag, out_tag):
    segment = (in_tag, out_tag)
    return sum([s['peak'] for s in stats[segment].values()])

  def testFlightComputerSensor(self):
    stats = self._GetSegmentStats('FlightComputerSensor')

    # Each Controller may receive a peak of 5 FlightComputerSensor messages.
    # - FcA -> ControllerA on local switch (1 copy).
    # - FcB -> ControllerA over A and B networks (2 copies).
    # - FcC -> ControllerA over A and B networks (2 copies).
    self.assertEqual(5, self._GetPeak(stats, 'switches.fc_a.4',
                                      'aio_nodes.controller_a'))

    # RecorderWing may receive a peak of 6 FlightComputerSensor messages.
    # - FcA -> RecorderWing over A and B networks (2 copies).
    # - FcB -> RecorderWing over A and B networks (2 copies).
    # - FcC -> RecorderWing over A and B networks (2 copies).
    self.assertEqual(6, self._GetPeak(stats, 'switches.recorder_wing.4',
                                      'aio_nodes.recorder_wing'))

    # RecorderGs may receive a peak of 12 FlightComputerSensor messages.
    # - FcA -> RecorderGs over A and B networks times 2 trunks (4 copies).
    # - FcB -> RecorderGs over A and B networks times 2 trunks (4 copies).
    # - FcC -> RecorderGs over A and B networks times 2 trunks (4 copies).
    self.assertEqual(12, self._GetPeak(stats, 'switches.recorder_gs.4',
                                       'aio_nodes.recorder_gs'))

  def testControllerCommand(self):
    stats = self._GetSegmentStats('ControllerCommand')

    # Each Motor may receive a peak of 10 ControllerCommand messages.
    # - Host -> MotorPbi over A and B networks times 2 trunks (4 copies).
    # - ControllerA -> MotorPbi over A and B networks (2 copies).
    # - ControllerB -> MotorPbi over A and B networks (2 copies).
    # - ControllerC -> MotorPbi over A and B networks (2 copies).
    self.assertEqual(10, self._GetPeak(stats, 'switches.motor_pbi.5',
                                       'aio_nodes.motor_pbi'))

    # Host may receive a peak of 12 ControllerCommand messages.
    # - ControllerA -> Host over A and B networks times 2 trunks (4 copies).
    # - ControllerB -> Host over A and B networks times 2 trunks (4 copies).
    # - ControllerC -> Host over A and B networks times 2 trunks (4 copies).
    self.assertEqual(12, self._GetPeak(stats, 'switches.host.5',
                                       'aio_nodes.host'))

  def testMotorStacking(self):
    stats = self._GetSegmentStats('MotorStacking')

    # Each Motor may receive a peak of 6 MotorStacking messages.
    # - MotorPbo -> MotorPbi over A and B networks (2 copies).
    # - MotorSbi -> MotorPbi over A and B networks (2 copies).
    # - MotorSbo -> MotorPbi over A and B networks (2 copies).
    self.assertEqual(6, self._GetPeak(stats, 'switches.motor_pbi.5',
                                      'aio_nodes.motor_pbi'))

  def testGetNodeBandwidthStatistics(self):
    switches = self._network_config.GetSwitches()
    message_types = self._network_config.all_messages
    path_finder = network_util.PathFinder(switches, message_types)
    node_stats = network_util.GetNodeBandwidthStatistics(path_finder,
                                                         message_types)
    for aio_node in ['aio_nodes.controller_a', 'aio_nodes.controller_b',
                     'aio_nodes.controller_c']:
      self.assertEqual(node_stats[aio_node].send, {'ControllerCommand': 100})
      self.assertEqual(node_stats[aio_node].receive, {
          'FlightComputerSensor': 50,  # 2 networks for all but the local Fc.
          'MotorStatus': 800,  # 4 motors * 2 networks.
      })
      self.assertEqual(node_stats[aio_node].multicast_packet_rate['tx'], 100)
      self.assertEqual(node_stats[aio_node].multicast_packet_rate['rx'], 850)

    sensor_messages_sent = 0
    for aio_node in ['aio_nodes.fc_a', 'aio_nodes.fc_b', 'aio_nodes.fc_c']:
      self.assertEqual(node_stats[aio_node].send, {'FlightComputerSensor': 10})
      self.assertEqual(node_stats[aio_node].receive, {})
      self.assertEqual(node_stats[aio_node].multicast_packet_rate['tx'], 10)
      self.assertEqual(node_stats[aio_node].multicast_packet_rate['rx'], 0)
      sensor_messages_sent += node_stats[aio_node].send['FlightComputerSensor']

    for aio_node in ['aio_nodes.motor_pbo', 'aio_nodes.motor_sbi',
                     'aio_nodes.motor_pbi', 'aio_nodes.motor_sbo']:
      self.assertEqual(node_stats[aio_node].send,
                       {'MotorStatus': 100, 'MotorStacking': 1000})
      # 2 networks * each other motor, no loopback packets.
      self.assertEqual(node_stats[aio_node].receive, {
          'ControllerCommand': 1000,
          'MotorStacking': 6000})
      self.assertEqual(node_stats[aio_node].multicast_packet_rate['tx'], 1100)
      self.assertEqual(node_stats[aio_node].multicast_packet_rate['rx'], 7000)

    # Multiply by two for the two networks, the by 2 for the two paths from the
    # wing to the ground.
    self.assertEqual(node_stats['aio_nodes.host'].receive[
        'FlightComputerSensor'], sensor_messages_sent * 2 * 2)


if __name__ == '__main__':
  unittest.main()

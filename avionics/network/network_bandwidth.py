#!/usr/bin/python
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


"""Generates bandwidth estimates from network.yaml and packet definitions."""

import collections
import ctypes
import json
import sys

import gflags

from makani.avionics.common import pack_aio_header as aio_header
from makani.avionics.common import pack_avionics_messages
from makani.avionics.common import pack_eop_messages
from makani.avionics.common import pack_winch_messages
from makani.avionics.network import network_config
from makani.avionics.network import network_util
from makani.avionics.network import node_locations
from makani.control import pack_control_telemetry as control_telemetry
from makani.control import pack_ground_telemetry as ground_telemetry

gflags.DEFINE_bool('print_bandwidth_stats', False,
                   'Indicates whether to print bandwidth stats.')
gflags.DEFINE_bool('validate_bandwidth_stats', False,
                   'Indicates whether or not to validate bandwidth stats '
                   'against limits.')
gflags.DEFINE_bool('export_json', False,
                   'Indicates whether or not to print a JSON file with '
                   'per (message, link) stats.')
gflags.DEFINE_multistring('ignore_node', [],
                          'Ignore traffic from specified node when calculating '
                          'bandwidth.')
gflags.DEFINE_multistring('ignore_location', [],
                          'Ignore traffic from specified locations when '
                          'calculating bandwidth.')
gflags.DEFINE_string('network_file', None,
                     'Full path to the yaml file that describes the network.')
gflags.MarkFlagAsRequired('network_file')

FLAGS = gflags.FLAGS


class BandwidthException(Exception):
  pass


Stat = collections.namedtuple(  # pylint: disable=invalid-name
    'Stat', ['bytes_per_sec', 'packets_per_sec',
             'bytes_peak', 'packets_peak', 'utilization'])


def _GetBandwidth(switches, link):
  """Get the bandwidth for a switch port in bits per second."""
  # Handle internal switch port to switch port bandwidth limitations.
  if network_util.IsSwitchLink(*link):
    return 1e8
  # Check egress bandwidth for most hops.  In the case of an AIO node talking
  # to a switch, check ingress bandwidth, which should be equal in all cases.
  if network_util.IsAioNode(link[0]):
    switch_name, port = network_util.TagToPort(link[1])
  else:
    switch_name, port = network_util.TagToPort(link[0])
  switch = switches[switch_name]
  if 'config' in switch and 'bandwidth' in switch['config']:
    if port in switch['config']['bandwidth']:
      return float(switch['config']['bandwidth'][port])
    else:
      return float(switch['config']['bandwidth']['default'])
  else:
    return 1e8


def _GetMessageSize(message):
  """Returns the message size in bytes."""
  # TODO: Clean-up nonstandard messages.
  if message.name in ['ControlTelemetry', 'ControlSlowTelemetry']:
    header_size = ctypes.sizeof(aio_header.AioHeader())
    payload_size = ctypes.sizeof(getattr(control_telemetry, message.name)())
  elif message.name == 'ControlDebug':
    header_size = ctypes.sizeof(aio_header.AioHeader())
    payload_size = ctypes.sizeof(getattr(control_telemetry,
                                         message.name + 'Message')())
  elif message.name == 'GroundTelemetry':
    header_size = ctypes.sizeof(aio_header.AioHeader())
    payload_size = ctypes.sizeof(getattr(ground_telemetry, message.name)())
  elif message.eop_message:
    header_size = ctypes.sizeof(pack_eop_messages.EopHeader())
    payload_size = (
        ctypes.sizeof(
            getattr(pack_eop_messages, message.name + 'Message')()))
  elif message.winch_message:
    header_size = 0
    payload_size = (
        ctypes.sizeof(
            getattr(pack_winch_messages, message.name + 'Message')()))
  elif message.aio_message:
    header_size = ctypes.sizeof(aio_header.AioHeader())
    payload_size = (
        ctypes.sizeof(
            getattr(pack_avionics_messages, message.name + 'Message')()))
  else:
    assert ValueError('Unexpected message type.')

  # TODO: This formula assumes packed messages types and needs
  # to be updated.
  size = (
      14 +  # Ethernet header.
      20 +  # IP header.
      8 +   # UDP header.
      header_size +
      payload_size +  # Actual message.
      4)    # Ethernet FCS.
  return size


def GetFullBandwidthStatistics(config):
  """Gather the bandwidth statistics for each link, message, and source.

  Args:
    config: A NetworkConfig.
  Returns:
    stats: A dictionary of link statistics, indexed by a
        (sender, message, source, sink) tuple denoting sender and message type.
  """
  Key = collections.namedtuple(  # pylint: disable=invalid-name
      'Key', ['sender', 'message', 'source', 'sink'])

  output = dict()
  message_types = config.all_messages
  switches = config.GetSwitches()
  locations = node_locations.GetNodeLocations(config)
  path_finder = network_util.PathFinder(switches, message_types)

  ignore_senders = FLAGS.ignore_node
  for location, senders in locations.iteritems():
    if location in FLAGS.ignore_location:
      ignore_senders += [s.snake_name for s in senders]

  for message in message_types:
    if message.frequency_hz <= 0:
      continue

    senders = [s for s in message.all_senders
               if s.snake_name not in ignore_senders]
    graph = network_util.MessageGraph(path_finder, message)
    message_stats = network_util.GetSegmentStats(graph, senders)
    message_size = _GetMessageSize(message)

    for segment, senders in message_stats.iteritems():
      for sender, stats in senders.iteritems():
        packets_per_sec = stats['packets_per_sec']
        bytes_per_sec = packets_per_sec * message_size
        packets_peak = stats['peak']
        bytes_peak = packets_peak * message_size

        key = Key(
            sender=sender.snake_name,
            message=message.name,
            source=segment[0],
            sink=segment[1])
        output[key] = Stat(
            bytes_per_sec=bytes_per_sec,
            packets_per_sec=packets_per_sec,
            bytes_peak=bytes_peak,
            packets_peak=packets_peak,
            utilization=bytes_per_sec * 8 / _GetBandwidth(switches, segment))

  return output


def _GetAggregateStatistics(config, get_dict_key_function):
  """Gather the bandwidth statistics for some key function.

  Args:
    config: A NetworkConfig.
    get_dict_key_function: Take the Key containing sender, link, and message,
      and return an aggregation key for the dictionaryto be generated.
  Returns:
    aggregate_stats: A dictionary of link statistics, indexed by keys created
      from the provided function.
  """
  aggregate_stats = collections.defaultdict(lambda: Stat(0, 0, 0, 0, 0.0))
  switches = config.GetSwitches()

  for key, stat in GetFullBandwidthStatistics(config).iteritems():
    link = (key.source, key.sink)
    aggregate_key = get_dict_key_function(key)
    s = aggregate_stats[aggregate_key]
    bytes_per_sec = s.bytes_per_sec + stat.bytes_per_sec
    aggregate_stats[aggregate_key] = Stat(
        s.bytes_per_sec + stat.bytes_per_sec,
        s.packets_per_sec + stat.packets_per_sec,
        s.bytes_peak + stat.bytes_peak,
        s.packets_peak + stat.packets_peak,
        bytes_per_sec * 8 / _GetBandwidth(switches, link))
  return aggregate_stats


def GetLinkBandwidthStatistics(config):
  return _GetAggregateStatistics(config,
                                 lambda x: (x.source, x.sink))


def _PrintBandwidthStatistics(config):
  """Prints the bandwidth statistics for each link."""

  print ''
  print 'Bandwidth Statistics'
  print '===================='
  link_stats = GetLinkBandwidthStatistics(config)

  for link, stat in sorted(link_stats.iteritems()):
    print ('  (%s -> %s): %.2f Mbps (%.2f%%), %d packets/s, '
           '%d bytes peak, %d packets peak.') % (
               link[0], link[1], stat.bytes_per_sec * 8 / 1e6,
               stat.utilization * 100, stat.packets_per_sec,
               stat.bytes_peak, stat.packets_peak)


def _ValidateBandwidthStatistics(config):
  """Validate the bandwidth statistics for each link."""
  link_stats = GetLinkBandwidthStatistics(config)
  failed = False
  for link, stat in sorted(link_stats.iteritems()):
    if stat.utilization >= 1.0:
      sys.stderr.write(('ERROR: %s -> %s exceeds bandwidth limits: '
                        '%.2f Mbps (%.2f%%)\n') % (
                            link[0], link[1], stat.bytes_per_sec * 8 / 1e6,
                            stat.utilization * 100))
      failed = True
  if failed:
    raise BandwidthException('Link exceeds bandwidth limits!')


def _ExportBandwidthStatistics(config):
  """Print a JSON-formatted list of statistics.

  Stats take the form of dicts with keys:
     segment_source, segment_sink, message, bytes_per_sec, packets_per_sec,
     bytes_peak, packets_peak, segment_utilization.

  Args:
    config: A NetworkConfig.
  """
  LinkMessageKey = collections.namedtuple(  # pylint: disable=invalid-name
      'LinkMessageKey', ['message', 'source', 'sink'])
  link_message_stats = _GetAggregateStatistics(
      config, lambda x: LinkMessageKey(x.message, x.source, x.sink))

  stats = []
  for key, stat in link_message_stats.iteritems():
    # The network dashboard does not support internal switch links.
    if network_util.IsSwitchLink(key.source, key.sink):
      continue

    stats.append({'source': key.source, 'sink': key.sink,
                  'message': key.message,
                  'bytes_per_sec': stat.bytes_per_sec,
                  'packets_per_sec': stat.packets_per_sec,
                  'bytes_peak': stat.bytes_peak,
                  'packets_peak': stat.packets_peak,
                  'segment_utilization': stat.utilization})

  print json.dumps(stats, sort_keys=True, indent=2)


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    sys.stderr.write('\nError: %s\n\nUsage: %s ARGS\n%s\n'
                     % (e, argv[0], FLAGS))
    sys.exit(1)

  config = network_config.NetworkConfig(FLAGS.network_file)

  if FLAGS.print_bandwidth_stats:
    _PrintBandwidthStatistics(config)
  if FLAGS.validate_bandwidth_stats:
    _ValidateBandwidthStatistics(config)
  if FLAGS.export_json:
    _ExportBandwidthStatistics(config)

if __name__ == '__main__':
  main(sys.argv)

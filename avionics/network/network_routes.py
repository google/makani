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


"""Set of various AIO network tools."""
# TODO: Rename this file to something more accurate.

import re
import sys

import gflags
from makani.avionics.network import network_config
from makani.avionics.network import network_util

gflags.DEFINE_bool('print_routes', False,
                   'Indicates whether to print multicast routes.')

gflags.DEFINE_string('network_file', None,
                     'Full path to the yaml file that describes the network.')
gflags.MarkFlagAsRequired('network_file')

gflags.DEFINE_bool('write_dot_files', False,
                   'Indicates whether to write a dot network description file.')

FLAGS = gflags.FLAGS


def _PrintMulticastRoutes(forwarding_maps):
  """Prints the multicast routing tables for each switch."""

  print ''
  print 'Multicast Routes'
  print '================'

  for switch, forwarding_map in sorted(forwarding_maps.iteritems()):
    print '  %s multicast routes:' % switch
    for message_type, out_ports in sorted(forwarding_map.iteritems()):
      print '    [%s] = 0x%X' % (message_type, out_ports)


def _WriteDotFiles(path_finder):
  """Writes a pair of dot files showing the network topology.

  full.dot is the full network, with each port listed as a separate node.
  reduced.dot is a simplified view with a node for each switch or aio_node.

  Args:
    path_finder: A network_util.PathFinder over the network.
  """
  links = path_finder.GetAllConnections()
  with open('full.dot', 'w') as f:
    f.write('graph network {\n')
    for link in links:
      source = link[0].replace('.', '_')
      dest = link[1].replace('.', '_')
      f.write('  %s -- %s;\n' % (source, dest))
    f.write('}\n')
    print 'Wrote full.dot.'
  with open('reduced.dot', 'w') as f:
    f.write('graph network {\n')
    for link in links:
      re0 = r'switches.([a-z_0-9]+).[0-9]+'
      re1 = r'aio_nodes\.([a-z_0-9]+)'
      source = re.sub(re0, r'switch_\1', link[0])
      source = re.sub(re1, r'\1', source)
      dest = re.sub(re0, r'switch_\1', link[1])
      dest = re.sub(re1, r'\1', dest)
      if source != dest:
        f.write('  %s -- %s;\n' % (source, dest))
    f.write('}\n')
    print 'Wrote reduced.dot.'
    print('Now try e.g. \'fdp ./reduced.dot -Tpng:gd > reduced.png && '
          'eog reduced.png\'')


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    sys.stderr.write('\nError: %s\n\nUsage: %s ARGS\n%s\n'
                     % (e, argv[0], FLAGS))
    sys.exit(1)

  config = network_config.NetworkConfig(FLAGS.network_file)
  message_types = config.all_messages

  path_finder = network_util.PathFinder(config.GetSwitches(), message_types)

  if FLAGS.print_routes:
    forwarding_maps = network_util.MakeForwardingMaps(
        message_types, path_finder)

  if FLAGS.print_routes:
    _PrintMulticastRoutes(forwarding_maps)

  if FLAGS.write_dot_files:
    _WriteDotFiles(path_finder)

if __name__ == '__main__':
  main(sys.argv)

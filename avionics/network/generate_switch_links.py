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

"""Generate the switch_links.c file."""

import os
import re
import sys
import textwrap

from makani.avionics.network import network_config
from makani.avionics.network import network_util
from makani.lib.python import c_helpers
from makani.lib.python import string_util


def _GetNodeLinks(config):
  """Get dict mapping node index to local_links and remote_links lists.

  A local_link connects two AIO nodes on the same switch.
  A remote_link connects two switches.

  Args:
    config: A NetworkConfig.

  Returns:
    A dict mapping node index to dict containing 'local_links' and
    'remote_links' lists, where each element in the list is a tuple
    of (local_port, remote_name, remote_port).
  """
  def _GetLinkName(switch, port):
    if 'port_names' not in switch:
      return None
    if port not in switch['port_names']:
      return None
    return string_util.SnakeToCamel(switch['port_names'][port])

  switches = config.GetSwitches()
  node_links = {}
  for switch in switches.values():
    local_nodes = []
    remote_links = []
    ports = switch['ports']
    for local_port, remote_tag in ports.iteritems():
      if network_util.IsAioNode(remote_tag):
        local_nodes.append((local_port, network_util.TagToAioNode(remote_tag)))
      elif network_util.IsPort(remote_tag):
        remote_switch, remote_port = network_util.TagToPort(remote_tag)
        remote_aio = ('config' in switches[remote_switch] and
                      'tms570' in switches[remote_switch]['config'])
        remote_links.append((local_port, remote_switch, remote_port,
                             remote_aio, _GetLinkName(switch, local_port)))

    for local_node in local_nodes:
      # Nodes with a TMS570 and Q7 have a local link connecting each other.
      local_links = []
      for remote_node in local_nodes:
        # Remote port not valid.
        local_links.append((remote_node[0], remote_node[1], -1, False,
                            _GetLinkName(switch, remote_node[0])))

      aio_node = config.GetAioNode(local_node[1])
      node_links[aio_node] = {
          'local_links': local_links,
          'remote_links': remote_links
      }
  return node_links


def _GenerateSource(header_file, config):
  """Generate the source file as a string.

  Args:
    header_file: The relative path to the header file.
    config: A NetworkConfig.

  Returns:
    The source file as a string.
  """

  def _LinkToString(link):
    return textwrap.dedent("""\
        {{
          .local_port = {local_port},
          .remote_port = {remote_port},
          .remote_name = "{remote_name}",
          .remote_aio = {remote_aio},
          .link_name = {link_name},
        }}""").format(local_port=link[0],
                      remote_port=link[2],
                      remote_name=string_util.SnakeToCamel(link[1]),
                      remote_aio=str(link[3]).lower(),
                      link_name=('"%s"' % link[4]) if link[4] else 'NULL')

  def _LinksToStrings(name, links):
    if links:
      strings = [textwrap.dedent("""\
          static const SwitchLinkInfo {name}[] = {{""".format(name=name))]
      entries = ', '.join([_LinkToString(link) for link in links])
      strings += [c_helpers.Indent(entries)]
      strings += ['};\n']
      return strings, name
    return [], 'NULL'

  strings = [textwrap.dedent("""\
      #include "{header_file}"

      #include <assert.h>
      #include <stdbool.h>
      #include <stddef.h>

      #include "avionics/network/aio_node.h"
      """.format(header_file=header_file))]

  node_links = _GetNodeLinks(config)
  map_strings = []
  for node, links in node_links.iteritems():
    local_links, local_address = _LinksToStrings(
        'kLocalLinks' + node.camel_name, links['local_links'])
    strings.extend(local_links)
    remote_links, remote_address = _LinksToStrings(
        'kRemoteLinks' + node.camel_name, links['remote_links'])
    strings.extend(remote_links)

    map_strings.append('  [{enum_name}] = {{{num_local}, {local_address}, '
                       '{num_remote}, {remote_address}}},'.format(
                           enum_name=node.enum_name,
                           num_local=len(links['local_links']),
                           local_address=local_address,
                           num_remote=len(links['remote_links']),
                           remote_address=remote_address))

  strings.append('static const SwitchLinkMap kSwitchLinkMap[kNumAioNodes] = {')
  strings.extend(map_strings)
  strings.append('};')

  strings.append(textwrap.dedent("""
      const SwitchLinkMap *GetSwitchLinkMap(AioNode aio_node) {
        if (0 <= aio_node && aio_node < kNumAioNodes) {
          return &kSwitchLinkMap[aio_node];
        }
        assert(false);
        return NULL;
      }
      """))
  return '\n'.join(strings)


def _GenerateHeader(header_file):
  """Generate the header file as a string.

  Args:
    header_file: The relative path to the header file.

  Returns:
    The source file as a string.
  """
  guard = re.sub('[/.]', '_', header_file).upper() + '_'
  strings = [textwrap.dedent("""\
      #ifndef {guard}
      #define {guard}

      #include <stdint.h>

      #include "avionics/network/aio_node.h"
      """.format(guard=guard))]

  strings.append(textwrap.dedent("""\
      typedef struct {
        int32_t local_port;
        int32_t remote_port;
        const char *remote_name;
        bool remote_aio;
        const char *link_name;
      } SwitchLinkInfo;

      typedef struct {
        int32_t num_local_links;
        const SwitchLinkInfo *local_links;
        int32_t num_remote_links;
        const SwitchLinkInfo *remote_links;
      } SwitchLinkMap;

      const SwitchLinkMap *GetSwitchLinkMap(AioNode aio_node);
      """))

  strings.append('#endif  // {guard}\n'.format(guard=guard))
  return '\n'.join(strings)


def main(argv):
  flags, argv = network_config.ParseGenerationFlags(argv)
  config = network_config.NetworkConfig(flags.network_file)
  source_file = os.path.join(flags.output_dir, 'switch_links.c')
  header_file = os.path.join(flags.output_dir, 'switch_links.h')
  rel_header_file = os.path.relpath(header_file, flags.autogen_root)

  with open(source_file, 'w') as f:
    f.write(_GenerateSource(rel_header_file, config))
  with open(header_file, 'w') as f:
    f.write(_GenerateHeader(rel_header_file))


if __name__ == '__main__':
  main(sys.argv)

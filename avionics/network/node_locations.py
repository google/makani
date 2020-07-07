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

"""Identity node locations given the core switch location."""

from makani.avionics.network import network_util


def GetNodeLocations(network_config):
  """Identify node location given core switch location.

  Args:
    network_config: A NetworkConfig.

  Returns:
    A dict of sets of AIO node names, where the key name indicates the
    node location.

  Raises:
    ValueError: If a node appears in more than one location.
  """
  nodes = {
      'wing': set(),
      'ground_station': set(),
      'remote_command': set(),
      'test_fixture': set()
  }
  core_switch_locations = {
      'cs_a': 'wing',
      'cs_b': 'wing',
      'cs_dyno_a': 'test_fixture',
      'cs_dyno_b': 'test_fixture',
      'cs_gs_a': 'ground_station',
      'cs_gs_b': 'ground_station',
      'joystick_radio_ground': 'remote_command',
      'remote_command': 'remote_command',
  }

  path_finder = network_util.PathFinder(network_config.GetSwitches(),
                                        network_config.aio_messages)
  destinations = ['aio_nodes.operator']
  for aio_node in network_config.aio_nodes:
    source = network_util.AioNodeToTag(aio_node)
    try:
      switches = path_finder.GetFirstSwitch(None, source, destinations,
                                            core_switch_locations.keys())
    except KeyError:
      switches = []
    for switch in switches:
      nodes[core_switch_locations[switch]].add(aio_node)

  # Remove test fixtures.
  # TODO: Eliminate after we support multiple network configurations.
  test_fixtures = [network_config.GetAioNode(n) for n in [
      'simulated_joystick', 'simulator', 'torque_cell', 'visualizer']]
  for loc in nodes:
    if loc != 'test_fixture':
      nodes[loc] -= set(test_fixtures)
  nodes['test_fixture'] |= set(test_fixtures)

  # Check for potential duplicate listings.
  for loc_a in nodes:
    for loc_b in nodes:
      intersection = nodes[loc_a] & nodes[loc_b]
      if loc_a != loc_b and intersection:
        names = [node.snake_name for node in intersection]
        raise ValueError(
            'Nodes appear both in {loc_a} and {loc_b}: {nodes}'.format(
                loc_a=loc_a, loc_b=loc_b, nodes=names))
  return nodes

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

"""Wrapper for network.yaml."""

import collections
import os
import sys

import gflags
import makani
from makani.lib.python import string_util
import yaml


class NetworkConfigException(Exception):
  pass


def _GetPortNumber(port, port_names):
  if isinstance(port, int):
    return port
  else:
    for p, name in port_names.iteritems():
      if name == port:
        return p
  raise NetworkConfigException('Invalid port specification: {}.'.format(port))


def _ConvertPortNumbersInList(names_or_numbers, port_names):
  return [_GetPortNumber(n, port_names) for n in names_or_numbers]


def _RangeParse(node_select):
  nodes = set()
  for item in node_select:
    if isinstance(item, list):
      nodes.update(range(*item))
    else:
      nodes.add(item)
  return nodes


def _ValidateMessageTypes(message_types, supplied_types):
  for supplied in supplied_types:
    for m in message_types:
      if supplied == m['name']:
        break
    else:
      raise NetworkConfigException('Message type %s is invalid.' % supplied)


# Flatten lists of lists, and make raw elements into lists of one item; yaml
# doesn't support merging lists automatically.
def _FlattenList(l):
  """Flattens lists of lists into plain lists, recursively.

  For example, [[4, 5, 6], [7, 8], []] will become [4, 5, 6, 7, 8].

  Non-list elements will get wrapped in a list, so 'foo' becomes ['foo'].
  None becomes [].

  Args:
    l: a list, or not.

  Returns:
    A flattened list.
  """
  ret = []
  if l is None:
    pass
  elif not isinstance(l, list):
    ret = [l]
  else:
    for item in l:
      ret.extend(_FlattenList(item))

  return ret


def _FlattenNodeList(l):
  """Replaces a list of list of nodes with a list of node names."""
  return [n['name'] for n in _FlattenList(l)]


class AioNode(object):
  """Wrapper class for AIO node definition in network.yaml."""

  def __init__(self, config, kind, instance):
    self._config = config
    self._kind = kind
    self._instance = instance
    self._enum_value = config._aio_enum_value  # pylint: disable=protected-access
    config._aio_enum_value += 1  # pylint: disable=protected-access
    for i, node in enumerate(kind['instances']):
      if node['name'] == instance['name']:
        self._index = i
        break
    else:
      raise NetworkConfigException('Node name not found under this label name.')
    if not string_util.IsSnakeCase(self.snake_name):
      raise NetworkConfigException('AioNode name is not snake case: %s'
                                   % self.snake_name)

  def __hash__(self):
    return self.snake_name.__hash__()

  def __eq__(self, other):
    return self.snake_name == other.snake_name

  def __cmp__(self, other):
    return cmp(self.enum_value, other.enum_value)

  @property
  def label_name(self):
    return self._kind['label_name']

  @property
  def snake_name(self):
    return self._instance['name']

  @property
  def camel_name(self):
    return string_util.SnakeToCamel(self.snake_name)

  @property
  def enum_name(self):
    return 'kAioNode' + self.camel_name

  @property
  def application_path(self):
    return self._kind['application']

  @property
  def bootloader_path(self):
    return self._kind.get('bootloader',
                          self._config._yaml['default_bootloader'])  # pylint: disable=protected-access

  @property
  def bootloader_application_path(self):
    return self._kind.get(
        'bootloader_application',
        self._config._yaml['default_bootloader_application'])  # pylint: disable=protected-access

  @property
  def label_value(self):
    return self._index

  @property
  def ip_octet(self):
    return self._instance['ip']

  @property
  def ip(self):
    return '192.168.1.%d' % self.ip_octet

  @property
  def enum_value(self):
    return self._enum_value

  @property
  def tms570_node(self):
    return self.snake_name in self._config._yaml['tms570s']  # pylint: disable=protected-access

  @property
  def q7_node(self):
    return self.snake_name in self._config._yaml['q7s']  # pylint: disable=protected-access


class MessageRoute(object):
  """Wrapper class for message route definitions in network.yaml."""

  def __init__(self, config, route):
    self._config = config
    self._route = route

    required_fields = ['senders', 'receivers']
    if not all(x in route for x in required_fields):
      raise NetworkConfigException(
          'MessageRoute missing one or more required fields: %s' %
          ', '.join(required_fields))

  @property
  def senders(self):
    return frozenset(self._config.GetAioNode(name)
                     for name in self._route['senders'])

  @property
  def receivers(self):
    return frozenset(self._config.GetAioNode(name)
                     for name in self._route['receivers'])


class MessageType(object):
  """Wrapper class for message type definition in network.yaml."""

  def __init__(self, config, message, type_name, enum_value):
    self._config = config
    self._message = message
    self._type_name = type_name.capitalize()
    self._enum_value = enum_value

    required_fields = ['name', 'freq', 'routes']
    if not all(x in self._message for x in required_fields):
      raise NetworkConfigException(
          'Message %s missing one or more required fields: %s' %
          (self._message.get('name', ''), ', '.join(required_fields)))
    if not string_util.IsCamelCase(self.name):
      raise NetworkConfigException('MessageType name is not camel case: %s'
                                   % self.camel_name)

    self._routes = [MessageRoute(config, r) for r in message['routes']]

  def __hash__(self):
    return self.name.__hash__()

  def __eq__(self, other):
    return self.name == other.name

  def __cmp__(self, other):
    type_name_compare = cmp(self.type_name, other.type_name)
    if type_name_compare == 0:
      return cmp(self.enum_value, other.enum_value)
    return type_name_compare

  @property
  def name(self):
    return self._message['name']

  @property
  def snake_name(self):
    return string_util.CamelToSnake(self.name)

  @property
  def type_name(self):
    return self._type_name

  @property
  def enum_prefix(self):
    # TODO: Rename MessageType to AioMessageType.
    if self.type_name == 'Aio':
      return 'MessageType'
    return '%sMessageType' % self.type_name

  @property
  def enum_name(self):
    return 'k%s%s' % (self.enum_prefix, self.name)

  @property
  def enum_value(self):
    return self._enum_value

  @property
  def ip(self):
    if self.type_name == 'Aio':
      return '239.0.0.%d' % self.enum_value
    elif self.type_name == 'Eop':
      return '239.0.2.%d' % self.enum_value
    elif self.type_name == 'Winch':
      return '239.0.1.%d' % self.enum_value
    raise ValueError('Unknown message type name: ' + self.type_name)

  @property
  def frequency_hz(self):
    return self._message['freq']

  @property
  def all_senders(self):
    return frozenset(name for route in self._routes for name in route.senders)

  @property
  def all_receivers(self):
    return frozenset(name for route in self._routes for name in route.receivers)

  @property
  def routes(self):
    return self._routes

  @property
  def inhibit_routing(self):
    return self._message.get('inhibit_routing', False)

  @property
  def inhibit_cvt(self):
    return self._message.get('inhibit_cvt', False)

  @property
  def aio_message(self):
    return self.type_name == 'Aio'

  @property
  def eop_message(self):
    return self.type_name == 'Eop'

  @property
  def winch_message(self):
    return self.type_name == 'Winch'


class NetworkConfig(object):
  """Wrapper for the network.yaml file.

  Provides an interface to access information about AioNodes and MessageTypes.
  """

  def _PreprocessMessageTypes(self, key, y):
    """Preprocesses a message list.

    Args:
      key: YAML file key for message list (e.g., aio_messages).
      y: Loaded YAML file.

    Returns:
      The processed message list.
    """
    if key in y:
      for m in y[key]:
        if 'routes' in m:
          for r in m['routes']:
            r['receivers'] = _FlattenNodeList(r['receivers'])
            r['senders'] = _FlattenNodeList(r['senders'])
        else:
          m['routes'] = []
        if 'receivers' in m and 'senders' in m:
          m['routes'].append({
              'receivers': _FlattenNodeList(m['receivers']),
              'senders': _FlattenNodeList(m['senders'])})
          m.pop('receivers')
          m.pop('senders')
      return y[key]
    return []

  def _PreprocessYamlFile(self, yaml_file):
    """Read the YAML file and prepare it for processing.

    Flatten lists, generate masks, convert port names into numbers, etc.

    Args:
      yaml_file: Path to the network.yaml file.

    Returns:
      The parsed YAML file.

    Raises:
      NetworkConfigException: if there is overlap between C network ports and
          A or B network ports.
    """
    if not yaml_file:
      yaml_file = os.path.join(makani.HOME, 'avionics/network/network.yaml')
    with open(yaml_file, 'r') as f:
      y = yaml.full_load(f)
      y['tms570s'] = _FlattenNodeList(y['tms570s'])
      y['q7s'] = _FlattenNodeList(y['q7s'])

      all_message_types = []
      all_message_types += self._PreprocessMessageTypes('aio_messages', y)
      all_message_types += self._PreprocessMessageTypes('eop_messages', y)
      all_message_types += self._PreprocessMessageTypes('winch_messages', y)

      for switch_name, switch in y['switches'].iteritems():
        if 'config' in switch:
          # Note: If config is shared and port name/number assignments are ever
          # allowed to differ between config users, we'll need to clone it
          # before modifying it.
          config = switch['config']
          for l in ['network_a', 'network_b', 'network_c', 'unicast']:
            if l in config:
              config[l] = _RangeParse(config[l])
          if set(config.get('network_c', [])) & (
              set(config.get('network_a', []))
              | set(config.get('network_b', []))):
            raise NetworkConfigException(
                'A port on %s is assigned to network C as well as A or B.'
                % switch_name)
          if 'port_names' in switch:
            port_names = switch['port_names']
            if 'trunk' in config:
              trunk = config['trunk']
              trunk['ports'] = _ConvertPortNumbersInList(trunk['ports'],
                                                         port_names)
              if 'unicast_learning' in trunk:
                trunk['unicast_learning'] = _ConvertPortNumbersInList(
                    trunk['unicast_learning'], port_names)
              if 'network_c_transit' in trunk:
                trunk['network_c_transit'] = _ConvertPortNumbersInList(
                    trunk['network_c_transit'], port_names)
              overrides = trunk['override_message_routes']
              for k, v in overrides.iteritems():
                overrides[k] = _ConvertPortNumbersInList(v, port_names)
              _ValidateMessageTypes(all_message_types, overrides.iterkeys())
            bandwidth = {}
            for port_name, port_bandwidth in config['bandwidth'].iteritems():
              if port_name == 'default':
                bandwidth['default'] = port_bandwidth
              else:
                bandwidth[_GetPortNumber(
                    port_name, port_names)] = port_bandwidth
            config['bandwidth'] = bandwidth
            config['restrict'] = {
                _GetPortNumber(k, port_names): v
                for k, v in config.get('restrict', {}).iteritems()}
            for v in config['restrict'].itervalues():
              _ValidateMessageTypes(all_message_types, v)
    self._yaml = y
    return y

  def _GenerateAioNodes(self):
    """Generate the list of AIO nodes.

    Ensure a sorted ordering of AIO nodes as found in the YAML file.
    """
    self._aio_nodes = []
    self._aio_nodes_by_name = {}
    self._aio_nodes_by_label = collections.defaultdict(list)
    self._aio_enum_value = 0
    for kind in self._yaml['aio_nodes']:
      for instance in kind['instances']:
        node = AioNode(self, kind, instance)
        self._aio_nodes.append(node)
        self._aio_nodes_by_name[node.snake_name] = node
        self._aio_nodes_by_label[node.label_name].append(node)
    self._aio_nodes = tuple(self._aio_nodes)

  def _GenerateMessages(self, type_name, y):
    """Generate the list of AIO nodes.

    Args:
      type_name: Message type name (e.g., 'Aio' or 'Winch').
      y: A parsed YAML file.

    Raises:
      NetworkConfigException: if message indices are invalid.
    """
    key = type_name.lower() + '_messages'
    message_types = []
    message_types_by_name = {}

    if key in y:
      static_assignments = {m['name']: m['index'] for m in y[key]
                            if 'index' in m}
      value = 0
      used_values = set(static_assignments.values())

      if len(static_assignments) != len(used_values):
        raise NetworkConfigException('Duplicate message indices in %s.' % key)
      if used_values and (min(used_values) < 0 or max(used_values) > 255):
        raise NetworkConfigException('Invalid message indices in %s.' % key)

      for message in y[key]:
        if message['name'] in static_assignments:
          enum_value = static_assignments[message['name']]
        else:
          while value in used_values:
            value += 1
          used_values.add(value)
          enum_value = value

        message_type = MessageType(self, message, type_name, enum_value)
        message_types.append(message_type)
        message_types_by_name[message_type.name] = message_type

    setattr(self, '_%s' % key, message_types)
    setattr(self, '_%s_by_name' % key, message_types_by_name)

  def _ValidateAioNodeIps(self):
    """Ensure that IPs are not duplicated, and that unused IPs are declared."""
    used_ips = set()
    unused_ips_list = _RangeParse(self._yaml.get('unused_ips', []))
    unused_ips = set(unused_ips_list)
    next_ip = self._yaml['next_ip']
    unknown_ip = self._yaml['unknown_ip']
    for node in self._aio_nodes:
      ip = node.ip_octet
      if ip in used_ips:
        raise NetworkConfigException('IP address %d is used more than once.'
                                     % ip)
      if ip in unused_ips:
        raise NetworkConfigException('IP address %d is used and unused.' % ip)
      if ip >= next_ip and ip != unknown_ip:
        raise NetworkConfigException('An address at or above next_ip is in '
                                     'use.')
      used_ips.add(ip)

    covered_range = used_ips.union(unused_ips)
    expected_range = set(range(next_ip))
    expected_range.add(unknown_ip)
    missed_ips = covered_range.symmetric_difference(expected_range)
    if missed_ips:
      raise NetworkConfigException('Address range through "next_ip" isn\'t '
                                   'fully covered by used IPs and "unused_ips";'
                                   ' errors are %s.' % missed_ips)

  def _ValidateMessageSortOrder(self, message_types):
    """Ensure that messages are unique and sorted alphabetically."""
    names = [m.name for m in message_types]
    sorted_names = sorted(names, key=lambda s: s.lower())
    if names != sorted_names:
      raise NetworkConfigException('Messages are not in alphabetical order.')
    if len(set(names)) != len(names):
      raise NetworkConfigException('Duplicate message entry found.')

  def _ValidateAioNodeSortOrder(self):
    """Ensure that AIO nodes are unique and sorted alphabetically.

    Motors are not sorted alphabetically.

    Raises:
      NetworkConfigException: if nodes are not in order.
    """
    node_names = []
    for node in self._aio_nodes:
      if node.label_name != 'unknown':
        node_names.append(node.enum_name)
    sorted_node_names = sorted(node_names, key=lambda s: s.lower())
    for a, b in zip(node_names, sorted_node_names):
      if a != b and not ('Motor' in a and 'Motor' in b):
        raise NetworkConfigException('Node sort order violation near "%s".' % a)
    if len(set(node_names)) != len(node_names):
      raise NetworkConfigException('Duplicate name AIO node entry found.')

  def __init__(self, yaml_file=None):
    y = self._PreprocessYamlFile(yaml_file)
    self._GenerateAioNodes()
    self._GenerateMessages('Aio', y)
    self._GenerateMessages('Eop', y)
    self._GenerateMessages('Winch', y)
    self._ValidateAioNodeIps()
    self._ValidateMessageSortOrder(self.aio_messages)
    self._ValidateMessageSortOrder(self.eop_messages)
    self._ValidateMessageSortOrder(self.winch_messages)
    self._ValidateAioNodeSortOrder()

  @property
  def aio_nodes(self):
    return self._aio_nodes

  def GetAioNode(self, name):
    if name.startswith('kAioNode'):
      name = name[len('kAioNode'):]
    if string_util.IsCamelCase(name):
      name = string_util.CamelToSnake(name)
    name = name.lower()
    try:
      return self._aio_nodes_by_name[name]
    except KeyError:
      raise ValueError('Invalid node name: %s' % name)

  def GetAioMessageType(self, name):
    if name.startswith('kMessageType'):
      name = name[len('kMessageType'):]
    return self._aio_messages_by_name[name]

  @property
  def aio_labels(self):
    return sorted(self._aio_nodes_by_label.iterkeys())

  def GetAioNodesByLabel(self, label):
    if label not in self._aio_nodes_by_label:
      raise ValueError('Invalid label: %s' % label)
    return tuple(self._aio_nodes_by_label[label])

  @property
  def messages_by_type(self):
    return {
        'Aio': self.aio_messages,
        'Eop': self.eop_messages,
        'Winch': self.winch_messages
    }

  @property
  def all_messages(self):
    return self._aio_messages + self._eop_messages + self._winch_messages

  @property
  def aio_messages(self):
    return self._aio_messages

  @property
  def eop_messages(self):
    return self._eop_messages

  @property
  def winch_messages(self):
    return self._winch_messages

  # TODO: Split this section of network.yaml into its own file.
  def GetSwitchChips(self):
    return self._yaml['switch_chips']

  # TODO: Split this section of network.yaml into its own file.
  def GetSwitches(self):
    return self._yaml['switches']


def ParseGenerationFlags(argv):
  """Use gflags to parse arguments specific to network.yaml code generation."""

  gflags.DEFINE_string('autogen_root', makani.HOME,
                       'Root of the source tree for the output files.')

  gflags.DEFINE_string('output_dir', '.',
                       'Full path to the directory for output files.')

  gflags.DEFINE_string('network_file', None,
                       'Full path to the yaml file that describes the network.')

  try:
    argv = gflags.FLAGS(argv)
  except gflags.FlagsError, e:
    sys.stderr.write('\nError: %s\n\nUsage: %s ARGS\n%s\n'
                     % (e, argv[0], gflags.FLAGS))
    sys.exit(1)

  argv = gflags.FLAGS(argv)
  return gflags.FLAGS, argv

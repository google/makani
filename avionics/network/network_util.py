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

"""Utilities to generate switch and route information from network.yaml."""

import collections
import sys


def AioNodeToTag(aio_node):
  """Returns the tag for an AIO node."""
  return 'aio_nodes.%s' % aio_node.snake_name


def IsAioNode(tag):
  """Returns True iff tag represents an AIO node."""
  return tag.startswith('aio_nodes.')


def TagToAioNode(tag):
  """Returns the AIO node for a switch port tag."""
  _, aio_node = tag.split('.')
  return aio_node


def PortToTag(switch, port):
  """Returns the tag for a port."""
  return 'switches.%s.%d' % (switch, port)


def IsPort(tag):
  """Returns True iff tag represents a switch port."""
  return tag.startswith('switches.')


def TagToPort(tag):
  """Returns the switch and port for a switch port tag."""
  _, switch, port = tag.split('.')
  return switch, int(port)


def IsSwitchLink(a, b):
  """Returns True iff a link connects ports on the same switch."""
  return IsPort(a) and IsPort(b) and TagToPort(a)[0] == TagToPort(b)[0]


def IsSwitchLinkRoutable(switch, in_port, out_port, network_c_mode=False):
  """Returns True iff multicast packets can travel from in_port to out_port."""
  config = switch.get('config', {})
  if in_port == out_port:
    return False
  # On core switches, there is a set of ports which can't talk to each
  # other, in order to prevent loops via redundant connections.
  if 'isolate' in config:
    isolated = config['isolate']
    if in_port in isolated and out_port in isolated:
      return False
  if 'trunk' in config:
    trunk_ports = config['trunk']['ports']
    if in_port in trunk_ports and out_port in trunk_ports:
      return False
    # Allow blocking of network C traffic with the network_c_transit trunk
    # setting.
    if network_c_mode and 'network_c_transit' in config['trunk']:
      network_c_transit = config['trunk']['network_c_transit']
      if in_port in trunk_ports and in_port not in network_c_transit:
        return False
  if network_c_mode:
    # Network C rides along with network A traffic.
    network_a = config.get('network_a', [])
    network_c = config.get('network_c', [])
    if ((in_port not in network_a and in_port not in network_c)
        or (out_port not in network_a and out_port not in network_c)):
      return False
  else:
    # On access switches, don't let A-only ports talk to B-only ports.
    # However, ports on both networks can talk to either.
    if 'network_a' in config and 'network_b' in config:
      network_a = config['network_a']
      network_b = config['network_b']
      in_a = in_port in network_a
      in_b = in_port in network_b
      out_a = out_port in network_a
      out_b = out_port in network_b
      in_neither = not in_a and not in_b
      out_neither = not out_a and not out_b
      in_a_only = in_a and not in_b
      in_b_only = in_b and not in_a
      out_a_only = out_a and not out_b
      out_b_only = out_b and not out_a
      if ((in_a_only and out_b_only) or (in_b_only and out_a_only) or
          in_neither or out_neither):
        return False
  return True


class BadConnectionException(Exception):
  pass


class BadMessageException(Exception):
  pass


class BadRouteException(Exception):
  pass


class SwitchMismatchException(Exception):
  pass


class PathFinder(object):
  """Finds paths between AIO network nodes."""

  def __init__(self, switches, message_types, network_c=False):
    self._ValidateConnections(switches)
    self._switches = switches
    self._network_c = network_c
    if not network_c:
      destination_sets = self._ComputeDestinationSets(message_types)
    else:
      destination_sets = self._ComputeNetCDestinationSets(switches)
    self._routes = self._ComputeAllRoutes(destination_sets)

  def _ValidateConnections(self, switches):
    """Make sure each port is only used once and ends of a connection agree.

    Args:
      switches: the switches section from the processed YAML file.
    Raises:
      BadConnectionException: on any network error.
    """
    connections = {}

    for switch, sv in switches.iteritems():
      for port, pv in sv['ports'].iteritems():
        tag = PortToTag(switch, port)
        if not pv:
          sys.stderr.write('WARNING: %s is not connected.\n' % tag)
          continue
        if pv in connections:
          if tag not in connections or connections[pv] is not connections[tag]:
            raise BadConnectionException(
                '%s thinks it is connected to %s, but %s disagrees!'
                % (tag, pv, pv))
          record = connections[pv]
        else:
          record = {
              'count': 0,
          }
          connections[pv] = connections[tag] = record
          if IsAioNode(pv):  # We won't see another end of this connection.
            record['count'] += 1
        record['count'] += 1
    for tag, record in connections.iteritems():
      if record['count'] != 2:
        raise BadConnectionException(
            '%s is connected the wrong number of times: %s'
            % (tag, str(record)))

  def _ComputeDestinationSets(self, message_types):
    """Computes all {source: set(destinations)} used by the input messages."""
    destination_sets = collections.defaultdict(set)
    for m in message_types:
      for route in m.routes:
        for s in route.senders:
          sender_tag = AioNodeToTag(s)
          destination_set = destination_sets[sender_tag]
          for r in route.receivers:
            receiver_tag = AioNodeToTag(r)
            if receiver_tag != sender_tag:
              destination_set.add(receiver_tag)
    return destination_sets

  def _ComputeAllRoutes(self, destination_sets):
    """Computes all routes described by destination_sets.

    Args:
      destination_sets: A map from source to set of destinations.
    Returns:
      A map from source to a map from destination to a set of paths from source
      to that destination.  Each path is a list of port/AioNode tags starting
      from the source and ending with the destination.
      For example:
        { 'aio_nodes.fc_a' :  # Routes from fc_a.
          { 'aio_nodes.recorder_wing': [  #  routes from fc_a to recorder_wing.
              ['aio_nodes.fc_a', 'switches.fc_a.0', ...,  # The first route.
                'aio_nodes.recorder_wing],
              ['aio_nodes.fc_a', 'switches.fc_a.0', ...,  # The second route.
                'aio_nodes.recorder_wing],
                ...],
              ...
            ],
            'aio_nodes.motor_sbo': [ # routes from fc_a to motor_sbo.
              ...
          }
        }
    """
    path_sets = {}
    for source, destinations in destination_sets.iteritems():
      ps = path_sets[source] = {d: [] for d in destinations}
      paths = self._Dfs(source, destinations, coalesce=False)
      for path in paths:
        ps[path[-1]].append(path)
    return path_sets

  def _ComputeNetCDestinationSets(self, switches):
    edges = set()
    for switch_name, switch in switches.iteritems():
      for port in switch.get('config', {}).get('network_c', []):
        edges.add(PortToTag(switch_name, port))
    destination_sets = {}
    for source in edges:
      destination_sets[source] = edges.copy()
      destination_sets[source].remove(source)
    return destination_sets

  def _FindNeighbor(self, tag):
    """Returns the neighboring node, or None if no neighboring node is found."""
    if IsAioNode(tag):
      for switch, sv in self._switches.iteritems():
        for port, pv in sv['ports'].iteritems():
          if pv == tag:
            return PortToTag(switch, port)
      return None
    else:
      switch, port = TagToPort(tag)
      return self._switches[switch]['ports'][port]

  def FindAllPaths(self, coalesce=True):
    """Returns all of the paths in this network topology."""
    paths = []
    for source in self._routes:
      for destination in self._routes[source]:
        routes = self._routes[source][destination]
        if not routes:
          raise BadRouteException('No route from %s to %s' %
                                  (source, destination))
        paths.extend(routes)
    if coalesce:
      paths = self._CoalescePaths(paths)
    return paths

  def FindPaths(self, name, source, destinations, coalesce=True):
    r"""Finds the paths taken by a packet.

    If coalesce is True, the paths will account for multicast replication. For
    example, consider the graph:

               B - D1
              / \ /
        S -- A   x
              \ / \
               C - D2

    Result with coalesce=False:
      [[S, A, B, D1], [S, A, B, D2], [S, A, C, D1], [S, A, C, D2]]

    Result with coalesce=True:
      [[S, A, B, D1], [B, D2], [A, C, D1], [C, D2]]

    With coalesce=True, each link appears in the result exactly the number of
    times that a packet and its multicast replicas traverse it.

    Args:
      name: Message name.
      source: Source node.
      destinations: List of destination nodes.
      coalesce: See above.

    Returns:
      A list of paths taken by a packet, where each path is a list of
      port/AioNode tags starting from the source and ending with a destination.

    Raises:
      BadRouteException: if no route can be found for a destination.
    """
    paths = []
    for destination in destinations:
      if destination == source:
        continue
      if destination not in self._routes[source]:
        continue
      routes = self._routes[source][destination]
      routes = [r for r in routes if not self._IsRestrictedRoute(name, r)]
      if not routes:
        raise BadRouteException('No route from %s to %s for message %s.' %
                                (source, destination, name))
      paths.extend(routes)
    if coalesce:
      paths = self._CoalescePaths(paths)
    return paths

  def _CoalescePaths(self, paths):
    """Coalesce paths as described under FindPaths."""
    paths = sorted(paths)
    coalesced = [paths[0]]
    for p in range(1, len(paths)):
      last = paths[p - 1]
      cur = paths[p]
      path = []
      for i in range(1, len(cur)):
        if i > len(last) or last[i] != cur[i]:
          path.extend(cur[i-1:])
          break
      coalesced.append(path)
    return coalesced

  def _IsRestrictedRoute(self, message, route):
    """Restrict route by message type."""
    for i in range(len(route) - 1):
      egress = route[i]
      ingress = route[i + 1]
      if not IsPort(ingress) or not IsPort(egress):
        continue
      if IsSwitchLink(ingress, egress):
        continue
      out_switch_name, out_port = TagToPort(egress)
      out_switch = self._switches[out_switch_name]
      if 'config' in out_switch:
        restrict = out_switch['config'].get('restrict', {})
        if out_port in restrict and message not in restrict[out_port]:
          return True
        if 'trunk' in out_switch['config']:
          trunk = out_switch['config']['trunk']
          if out_port in trunk['ports']:
            override = trunk.get('override_message_routes', {})
            for override_message, override_ports in override.iteritems():
              if message == override_message and out_port not in override_ports:
                return True
    return False

  def _Dfs(self, source, destinations, coalesce=True):
    """Finds the paths taken by a packet."""
    # Persistent state for _Dfs.
    self._paths = []
    self._destinations = destinations
    self._coalesce = coalesce

    self._DfsHelper(source, [], 0)
    return self._paths

  def _DfsHelper(self, node, path, base):
    """Recursively visit a node.

    Path coalescing is implemented by treating the outgoing paths from a node as
    one "parent" path and zero or more "child" paths. Parent paths preserve the
    path base, while child traversals set the base to the current node. The
    returned paths begin at the base. Because only the parent path preserves the
    base, the incoming path component will only be counted once.  This gives an
    accurate representation of multicast bandwidth utilization.

    Args:
      node: Tag of the node to visit.
      path: Path so far, as an array of tags, not yet including node.
      base: Index of the starting node of this traversal in path.

    Returns:
      True iff a path was found.
    """

    # Found a cycle.
    if node in path:
      return False

    # Ignore paths that hit the same switch more than once.
    if IsPort(node) and len([n for n in path if IsSwitchLink(node, n)]) > 1:
      return False

    found_path = False
    path.append(node)

    if len(path) > 1:
      if node in self._destinations:
        self._paths.append(path[base:])
        path.pop()
        return True  # Destinations are always network leaves.
      elif IsAioNode(node):
        path.pop()
        return False  # AioNodes are always network leaves.

    if IsAioNode(node) or (len(path) >= 2 and IsSwitchLink(node, path[-2])):
      # The current node is an AIO node or an egress port, so there is a maximum
      # of one neighbor to traverse.
      neighbor = self._FindNeighbor(node)
      if neighbor:
        found_path = self._DfsHelper(neighbor, path, base)
    else:
      # The current node is an ingress port. Traverse all allowed egress ports.
      switch_name, in_port = TagToPort(node)
      switch = self._switches[switch_name]
      ports = switch['ports']
      if self._network_c:
        for net_c_port in switch.get('config', {}).get('network_c', []):
          ports[net_c_port] = 'network_c_device'
      for out_port in switch['ports']:
        if not IsSwitchLinkRoutable(switch, in_port, out_port,
                                    network_c_mode=self._network_c):
          continue

        found_path |= self._DfsHelper(PortToTag(switch_name, out_port),
                                      path, base)
        if found_path and self._coalesce:
          base = len(path) - 1

    path.pop()
    return found_path

  def GetSwitches(self):
    return self._switches

  def GetAllConnections(self):
    """List every network connection used in any route.

    Returns:
      A list of tuples of ports, listing each connection twice--once in each
      direction.
    """
    connections = set()
    for destination_set in self._routes.itervalues():
      for path_list in destination_set.itervalues():
        for path in path_list:
          for link in zip(path[:-1], path[1:]):
            connections.add(link)
    return connections

  def GetHops(self, name, source, destinations, unicast=False):
    """List all switch hops between source and destination.

    Args:
      name: Message name.
      source: Source node.
      destinations: List of destination nodes.
      unicast: Require unicast support on path.

    Returns:
      A list of paths taken by a packet, where each path is a list of switches
      that the packet passes through.
    """
    paths = self.FindPaths(name, source, destinations, coalesce=False)
    hops = []
    hop_set = set()
    for path in paths:
      unicast_path = True
      path_hops = []
      for index, hop in enumerate(path[:-1]):
        ingress = path[index]
        egress = path[index + 1]
        if not IsSwitchLink(ingress, egress):
          continue
        switch_name, port = TagToPort(hop)
        switch = self._switches[switch_name]
        unicast_path = ('config' in switch
                        and 'unicast' in switch['config']
                        and port in switch['config']['unicast'])
        path_hops.append(switch_name)

      if (not unicast or unicast_path) and tuple(path_hops) not in hop_set:
        hop_set.add(tuple(path_hops))
        hops.append(path_hops)
    return hops

  def GetFirstSwitch(self, name, source, destinations, switches):
    """List the first switch within switches that a packet routes through.

    Args:
      name: Message name.
      source: Source node.
      destinations: List of destination nodes.
      switches: Switches to consider.

    Returns:
      A list of the first switches that a packet routes through.
    """
    paths = self.GetHops(name, source, destinations)
    first_switches = []
    for path in paths:
      for hop in path:
        if hop in switches:
          first_switches.append(hop)
          break
    return first_switches

  def GetAttachedNodes(self, switch_name):
    """Get a list of nodes attached to the given switch."""
    aio_nodes = []
    switch = self._switches.get(switch_name)
    if switch:
      for tag in switch['ports'].values():
        if IsAioNode(tag):
          aio_nodes.append(TagToAioNode(tag))
    return aio_nodes


def MakeForwardingMaps(message_types, path_finder):
  """Creates a forwarding map for each switch.

  The bitmask tells which ports should receive a given message type,
  e.g. 0x31 forwards to ports 0, 4, and 5.

  Args:
    message_types: {message_types: type_data}, corresponding to the
        "message_types" field of the YAML file.
    path_finder: PathFinder.

  Returns:
    A nested dictionary of the form
        {switch_name: {message_type: port_bitmask}}.
  """

  forwarding_maps = collections.defaultdict(
      lambda: collections.defaultdict(lambda: 0))

  for m in message_types:
    if m.inhibit_routing:
      continue
    for route in m.routes:
      if not route.senders or not route.receivers:
        sys.stderr.write('WARNING: Skipping %s; no route found.\n' % m.name)
        continue
      for sender in route.senders:
        paths = path_finder.FindPaths(
            m.name, AioNodeToTag(sender),
            {AioNodeToTag(x) for x in route.receivers})
        for path in paths:
          for link in zip(path[:-1], path[1:]):
            if IsSwitchLink(*link):
              switch, out_port = TagToPort(link[1])
              forwarding_maps[switch][m.name] |= (1 << out_port)

  return forwarding_maps


def MakeNetworkCForwardingMap(path_finder):
  """Creates a network C forwarding map for each switch.

  The bitmask tells which ports should forward network C traffic.

  Args:
    path_finder: PathFinder.

  Returns:
    A dictionary of the form {switch_name: port_bitmask}.
  """
  forwarding_map = collections.defaultdict(lambda: 0)

  paths = path_finder.FindAllPaths()
  for path in paths:
    for link in zip(path[:-1], path[1:]):
      if IsSwitchLink(*link):
        switch, out_port = TagToPort(link[1])
        forwarding_map[switch] |= (1 << out_port)

  return forwarding_map


class MessageGraph(object):
  """Compute the multicast route graph for a given message type."""

  def __init__(self, path_finder, message):
    self._switches = path_finder.GetSwitches()
    self._message = message

    # Compute graph.
    # Map segment in_tag to a set of out_tags.
    self._segments = collections.defaultdict(set)
    # Map switch name to a set of egress ports.
    self._switch_egress = collections.defaultdict(set)
    for route in message.routes:
      # Iterate through all senders to build the graph because we use the
      # same forwarding maps for all network configurations.
      for sender in route.senders:
        paths = path_finder.FindPaths(
            message.name, AioNodeToTag(sender),
            {AioNodeToTag(x) for x in route.receivers}, coalesce=False)
        for path in paths:
          for source, dest in zip(path[:-1], path[1:]):
            self._segments[source].add(dest)
            if IsSwitchLink(source, dest):
              # Link switch_port -> internal switch_port.
              pass
            elif IsPort(source):
              # Link switch_port -> aio_node or external switch_port.
              switch_name, _ = TagToPort(source)
              self._switch_egress[switch_name].add(source)
            else:
              # Link aio_node -> switch_port.
              pass

  def GetMessage(self):
    return self._message

  def VisitSenders(self, visitor, senders, *args, **kwargs):
    for sender in senders:
      out_tag = AioNodeToTag(sender)
      self.VisitAioNode(out_tag, visitor, sender, *args, **kwargs)

  def VisitAioNode(self, in_tag, visitor, *args, **kwargs):
    self.VisitRemoteLinks(in_tag, visitor, *args, **kwargs)

  def VisitLocalSwitchLinks(self, in_tag, visitor, *args, **kwargs):
    """Visit internal switch port to switch port links."""
    switch_name, in_port = TagToPort(in_tag)
    switch = self._switches[switch_name]
    out_tags = [t for t in self._switch_egress[switch_name]
                if IsSwitchLinkRoutable(switch, in_port, TagToPort(t)[1])]
    visitor.HandleLocalSwitchLinks(self, in_tag, out_tags, *args, **kwargs)

  def VisitRemoteLinks(self, in_tag, visitor, *args, **kwargs):
    """Visit external switch port to remote port links."""
    out_tags = [t for t in self._segments[in_tag]
                if not IsSwitchLink(in_tag, t)]
    visitor.HandleRemoteLinks(self, in_tag, out_tags, *args, **kwargs)


class MessageGraphVisitor(object):
  """Visit each node in message graph."""

  def __init__(self):
    self._path = []

  def _Push(self, tag):
    if tag in self._path:
      raise BadConnectionException('Loop found at %s.' % self._path)
    self._path.append(tag)

  def _Pop(self):
    self._path.pop()

  def GetPath(self):
    return list(self._path)

  def HandleLocalSwitchLinks(self, graph, in_tag, out_tags, *args, **kwargs):
    """Follow internal switch port to switch port links."""
    self._Push(in_tag)
    for out_tag in out_tags:
      self.HandleSegment(graph, in_tag, out_tag, *args, **kwargs)
      graph.VisitRemoteLinks(out_tag, self, *args, **kwargs)
    self._Pop()

  def HandleRemoteLinks(self, graph, in_tag, out_tags, *args, **kwargs):
    """Follow external switch port to remote port links."""
    self._Push(in_tag)
    for out_tag in out_tags:
      self.HandleSegment(graph, in_tag, out_tag, *args, **kwargs)
      if IsAioNode(out_tag):
        self.HandleAioNode(graph, out_tag, *args, **kwargs)
      elif IsPort(out_tag):
        graph.VisitLocalSwitchLinks(out_tag, self, *args, **kwargs)
      else:
        raise BadConnectionException('Unexpected tag %s.' % out_tag)
    self._Pop()

  def HandleSegment(self, graph, in_tag, out_tag, *args, **kwargs):  # pylint: disable=unused-argument
    pass

  def HandleAioNode(self, graph, in_tag, *args, **kwargs):  # pylint: disable=unused-argument
    pass


class SegmentStatisticsVisitor(MessageGraphVisitor):
  """Collect packet statistics by segment and sender."""

  def __init__(self):
    self.stats = collections.defaultdict(
        lambda: collections.defaultdict(  # pylint: disable=g-long-lambda
            lambda: collections.defaultdict(int)))
    super(SegmentStatisticsVisitor, self).__init__()

  def HandleSegment(self, graph, in_tag, out_tag, sender):
    segment = (in_tag, out_tag)
    s = self.stats[segment][sender]
    s['packets_per_sec'] += graph.GetMessage().frequency_hz
    s['peak'] += 1


def GetSegmentStats(graph, senders):
  """Gather the bandwidth statistics for each segment.

  Args:
    graph: A MessageGraph object.
    senders: A list of AioNode objects for each sender to consider.

  Returns:
    A dictionary of one-way segment statistics, keyed by the segment tuple
    (in_tag, out_tag). The segment statistics describe the data flowing from
    the in_tag to the out_tag.

    A typical example is:
    {
        <(in_tag, out_tag)>: {
            <sender>: {
                'packets_per_sec': <freq>,
                'peak': <count>
            }
        }
    }
  """
  visitor = SegmentStatisticsVisitor()
  graph.VisitSenders(visitor, senders)
  return visitor.stats


def GetNodeBandwidthStatistics(path_finder, message_types):
  """Gather the bandwidth statistics for each endpoint.

  Args:
    path_finder: PathFinder.
    message_types: A list of MessageTypes.

  Returns:
    A dictionary of AIO node configurations, keyed by AIO node names.
    Each AIO node configuration contains 'send', 'receive', and aggregated
    rates for transmitting and receiving packets.

    A typical example is:
    {
        <aio_node>: {
            'send': {
                <message_type>: <freq>
            },
            'receive': {
                <message_type>: <freq>
            },
            'multicast_packet_rate': {
              'tx': <#_of_transmitted_packets_per_second>,
              'rx': <#_of_received_packets_per_second>,
            }
        }
    }
  """
  Stat = collections.namedtuple(  # pylint: disable=invalid-name
      'Stat', ['send', 'receive', 'multicast_packet_rate'])
  # Stats per node, per message.
  node_stats = collections.defaultdict(
      lambda: Stat(  # pylint: disable=g-long-lambda
          collections.defaultdict(lambda: 0),
          collections.defaultdict(lambda: 0),
          collections.defaultdict(lambda: 0)))

  for m in message_types:
    if not m.frequency_hz:
      continue

    graph = MessageGraph(path_finder, m)
    message_stats = GetSegmentStats(graph, m.all_senders)
    for segment, senders in message_stats.iteritems():
      for stats in senders.values():
        packets_per_sec = stats['packets_per_sec']
        if IsAioNode(segment[0]):
          # AioNode sends message.
          node = segment[0]
          node_stats[node].send[m.name] += packets_per_sec
          node_stats[node].multicast_packet_rate['tx'] += packets_per_sec
        elif IsAioNode(segment[1]):
          # AioNode receives message.
          node = segment[1]
          node_stats[node].receive[m.name] += packets_per_sec
          node_stats[node].multicast_packet_rate['rx'] += packets_per_sec
  return node_stats


class RecipientsVisitor(MessageGraphVisitor):
  """Collect recipients by sender."""

  def __init__(self):
    self.recipients = collections.defaultdict(set)
    self.paths = collections.defaultdict(list)
    super(RecipientsVisitor, self).__init__()

  def HandleAioNode(self, graph, in_tag, sender):
    self.recipients[sender].add(in_tag)
    self.paths[(sender, TagToAioNode(in_tag))].append(self.GetPath())


def CheckForUnintendedRecipients(graph):
  """Check for unintended recipients in multicast routing graph."""
  message = graph.GetMessage()
  visitor = RecipientsVisitor()
  graph.VisitSenders(visitor, message.all_senders)

  for sender in message.all_senders:
    # Actual receivers are those who receive messages from 'sender' as a
    # consequence of the multicast routes.
    actual_receivers = set(TagToAioNode(t) for t in visitor.recipients[sender])
    # Expected receivers are those from the union of receivers from all routes
    # with 'sender' in senders.
    expected_receivers = set()
    for route in [r for r in message.routes if sender in r.senders]:
      expected_receivers |= set(n.snake_name for n in route.receivers)

    unexpected_receivers = actual_receivers - expected_receivers
    if unexpected_receivers:
      print 'Unexpected network routes:'
      for r in unexpected_receivers:
        for p in visitor.paths[(sender, r)]:
          print p
      raise BadConnectionException(
          'Network routes cross: %s should not receive %s messages from %s.'
          % (list(unexpected_receivers), message.name, sender.snake_name))


def GetAttachedSwitch(node, switches):
  node_tag = AioNodeToTag(node)
  for switch_name, switch in switches.iteritems():
    for tag in switch['ports'].values():
      if node_tag == tag:
        return switch_name
  raise BadConnectionException('Could not find switch for node %s.' % node)

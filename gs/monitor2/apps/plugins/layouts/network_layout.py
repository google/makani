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

"""Layout to monitor network status."""
from makani.avionics.network import network_bandwidth
from makani.avionics.network import network_config
from makani.avionics.network import network_util
from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.layout import widgets
from makani.gs.monitor2.project import settings
from makani.lib.python import string_util


class NetworkLayoutError(Exception):
  pass


class NetworkLayout(base.BaseLayout):
  """Layout for network status."""

  # TODO: Test utility functions under different scenarios.

  _NAME = 'Network Status'
  # The desired number of columns to place views.
  _DESIRED_VIEW_COLS = 2

  def Initialize(self):
    config = network_config.NetworkConfig(settings.NETWORK_YAML)
    message_types = config.all_messages
    path_finder = network_util.PathFinder(config.GetSwitches(), message_types)
    self._node_stats = {}
    for tag, stats in network_util.GetNodeBandwidthStatistics(
        path_finder, message_types).iteritems():
      self._node_stats[self._ParseNetworkConfigTag(tag)[1]] = stats

    self._link_stats = network_bandwidth.GetLinkBandwidthStatistics(config)
    self._freq_by_message_type = {}
    self._status_message_by_aio_node = {}
    for m in message_types:
      self._freq_by_message_type[m.name] = m.frequency_hz
      if m.name in ['CoreSwitchSlowStatus', 'SlowStatus']:
        for sender in m.all_senders:
          self._status_message_by_aio_node[sender.camel_name] = m.name

  def Filter(self, messages):
    """Select and compute data to show.

    Args:
      messages: A struct_tree.StructTree containing a snapshot of messages
          received. It can be indexed as a nested dict. Indexing returns None
          if field does not exist.

    Returns:
      A PlotData object storing the generated data.
    """
    data = self._GetPlotData()

    for aio_node in self._node_stats:
      # Check error flags on all the nodes.
      message_type = self._GetStatusMessageType(aio_node)
      if message_type is None:
        continue
      # Index string to the ethernet stats.
      path = self._EthernetStatsPath(message_type, aio_node)
      error_fields = ['rx_fragment_errors', 'rx_alignment_errors',
                      'rx_fcs_errors', 'rx_symbol_errors', 'rx_jabber_errors',
                      'rx_in_range_errors', 'tx_dropped_packets']
      # Stoplight showing the status of this AIO node.
      node_stoplight = stoplights.STOPLIGHT_ANY
      # Detailed status of this AIO node.
      content = []
      for field in error_fields:
        # An array of error counts per port, or None if message is unavailable.
        error_stats = messages['%s.%s' % (path, field)]
        # Stoplight for this error field.
        field_stoplight = stoplights.STOPLIGHT_ANY
        # Indicators for each port.
        port_stats = {}
        if error_stats:
          for port, error_count in enumerate(error_stats):
            if error_count > 0:
              port_stoplight = stoplights.STOPLIGHT_WARNING
            else:
              port_stoplight = stoplights.STOPLIGHT_NORMAL
            port_stats[port] = {'value': error_count,
                                'stoplight': port_stoplight}
            # Stoplight of a field indicates the worst case of all subfields.
            field_stoplight = stoplights.MostSevereStoplight(field_stoplight,
                                                             port_stoplight)
        else:
          field_stoplight = stoplights.STOPLIGHT_UNAVAILABLE

        # Stoplight of the AIO node indicates the worst case of all fields.
        node_stoplight = stoplights.MostSevereStoplight(node_stoplight,
                                                        field_stoplight)
        content.append({
            'name': field,
            'stoplight': field_stoplight,
            'fields': port_stats,
        })
      data[self._AioNodeStatsName(aio_node)] = {
          'stoplight': node_stoplight,
          'content': content,
          'node': aio_node,
      }

    # Check each link.
    for link, link_stat in self._link_stats.iteritems():
      if not self._IsInterNodeLink(link):
        # Skip onboard links.
        continue
      link_name = self._LinkStatsName(link)
      if messages:
        sender_field, sender_freq = self._GetFieldAndFreqFromPortTag(
            link[0], 'tx_multicast_packet_rate')
        receiver_field, receiver_freq = self._GetFieldAndFreqFromPortTag(
            link[1], 'rx_multicast_packet_rate')

        if sender_field is None or receiver_field is None:
          # No such link.
          continue

        # The stoplight for this link.
        link_stoplight = stoplights.STOPLIGHT_ANY
        content = []

        # Check individual rates.
        sender_rate = messages[sender_field]
        receiver_rate = messages[receiver_field]

        # Check if the rates match.
        if sender_rate is not None and receiver_rate is not None:
          if sender_rate < 0:
            result_stoplight = stoplights.STOPLIGHT_ERROR
            content.append({'name': 'Invalid tx rate: %.2f' % sender_rate,
                            'stoplight': result_stoplight})
          elif receiver_rate < 0:
            result_stoplight = stoplights.STOPLIGHT_ERROR
            content.append({'name': 'Invalid rx rate: %.2f' % receiver_rate,
                            'stoplight': result_stoplight})
          elif (abs(sender_rate - receiver_rate) > 1 and
                abs(sender_rate - receiver_rate) /
                float(sender_rate + receiver_rate) > 0.1):
            result_stoplight = stoplights.STOPLIGHT_ERROR
            content.append({
                'name': 'tx rate (%.2f) != rx rate (%.2f)' % (sender_rate,
                                                              receiver_rate),
                'stoplight': result_stoplight,
            })
          else:
            # If they differ by no more than one or the difference is less than
            # 10% of their sum, treat them as equal.
            if sender_rate == getattr(link_stat, 'packets_per_sec'):
              result_stoplight = stoplights.STOPLIGHT_NORMAL
            else:
              # Equal rates but with unexpected value.
              result_stoplight = stoplights.STOPLIGHT_WARNING
            content.append({
                'name': 'tx/rx rate: %.2f/%.2f' % (sender_rate, receiver_rate),
                'stoplight': result_stoplight,
            })

          link_stoplight = stoplights.MostSevereStoplight(link_stoplight,
                                                          result_stoplight)
        link_stoplight = self._CheckLinkRate('tx', sender_rate, sender_freq,
                                             link_stat, link_stoplight,
                                             content)

        link_stoplight = self._CheckLinkRate('rx', receiver_rate,
                                             receiver_freq, link_stat,
                                             link_stoplight, content)
      else:
        link_stoplight = stoplights.STOPLIGHT_UNAVAILABLE
        content = []

      data[link_name] = {
          'stoplight': link_stoplight,
          'content': content,
          'src': self._ParseNetworkConfigTag(link[0]),
          'dest': self._ParseNetworkConfigTag(link[1]),
      }

    return data

  def Plot(self, data):
    """Defines a monitor layout.

    Args:
      data: A PlotData object, with fields defined in self.Filter.

    Returns:
      A layout definition.
    """
    views = []

    # Widgets for links.
    wing_a_network_link_widgets = []
    wing_b_network_link_widgets = []
    gs_a_network_link_widgets = []
    gs_b_network_link_widgets = []
    other_link_widgets = []
    link_stats = self._link_stats.keys()
    # Rename the link so that port '1' becomes '0001', so it can be sorted.
    # We assume the max port ID is 9999.
    naming = lambda s: s[:s.rfind('.')] + '.' + s[s.rfind('.') + 1:].zfill(4)
    link_names = ['%s -> %s' % (naming(l[0]), naming(l[1])) for l in link_stats]
    ordering = sorted(range(len(link_names)), key=lambda x: link_names[x])
    for i in ordering:
      link = link_stats[i]
      widget = widgets.AttributesWidget('%s -> %s' % (link[0], link[1]),
                                        data[self._LinkStatsName(link)])
      if self._IsInterNodeLink(link):
        if self._IsWingNetwork(link, wing=True):
          if self._IsNetwork(link, network_a=True):
            wing_a_network_link_widgets.append(widget)
          elif self._IsNetwork(link, network_a=False):
            wing_b_network_link_widgets.append(widget)
          else:
            other_link_widgets.append(widget)
        elif self._IsWingNetwork(link, wing=False):
          if self._IsNetwork(link, network_a=True):
            gs_a_network_link_widgets.append(widget)
          elif self._IsNetwork(link, network_a=False):
            gs_b_network_link_widgets.append(widget)
          else:
            other_link_widgets.append(widget)
        else:
          other_link_widgets.append(widget)

    views.append(('Wing Network A', wing_a_network_link_widgets))
    views.append(('Wing Network B', wing_b_network_link_widgets))
    views.append(('GS Network A', gs_a_network_link_widgets))
    views.append(('GS Network B', gs_b_network_link_widgets))
    views.append(('Other links', other_link_widgets))

    # Widgets for AIO nodes.
    aio_node_widgets = []
    for aio_node in sorted(self._node_stats):
      aio_node_widgets.append(widgets.AttributesWidget(
          aio_node, data[self._AioNodeStatsName(aio_node)]))
    views.append(('Aio Nodes', aio_node_widgets))

    return base.AssembleLayout(views, self._DESIRED_VIEW_COLS,
                               self._ORDER_HORIZONTALLY)

  def _IsWingNetwork(self, link, wing):
    is_wing = None
    for port in link:
      if port.startswith('switches.cs_'):
        is_port_on_wing = (not port.startswith('switches.cs_gs_'))
        if is_wing is None:
          is_wing = is_port_on_wing
        elif is_wing != is_port_on_wing:
          return False

    if is_wing is None:
      is_wing = 'servo' in link[0] and 'servo' in link[1]

    return is_wing if wing else not is_wing

  def _IsNetwork(self, link, network_a):
    """Test if a link is in network A or network B."""
    if not self._IsInterNodeLink(link):
      return False
    is_network_a = None
    for port in link:
      port_id = int(port[port.rfind('.') + 1:])
      is_core_switch = port.startswith('switches.cs_')
      if not is_core_switch:
        is_port_in_network_a = not port_id & 1
        if is_network_a is None:
          is_network_a = is_port_in_network_a
        elif is_network_a != is_port_in_network_a:
          return False

    if is_network_a is None:
      return False

    return is_network_a if network_a else not is_network_a

  def _IsInterNodeLink(self, link):
    source, sink = link
    return len(source.split('.')) == 3 and len(sink.split('.')) == 3

  def _GetFieldAndFreqFromPortTag(self, tag, field_name):
    """Get the field path and the frequency of the status message.

    Args:
      tag: The tag of a network port. E.g. `aio_nodes.CsA.4'.
      field_name: A field name in the EthernetStats struct.

    Returns:
      A tuple including the full name of the field and the frequency of its
      status message.
    """
    _, aio_node, port = self._ParseNetworkConfigTag(tag)
    message_type = self._GetStatusMessageType(aio_node)
    if message_type is None:
      return None, None
    field_name = '%s.%s' % (
        self._EthernetStatsPath(message_type, aio_node, port), field_name)
    return field_name, self._freq_by_message_type[message_type]

  def _GetStatusMessageType(self, aio_node):
    """Get the shortname for an AIO node's status message type."""
    if aio_node in self._status_message_by_aio_node:
      return self._status_message_by_aio_node[aio_node]
    else:
      return None

  def _EthernetStatsPath(self, message_type, aio_node, port=None):
    """Get the path string to the EthernetStats struct."""
    if port is None:
      return '%s.%s.switch_stats.stats[:]' % (message_type, aio_node)
    else:
      return '%s.%s.switch_stats.stats[%d]' % (message_type, aio_node, port)

  def _LinkStatsName(self, link):
    return 'link__%s__%s' % (link[0].replace('.', '_'),
                             link[1].replace('.', '_'))

  def _AioNodeStatsName(self, aio_node):
    return 'node__%s' % aio_node.replace('.', '_')

  def _ParseNetworkConfigTag(self, tag):
    """From a network tag, return category, aio_node, and port.

    Args:
      tag: The network tag of a port.

    Returns:
      A tuple of category, aio_node, and port. Port is None if not present.

    Raises:
      NetworkLayoutError: Raised if parsing failed.
    """

    items = tag.split('.')
    if len(items) == 2:
      items.append(None)
    elif len(items) == 3:
      # Port number.
      items[2] = int(items[2])
    else:
      raise NetworkLayoutError('Cannot parse network config tag: %s.' % tag)

    # Set AIO node name to camel case.
    items[1] = string_util.SnakeToCamel(items[1], False)
    return tuple(items)

  def _CheckLinkRate(self, direction, rate, freq, link_stat, stoplight,
                     content):
    """Perform common checks for link rates.

    Args:
      direction: Usually it is 'tx' or 'rx'.
      rate: The packet rate.
      freq: The frequency of the message.
      link_stat: The structure holding expected packet rates.
      stoplight: The stoplight so far.
      content: A list holding results for various checks.

    Returns:
      stoplight: The resuling stoplight.
    """
    expected_packets_per_sec = getattr(link_stat, 'packets_per_sec')
    if rate is None:
      result_stoplight = stoplights.STOPLIGHT_UNAVAILABLE
      content.append({
          'name': '%s rate unavailable.' % direction,
          'stoplight': result_stoplight,
      })
      stoplight = stoplights.MostSevereStoplight(stoplight, result_stoplight)
    elif rate * freq != expected_packets_per_sec:
      result_stoplight = stoplights.STOPLIGHT_WARNING
      content.append({
          'name': '%s rate (%.2f) != expected (%.2f)' % (
              direction, rate * freq, expected_packets_per_sec),
          'stoplight': result_stoplight,
      })
      stoplight = stoplights.MostSevereStoplight(stoplight, result_stoplight)
    return stoplight

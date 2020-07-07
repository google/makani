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

"""Indicators for network status."""

from makani.avionics.common import cvt
from makani.avionics.common import pack_avionics_messages
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.lib.python import struct_tree


# TODO: Find a global definition source for it.
_XLR_RSSI_WARNING_THRESHOLD = -112 + 20

# The Microhard pDDL radios support the joystick traffic without
# packet drops down to an RSSI of -92 dBm. The additional free space
# loss from perch to maximal glide range is 11 dB.
_PDDL_RSSI_WARNING_THRESHOLD = -92 + 11


def _IsSwitchCommsLinkUp(switch_stats, port):
  return (switch_stats and (switch_stats.link_status_bits & (1 << port)) and
          switch_stats.stats[port].rx_multicast_packet_rate > 0)


class BaseCommsStatusIndicator(indicator.BaseAttributeIndicator):
  """Base class for comms status."""

  def __init__(self, name, node_a, port_a, node_b, port_b, message_type,
               show_label, ignore_error=False):
    super(BaseCommsStatusIndicator, self).__init__([
        (message_type, node_a, 'switch_stats'),
        (message_type, node_b, 'switch_stats'),
    ], name)

    self._ignore_error = ignore_error
    self._port_a = port_a
    self._port_b = port_b
    self._show_label = show_label

  def _ShowLinkStatus(self, is_up):
    return 'Up' if is_up else 'Down'

  def _DictToString(self, results, item_length=10):
    text = []
    keys = sorted(results.keys())
    if self._show_label:
      text.append(' '.join(k.rjust(item_length) for k in keys))
    text.append(' '.join(results[k].rjust(item_length) for k in keys))

    return '\n'.join(text)

  def _Filter(self, status_a, status_b):
    results = {}
    total_links = 0
    total_up_links = 0
    if self._port_a is None:
      results['Link A'] = '--'
    else:
      is_a_up = _IsSwitchCommsLinkUp(status_a, self._port_a)
      results['Link A'] = self._ShowLinkStatus(is_a_up)
      total_links += 1
      if is_a_up:
        total_up_links += 1

    if self._port_b is None:
      results['Link B'] = '--'
    else:
      is_b_up = _IsSwitchCommsLinkUp(status_b, self._port_b)
      results['Link B'] = self._ShowLinkStatus(is_b_up)
      total_links += 1
      if is_b_up:
        total_up_links += 1

    if self._ignore_error:
      stoplight = stoplights.STOPLIGHT_ANY
    else:
      if total_links == 0:
        stoplight = stoplights.STOPLIGHT_ANY
      elif total_up_links == total_links:
        stoplight = stoplights.STOPLIGHT_NORMAL
      elif total_up_links == 0:
        stoplight = stoplights.STOPLIGHT_ERROR
      else:
        stoplight = stoplights.STOPLIGHT_WARNING

    return self._DictToString(results), stoplight


class CommsStatusPoFIndicator(BaseCommsStatusIndicator):

  def __init__(self, name, show_label=True):
    super(CommsStatusPoFIndicator, self).__init__(
        name, 'CsGsA', 20, 'CsGsB', 20, 'CoreSwitchSlowStatus', show_label,
        ignore_error=True)


class CommsStatusEoPIndicator(BaseCommsStatusIndicator):

  def __init__(self, name, show_label=True):
    super(CommsStatusEoPIndicator, self).__init__(
        name, 'CsGsA', None, 'CsGsB', None, 'CoreSwitchSlowStatus', show_label,
        ignore_error=True)


class CommsStatusWifiIndicator(BaseCommsStatusIndicator):

  def __init__(self, name, show_label=True):
    super(CommsStatusWifiIndicator, self).__init__(
        name, 'CsGsA', 22, 'CsGsB', 18, 'CoreSwitchSlowStatus', show_label)


class JoystickRadioStatusIndicator(indicator.BaseAttributeIndicator):
  """Joystick radio status."""

  def __init__(self, name):
    super(JoystickRadioStatusIndicator, self).__init__([
        ('JoystickMonitorStatus', 'JoystickA', 'microhard_status'),
        ('TetherDown', 'CsB', 'comms_status'),
    ], name)

  def _HandleStatus(self, connected, rssi):
    if connected:
      if rssi < _PDDL_RSSI_WARNING_THRESHOLD:
        stoplight = stoplights.STOPLIGHT_WARNING
      else:
        stoplight = stoplights.STOPLIGHT_NORMAL
      return '% 4d' % rssi, stoplight
    else:
      return ' n/a', stoplights.STOPLIGHT_WARNING

  def _Filter(self, down_status, up_status):
    if struct_tree.IsValidElement(down_status):
      text, down_stoplight = self._HandleStatus(
          down_status.connected, down_status.rssi)
    else:
      text, down_stoplight = '--', stoplights.STOPLIGHT_WARNING
    result = '%s dBm down, ' % text

    if struct_tree.IsValidElement(up_status):
      is_connected = (up_status.links_up &
                      pack_avionics_messages.kTetherCommsLinkJoystick)
      text, up_stoplight = self._HandleStatus(
          is_connected, up_status.received_signal_strength)
    else:
      text, up_stoplight = '--', stoplights.STOPLIGHT_WARNING

    result += '%s dBm up' % text
    stoplight = stoplights.MostSevereStoplight(down_stoplight, up_stoplight)
    return result, stoplight


class TetherLongRangeRadioStatusIndicator(indicator.SingleAttributeIndicator):

  def __init__(self, name):
    super(TetherLongRangeRadioStatusIndicator, self).__init__(
        ('TetherDown', 'CsGsA'), name)

  def _Filter(self, tether_down):
    if not struct_tree.IsValidElement(tether_down):
      return 'Link down', stoplights.STOPLIGHT_ERROR

    down_signal_strength = tether_down.received_signal_strength
    up_signal_strength = tether_down.comms_status.received_signal_strength
    text = '% 4d dBm down, % 4d dBm up' % (
        down_signal_strength, up_signal_strength)

    if (down_signal_strength < _XLR_RSSI_WARNING_THRESHOLD or
        up_signal_strength < _XLR_RSSI_WARNING_THRESHOLD):
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    return text, stoplight


class BaseTetherCommsStatusIndicator(indicator.BaseAttributeIndicator):
  """Base class for tether comms status."""

  def __init__(self, name, link_type, sources_per_link, link_names, show_label,
               ignore_error=False):
    super(BaseTetherCommsStatusIndicator, self).__init__(
        self._PackArguments(sources_per_link), name)
    assert len(sources_per_link) == len(link_names)
    self._sources_per_link = sources_per_link
    self._link_names = link_names
    self._link_type = link_type
    self._show_label = show_label
    self._ignore_error = ignore_error

  def _PackArguments(self, sources_per_link):
    """Construct the list of arguments telling statuses of various links.

    Args:
      sources_per_link: A list of source list. Each source list contains
          TetherDownSources for a particular link.

    Returns:
      The packed argument list is in the form of
      [<status_link_0>, <valid_link_0>, <status_link_1>, <valid_link_1>, ...]
    """
    attributes = []
    for sources in sources_per_link:
      for source in sources:
        attributes.append(
            ('filtered', 'merge_tether_down', 'comms_status[%d]' % source))
        attributes.append(
            ('filtered', 'merge_tether_down',
             'comms_status_valid[%d]' % source))
    return attributes

  def _UnpackArguments(self, *attributes):
    """Unpack attributes to comms status, valid bits, and indices per link."""
    comms_status = attributes[0:len(attributes):2]
    valid = attributes[1:len(attributes):2]
    # A dictionary of source indices in `comms_status` and `valid` arrays
    # per link. E.g., comms_status[source_indice_per_link[0][1]] is the
    # comms_status for source 1 of link 0.
    source_indices_per_link = {}
    source_idx = 0
    for link_idx, sources in enumerate(self._sources_per_link):
      source_indices_per_link[link_idx] = range(
          source_idx, source_idx + len(sources))
      source_idx += len(sources)
    return comms_status, valid, source_indices_per_link

  def _ShowLinkStatus(self, is_up):
    return 'Up' if is_up else 'Down'

  def _DictToString(self, results, item_length=10):
    text = []
    keys = sorted(results.keys())
    if self._show_label:
      text.append(' '.join(k.rjust(item_length) for k in keys))
    text.append(' '.join(results[k].rjust(item_length) for k in keys))

    return '\n'.join(text)

  def _Filter(self, *attributes):
    comms_status, valid, source_indices_per_link = self._UnpackArguments(
        *attributes)

    results = {}
    total_links = 0
    total_up_links = 0
    # Iterate through all links in the order of _sources_per_link.
    for link_index in range(len(self._sources_per_link)):
      link_name = self._link_names[link_index]
      link_up = False
      for source_idx in source_indices_per_link[link_index]:
        if (valid[source_idx] and comms_status[source_idx].no_update_count <
            common.MAX_NO_UPDATE_COUNT_COMMS_STATUS):
          link_up = comms_status[source_idx].links_up & self._link_type
          break
      # Links are regarded as DOWN, if comms_status is obsolete.
      results[link_name] = self._ShowLinkStatus(link_up)
      total_links += 1
      total_up_links += (1 if link_up else 0)

    if self._ignore_error:
      stoplight = stoplights.STOPLIGHT_ANY
    else:
      if total_links == 0:
        stoplight = stoplights.STOPLIGHT_ANY
      elif total_up_links == total_links:
        stoplight = stoplights.STOPLIGHT_NORMAL
      elif total_up_links == 0:
        stoplight = stoplights.STOPLIGHT_ERROR
      else:
        stoplight = stoplights.STOPLIGHT_WARNING

    return self._DictToString(results), stoplight


class TetherCommsStatusPoFIndicator(BaseTetherCommsStatusIndicator):

  def __init__(self, name, show_label=True):
    super(TetherCommsStatusPoFIndicator, self).__init__(
        name, cvt.kTetherCommsLinkPof,
        [[cvt.kTetherDownSourceCsGsA, cvt.kTetherDownSourceCsA],
         [cvt.kTetherDownSourceCsB]], ['Link A', 'Link B'], show_label,
        ignore_error=True)


class TetherCommsStatusEoPIndicator(BaseTetherCommsStatusIndicator):

  def __init__(self, name, show_label=True):
    super(TetherCommsStatusEoPIndicator, self).__init__(
        name, cvt.kTetherCommsLinkEop,
        [[cvt.kTetherDownSourceCsGsA, cvt.kTetherDownSourceCsA],
         [cvt.kTetherDownSourceCsB]], ['Link A', 'Link B'], show_label,
        ignore_error=True)


class TetherCommsStatusWifiIndicator(BaseTetherCommsStatusIndicator):

  def __init__(self, name, show_label=True):
    super(TetherCommsStatusWifiIndicator, self).__init__(
        name, cvt.kTetherCommsLinkWifi,
        [[cvt.kTetherDownSourceCsGsA, cvt.kTetherDownSourceCsA],
         [cvt.kTetherDownSourceCsB]], ['Link A', 'Link B'], show_label)

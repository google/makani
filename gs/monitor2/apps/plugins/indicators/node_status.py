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

import collections

from makani.analysis.checks import check_range
from makani.avionics.common import cvt
from makani.avionics.network import aio_node
from makani.control import system_params
from makani.control import system_types
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.lib.python import c_helpers

_TETHER_NODE_FLAGS_HELPER = c_helpers.EnumHelper('TetherNodeFlag', cvt)
_AIO_NODE_HELPER = c_helpers.EnumHelper('AioNode', aio_node)
_SYSTEM_PARAMS = system_params.GetSystemParams().contents
_WING_TMS570_NODES = common.WingTms570Nodes()


class BaseTetherNodeStatusIndicator(indicator.BaseAttributeIndicator):
  """Base indicator class for individual node status."""

  def __init__(self, name, node_name):
    node_id = _AIO_NODE_HELPER.Value(node_name)
    super(BaseTetherNodeStatusIndicator, self).__init__([
        ('filtered', 'merge_tether_down', 'node_status[%d]' % node_id),
        ('filtered', 'merge_tether_down',
         'node_status_valid[%d]' % node_id),
    ], name)

  def _IsValidInput(self, node_status, valid):
    return valid


class BaseTetherAllNodeStatusIndicator(indicator.BaseAttributeIndicator):
  """Base indicator class for aggregated node status."""

  def __init__(self, name):
    super(BaseTetherAllNodeStatusIndicator, self).__init__([
        ('filtered', 'merge_tether_down', 'node_status'),
        ('filtered', 'merge_tether_down', 'node_status_valid'),
    ], name)


class BaseTetherNodeStatusFieldIndicator(BaseTetherNodeStatusIndicator):

  def __init__(self, name, node_name, node_field):
    super(BaseTetherNodeStatusFieldIndicator, self).__init__(
        name, node_name)
    self._node_field = node_field

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, node_status, valid):
    value = getattr(node_status, self._node_field)
    return '% 4.1f' % value, stoplights.STOPLIGHT_NORMAL


class TetherNodeTempIndicator(BaseTetherNodeStatusFieldIndicator):

  def __init__(self, name, node_name):
    super(TetherNodeTempIndicator, self).__init__(
        name, node_name, 'board_temp')


class TetherNodeHumidityIndicator(BaseTetherNodeStatusFieldIndicator):

  def __init__(self, name, node_name):
    super(TetherNodeHumidityIndicator, self).__init__(
        name, node_name, 'board_humidity')


class BaseTetherNodeStatusFlagsIndicator(BaseTetherNodeStatusIndicator):

  def __init__(self, name, node_name, node_flags):
    super(BaseTetherNodeStatusFlagsIndicator, self).__init__(
        name, node_name)
    self._node_flags = node_flags

  def _ActivatedFlags(self, node_status):
    flags = node_status.flags
    activated_flags = set()
    for flag_name in self._node_flags:
      if flags & _TETHER_NODE_FLAGS_HELPER.Value(flag_name):
        activated_flags.add(flag_name)
    return activated_flags


class TetherNodeNetworkIndicator(BaseTetherNodeStatusFlagsIndicator):
  """Show network status for individual node from the TetherDown."""

  def __init__(self, name, node_name, enabled=True):
    super(TetherNodeNetworkIndicator, self).__init__(
        name, node_name, ['NetworkAGood', 'NetworkBGood'])
    # If not `enabled`, the indicator always shows grey. We allow this
    # scenario so that the indicators can still take a grid cell for the
    # purpose of alignment in the status layout.
    self._enabled = enabled

  def _IsValidInput(self, node_status, valid):
    return valid and self._enabled

  @indicator.ReturnIfInputInvalid(
      'A: --      B: --    ', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, node_status, valid):
    activated_flags = self._ActivatedFlags(node_status)
    network_labels = ['A', 'B']
    chars_per_network = 6
    total_links = len(network_labels)
    outputs = []
    for idx, flag in enumerate(self._node_flags):
      text = '%s: ' % network_labels[idx]
      if flag in activated_flags:
        text += 'Up'.ljust(chars_per_network)
      else:
        text += 'Down'.ljust(chars_per_network)
      outputs.append(text)

    total_up_links = len(activated_flags)

    if not total_up_links:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif total_up_links < total_links:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    return '  '.join(outputs), stoplight


class TetherNodeFailureIndicator(BaseTetherNodeStatusFlagsIndicator):

  def __init__(self, name, node_name):
    super(TetherNodeFailureIndicator, self).__init__(
        name, node_name, ['SelfTestFailure', 'AnyWarning', 'AnyError'])

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, node_status, valid):
    activated_flags = self._ActivatedFlags(node_status)
    failures = []
    warnings = []

    for flag_name in ['SelfTestFailure', 'AnyError']:
      if flag_name in activated_flags:
        failures.append(flag_name)

    for flag_name in ['AnyWarning']:
      if flag_name in activated_flags:
        warnings.append(flag_name)

    reports = failures + warnings
    text = ', '.join(reports) if reports else 'No Failures'

    if failures:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif warnings:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    return text, stoplight


class TetherNodePowerIndicator(BaseTetherNodeStatusFlagsIndicator):

  def __init__(self, name, node_name):
    super(TetherNodePowerIndicator, self).__init__(
        name, node_name, ['PowerGood'])

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, node_status, valid):
    activated_flags = self._ActivatedFlags(node_status)
    if activated_flags:
      text = 'Power Bad'
      stoplight = stoplights.STOPLIGHT_ERROR
    else:
      text = 'Power Good'
      stoplight = stoplights.STOPLIGHT_NORMAL
    return text, stoplight


class BaseNodeStatusSummary(BaseTetherAllNodeStatusIndicator):
  """Summary indicator for node status."""

  def __init__(self, name, field, normal_ranges, warning_ranges, char_limit):
    super(BaseNodeStatusSummary, self).__init__(name)
    self._normal_ranges = check_range.BuildRanges(normal_ranges)
    self._warning_ranges = check_range.BuildRanges(warning_ranges)
    self._char_limit = char_limit
    self._field = field

  def _SetMaxStatusValue(self, node_name, status_value, max_status_value,
                         node_with_max_value, category):
    if status_value > max_status_value[category]:
      max_status_value[category] = status_value
      node_with_max_value[category] = node_name

  def _DisplaySummary(self, max_status_value, node_with_max_value, category,
                      nodes, char_limit):
    if node_with_max_value[category] is None:
      return '--'

    text = 'Max: %.0f (%s)' % (
        max_status_value[category], node_with_max_value[category])

    if nodes:
      char_limit -= len(text) + 3
      catalog = common.TimeCycle(
          [n for n in nodes if n != node_with_max_value[category]],
          char_limit, ',')
      text += ' | ' + catalog

    return text

  def _Filter(self, node_status, valid):
    status_values = {}
    warnings = set()
    errors = set()
    any_value = False
    max_status_value = {t: float('-inf')
                        for t in ['error', 'warning', 'normal']}
    node_with_max_value = {t: None for t in ['error', 'warning', 'normal']}
    for node_name in _WING_TMS570_NODES:
      aio_node_id = _AIO_NODE_HELPER.Value(node_name)
      if valid[aio_node_id]:
        any_value = True
        status_value = getattr(node_status[aio_node_id], self._field)
        status_values[node_name] = status_value
        if status_value not in self._normal_ranges:
          if status_value not in self._warning_ranges:
            errors.add(node_name)
            self._SetMaxStatusValue(node_name, status_value, max_status_value,
                                    node_with_max_value, 'error')
          else:
            warnings.add(node_name)
            self._SetMaxStatusValue(node_name, status_value, max_status_value,
                                    node_with_max_value, 'warning')
        else:
          self._SetMaxStatusValue(node_name, status_value, max_status_value,
                                  node_with_max_value, 'normal')

    if not any_value:
      text = '--'
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif errors:
      text = self._DisplaySummary(
          max_status_value, node_with_max_value, 'error', errors,
          self._char_limit)
      stoplight = stoplights.STOPLIGHT_ERROR
    elif warnings:
      text = self._DisplaySummary(
          max_status_value, node_with_max_value, 'warning', warnings,
          self._char_limit)
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      text = self._DisplaySummary(max_status_value, node_with_max_value,
                                  'normal', None, self._char_limit)
      stoplight = stoplights.STOPLIGHT_NORMAL

    return text, stoplight


class TetherMaxBoardTemperature(BaseNodeStatusSummary):
  """Indicator for board temperature."""

  def __init__(self, name, normal_ranges, warning_ranges, char_limit):
    super(TetherMaxBoardTemperature, self).__init__(
        name, 'board_temp', normal_ranges, warning_ranges, char_limit)


class TetherMaxBoardHumidity(BaseNodeStatusSummary):
  """Indicator for board humidity."""

  def __init__(self, name, normal_ranges, warning_ranges, char_limit):
    super(TetherMaxBoardHumidity, self).__init__(
        name, 'board_humidity', normal_ranges, warning_ranges, char_limit)


class BaseNodeStatusFlagsSummary(BaseTetherAllNodeStatusIndicator):
  """Summary indicator for node status."""

  _MAX_CHARS = 20

  def __init__(self, name, flag_names, normal_if_set, is_error,
               merge_flags=True, nodes_to_exclude=None):
    super(BaseNodeStatusFlagsSummary, self).__init__(name)
    self._flag_names = flag_names
    self._normal_if_set = normal_if_set
    self._is_error = is_error
    self._merge_flags = merge_flags
    self._nodes_to_exclude = nodes_to_exclude

  def _Filter(self, node_status, valid):
    if self._merge_flags:
      bad_nodes = set()
    else:
      bad_nodes = collections.defaultdict(list)
    error = False
    warning = False
    any_value = False

    for node_name in _WING_TMS570_NODES:
      if self._nodes_to_exclude and node_name in self._nodes_to_exclude:
        continue
      aio_node_id = _AIO_NODE_HELPER.Value(node_name)
      if valid[aio_node_id]:
        any_value = True
        flags = node_status[aio_node_id].flags
        num_caught = 0
        for flag_name in self._flag_names:
          if (bool(flags & _TETHER_NODE_FLAGS_HELPER.Value(flag_name)) !=
              self._normal_if_set):
            num_caught += 1
            if self._merge_flags:
              bad_nodes.add(node_name)
            else:
              bad_nodes[flag_name].append(node_name)

        if self._is_error is None:
          if num_caught == len(self._flag_names):
            error = True
          elif num_caught:
            warning = True
        elif self._is_error:
          error |= bool(num_caught)
        else:
          warning |= bool(num_caught)

    if not any_value:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif error:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif warning:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    if bad_nodes:
      if self._merge_flags:
        return common.TimeCycle(sorted(bad_nodes), self._MAX_CHARS), stoplight
      else:
        return self._ShowDetailedReport(bad_nodes), stoplight
    elif any_value:
      return 'Normal', stoplight
    else:
      return '--', stoplight

  def _ShowDetailedReport(self, bad_nodes):
    lines = [
        flag + ': ' + common.TimeCycle(bad_nodes[flag], self._MAX_CHARS)
        for flag in sorted(bad_nodes)]
    return '\n'.join(lines)


class TetherNodePowerSummary(BaseNodeStatusFlagsSummary):

  def __init__(self, name):
    super(TetherNodePowerSummary, self).__init__(
        'Power', ['PowerGood'], normal_if_set=False, is_error=True)


class TetherNodeNetworkSummary(BaseNodeStatusFlagsSummary):

  def __init__(self, name):
    super(TetherNodeNetworkSummary, self).__init__(
        'Network', ['NetworkAGood', 'NetworkBGood'],
        normal_if_set=True, is_error=None, merge_flags=False,
        nodes_to_exclude=common.NETWORK_STATUS_NODES_TO_EXCLUDE)

  def _ShowDetailedReport(self, bad_nodes):
    keys = {'NetworkAGood': 'A', 'NetworkBGood': 'B'}
    lines = [
        keys[flag] + ': ' +
        common.TimeCycle(bad_nodes[flag], self._MAX_CHARS)
        for flag in sorted(bad_nodes)]
    return '\n'.join(lines)


class TetherNodeSelfTestSummary(BaseNodeStatusFlagsSummary):

  def __init__(self, name):
    super(TetherNodeSelfTestSummary, self).__init__(
        'Self Test', ['SelfTestFailure'], normal_if_set=False, is_error=True)


class TetherNodeErrorSummary(BaseNodeStatusFlagsSummary):

  def __init__(self, name):
    nodes_to_exclude = []
    if _SYSTEM_PARAMS.wing_serial == system_types.kWingSerial01:
      # Eliminate MVLV from error reporting since 4 of its temperature sensors
      # are not reliable due to common mode noise in Gin configuration (SN01)
      nodes_to_exclude = ['Mvlv']
    super(TetherNodeErrorSummary, self).__init__(
        'Error', ['AnyError'], normal_if_set=False, is_error=True,
        nodes_to_exclude=nodes_to_exclude)


class TetherNodeWarningSummary(BaseNodeStatusFlagsSummary):

  def __init__(self, name):
    super(TetherNodeWarningSummary, self).__init__(
        'Warning', ['AnyWarning'], normal_if_set=False, is_error=False)

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

"""Common varibles for indicators and layouts."""
import collections
import datetime

from makani.analysis.checks import avionics_util
from makani.avionics.common import pack_avionics_messages as avionics_messages
from makani.avionics.network import aio_node
from makani.common.c_math import util as c_util
from makani.control import control_types
from makani.control import system_params
from makani.gs.monitor2.apps.layout import stoplights
from makani.lib.python import c_helpers
import numpy

_AIO_NODE_HELPER = c_helpers.EnumHelper('AioNode', aio_node)

_SYSTEM_PARAMS = system_params.GetSystemParams().contents

FULL_COMMS_MODE = 0
SPARSE_COMMS_MODE = 1

MAX_NO_UPDATE_COUNT_BATT_STATUS = 32
MAX_NO_UPDATE_COUNT_COMMS_STATUS = 32
MAX_NO_UPDATE_COUNT_CONTROLLER_COMMAND = 32
MAX_NO_UPDATE_COUNT_CONTROL_TELEMETRY = 32
MAX_NO_UPDATE_COUNT_GPS_STATUS = 32
MAX_NO_UPDATE_COUNT_MOTOR_STATUS = 20
MAX_NO_UPDATE_COUNT_SERVO_STATUS = 20
MAX_NO_UPDATE_COUNT_TETHER_RELEASE = 32

NETWORK_STATUS_NODES_TO_EXCLUDE = ['CsA', 'CsB']


def IsWingTms570Node(node_name):
  node_idx = _AIO_NODE_HELPER.Value(node_name)
  return aio_node.IsWingNode(node_idx) and aio_node.IsTms570Node(node_idx)


def WindAziToWindG(azimuth):
  """Convert wind azimuth to wind direction in ground frame.

  Args:
    azimuth: Wind azimuth [rad]

  Returns:
    Where wind comes from in ground frame.
  """
  gs_params = _SYSTEM_PARAMS.ground_station
  # phi = azimuth                 : Wind azimuth is where wind blows towards.
  # phi_g = phi + azi_ref_offset  : Convert it to ground frame.
  # wind_direction = phi_g - pi : Convert it to where wind comes from.
  return c_util.Wrap(
      azimuth + gs_params.azi_ref_offset - numpy.pi,
      0, numpy.pi * 2.0)


def WindAziToWindNed(azimuth):
  """Convert wind azimuth to wind direction in NED.

  Args:
    azimuth: Wind azimuth [rad]

  Returns:
    Where wind comes from in NED.
  """
  ground_frame_params = _SYSTEM_PARAMS.ground_frame
  wind_dir_g = WindAziToWindG(azimuth)
  return c_util.Wrap(
      wind_dir_g + ground_frame_params.heading, 0.0, numpy.pi * 2.0)


def WingTms570Nodes():
  return [node_name for node_name in _AIO_NODE_HELPER.ShortNames()
          if IsWingTms570Node(node_name)]


# TODO: Rewrite using range abstractions from analysis/checks.
def IsInTargetRange(
    target, error_ref, warning_percentage, error_percentage, value):
  """Test if a value falls into warning or error range around a target.

  Args:
    target: The target value.
    error_ref: The error reference as the base for warning/error percentage.
    warning_percentage: The percentage of error_ref to draw warning bounds.
    error_percentage: The percentage of error_ref to draw error bounds.
    value: The value of the variable.

  Returns:
    0 for normal, 1 for warning, and 2 or error.
  """
  warning_lower_bound = target - error_ref * warning_percentage
  warning_upper_bound = target + error_ref * warning_percentage
  error_lower_bound = target - error_ref * error_percentage
  error_upper_bound = target + error_ref * error_percentage

  if warning_lower_bound <= value <= warning_upper_bound:
    return 0
  elif error_lower_bound <= value <= error_upper_bound:
    return 1
  else:
    return 2


def TimeCycle(items, char_limit, delimiter=','):
  """Return a string that time-cycles through items given the char limit."""
  groups = []
  current_group = []
  current_length = 0
  item_idx = 0
  while item_idx < len(items):
    item = items[item_idx]
    if current_length + len(item) < char_limit:
      current_group.append(item)
      current_length += len(item) + len(delimiter)
      item_idx += 1
    elif current_length > 0:
      groups.append(current_group)
      current_length = 0
      current_group = []
    else:
      groups.append([item])
      item_idx += 1
  if current_length:
    groups.append(current_group)

  if not groups:
    return ''
  num_groups = len(groups)
  idx = datetime.datetime.now().second % num_groups
  return delimiter.join(groups[idx])


def CheckFlags(node_name, report_per_node, warnings, errors,
               flags, warning_helper, error_helper):
  """Check the status flags in each node and bookkeep the results.

  Args:
    node_name: Short name of the node.
    report_per_node: Structure to record warning/error messages per node.
        Its type should be collections.defaultdict(list).
    warnings: Structure to record nodes that raise each warning type.
        Its type should be collections.defaultdict(list).
    errors: Structure to record nodes that raise each error type.
        Its type should be collections.defaultdict(list).
    flags: The status flags to check against.
    warning_helper: The EnumHelper for warnings.
    error_helper: The EnumHelper for errors.

  Returns:
    True if there are any warnings/errors.
  """

  any_warning_or_error = False
  if warning_helper:
    for warning_value in warning_helper.Values():
      warning_name = warning_helper.ShortName(warning_value)
      if avionics_util.CheckWarning(flags, warning_value):
        report_per_node[node_name].append(('WARNING', warning_name))
        warnings[warning_name].append(node_name)
        any_warning_or_error = True
  if error_helper:
    for error_value in error_helper.Values():
      error_name = error_helper.ShortName(error_value)
      if avionics_util.CheckError(flags, error_value):
        report_per_node[node_name].append(('ERROR', error_name))
        errors[error_name].append(node_name)
        any_warning_or_error = True
  return any_warning_or_error


def _CappedListToStr(sequence, max_count):
  if len(sequence) <= max_count:
    return ','.join(sequence)
  else:
    truncated = sequence[:max_count]
    return ','.join(truncated) + ', ...'


def _CapStr(string, max_chars):
  if len(string) > max_chars:
    return string[:max_chars]
  return string


_MAX_NODES_PER_REPORT = 4
_MAX_WARNINGS_OR_ERRORS_PER_SERVO = 2
_MAX_FAULTY_NODES_TO_SUMMARIZE = 4
_MAX_REPORT_CATEGORIES_TO_SUMMARIZE = 2


def InvertDictOfLists(report_per_node):
  """Invert a dict of lists so that values in the lists become keys."""
  results = collections.defaultdict(list)
  for node, values in report_per_node.iteritems():
    for value in values:
      results[value].append(node)
  return results


def SummarizeWarningsAndErrors(
    any_node, report_per_node, warnings, errors, any_warning_or_error,
    node_prefix_len=0, max_chars_per_line=30, force_time_cycle=True,
    num_lines=None):
  """Produce a string summarizing the warnings and errors for each node."""

  def PadLineBreaks(text, line_count):
    if line_count is not None:
      current_lines = text.count('\n') + 1
      # Padding one more line break because HTML ignores the last trailing '\n'.
      if current_lines < line_count + 1:
        text += '\n' * (line_count + 1 - current_lines)
    return text

  def OneReport(report):
    if isinstance(report, tuple):
      if len(report) == 2:
        return '(%s)%s' % (report[0], report[1])
      else:
        assert False
    else:
      return report

  def MultiLineReport(report, max_chars_per_line):
    if isinstance(report, tuple):
      if len(report) == 2:
        return '[%s]\n%s' % (_CapStr(report[0], max_chars_per_line),
                             _CapStr(report[1], max_chars_per_line))
      else:
        assert False
    else:
      return report

  if not any_node:
    return PadLineBreaks('--', num_lines), stoplights.STOPLIGHT_UNAVAILABLE

  elif any_warning_or_error:
    # If there are too many errors or warning, show one each second.
    # Otherwise, display by either category or node, whichever appears
    # more concise.
    text = []
    num_report_categories = len(warnings) + len(errors)
    if (num_report_categories > _MAX_REPORT_CATEGORIES_TO_SUMMARIZE or
        len(report_per_node) > _MAX_FAULTY_NODES_TO_SUMMARIZE or
        force_time_cycle):
      num_reports = sum(len(report) for report in report_per_node.values())
      idx = datetime.datetime.now().second % num_reports
      for node in sorted(report_per_node):
        reports = report_per_node[node]
        if idx >= len(reports):
          idx -= len(reports)
        else:
          report = reports[idx]
          node_name = node[node_prefix_len:]
          text.append('%s:\n%s' % (_CapStr(node_name, max_chars_per_line),
                                   MultiLineReport(report, max_chars_per_line)))
          break
    elif num_report_categories < len(report_per_node):
      # Print by category.
      for category, details in [('WARNING', warnings), ('ERROR', errors)]:
        for name in sorted(details):
          nodes = details[name]
          if node_prefix_len:
            nodes = [n[node_prefix_len:] for n in nodes]
          text.append('%s %s: %s' % (
              category, name,
              _CappedListToStr(nodes, _MAX_NODES_PER_REPORT)))
    else:
      # Print by node.
      for node in sorted(report_per_node):
        text.append(
            '%s: %s' % (
                node[node_prefix_len:], _CappedListToStr(
                    [OneReport(x) for x in report_per_node[node]],
                    _MAX_WARNINGS_OR_ERRORS_PER_SERVO)))
    if errors:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif warnings:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL
    return PadLineBreaks('\n'.join(text), num_lines), stoplight
  else:
    return PadLineBreaks('Normal', num_lines), stoplights.STOPLIGHT_NORMAL


def IsActuatorStateArmed(state):
  return state in (avionics_messages.kActuatorStateArmed,
                   avionics_messages.kActuatorStateRunning)


def IsActuatorStateError(state):
  return state == avionics_messages.kActuatorStateError


def FillLines(lines, num_lines):
  lines = lines[:num_lines]
  if len(lines) < num_lines:
    lines += [''] * (num_lines - len(lines))
  return '\n'.join(lines)


class DurationWatcher(object):
  """Class to check if a value has fallen in a given range for N seconds."""

  def __init__(self, bounds, duration):
    self._bounds = bounds
    self._duration = duration
    self._start_time = None

  def _IsMet(self, value):
    return self._bounds[0] <= value <= self._bounds[1]

  def Check(self, value):
    """Return true if value falls within bounds for over [duration] seconds."""
    if not self._IsMet(value):
      self._start_time = None
      return False

    if self._start_time is None:
      self._start_time = datetime.datetime.now()

    elapse = datetime.datetime.now() - self._start_time
    return elapse.total_seconds() > self._duration


def AnyDetwistFlightMode(flight_mode):
  return flight_mode in [
      control_types.kFlightModeHoverFullLength,
      control_types.kFlightModeHoverAccel,
      control_types.kFlightModeTransIn,
      control_types.kFlightModeCrosswindNormal,
      control_types.kFlightModeCrosswindPrepTransOut,
      control_types.kFlightModeHoverTransOut,
      control_types.kFlightModeHoverPrepTransformGsDown,
  ]

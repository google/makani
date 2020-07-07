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

"""Utilities to set stoplight color."""

from makani.analysis.checks import check_range
from makani.gs.monitor2.apps.layout import checks

# Nothing is checked so any value is fine.
STOPLIGHT_ANY = 4
# Indicates that values are OK.
STOPLIGHT_NORMAL = 3
# Indicates that values should raise warnings.
STOPLIGHT_WARNING = 2
# Indicates that values should raise errors.
# This ususally means flight should end.
STOPLIGHT_ERROR = 1
# Indicates that values are not available.
STOPLIGHT_UNAVAILABLE = 0


def Set(assignments, callback, *args):
  """Set the stoplight using assignments and a condition callback.

  Args:
    assignments: A dict mapping returns of the callback to the stoplight value.
    callback: The callback to compute the conditional assignment.
    *args: Input arguments to the callback functions.

  Example:
    Set({1: STOPLIGHT_NORMAL, -1: STOPLIGHT_WARNING, 0: STOPLIGHT_UNAVAILABLE},
        callback)

  Returns:
    Value of the stoplight.

  Raises:
    KeyError: Raised if assignment is invalid.
  """

  key = callback(*args)
  if key not in assignments:
    raise KeyError("Cannot determine stoplight due to invalid assignment.")
  return assignments[key]


def SetByAioUpdate(messages, message_type, source, stoplight_if_updated,
                   stoplight_if_stale, stoplight_if_aio_unavailable):
  """Set a stoplight based on AIO message availability.

  Args:
    messages: The full StructTree of available data. If empty, then data
        is unavailable.
    message_type: The specific AIO message type being requested. If None,
        the function checks whether AIO messages of any type has been updated.
    source: The specific AIO node being requested. If None, the function checks
        whether AIO messages from any node has been updated.
    stoplight_if_updated: The stoplight when message is freshly available.
    stoplight_if_stale: The stoplight when the message is stale.
    stoplight_if_aio_unavailable: The stoplight when AIO is not available.

  Returns:
    The stoplight indicating the AIO update status.
  """
  if messages:
    index_str = "[:]" if message_type is None else message_type
    index_str += "[:]" if source is None else "." + source
    data = messages[index_str]
  else:
    data = None
  return Set({
      1: stoplight_if_updated,
      -1: stoplight_if_stale,
      0: stoplight_if_aio_unavailable,
  }, checks.CheckForExistence, data, messages)


def SetByDictValues(dictionary, expected_values, stoplight_if_partial,
                    stoplight_if_fail):
  """Set a stoplight based on whether all the values in a dict are expected.

  Args:
    dictionary: A dictionary whose values need to be checked.
    expected_values: All possible values expected for this dictionary.
    stoplight_if_partial: Stoplight if not all values are expected.
    stoplight_if_fail: Stoplight if no values are expected.

  Returns:
    Stoplight indicating the check result.
  """

  passed_results = {key for key, value in dictionary.iteritems()
                    if value in expected_values}
  return checks.CheckForSize(passed_results, len(dictionary), STOPLIGHT_NORMAL,
                             stoplight_if_partial, stoplight_if_fail)


def MostSevereStoplight(*stoplights):
  return min(stoplights)


def SetByRanges(value, normal_ranges, warning_ranges, error_ranges=None):
  if value in normal_ranges:
    return STOPLIGHT_NORMAL
  elif value in warning_ranges:
    return STOPLIGHT_WARNING
  elif error_ranges is None or value in error_ranges:
    return STOPLIGHT_ERROR
  else:
    return STOPLIGHT_UNAVAILABLE


def SetByLimits(value, limits):
  return SetByRanges(value, limits["normal"], limits["warning"])


def SetByStoplightLimits(value, limits):
  return SetByRanges(value, check_range.Interval([limits.low, limits.high]),
                     check_range.Interval([limits.very_low, limits.very_high]))

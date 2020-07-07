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

"""Utilities to inspect values for anomaly."""

import re

from makani.analysis.checks import check_range
from makani.gs.monitor2.apps.layout import stoplights
import numpy


def _StoplightByRange(value, normal_ranges, warning_ranges):
  """Determine the stoplight according to what range the value falls into."""
  if isinstance(normal_ranges, check_range.AllInclusiveRange):
    return stoplights.STOPLIGHT_ANY

  if value in normal_ranges:
    return stoplights.STOPLIGHT_NORMAL
  elif value in warning_ranges:
    return stoplights.STOPLIGHT_WARNING
  else:
    return stoplights.STOPLIGHT_ERROR


def _GetAioNodeAndMessageNameFromField(field_path):
  match = re.search(r'(?P<message>\w+)\.(?P<aio_node>\w+)\.', field_path)
  if match is not None:
    return match.group('aio_node'), match.group('message')
  else:
    return None, None


def TetherDownTimestampAttributePath():
  return ('filtered', 'merge_tether_down', 'timestamp_sec')


def AioTimestampAttributePath(message_name, aio_node):
  return (message_name, aio_node, 'capture_info.timestamp')


def AioTimestamp(data, message_name, aio_node):
  if message_name == 'filtered' and aio_node == 'merge_tether_down':
    return data['filtered.merge_tether_down.timestamp_sec']
  else:
    return data['%s.%s.capture_info.timestamp' % (message_name, aio_node)]


def AioTimestampFromField(data, field):
  aio_node, message_name = _GetAioNodeAndMessageNameFromField(field)
  return AioTimestamp(data, message_name, aio_node)


def CheckByRange(value, normal_ranges, warning_ranges, name):
  """Check the value of a given field and determine its stoplight.

  Args:
    value: Value of the field.
    normal_ranges: A BaseRange object in which values are regarded as normal.
        It can also be a list of basic objects that create BaseRange objects.
        Please refer to check_range.RangeChecker for details.
    warning_ranges: A BaseRange object in which values should raise warnings,
        if they fall out of normal_ranges. Values falling out of
        warning_ranges should raise errors.
        It can also be a list of basic objects that create BaseRange objects.
        Please refer to check_range.RangeChecker for details.
    name: Name of the check.

  Returns:
    A list of dicts each consists of name, value, as well as
    their stoplights. E.g., [
        {
            'name': 'MotorStatus.MotorPbi.status',
            'value': ...,
            'stoplight': stoplight,
        }
    ]
  """
  if value is None or (isinstance(value, numpy.ndarray) and value.size == 0):
    return []

  normal_ranges = check_range.BuildRanges(normal_ranges)
  warning_ranges = check_range.BuildRanges(warning_ranges)
  data = [{
      'name': name,
      'value': value,
      'stoplight': _StoplightByRange(value, normal_ranges, warning_ranges),
  }]
  return data


def CheckForFailure(values, target_ranges, error_if_fail, name):
  """Checking values against normal ranges.

  Args:
    values: Values of the field.
    target_ranges: A BaseRange object in which values are regarded as normal.
        It can also be a list of basic objects that create BaseRange objects.
        Please refer to check_range.RangeChecker for details.
    error_if_fail: True if failed values raise errors,
        otherwise they raise warnings.
    name: Name of the check.

  Returns:
    Check results in the form of {
        warning_name: {
            'total': ...,  # Total number of samples.
            'count': ...,  # Number of samples in this range.
            'range': [min_value, max_value],
        },
        error_name: {...}
    }
  """
  if error_if_fail:
    return CheckByRange(values, target_ranges, check_range.AllExclusiveRange(),
                        name)
  else:
    return CheckByRange(values, target_ranges, check_range.AllInclusiveRange(),
                        name)

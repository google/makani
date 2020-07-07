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

"""Utilities to automatically generates a monitor layout."""

from makani.gs.monitor2.apps.layout import widgets
from makani.gs.monitor2.project import settings


def GenerateScenario(obj, name):
  """Automatically generates a monitor layout according to a JSON object.

  Args:
    obj: A JSON object of the data struct to be visualized.
    name: Name of the layout.

  Returns:
    A JSON object about the monitor layout to visualize the data.
  """

  template = {
      'title': name,
      'signals': {},
      'filters': [],
      'canvas': {
          'grid_width': settings.CSS_GRID_COLUMNS,
          'row_height_px': 40
      },
      'views': []
  }
  stripe = []
  for message_type in sorted(obj):
    messages = obj[message_type]
    for aio_node in sorted(messages):
      details = messages[aio_node]
      prefix = '%s.%s' % (message_type, aio_node)
      view = {
          'name': prefix,
          'grid_width': settings.CSS_GRID_COLUMNS,
          'rows': 3,
          'indicators': _GetIndicators(prefix, details),
      }
      stripe.append(view)
  template['views'].append(
      {'stripe': stripe, 'grid_width': settings.CSS_GRID_COLUMNS})
  return template


def _IsLeafList(obj):
  """Return True if `obj` is a list of non-compound values.

  Since we are traversing a C structure, we assume elements in its list-type
  member variable are homogeneous, therefore we only test for its first element.

  Args:
    obj: The list to check.

  Returns:
    True if `obj` is a list of non-compound values.
  """

  if not obj:
    return True
  else:
    return not isinstance(obj[0], (dict, list, set, tuple))


def _GetIndicators(name, obj):
  """Recursively traverse A JSON object to populate indicators."""
  indicators = []
  if isinstance(obj, dict):
    for key in sorted(obj):
      value = obj[key]
      if name:
        kid_name = '%s.%s' % (name, key)
      else:
        kid_name = key
      indicators += _GetIndicators(kid_name, value)
  elif isinstance(obj, list):
    if _IsLeafList(obj):
      indicators += _AutoGenVectorIndicator(name, obj)
    else:
      for idx, item in enumerate(obj):
        kid_name = '%s.%d' % (name, idx)
        indicators += _GetIndicators(kid_name, item)
  else:
    indicators += _AutoGenScalarIndicator(name)
  return indicators


def _GetTimeStampIdx(name):
  items = name.split('.')
  if len(items) >= 2:
    return '%s.%s.capture_info.timestamp' % (items[0], items[1])
  else:
    return 'timestamp'


def _RemoveMessageTypeAioNodeFromNames(name):
  items = name.split('.')
  return '.'.join(items[2:])


def _AutoGenVectorIndicator(name, obj):
  """Generate indicators for a vector."""
  keys = ['[%d]' % i for i in range(len(obj))]
  indicator_src = None
  widget = widgets.ListIndicatorWidget(
      _RemoveMessageTypeAioNodeFromNames(name), keys, name, indicator_src)
  return widget.Indicators()


def _AutoGenScalarIndicator(name):
  """Generate an indicator for a scalar."""
  indicator_src = None
  widget = widgets.ScalarIndicatorWidget(
      _RemoveMessageTypeAioNodeFromNames(name), name, indicator_src)
  return widget.Indicators()

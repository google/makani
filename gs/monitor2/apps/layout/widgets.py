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

"""Widgets used to visualize data.

These widgets are used by apps.layout.base.AssembleLayout to generate
specifications for layout views.
"""

from makani.gs.monitor2.project import settings


class BaseWidget(object):
  """Base widget class."""

  def __init__(self):
    self._indicators = []

  def SetCols(self, cols):
    for indicator in self._indicators:
      # Minus one to allow some margin.
      indicator['cols'] = cols

  def SetRows(self, rows):
    for indicator in self._indicators:
      indicator['rows'] = rows

  def Indicators(self):
    return self._indicators


class DictIndicatorWidget(BaseWidget):
  """A widget to visualize a dict of values as a table."""

  def __init__(self, name, value_src, indicator_src,
               precision=None, keys=None, message_src=None,
               mode='horizontal', font_size=None):
    super(DictIndicatorWidget, self).__init__()
    self._indicators = [{
        'template': 'DictIndicator',
        'precision': precision,
        'keys': keys,
        'mode': mode,
        'cols': settings.CSS_GRID_COLUMNS,
        'name': name,
        'src': value_src,
        'indicator_src': indicator_src,
        'message_src': message_src,
        'font_size': font_size,
    }]


class ListIndicatorWidget(BaseWidget):
  """A widget to visualize a list of values as a table."""

  def __init__(self, name, keys, value_src, indicator_src,
               precision=None, message_src=None,
               mode='horizontal', font_size=None):
    super(ListIndicatorWidget, self).__init__()
    self._indicators = [{
        'template': 'ListIndicator',
        'mode': mode,
        'cols': settings.CSS_GRID_COLUMNS,
        'name': name,
        'keys': keys,
        'src': value_src,
        'indicator_src': indicator_src,
        'message_src': message_src,
        'precision': precision,
        'font_size': font_size,
    }]


class ScalarIndicatorWidget(BaseWidget):
  """A widget to visualize a scalar as a key:value pair."""

  def __init__(self, name, value_src, indicator_src, precision=None,
               font_size=None):
    super(ScalarIndicatorWidget, self).__init__()
    self._indicators = [{
        'template': 'ScalarIndicator',
        'cols': settings.CSS_GRID_COLUMNS,
        'mode': 'horizontal',
        'name': name,
        'src': value_src,
        'indicator_src': indicator_src,
        'precision': precision,
        'font_size': font_size,
    }]


class ScatterWidget(BaseWidget):
  """A widget to visualize scattered points."""

  def __init__(self, x_src, y_src):
    super(ScatterWidget, self).__init__()
    self._indicators = [
        {
            'template': 'Scatter',
            'cols': settings.CSS_GRID_COLUMNS,
            'rows': 40,
            'x_attrs': {
                'scale': 'linear',
                'domain': [0, 200],
                'src': x_src,
            },
            'y_attrs': {
                'domain': [-30, 30],
                'scale': 'linear',
                'src': y_src,
            }
        }
    ]


class DictLinesWidget(BaseWidget):
  """A widget to visualize a dictionary of {x, y} sequences."""

  def __init__(self, series_src, keys, interactive=False, aspect_ratio=1.8,
               use_markers=True):
    super(DictLinesWidget, self).__init__()
    self._indicators = [
        {
            'template': 'LineChartFromDict',
            'keys': keys,
            'cols': settings.CSS_GRID_COLUMNS,
            'rows': 40,
            'series_src': series_src,
            'interactive': interactive,
            'use_markers': use_markers,
            'x_attrs': {
                'scale': 'linear',
            },
            'y_attrs': {
                'scale': 'linear',
            },
            'aspect_ratio': aspect_ratio
        }
    ]


class DictChartWidget(BaseWidget):
  """A widget to visualize a dict of values in the form of a 2D plot."""

  def __init__(self, name, value_src, series_srcs, xlabel, ylabel, xlim, ylim,
               polygons, history_len, line_properties, indicator_src,
               show_legend=True, dynamic_polygons=None, aspect_ratio=1.0,
               precision=None, keys=None, secondary_keys=None, message_src=None,
               heading=None, panel_ratio=0.33, font_size=None, **kwargs):
    super(DictChartWidget, self).__init__()
    rows = max(len(keys) + 4, 10)
    self._panel_ratio = panel_ratio
    self._has_panel = bool(keys or secondary_keys)  # If no keys, no panel.
    self._indicators = []
    if self._has_panel:
      self._indicators.append({
          'template': 'DictIndicator',
          'mode': 'vertical',
          'precision': precision,
          'heading': heading,
          'keys': keys,
          'secondary_keys': secondary_keys,
          'rows': rows,
          'name': name,
          'src': value_src,
          'indicator_src': indicator_src,
          'message_src': message_src,
          'font_size': font_size,
      })

    self._indicators.append({
        'title': kwargs.get('title', None),
        'template': 'Sketch2DWindow',
        'dynamic_polygons': dynamic_polygons,
        'polygons': polygons,
        'cols': settings.CSS_GRID_COLUMNS,
        'rows': rows,
        'history_len': history_len,
        'series_srcs': series_srcs,
        'interactive': kwargs.get('interactive', False),
        'line_properties': line_properties,
        'show_legend': show_legend,
        'x_attrs': {
            'scale': 'linear',
            'domain': xlim,
            'label': xlabel,
            'num_ticks': kwargs.get('num_xticks', None),
            'invert': kwargs.get('invert_xaxis', False),
        },
        'y_attrs': {
            'scale': 'linear',
            'domain': ylim,
            'label': ylabel,
            'num_ticks': kwargs.get('num_yticks', None),
            'invert': kwargs.get('invert_yaxis', False),
        },
        'aspect_ratio': aspect_ratio,
    })
    self.SetCols(settings.CSS_GRID_COLUMNS)

  def SetCols(self, cols):
    if self._has_panel:
      panel_cols = max(1, int(self._panel_ratio * cols + 0.5))
      self._indicators[0]['cols'] = panel_cols
      self._indicators[1]['cols'] = cols - panel_cols
    else:
      self._indicators[0]['cols'] = cols


class DictTrailsWidget(BaseWidget):
  """A widget to visualize a dict of values in the form of strip charts."""

  def __init__(self, name, x_value_src, y_value_src, indicator_src,
               precision=None, keys=None, secondary_keys=None, chart_keys=None,
               trail_length=80, message_src=None, heading=None,
               interactive=False, aspect_ratio=1.8,
               has_panel=True, panel_ratio=0.33, font_size=None, **kwargs):
    super(DictTrailsWidget, self).__init__()
    rows = max(len(keys) + 4, 10)
    self._panel_ratio = panel_ratio
    self._has_panel = has_panel
    self._indicators = []
    if self._has_panel:
      self._indicators.append({
          'template': 'DictIndicator',
          'mode': 'vertical',
          'precision': precision,
          'heading': heading,
          'keys': keys,
          'secondary_keys': secondary_keys,
          'rows': rows,
          'name': name,
          'src': y_value_src,
          'indicator_src': indicator_src,
          'message_src': message_src,
          'font_size': font_size,
      })

    if chart_keys:
      full_keys = chart_keys
    elif secondary_keys:
      assert keys
      full_keys = []
      for k in keys:
        for s in secondary_keys:
          full_keys.append(k + '.' + s)
    else:
      full_keys = keys

    self._indicators.append({
        'template': 'StripChartFromDict',
        'keys': full_keys,
        'rows': rows,
        'interactive': interactive,
        'x_attrs': {
            'scale': 'linear',
            'domain': [0, trail_length],
            'src': x_value_src,
            'label': kwargs.get('xlabel', None),
            'num_ticks': kwargs.get('num_xticks', None),
            'invert': kwargs.get('invert_xaxis', False),
        },
        'y_attrs': {
            'scale': 'linear',
            'src': y_value_src,
            'domain': kwargs.get('ylim', None),
            'label': kwargs.get('ylabel', None),
            'num_ticks': kwargs.get('num_yticks', None),
            'invert': kwargs.get('invert_yaxis', False),
        },
        'aspect_ratio': aspect_ratio,
    })
    self.SetCols(settings.CSS_GRID_COLUMNS)

  def SetCols(self, cols):
    if self._has_panel:
      panel_cols = max(1, int(self._panel_ratio * cols + 0.5))
      self._indicators[0]['cols'] = panel_cols
      self._indicators[1]['cols'] = cols - panel_cols
    else:
      self._indicators[0]['cols'] = cols


class ListTrailsWidget(BaseWidget):
  """A widget to visualize a list of values in the form of strip charts."""

  def __init__(self, name, keys, x_value_src, y_value_src, indicator_src,
               precision=None, trail_length=80, message_src=None,
               interactive=False, aspect_ratio=1.8,
               has_panel=True, panel_ratio=0.33, font_size=None, **kwargs):
    super(ListTrailsWidget, self).__init__()
    rows = max(len(keys) + 4, 10)
    self._panel_ratio = panel_ratio
    self._has_panel = has_panel
    self._indicators = []
    if self._has_panel:
      self._indicators.append({
          'template': 'ListIndicator',
          'mode': 'vertical',
          'precision': precision,
          'rows': rows,
          'indicator_src': indicator_src,
          'message_src': message_src,
          'name': name,
          'keys': keys,
          'src': y_value_src,
          'font_size': font_size,
      })
    self._indicators.append({
        'template': 'StripChartFromList',
        'rows': rows,
        'keys': keys,
        'indicator_src': indicator_src,
        'interactive': interactive,
        'x_attrs': {
            'scale': 'linear',
            'domain': [0, trail_length],
            'src': x_value_src,
            'label': kwargs.get('xlabel', None),
            'num_ticks': kwargs.get('num_xticks', None),
            'invert': kwargs.get('invert_xaxis', False),
        },
        'y_attrs': {
            'scale': 'linear',
            'src': y_value_src,
            'domain': kwargs.get('ylim', None),
            'label': kwargs.get('num_ylabel', None),
            'num_ticks': kwargs.get('num_yticks', None),
            'invert': kwargs.get('invert_yaxis', False),
        },
        'aspect_ratio': aspect_ratio,
    })
    self.SetCols(settings.CSS_GRID_COLUMNS)

  def SetCols(self, cols):
    if self._has_panel:
      panel_cols = max(1, int(self._panel_ratio * cols + 0.5))
      self._indicators[0]['cols'] = panel_cols
      self._indicators[1]['cols'] = cols - panel_cols
    else:
      self._indicators[0]['cols'] = cols


class AttributesWidget(BaseWidget):
  """A widget to visualize results with multiple attributes."""

  def __init__(self, name, src, precision=None):
    """Initialize the widget.

    Args:
      name: Name of the widget.
      src: Name of the data field.
      precision: Number of digits after the decimal.
    """
    super(AttributesWidget, self).__init__()
    self._indicators = [
        {
            'template': 'AttributesIndicator',
            'cols': settings.CSS_GRID_COLUMNS,
            'name': name,
            'precision': precision,
            'src': src,
        }
    ]


class Sketch2DWidget(BaseWidget):
  """A widget to visualize a dictionary of (x, y) sequences."""

  # TODO: Get rid of this widget class; replace with
  # DictChartWidgets with has_panel=False (keys and secondary_keys = None).

  def __init__(self, series_srcs, xlabel, ylabel, xlim, ylim, polygons,
               history_len, line_properties, marker_properties,
               show_legend=True, rows=16, dynamic_polygons=None, markers=None,
               aspect_ratio=1.8, **kwargs):
    super(Sketch2DWidget, self).__init__()
    self._indicators = [
        {
            'title': kwargs.get('title', None),
            'template': 'Sketch2DWindow',
            'dynamic_polygons': dynamic_polygons,
            'polygons': polygons,
            'markers': markers,
            'cols': settings.CSS_GRID_COLUMNS,
            'rows': rows,
            'history_len': history_len,
            'series_srcs': series_srcs,
            'interactive': kwargs.get('interactive', False),
            'line_properties': line_properties,
            'marker_properties': marker_properties,
            'show_legend': show_legend,
            'x_attrs': {
                'scale': 'linear',
                'domain': xlim,
                'label': xlabel,
                'num_ticks': kwargs.get('num_xticks', None),
                'invert': kwargs.get('invert_xaxis', False),
            },
            'y_attrs': {
                'scale': 'linear',
                'domain': ylim,
                'label': ylabel,
                'num_ticks': kwargs.get('num_yticks', None),
                'invert': kwargs.get('invert_yaxis', False),
            },
            'aspect_ratio': aspect_ratio,
        }
    ]


class TapeIndicator(BaseWidget):
  """A widget to visualize a dictionary of (x, y) sequences."""

  def __init__(self, title, color, position_src, annotation_src, **kwargs):
    super(TapeIndicator, self).__init__()
    self._indicators = [
        {
            'template': 'TapeIndicator',
            'title': title,
            'aspectRatio': kwargs.get('aspect_ratio', 1.8),
            'srcPosition': position_src,
            'srcAnnotation': annotation_src,
            'leftBar': {
                'color': color,
                'position': 0.0,
                'title': ''
            },
            # Example for kwargs['right_bars']:
            # [
            #     {
            #         'position': 0.08,
            #         'textBelow': 'STOP'
            #     },
            #     {
            #         'position': 0.4,
            #         'textAbove': 'Low',
            #         'textBelow': 'Very Low'
            #     },
            # ],
            'rightBars': kwargs.get('right_bars', []),
            'yLimits': [0.0, 1.0],
            # Example for kwargs['y_ticks']:
            # [
            #    {
            #        'position': 0.08,
            #        'label': '0.08 (Stop)'
            #    },
            #    {
            #        'position': 0.2,
            #        'label': '0.2'
            #    }
            #]
            'yTicks': kwargs.get('y_ticks', []),
        }
    ]

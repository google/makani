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

"""Base indicators class for modular plots and updates."""
import copy
import logging

from makani.analysis.checks import check_range
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.layout import widgets
from makani.lib.python import struct_tree
import numpy


class BaseIndicator(object):
  """Base class for indicators with local memory.

  The Filter() function computes data to update the indicator, including
  both values and stoplights. The returned variables are "used" as the list
  of arguments to Plot(), which feeds data to the visualization widgets.
  """

  # TODO: No private memory is supported in indicators at this time.

  def __init__(self, name=None):
    self._label = name

  def Name(self):
    return self._label

  def _GetLimits(self, param_limits):
    return {
        'normal': check_range.Interval(
            [param_limits.low, param_limits.high]),
        'warning': check_range.Interval(
            [param_limits.very_low, param_limits.very_high]),
    }

  def Plot(self, *args):
    """Plot the widget for this indicator."""
    return widgets.ScalarIndicatorWidget(self._label, *args)

  def Filter(self, messages):
    """Compute values used for the indicator.

    This function has to be overridden in a derived class.

    Args:
      messages: The StructTree object as a snapshot of all messages received.

    Returns:
      Objects to be passed to Plot() as arguments.
      The default Plot() function uses a scalar indicator, which
      needs two extra objects: data to show and the stoplight value.
    """

    raise NotImplementedError

  def _GetAvailableValues(self, messages, message_name, aio_nodes,
                          aio_node_prefix, field):
    """Helper function to retrieve the same field from multiple AIO nodes."""
    values = {}
    for aio_node in aio_nodes:
      value = messages['%s.%s%s.%s' % (
          message_name, aio_node_prefix, aio_node, field)]
      if value is not None:
        values[aio_node] = value
    return values


def ReturnCallbackOutputIfInputInvalid(callback):
  """Decorator that calls the _IsValidInput member method."""

  def Decorator(func):
    """Define the decorator's wrapper function."""
    def Wrapper(self, *attributes):
      if self._IsValidInput(*attributes):  # pylint: disable=protected-access
        return func(self, *attributes)
      else:
        return callback()
    return Wrapper

  return Decorator


def ReturnIfInputInvalid(*default_returns):
  """Decorator that calls the _IsValidInput member method."""

  def Decorator(func):
    """Define the decorator's wrapper function."""
    def Wrapper(self, *attributes):
      if self._IsValidInput(*attributes):  # pylint: disable=protected-access
        return func(self, *attributes)
      else:
        if len(default_returns) == 1:
          # If there is only one return value in the tuple, then return it.
          return default_returns[0]
        else:
          return default_returns
    return Wrapper

  return Decorator


class BaseAttributeIndicator(BaseIndicator):
  """A base indicator that depends on specific attributes in the messages."""

  def __init__(self, attribute_paths, name=None,
               missing_stoplight=stoplights.STOPLIGHT_UNAVAILABLE,
               default='--', **widget_kwargs):
    """Initialize the attribute indicator.

    Args:
      attribute_paths: A list of attribute paths. Each path is defined by
          [(<message_name>[, <aio_node>, <field_path>]), ...].
          If <aio_node> is None, then we will use one of the available source.
      name: Name of the indicator.
      missing_stoplight: Stoplight if not all attributes are available.
      default: The default output.
      **widget_kwargs: The kwargs for the widget.
    """
    super(BaseAttributeIndicator, self).__init__(name)
    self._attribute_paths = attribute_paths
    self._missing_stoplight = missing_stoplight
    self._default = default
    self._widget_kwargs = widget_kwargs

  def _GetCTypeFieldByString(self, obj, fields):
    for field in fields.split('.'):
      obj = getattr(obj, field)
    return obj

  def _Filter(self, *attributes):
    raise NotImplementedError

  def _IsValidInput(self, *attributes):
    """Whether the input is valid (Required for @ReturnIfInputInvalid)."""
    for attribute in attributes:
      if struct_tree.IsValidElement(attribute):
        return True
    return False

  def _GetAllAttributes(self, messages):
    """Obtain the attributes given messages.

    Args:
      messages: A StructTree object as the snapshot of all messages.

    Returns:
      The list of attributes corresponding to self._attribute_paths.

    Raises:
      ValueError: Raised if the attribute path is invalid.
    """
    attributes = []
    for path in self._attribute_paths:
      if len(path) == 3:
        # Obtain a specific field in a particular message from one source.
        message_name, aio_node, field_path = path
      elif len(path) == 2:
        # Obtain a specific message from one source.
        message_name, aio_node = path
        field_path = None
      elif len(path) == 1:
        # Obtain a specific message from all sources.
        attributes.append(messages.Subtree(path[0]))
        continue
      else:
        raise ValueError(
            'Attribute-depending indicator "%s" expects attribute paths to be '
            '(message_name[, aio_node[, field_path]])' % self._label)

      # Handle the special case where aio_node is set to None, meaning
      # "Pick any AIO node that sends this message type".
      if message_name != 'filtered' and aio_node is None:
        if message_name in messages:
          aio_node = min(messages.Data()[message_name].keys())
        else:
          attributes.append(None)
          continue
      assert aio_node and message_name

      # Construct the index to the attribute.
      parts = [message_name, aio_node]
      if field_path:
        parts.append(field_path)
      index = '.'.join(parts)

      attributes.append(messages[index])

    return attributes

  def Filter(self, messages):
    """Obtain the attribute from the messages and compute the data to show.

    Args:
      messages: The StructTree object as a snapshot of all messages received.

    Returns:
      The data to show and the stoplight value, as arguments to the
          scalar indicator.
    """
    if not messages:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE

    attributes = self._GetAllAttributes(messages)
    return self._Filter(*attributes)

  def Plot(self, *args):
    """Plot the widget for this indicator."""
    return widgets.ScalarIndicatorWidget(
        self._label, *args, **self._widget_kwargs)


class DictAttributeIndicator(BaseAttributeIndicator):
  """A base indicator that computes several attributes into a dict of values."""

  def __init__(self, attribute_paths, keys,
               name=None, missing_stoplight=stoplights.STOPLIGHT_UNAVAILABLE,
               default=None, precision=2, **widget_kwargs):
    super(DictAttributeIndicator, self).__init__(
        attribute_paths, name=name, missing_stoplight=missing_stoplight,
        default=default, **widget_kwargs)
    self._keys = keys
    self._precision = precision

  def Plot(self, *args):
    """Plot the widget for this indicator."""
    value_src, indicator_src = args
    return widgets.DictIndicatorWidget(
        self._label, value_src, indicator_src, self._precision,
        self._keys, **self._widget_kwargs)


class SingleAttributeIndicator(BaseAttributeIndicator):
  """A base indicator that depends on a single attribute in the messages."""

  def __init__(self, attribute_path, name=None, **base_kwargs):
    """Initialize the attribute indicator.

    Args:
      attribute_path: An attribute path defined by a tuple
          (<message_name>[, <aio_node>, <field_path>]). If <aio_node> is None,
          then we will use any of the available source nodes.
      name: Name of the indicator.
      **base_kwargs: Kwargs for the base class.
    """
    super(SingleAttributeIndicator, self).__init__(
        [attribute_path], name=name, **base_kwargs)

  def Filter(self, messages):
    """Obtain the attribute from the messages and compute the data to show.

    Args:
      messages: The StructTree object as a snapshot of all messages received.

    Returns:
      The data to show and the stoplight value, as arguments to the
          scalar indicator.
    """
    if not messages:
      return self._default, stoplights.STOPLIGHT_UNAVAILABLE

    attribute = self._GetAllAttributes(messages)[0]
    if not struct_tree.IsValidElement(attribute):
      return self._default, stoplights.STOPLIGHT_UNAVAILABLE

    return self._Filter(attribute)


class BaseAttributeChart(BaseAttributeIndicator):
  """A chart whose data depends on a specific attribute."""

  def __init__(self, attribute_paths, labels, name=None,
               missing_stoplight=stoplights.STOPLIGHT_UNAVAILABLE,
               **base_kwargs):
    """Initialize the object.

    Args:
      attribute_paths: A list of attribute paths. Each path is defined by
          [(<message_name>[, <aio_node>, <field_path>]), ...].
          If <aio_node> is None, then we will use one of the available source.
      labels: A list of keys labeling the data sequences to show.
      name: Title of the chart.
      missing_stoplight: The stoplight if there are AIO messages but
          none of the desired fields are available.
      **base_kwargs: The kwargs for base class.
    """

    super(BaseAttributeChart, self).__init__(
        attribute_paths, name, missing_stoplight=missing_stoplight,
        **base_kwargs)
    self._labels = labels

  def Filter(self, messages):
    """Obtain data needed to plot a list chart.

    Args:
      messages: The StructTree object as a snapshot of all messages received.

    Returns:
      Objects as arguments to the ListTrailsWidget. The three objects should be:
          x_value: The x value shared by all points, usually the timestamp.
          y_values: A list of y values to be shown that correspond to the same
              x value.
          stoplight: Value of the stoplight.
    """
    attributes = self._GetAllAttributes(messages)

    # A derived class should overwrite the _Filter function.
    # Returns three or four objects:
    #     timestamp: The timestamp of the values.
    #     values: The list of values.
    #     stoplight: The stoplight of the indicator.
    #     message: (optional) The message to show.
    args = self._Filter(*attributes)
    if len(args) == 3:
      args += (None,)
    return args


class BaseAttributeListChart(BaseAttributeChart):
  """A BaseAttributeChart that renders from a list of values."""

  def Plot(self, timestamp, values, stoplight, message):
    """Use a list chart to show the values."""
    return widgets.ListTrailsWidget(
        self._label, self._labels, timestamp, values, stoplight,
        message_src=message, **self._widget_kwargs)

  def Filter(self, messages):
    args = super(BaseAttributeListChart, self).Filter(messages)
    assert isinstance(args[1], list) or args[1] is None
    return args


class BaseAttributeDictChart(BaseAttributeChart):
  """A BaseAttributeChart that renders from a {key: values} dict."""

  def Plot(self, timestamps, values, stoplight, message):
    """Plot the widget for this indicator."""
    widget_kwargs = copy.copy(self._widget_kwargs)
    if 'keys' not in self._widget_kwargs:
      widget_kwargs['keys'] = self._labels
    return widgets.DictTrailsWidget(
        self._label, timestamps, values, stoplight,
        message_src=message, **widget_kwargs)

  def Filter(self, messages):
    args = super(BaseAttributeDictChart, self).Filter(messages)
    assert ((isinstance(args[0], dict) or args[0] is None)
            and (isinstance(args[1], dict) or args[1] is None))
    return args


def RegisterModes(*modes):
  """Decorator to indicate the acceptable modes for an indicator."""
  def Decorator(func):
    def Wrapper(self, mode, *args, **kwargs):
      if mode not in modes:
        raise ValueError('Mode "%s" is not valid for "%s" indicator.' %
                         (mode, self.__class__.__name__))

      return func(self, mode, *args, **kwargs)
    return Wrapper
  return Decorator


class MultiModeIndicator(BaseAttributeIndicator):
  """Base indicator for message telemetries."""

  def __init__(self, mode, name=None, **base_kwargs):
    self._mode = mode
    attributes = self._GetMessageAttributes()
    super(MultiModeIndicator, self).__init__(attributes, name, **base_kwargs)

  def _GetMessageAttributes(self):
    """Return the arguments to indicator according to message source.

    Returns:
      [(<message_short_name>, [<aio_node_short_name>[, <message_field>]]), ...]
          If `aio_node_short_name` is None, it refers to any aio node that has
          this message sent.
    """
    raise NotImplementedError


class MultiModeChart(BaseAttributeChart):
  """Base class similar to MultiModeIndicator but renders with list charts."""

  def __init__(self, mode, labels, name=None, **base_kwargs):
    self._mode = mode
    super(MultiModeChart, self).__init__(
        self._GetMessageAttributes(), labels, name, **base_kwargs)

  def _GetMessageAttributes(self):
    raise NotImplementedError

  def _IsValidInput(self, *attributes):
    raise NotImplementedError


class MultiModeListChart(BaseAttributeListChart):
  """Base class similar to MultiModeIndicator but renders with list charts."""

  def __init__(self, mode, labels, name=None, **base_kwargs):
    self._mode = mode
    super(MultiModeListChart, self).__init__(
        self._GetMessageAttributes(), labels, name, **base_kwargs)

  def _GetMessageAttributes(self):
    raise NotImplementedError

  def _IsValidInput(self, *attributes):
    raise NotImplementedError


class MultiModeDictChart(BaseAttributeDictChart):
  """Base class similar to MultiModeIndicator but renders with dict charts."""

  def __init__(self, mode, labels, name=None, **base_kwargs):
    self._mode = mode
    super(MultiModeDictChart, self).__init__(
        self._GetMessageAttributes(), labels, name, **base_kwargs)

  def _GetMessageAttributes(self):
    raise NotImplementedError

  def _IsValidInput(self, *attributes):
    raise NotImplementedError


class BaseSketch2D(BaseAttributeIndicator):
  """Base class to show trails of 2D points."""

  def __init__(self, attribute_paths=None, name=None, xlim=None, ylim=None,
               xlabel='', ylabel='', background_polygons=None,
               line_properties=None, marker_properties=None, history_len=100,
               show_legend=True, rows=16, **base_kwargs):
    """Initialize the indicator.

    Args:
      attribute_paths: A list of attribute paths. Each path is defined by
          [(<message_name>[, <aio_node>, <field_path>]), ...].
          If <aio_node> is None, then we will use one of the available source.
      name: Name of the indicator.
      xlim: [lower_bound, upper_bound] of the X-axis.
      ylim: [lower_bound, upper_bound] of the Y-axis.
      xlabel: String label of the X-axis.
      ylabel: String label of the Y-axis.
      background_polygons: A list of polygons to render as the background,
          formatted as [
              {'x': [<x_coord>], 'y': [<y_coord>], 'color': <html_color>},
              ...
          ].
      line_properties: Line properties of each data series, formatted as {
          <series_name>: {'color': <html_color>}, ...}.
      marker_properties: Marker properties of each data series, formatted as {
          <series_name>: {'color': <html_color>}, ...}.
      history_len: Number of points to track for each data series.
      show_legend: True if legend is visible.
      rows: Number of rows to set the height of the widget.
      **base_kwargs: kwargs to forward to Sketch2DWidget.
    """

    super(BaseSketch2D, self).__init__(attribute_paths, name, **base_kwargs)
    self._xlim = xlim
    self._ylim = ylim
    self._polygons = [] if background_polygons is None else background_polygons
    self._line_properties = {} if line_properties is None else line_properties
    self._marker_properties = (
        {} if marker_properties is None else marker_properties)
    self._xlabel = xlabel
    self._ylabel = ylabel
    self._show_legend = show_legend
    self._history_len = history_len
    self._rows = rows

  def Plot(self, points, dynamic_polygons, markers):
    return widgets.Sketch2DWidget(points, self._xlabel, self._ylabel,
                                  self._xlim, self._ylim, self._polygons,
                                  self._history_len, self._line_properties,
                                  show_legend=self._show_legend,
                                  rows=self._rows, aspect_ratio=1.0,
                                  title=self._label,
                                  dynamic_polygons=dynamic_polygons,
                                  markers=markers,
                                  marker_properties=self._marker_properties,
                                  **self._widget_kwargs)

  def _PolygonForCircle(self, radius, center_x, center_y, num_samples,
                        color, rad_range=None):
    """Return coordinates/color of a polygon representing part of a cicle."""
    circle_x = []
    circle_y = []

    if not rad_range:
      rad_range = [0.0, numpy.pi * 2]
    rads = numpy.linspace(rad_range[0], rad_range[1], num_samples)

    for rad in rads:
      circle_x.append(numpy.cos(rad) * radius + center_x)
      circle_y.append(numpy.sin(rad) * radius + center_y)
    return {'x': circle_x, 'y': circle_y, 'color': color}

  def _Filter(self, *attributes):
    raise NotImplementedError

  def Filter(self, messages):
    """Obtain the attribute from the messages and compute the data to show.

    Args:
      messages: The StructTree object as a snapshot of all messages received.

    Returns:
      The data to show.
    """
    if not messages:
      return {}, {}, {}

    results = self._Filter(*self._GetAllAttributes(messages))
    if not results:
      return {}, {}, {}

    try:
      assert isinstance(results, dict)
    except AssertionError, e:
      logging.error('Indicator %s is not returning a dict when calling the '
                    '_Filter method.', str(self))
      raise e

    pointers = {}
    if results.get('pointers') and self._line_properties:
      assert set(results['pointers']) == set(self._line_properties)
      pointers = results['pointers']

    segments = results['segments'] if 'segments' in results else {}

    markers = {}
    if results.get('markers') and self._marker_properties:
      assert set(results['markers']) == set(self._marker_properties)
      markers = results['markers']

    return pointers, segments, markers


class DictChartIndicator(BaseAttributeIndicator):
  """Base class to show a 2D x-y chart alongside a dict."""

  def __init__(self, attribute_paths=None, name=None, xlim=None, ylim=None,
               xlabel='', ylabel='', background_polygons=None,
               line_properties=None, history_len=100, show_legend=True,
               precision=1, keys=None, **base_kwargs):
    """Initialize the indicator.

    Args:
      attribute_paths: A list of attribute paths. Each path is defined by
          [(<message_name>[, <aio_node>, <field_path>]), ...].
          If <aio_node> is None, then we will use one of the available source.
      name: Name of the indicator.
      xlim: [lower_bound, upper_bound] of the X-axis.
      ylim: [lower_bound, upper_bound] of the Y-axis.
      xlabel: String label of the X-axis.
      ylabel: String label of the Y-axis.
      background_polygons: A list of polygons to render as the background,
          formatted as [
              {'x': [<x_coord>], 'y': [<y_coord>], 'color': <html_color>},
              ...
          ].
      line_properties: Line properties of each data series, formatted as {
          <series_name>: {'color': <html_color>}, ...}.
      history_len: Number of points to track for each data series.
      show_legend: True if legend is visible.
      precision: Number of digits to display after decimal point.
      keys: Keys whose values should be displayed in dict.
      **base_kwargs: kwargs to forward to DictChartWidget.
    """

    super(DictChartIndicator, self).__init__(attribute_paths, name,
                                             **base_kwargs)
    self._xlim = xlim
    self._ylim = ylim
    self._polygons = [] if background_polygons is None else background_polygons
    self._line_properties = {} if line_properties is None else line_properties
    self._xlabel = xlabel
    self._ylabel = ylabel
    self._show_legend = show_legend
    self._history_len = history_len
    self._precision = precision
    self._keys = keys

  def Plot(self, values, points, dynamic_polygons, stoplight):
    return widgets.DictChartWidget(self._label, values, points, self._xlabel,
                                   self._ylabel, self._xlim, self._ylim,
                                   self._polygons, self._history_len,
                                   self._line_properties, stoplight,
                                   show_legend=self._show_legend,
                                   dynamic_polygons=dynamic_polygons,
                                   precision=self._precision, keys=self._keys,
                                   **self._widget_kwargs)

  def _Filter(self, *attributes):
    raise NotImplementedError


class MultiModeSketch2D(BaseSketch2D):
  """Multi-mode base class to show trails of 2D points."""

  def __init__(self, mode, *base_args, **base_kwargs):
    self._mode = mode
    attribute_paths = self._GetMessageAttributes()
    super(MultiModeSketch2D, self).__init__(
        attribute_paths, *base_args, **base_kwargs)

  def _GetMessageAttributes(self):
    """Return the arguments to indicator according to message source.

    Returns:
      [(<message_short_name>, [<aio_node_short_name>[, <message_field>]]), ...]
          If `aio_node_short_name` is None, it refers to any aio node that has
          this message sent.
    """
    raise NotImplementedError

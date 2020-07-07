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

"""Basic data structures and functions needed to populate a layout."""

import copy
import logging

from makani.avionics.linux.swig import aio_helper
from makani.gs.monitor2.project import settings
from makani.lib.python import debug_util
from makani.lib.python import struct_tree


class MonitorLayoutException(Exception):
  pass


class BaseLayout(object):
  """A base class for various monitor layouts.

  A layout is divided into views, where each view is a group of widgets.
  A layout class has three main functions:
      Initialize: Called when the layout is created.
      Filter: Compute data from the received messages for the client to show.
      Plot: Plot the layout. Called when the layout is created.

  There are two ways to derive a layout class:
  1. Override Filter() and Plot() functions, where Filter() creates a
     PlotData object and assigns computed values to its attributes, and Plot()
     uses the PlotData object as inputs to create a list of widget groups
     (a.k.a. views).
  2. In the Initialize() function, call _AddIndicators() to add a group of
     Indicator objects, where each indicator is an atomic unit of a widget
     and its value computation.

  The user can also call _UpdateProperties to define the display properties
  of a view. Valid options include:
     'cols': How many columns should this view span across.
  """

  # A dict of all derived classes: {<layout_name>: <layout_class>}.
  _layouts = {}
  # Layout name by base module name.
  _names_by_module_name = {}
  # File suffix as an identification for a layout.
  SUFFIX = '_layout'
  # A dict of the PlotData object for each type of layout:
  #     {<layout_name>: PlotData}
  _plot_data = {}
  # Name of the base layout class.
  _NAME = 'abstract'
  # The desired number of columns to place views.
  _DESIRED_VIEW_COLS = 2
  # True if views are ordered horizontally, row by row.
  _ORDER_HORIZONTALLY = True
  # The default font size.
  _DEFAULT_FONT_SIZE = 14

  def __init__(self):
    """Initialize the layout.

    The ordering and detail of views are defined in self._views.
    It stays constant after initialization of the layout, and is only used if
    the layout is created using the second method described above.

    Attributes:
      _views: The list of view configurations in the form of
          [(<view name>, [(<indicator>, [var_names])], <properties>)],
          where `view_name` is the name of the view, `indicator` is an Indicator
          class, `var_names` are output variable names of the indicator, and
          `properties` is a dict of display properties.

          And example of a view with a single indicator is:
          [('Motor', [(MotorStatusIndicator(), ['var_0_0_0'])], {'cols': 2})]

      _view_index: Assistant structure for quick lookup.
          {<view name>: <index in self._views>}.
    """

    self._views = []
    self._view_index = {}
    self._tracebacks = []

  @classmethod
  def OrderHorizontally(cls):
    return cls._ORDER_HORIZONTALLY

  @classmethod
  def DefaultFontSize(cls):
    return cls._DEFAULT_FONT_SIZE

  @classmethod
  def Name(cls):
    return cls._NAME

  @classmethod
  def _ModuleName(cls):
    module_name = cls.__module__.split('.')[-1].lower()
    if not module_name.endswith(BaseLayout.SUFFIX):
      raise MonitorLayoutException('A layout module must end with "%s".' %
                                   BaseLayout.SUFFIX)
    return module_name[:-len(BaseLayout.SUFFIX)]

  @classmethod
  def Layouts(cls):
    return BaseLayout._layouts

  @classmethod
  def NamesByModuleName(cls):
    return BaseLayout._names_by_module_name

  @classmethod
  def Register(cls):
    cls._names_by_module_name[cls._ModuleName()] = cls.Name()
    cls._layouts[cls.Name()] = cls

  @classmethod
  def _GetPlotData(cls):
    if cls.Name() not in cls._plot_data:
      # The PlotData definition for a layout may be missing due to
      # server restart. In this case we register it on demand here.
      layout = cls()
      layout.Initialize()
      layout.Scenario()
    return PlotData(cls._plot_data[cls.Name()])

  def Export(self, memory):
    """Export attributes to memory in the form of a dictionary."""
    if isinstance(memory, dict):
      memory.clear()
      memory.update(self.__dict__)
    else:
      logging.error('A dict is needed to export the memory of a layout.')

  def Import(self, memory):
    """Import attributes from memory in the form of a dictionary."""
    self.__dict__.update(memory)

  def Scenario(self):
    plot_data = PlotData()
    scenario = self.Plot(plot_data)
    if self._NAME not in self._plot_data:
      # Register the PlotData for this layout type.
      self._plot_data[self._NAME] = plot_data
    return scenario

  def Initialize(self):
    """An empty function which can be overwritten.

    This function is called to initialize the layout object when no
    previous memory is found. Any attributes will later be exported
    to memory.
    """
    raise NotImplementedError

  def _GetView(self, view_name):
    """Get a view's details by its name. Create the view if one doesn't exist.

    Args:
      view_name: Name of the view.

    Returns:
      A tuple in the form of [(<view name>, [(<indicator>, [var_names])],
                              <properties>)]
    """
    if view_name not in self._view_index:
      self._view_index[view_name] = len(self._views)
      self._views.append([view_name, [], {}])
    view_index = self._view_index[view_name]
    return self._views[view_index]

  def _UpdateProperties(self, view_name, properties):
    """Update a view's display properties.

    Args:
      view_name: The name of the view.
      properties: A dict of display properties to be updated.
    """
    _, _, view_properties = self._GetView(view_name)
    view_properties.update(properties)

  def _AddBreak(self):
    self._views.append(None)

  def _AddIndicators(self, view_name, indicators, properties=None):
    """"Append indicators to a view.

    This function is called when a layout is being initialized.

    Args:
      view_name: Name of the view that includes the indicators.
      indicators: The list of indicators to add.
      properties: A dict of display properties to be updated.
    """

    # Add indicators.
    _, indicator_list, view_properties = self._GetView(view_name)
    view_index = self._view_index[view_name]
    num_indicators = len(indicator_list)

    for i, indicator in enumerate(indicators):
      # Call Filter() with empty data to get the number of output variables.
      empty_message_snapshot = {}
      empty_message_snapshot['filtered'] = aio_helper.GetFilteredData()
      outputs = indicator.Filter(struct_tree.StructTree(
          empty_message_snapshot, fail_silently=True, readonly=True))
      output_count = len(outputs) if isinstance(outputs, tuple) else 1
      # Assign a unique name, labeled by view_index, indicator_index (i),
      # and variable index (n). This name is then used by Plot() to populate
      # the widgets, and later by Filter() to assign computed values to
      # widget inputs.
      var_names = ['var_%d_%d_%d' % (view_index, i + num_indicators, n)
                   for n in range(output_count)]
      indicator_list.append((indicator, var_names))

    if properties is not None:
      view_properties.update(properties)

  def Plot(self, display_data):
    """Generate JSON configuration to layout the plots.

    Args:
      display_data: A PlotData object.

    Returns:
      A dictionary as a JSON configuration for the layout.
    """
    def PlotIndicator(indicator, display_data, var_names):
      try:
        return indicator.Plot(*[display_data[var] for var in var_names])
      except TypeError, e:
        raise MonitorLayoutException(
            'Cannot plot indicator "%s":\n%s' % (
                type(indicator).__name__, e.message))

    scenario = []
    for view in self._views:
      if view is None:
        scenario.append(None)
      else:
        view_name, indicators, properties = view
        scenario.append((view_name,
                         [PlotIndicator(indicator, display_data, var_names)
                          for indicator, var_names in indicators],
                         properties))
    return AssembleLayout(scenario, self._DESIRED_VIEW_COLS,
                          self._ORDER_HORIZONTALLY)

  def Filter(self, messages):
    """Filter the incoming data and update local memory.

    Process received messages and produce data to plot.

    Args:
      messages: A StructTree object as a nested structure of values.

    Returns:
      A PlotData object.
    """
    data = self._GetPlotData()
    for view in self._views:
      if view is None:
        continue
      for indicator, var_names in view[1]:
        # An indicator's Filter function returns an object or a tuple of values.
        try:
          outputs = indicator.Filter(messages)
        except Exception:  # pylint: disable=broad-except
          traceback_string = debug_util.FormatTraceback()
          # indicator.Filter may introduce any kind of exception.
          logging.error('Indicator "%s" encountered an error:\n%s',
                        indicator.Name(), traceback_string)
          self._tracebacks.append((indicator.Name(), traceback_string))
          continue
        if not isinstance(outputs, tuple):
          outputs = (outputs,)
        assert len(outputs) == len(var_names)
        for i, output in enumerate(outputs):
          data[var_names[i]] = output
    return data

  def ErrorReport(self):
    return self._tracebacks

  def ClearErrors(self):
    del self._tracebacks[:]


class PlotData(object):
  """Data used for plotting.

  The class serves two purposes:
  - When the layout is first being created, the Layout.Plot function gets
    attribute names from PlotData to tell the client what to extract when data
    becomes available. This in fact builds a template PlotData, which registers
    attributes needed by the client. The attributes will be cross-checked later.
  - When the layout is being updated, the Layout.Filter function computes data
    to be plotted and adds to the PlotData, which is then serialized and sent
    over to the client.
  """

  def __init__(self, template=None):
    """Initialize the PlotData.

    Args:
      template: A template PlotData. Only attributes appearing in the template
          can be assigned. If None, then this PlotData is used to create a
          template.
    """
    if template is None:
      # Use __dict__ to avoid triggering self.__setattr__.
      self.__dict__['_attributes'] = {}
      self.__dict__['_is_template'] = True
    else:
      self.__dict__['_attributes'] = copy.copy(template.__dict__['_attributes'])
      self.__dict__['_is_template'] = False

  def __getattr__(self, attribute):
    """Returns value of the attribute, or its name if none exists."""
    if not self.__dict__['_is_template']:
      if attribute in self.__dict__['_attributes']:
        return self.__dict__['_attributes'][attribute]
      else:
        raise MonitorLayoutException('Attribute "%s" in PlotData is not '
                                     'defined in the layout\'s "Plot" function.'
                                     % attribute)
    else:
      self.__dict__['_attributes'][attribute] = None
      return attribute

  def __setattr__(self, attribute, value):
    """Sets value of an attribute."""
    if not self.__dict__['_is_template']:
      if attribute in self.__dict__['_attributes']:
        self.__dict__['_attributes'][attribute] = value
      else:
        raise MonitorLayoutException('Attribute "%s" in PlotData is not '
                                     'defined in the layout\'s "Plot" function.'
                                     % attribute)
    else:
      raise MonitorLayoutException('Cannot set attribute value in a '
                                   'template PlotData object')

  def __setitem__(self, attribute, value):
    return self.__setattr__(attribute, value)

  def __getitem__(self, attribute):
    return self.__getattr__(attribute)

  def Json(self):
    return self.__dict__['_attributes']


def AssembleLayout(views, desired_view_cols, order_horizontally):
  """Expand views defined in Plot() into full fledged JSON specifications.

  A "view" corresponds to a section in a layout page. Each view has a name and
  a list of widgets, each widget visualizes one piece of information.

  In the client, the canvas is divided into a grid. The grid will be further
  divided into columns to place views. By default, each view occupies a
  column, but a view can be configured to span across multiple columns.

  Args:
    views: A list of views, each with a list of widgets. E.g.,
    [
        # Defines a view named "Motor".
        ('Motor', [
            # A widget showing the current motor command.
            widgets.ScalarIndicatorWidget('Motor Command', data.motor_command,
                                          data.status_indicator),
            # A widget showing the desired motor speeds.
            widgets.ListTrailsWidget('Motor Speed', motor_names, data.timestamp,
                                     data.motor_speed, data.status_indicator),
        ]),
        ... # More views.
    ]
    desired_view_cols: The desired number of columns.
    order_horizontally: True if views are organized in a horizontal order.

  Returns:
    A JSON object specifying the layout.
  """

  grid_width = settings.CSS_GRID_COLUMNS

  # Compute the default width of a column. If multiple views are present,
  # each column takes a portion of the grid width.
  width_per_col = max(1, grid_width / desired_view_cols)

  if order_horizontally:
    view_stripes = _AssembleLayoutHorizontally(views, grid_width, width_per_col)
  else:
    view_stripes = _AssembleLayoutVertically(views, grid_width, width_per_col)

  # The JSON specification of the entire layout page.
  scenario = {
      'canvas': {
          'grid_width': settings.CSS_GRID_COLUMNS,
          'row_height_px': 15
      },
      'views': view_stripes,
  }
  return scenario


def _GetViewSpecs(view_args, width_per_col, grid_width, col_width=None):
  """Convert a single view into its JSON specifications.

  Args:
    view_args: A tuple of (title, widgets[, properties]).
    width_per_col: The width for each column.
    grid_width: The width of the canvas (in CSS style).
    col_width: The pre-determined width of the view
        (in units defined by layout._DESIRED_VIEW_COLS).

  Returns:
    A dict with parameters describing the view.
  """

  if len(view_args) == 2:
    title, widgets = view_args
    properties = {}
  else:
    assert len(view_args) == 3
    title, widgets, properties = view_args
  view = {
      'name': title,
      'cols': 1,  # By default, each view occupies one column.
      'indicators': []
  }
  view.update(properties)
  if col_width:
    view['cols'] = col_width
  view['grid_width'] = view['cols'] * width_per_col

  for widget in widgets:
    # Note that each grid cell is logically split into a finer grid.
    # Let each widget span horizontally across the entire view.
    widget.SetCols(grid_width)
    view['indicators'] += widget.Indicators()
  return view


def _AssembleLayoutVertically(views, grid_width, width_per_col):
  """Get the JSON specification of all indicators in vertical order.

  Args:
    views: A list of tuples in the shape of (title, widgets[, properties]).
    grid_width: The total width of the canvas.
    width_per_col: The with of each column.

  Returns:
    A list of view groups, each representing a column that has these fields:
        'grid_width': Width of the column.
        'stripe': List of view specifications.
  """
  view_columns = []

  # Views in groups, each representing a column.
  columns = []
  # The list of views in the current column.
  current_column = []
  # The width of the column.
  default_view_width = 1
  current_column_width = default_view_width

  # TODO: Add a View class.
  # Compute the views per each column, and the largest width amongst all views.
  for view_args in views:
    if view_args is None:
      # End of the last column
      columns.append((current_column, current_column_width))
      current_column = []
      current_column_width = default_view_width
      continue
    elif len(view_args) == 3 and 'cols' in view_args[2]:
      # view_args is a tuple of (title, widgets, properties).
      current_column_width = max(current_column_width, view_args[2]['cols'])
    current_column.append(view_args)

  if current_column:
    columns.append((current_column, current_column_width))

  # For each column, obtain a list of JSON specifications.
  for col_views, col_width in columns:
    view_columns.append({
        'grid_width': col_width * width_per_col,
        'stripe': [
            _GetViewSpecs(view_args, width_per_col, grid_width, None)
            for view_args in col_views],
    })
  return view_columns


def _AssembleLayoutHorizontally(views, grid_width, width_per_col):
  """Get the JSON specification of all indicators in horizontal order.

  Args:
    views: A list of tuples in the shape of (title, widgets[, properties]).
    grid_width: The total width of the canvas.
    width_per_col: The with of each column.

  Returns:
    A list of view groups, each representing a row that includes these fields:
        'stripe': List of view specifications.
  """
  view_rows = []
  current_row = []
  accumulated_width = 0

  for view_args in views:
    view = (_GetViewSpecs(view_args, width_per_col, grid_width)
            if view_args is not None else None)
    if not view or accumulated_width + view['grid_width'] > grid_width:
      view_rows.append({'stripe': current_row, 'grid_width': grid_width})
      accumulated_width = 0
      current_row = []
    if view:
      accumulated_width += view['grid_width']
      current_row.append(view)

  if current_row:
    view_rows.append({'stripe': current_row, 'grid_width': grid_width})
  return view_rows

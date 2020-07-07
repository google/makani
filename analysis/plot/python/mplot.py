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

"""Helper classes for plotting data from HDF5 log files."""

import collections
import copy
import inspect
import subprocess

import h5py
from matplotlib import pyplot
import numpy as np

# xdotool (install via "sudo apt-get install xdotool") can be used to
# bring plot windows to the foreground. The pipes render stdout and
# stderr quiet.
#
# NOTE: Allegedly it's possible to bring a window to the
# foreground using some combination of QtWidget methods (show,
# activateWindow, raise), but I can't get them to work.
_XDOTOOL_EXISTS = (subprocess.call(['which', 'xdotool'],
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE) == 0)

_COLOR_CYCLE = [
    'black', 'brown', 'red', 'orange', 'yellow', 'green', 'blue',
    'violet', 'gray'
]


def PlotVec3(x, y, label='', labels=None, linestyle='-', colors=None,
             scale=1.0):
  """Plot a Vec3 field."""
  if label:
    label += ' '
  if labels is None:
    labels = [label + d for d in ['x', 'y', 'z']]
  if colors is None:
    colors = ['b', 'g', 'r']
  pyplot.plot(x, scale * y['x'], linestyle=linestyle, color=colors[0],
              label=labels[0])
  pyplot.plot(x, scale * y['y'], linestyle=linestyle, color=colors[1],
              label=labels[1])
  pyplot.plot(x, scale * y['z'], linestyle=linestyle, color=colors[2],
              label=labels[2])


def PlotBitMask(x, bits, helper):
  points = -np.ones((bits.shape[0], len(helper)))
  for i in range(len(helper)):
    points[np.bitwise_and(2**i, bits) != 0, i] = i
  pyplot.plot(x, points, 'o')
  pyplot.yticks(helper.Values(), helper.ShortNames())


def PlotComponents(x, y, index_label_pairs, colors=None,
                   scale=1.0, linestyle='-'):
  if colors is None:
    colors = [_COLOR_CYCLE[i % len(_COLOR_CYCLE)]
              for i in range(len(index_label_pairs))]
  for i, (index, comp_label) in enumerate(index_label_pairs):
    pyplot.plot(x, scale * y[:, index], label=comp_label,
                color=colors[i], linestyle=linestyle)


class UnlabeledPlotException(Exception):
  """Thrown if a plot is missing axes labels and pedantry is enabled."""
  pass


class PlotGroup(object):
  """Class for managing a related set of plots."""

  def __init__(self, *args, **kwargs):
    """Constructor.

    Args:
      *args: Arguments to this constructor will be passed as the first
          arguments to any plotting function defined in this plot group and
          decorated by MFig below.
      **kwargs: Keyword args must specify 'parent' and 'title_prefix'.
    """
    self._parent = kwargs.get('parent')
    self._title_prefix = kwargs.get('title_prefix')
    if self._parent is None:
      self.linked_axes = None
      self.figure_count = 0
    else:
      self._parent.AddChild(self)
    self._children = []
    self.data = args

  def AddChild(self, child):
    self._children.append(child)

  def OpenAllPlots(self):
    """Run all methods whose name starts with 'Plot'."""
    for name, method in inspect.getmembers(self.__class__,
                                           predicate=inspect.ismethod):
      if name.startswith('Plot'):
        try:
          method(self)
        except Exception as e:  # pylint: disable=broad-except
          pyplot.close()
          print '%s failed: %s' % (name, e)

    for child in self._children:
      child.OpenAllPlots()

  def GetTitlePrefix(self):
    if self._parent and self._parent.GetTitlePrefix():
      prefix = self._parent.GetTitlePrefix()
    else:
      prefix = ''

    if self._title_prefix is not None:
      prefix += self._title_prefix + ':'

    return prefix

  def PrePlotSetup(self, link):
    """Call before making a plot."""
    if self._parent is not None:
      fig, ax, xlim = self._parent.PrePlotSetup(link)
    else:
      fig = pyplot.figure()
      self.figure_count += 1
      xlim = None
      if link:
        if self.linked_axes:
          xlim = self.linked_axes.get_xlim()
          ax = fig.add_subplot(1, 1, 1, sharex=self.linked_axes)
        else:
          ax = fig.add_subplot(1, 1, 1)
          self.linked_axes = ax
      else:
        ax = fig.add_subplot(1, 1, 1)
    return fig, ax, xlim

  def PostPlotSetup(self, fig, ax, xlim):
    """Call after making a plot."""
    if self._parent is not None:
      self._parent.PostPlotSetup(fig, ax, xlim)
    else:
      manager = pyplot.get_current_fig_manager()
      manager.toolbar.pan()
      manager.toolbar.hide()
      x, y = -10, 14
      width, height = 500, 410
      if self.figure_count % 12 in [2, 6, 10]:
        x += width
      if self.figure_count % 12 in [3, 7, 11]:
        x += 2.0 * width
      if self.figure_count % 12 in [4, 8, 0]:
        x += 3.0 * width

      if self.figure_count % 12 in [5, 6, 7, 8]:
        y += height
      if self.figure_count % 12 in [9, 10, 11, 0]:
        y += 2.0 * height
      manager.window.setGeometry(x, y, width, height - 40)
      if xlim:
        self.linked_axes.set_xlim(xlim)

  @staticmethod
  def MFig(title, xlabel=None, ylabel=None, link=True, pedantic=True,
           xticks=None, xticklabels=None, xticklabels_orientation='horizontal',
           axvline_values=None, collect_labels=False, axgrid=True):
    """Decorator for conveniently creating plots.

    Functions with this decorator will naturally get window
    positioning, title, legends, and optionally linked axes.
    They will be called with PlotGroup's curried arguments,
    followed by any other arguments.  The body of the function
    should consist of calls similar to pyplot.plot.

    Args:
      title: Title for the figure.
      xlabel: Label for the x axis, required if pedantic is True.
      ylabel: Label for the y axis, required if pedantic is True.
      link: Whether to link this plot's x-axis to other plots from this
          PlotGroup (default true).
      pedantic: Whether xlabel and ylabel must be set.
      xticks: List of values to place ticks on the x-axis.
      xticklabels: List of labels for the ticks placed on the x-axis.
      xticklabels_orientation: Orientation of the `xticklabels`. Possible values
          are 'horizontal', 'vertical', or a number indicating the angle in
          degrees.
      axvline_values: List of x values to draw a vertical dashed line.
      collect_labels: Whether to remove repeated labels, if any.
      axgrid: Whether to display a grid.

    Returns:
      The wrapped plotting function.
    """
    def WrapPlot(plot_func):
      """Decorator."""
      def WrappedPlot(self, *args, **kwargs):
        """Wrapped plotting function."""
        if pedantic and (not xlabel or not ylabel):
          raise UnlabeledPlotException()

        fig, ax, xlim = self.PrePlotSetup(link)

        fig.canvas.set_window_title(self.GetTitlePrefix() + title)
        fig.suptitle(self.GetTitlePrefix() + title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        # pyplot.hold(True)
        plot_func(self, *(self.data + args), **kwargs)
        # pyplot.hold(False)
        ax.grid(axgrid)
        if xticks:
          ax.set_xticks(xticks)
        if xticklabels:
          ax.set_xticklabels(xticklabels, rotation=xticklabels_orientation)
        if axvline_values:
          for x in axvline_values:
            ax.axvline(x=x, linestyle='--', linewidth=0.5)

        if collect_labels:
          handles, labels = ax.get_legend_handles_labels()
          by_label = collections.OrderedDict(zip(labels, handles))
          legend = pyplot.legend(by_label.values(), by_label.keys())
        else:
          legend = pyplot.legend(loc='lower left', prop={'size': 10})
        if legend:
          legend.draggable()

        self.PostPlotSetup(fig, ax, xlim)
        pyplot.show(block=False)
        if _XDOTOOL_EXISTS:
          window = pyplot.get_current_fig_manager().window
          subprocess.check_call([
              'xdotool', 'windowactivate', str(window.effectiveWinId())])
      return WrappedPlot
    return WrapPlot


class CollatedH5Data(object):
  """Helper class for combining log files and collating their contents."""

  def __init__(self, filenames, node_message_type_pairs):
    """Constructor.

    Args:
      filenames: Files names to concatenate.
      node_message_type_pairs: List of pairs consisting of node short names
          and message_type short names.
    """
    if not hasattr(filenames, '__iter__'):
      filenames = [filenames]

    self.filenames = copy.copy(filenames)
    # Pre-populate the date structure
    self._nodes = {}
    for node, message_type in node_message_type_pairs:
      if node not in self._nodes:
        self._nodes[node] = {}
      self._nodes[node][message_type] = []

    files = [None for _ in range(len(filenames))]
    for i, filename in enumerate(filenames):
      files[i] = h5py.File(filename, 'r')

      # Test if a given message was sent by a node in this log file,
      # and add it to the corresponding list if so.
      for node, message_type in node_message_type_pairs:
        node_long = 'kAioNode' + node
        message_type_long = 'kMessageType' + message_type
        if (node_long in files[i]['messages'] and
            message_type_long in files[i]['messages'][node_long]):
          self._nodes[node][message_type].append(
              files[i]['messages'][node_long][message_type_long])
    if 'parameters' in files[0]:
      self.params = copy.copy(files[0]['parameters'])

    for node, message_type in node_message_type_pairs:
      if self._nodes[node][message_type]:
        # Concatenate the data.
        self._nodes[node][message_type] = np.hstack(
            self._nodes[node][message_type])
      else:
        self._nodes[node][message_type] = None

  def __getitem__(self, node):
    return self._nodes[node]

  def __contains__(self, node):
    return node in self._nodes

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

import inspect

import h5py
from matplotlib import pyplot
import numpy as np


class UnlabeledPlotException(Exception):
  """Thrown if a plot is missing axes labels and pedantry is enabled."""
  pass


class PlotGroup(object):
  """Class for managing a related set of plots."""

  def __init__(self, *args):
    """Constructor.

    Args:
      *args: Arguments to this constructor will be passed as the first
          arguments to any plotting function defined in this plot group and
          decorated by MFig below.
    """
    self.linked_axes = None
    self.figure_count = 0
    self.data = args

  def PostProcess(self, fig, ax):
    """Function to be run after each plot is completed."""
    pass

  def OpenAllPlots(self):
    """Run all methods whose name starts with 'Plot'."""
    for name, method in inspect.getmembers(self.__class__,
                                           predicate=inspect.ismethod):
      if name.startswith('Plot'):
        method(self)

  @staticmethod
  def MFig(title, xlabel=None, ylabel=None, link=True, pedantic=True):
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

    Returns:
      The wrapped plotting function.
    """
    def WrapPlot(plot_func):
      """Decorator."""
      def WrappedPlot(self, *args):
        """Wrapped plotting function."""
        if pedantic and (not xlabel or not ylabel):
          raise UnlabeledPlotException()
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
        fig.canvas.set_window_title(title)
        fig.suptitle(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        pyplot.hold(True)
        plot_func(self, *(self.data + args))
        pyplot.hold(False)
        ax.grid(True)
        pyplot.legend(loc='lower left', prop={'size': 10})
        manager = pyplot.get_current_fig_manager()
        x, y = -10, 14
        width, height = 800, 410
        if self.figure_count % 4 in [0, 2]:
          x += width
        if self.figure_count % 4 in [2, 3]:
          y += height
        manager.window.setGeometry(x, y, width, height)
        if xlim:
          self.linked_axes.set_xlim(xlim)
        self.PostProcess(fig, ax)
        pyplot.show(block=False)
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

    # Pre-populate the date structure
    self._nodes = {}
    for node, message_type in node_message_type_pairs:
      if node not in self._nodes:
        self._nodes[node] = {}
      self._nodes[node][message_type] = []

    files = [None for _ in range(len(filenames))]
    for i, filename in enumerate(filenames):
      files[i] = h5py.File(filename)

      # Test if a given message was sent by a node in this log file,
      # and add it to the corresponding list if so.
      for node, message_type in node_message_type_pairs:
        node_long = 'kAioNode' + node
        message_type_long = 'kMessageType' + message_type
        if message_type_long in files[i]['messages'][node_long]:
          self._nodes[node][message_type].append(
              files[i]['messages'][node_long][message_type_long])

    for node, message_type in node_message_type_pairs:
      if self._nodes[node][message_type]:
        # Concatenate the data.
        self._nodes[node][message_type] = np.hstack(
            self._nodes[node][message_type])
      else:
        self._nodes[node][message_type] = None

    for f in files:
      f.close()

  def __getitem__(self, node):
    return self._nodes[node]

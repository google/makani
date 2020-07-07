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

"""Plots relating to the planner."""

from makani.analysis.plot.python import mplot
from makani.control import control_types
from makani.lib.python import c_helpers
from matplotlib.pyplot import plot
from matplotlib.pyplot import yticks

MFig = mplot.PlotGroup.MFig  # pylint: disable=invalid-name

_FLIGHT_MODE_HELPER = c_helpers.EnumHelper('FlightMode', control_types)


class Plots(mplot.PlotGroup):
  """Plots of the planner."""

  @MFig(title='Flight Mode', ylabel='Mode', xlabel='Time [s]')
  def PlotFlightMode(self, c):
    plot(c['time'], c['flight_mode'])
    yticks(_FLIGHT_MODE_HELPER.Values(), _FLIGHT_MODE_HELPER.ShortNames())

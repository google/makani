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

"""Plots relating to fault detection."""

from makani.analysis.plot.python import mplot
from makani.control import control_types
from makani.lib.python import c_helpers
from matplotlib.pyplot import plot
from matplotlib.pyplot import title
from matplotlib.pyplot import xticks
from matplotlib.pyplot import yticks
import numpy as np

_FAULT_TYPE_HELPER = c_helpers.EnumHelper('FaultType', control_types)
_SUBSYSTEM_HELPER = c_helpers.EnumHelper('Subsystems', control_types,
                                         prefix='kSubsys')

MFig = mplot.PlotGroup.MFig  # pylint: disable=invalid-name


class Plots(mplot.PlotGroup):
  """Plots for faults."""

  @MFig(title='Faults', ylabel='Type', xlabel='Time [s]', link=False)
  def PlotFaults(self, c, tspan=None):
    if tspan is not None:
      selection = np.logical_and(c['time'] > tspan[0], c['time'] < tspan[1])
    else:
      selection = slice(None)
    code = [np.bitwise_or.reduce(c['faults']['code'][selection, i])
            for i in range(c['faults']['code'].shape[1])]

    faults = -np.ones((len(_SUBSYSTEM_HELPER), len(_FAULT_TYPE_HELPER)))
    for i in range(len(_FAULT_TYPE_HELPER)):
      faults[np.bitwise_and(2**i, code) != 0, i] = i
    plot(faults, 'o')
    xticks(_SUBSYSTEM_HELPER.Values(), _SUBSYSTEM_HELPER.ShortNames(),
           rotation='vertical')
    yticks(_FAULT_TYPE_HELPER.Values(), _FAULT_TYPE_HELPER.ShortNames())

  @MFig(title='Faults', ylabel='Type', xlabel='Time [s]')
  def PlotFaultTimes(self, c, subsystem=None):
    faults = -np.ones((c['faults']['code'].shape[0], len(_FAULT_TYPE_HELPER)))
    if subsystem is None:
      code = np.zeros((faults.shape[0],), dtype='i4')
      for i in range(c['faults']['code'].shape[1]):
        code = np.bitwise_or(c['faults']['code'][:, i], code)
    else:
      code = c['faults']['code'][:, _SUBSYSTEM_HELPER.Value(subsystem)]

    for i in range(len(_FAULT_TYPE_HELPER)):
      faults[np.bitwise_and(2**i, code) != 0, i] = i

    plot(c['time'], faults, 'o')

    yticks(_FAULT_TYPE_HELPER.Values(), _FAULT_TYPE_HELPER.ShortNames())
    if subsystem is not None:
      title('Faults: ' + subsystem)

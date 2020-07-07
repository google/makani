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

"""Useful groups of plots."""
from datetime import datetime
import multiprocessing
import os

import control_input
import control_output
import crosswind
import estimator
import faults
import gps
import ground_estimator
import hover
from makani.analysis.plot.python import mplot
import numpy as np
import planner
import trans_in
import wind

control_input = reload(control_input)
control_output = reload(control_output)
crosswind = reload(crosswind)
estimator = reload(estimator)
faults = reload(faults)
gps = reload(gps)
ground_estimator = reload(ground_estimator)
hover = reload(hover)
mplot = reload(mplot)
planner = reload(planner)
trans_in = reload(trans_in)
wind = reload(wind)


class FilePlotGroup(mplot.PlotGroup):

  def __init__(self, filename, *args, **kwargs):
    # We effectively want title_prefix='' in the function signature, but that
    # doesn't play nice with *args.
    modified_kwargs = kwargs.copy()
    modified_kwargs['title_prefix'] = (os.path.basename(filename) + ':'
                                       + kwargs.get('title_prefix', ''))
    super(FilePlotGroup, self).__init__(*args, **modified_kwargs)


def _GetTime(message):
  return (message['capture_header']['tv_sec']
          + 1e-6 * message['capture_header']['tv_usec'])


class ControllerPlots(FilePlotGroup):
  """Container of plot groups of controller data."""

  def __init__(self, data, controller='A',
               zero_time=False, zero_time_mode='start', parent=None):
    super(ControllerPlots, self).__init__(
        data.filenames[0], title_prefix='Controller ' + controller,
        parent=parent)

    if hasattr(data, 'params'):
      self.params = data.params

    if ('ControlDebug' in data['Controller' + controller] and
        data['Controller' + controller]['ControlDebug'] is not None):
      control_telem_name = 'ControlDebug'
    elif data['Controller' + controller]['ControlTelemetry'] is not None:
      control_telem_name = 'ControlTelemetry'
    else:
      control_telem_name = None

    if control_telem_name is not None:
      self.c = data['Controller' + controller][control_telem_name]['message']
      if zero_time:
        if zero_time_mode == 'start':
          # Zero time based on time at start of data.
          self.c['time'] -= self.c['time'][0]
        elif zero_time_mode == 'HoverAscend':
          # Zero time based on flight time definition (start of HoverAscend).
          self.c['time'] -= self.c['time'][np.where(
              self.c['flight_mode'] == 2)[0][0]]
        else:
          assert False, 'zero_time_mode not recognized: %s' % zero_time_mode
      self.c_seq = (data['Controller' + controller][control_telem_name]
                    ['aio_header']['sequence'])
    else:
      self.c = None

    if data['Simulator']['SimTelemetry'] is not None:
      self.s = data['Simulator']['SimTelemetry']['message']
      if zero_time:
        if self.c is not None:
          # Zero time to be synced w/ the controller.
          self.s['time'] += self.c['time'][0] - self.s['time'][0]
        elif zero_time_mode == 'start':
          # Ensure sim time starts at zero
          self.s['time'] -= self.s['time'][0]
        elif zero_time_mode == 'HoverAscend':
          assert False, 'Need controller to zero time on HoverAscend.'
    else:
      self.s = None

    if data['GsEstimator']['GroundTelemetry'] is not None:
      self.g = data['GsEstimator']['GroundTelemetry']['message']
    else:
      self.g = None

    self.control_input = control_input.Plots(
        self.c['control_input'], self.c, self.s, parent=self)
    self.control_output = control_output.Plots(
        self.c['control_output'], self.c, self.s, parent=self)
    self.crosswind = crosswind.Plots(
        self.c['crosswind'], self.c, self.s, parent=self)
    self.estimator = estimator.Plots(
        self.c['estimator'], self.c, self.s, self.params, parent=self)
    self.faults = faults.Plots(self.c, parent=self)
    self.ground_estimator = ground_estimator.Plots(
        self.g, self.s, self.params, parent=self)
    self.hover = hover.Plots(self.c['hover'], self.c, self.s, self.params,
                             parent=self)
    self.planner = planner.Plots(self.c, parent=self)
    self.trans_in = trans_in.Plots(
        self.c['trans_in'], self.c, self.s, parent=self, title_prefix='TransIn')
    self.wind = wind.Plots(self.c, parent=self)


class AvionicsPlots(FilePlotGroup):
  """Container of plot groups of avionics data."""

  def __init__(self, data, parent=None):
    super(AvionicsPlots, self).__init__(
        data.filenames[0], title_prefix='Avionics', parent=parent)

    self.supported_nodes = [
        'FcA',
        'FcB',
        'LightPort',
        'LightStbd',
        'GpsBaseStation'
    ]

    if hasattr(data, 'params'):
      self.params = data.params

    self.cn0_by_sat = {}

    pool_keys = []
    pool_inputs = []
    for node in self.supported_nodes:
      if node in data and 'NovAtelObservations' in data[node]:
        novatel_obs = data[node]['NovAtelObservations']['message']
        pool_keys.append(node)
        pool_inputs.append(novatel_obs)
    pool_size = len(pool_keys) if len(pool_keys) <= 4 else 4
    pool = multiprocessing.Pool(pool_size)
    pool_results = pool.map(gps.get_cn0_by_satellite, pool_inputs)
    for idx, res in enumerate(pool_results):
      self.cn0_by_sat[pool_keys[idx]] = res

    if self.cn0_by_sat:
      novatel_obs_first_node = (
          data[self.cn0_by_sat.keys()[0]]['NovAtelObservations'])
      # If the timestamp is provided, add it to the map under the 'timestamp'
      # key so that it is used for the x-axis.
      if ('capture_header' in novatel_obs_first_node.dtype.names and
          'tv_sec' in novatel_obs_first_node['capture_header'].dtype.names and
          'tv_usec' in novatel_obs_first_node['capture_header'].dtype.names):
        timestamp_to_datetime = np.vectorize(datetime.utcfromtimestamp,
                                             otypes=[np.object])
        self.cn0_by_sat['timestamp'] = timestamp_to_datetime(
            novatel_obs_first_node['capture_header']['tv_sec'] +
            1e-6 * novatel_obs_first_node['capture_header']['tv_usec'])

    self.idle_time_by_node = {}
    for node in self.supported_nodes:
      try:
        idle_time = data[node]['NovAtelSolution']['message']['idle_time']
        novatel_sol = data[node]['NovAtelSolution']
        if ('capture_header' in novatel_sol.dtype.names and
            'tv_sec' in novatel_sol['capture_header'].dtype.names and
            'tv_usec' in novatel_sol['capture_header'].dtype.names):
          timestamp_to_datetime = np.vectorize(datetime.utcfromtimestamp,
                                               otypes=[np.object])
          timestamp = timestamp_to_datetime(
              novatel_sol['capture_header']['tv_sec'] +
              1e-6 * novatel_sol['capture_header']['tv_usec'])
        self.idle_time_by_node[node] = {
            'idle_time': idle_time,
            'timestamp': timestamp
        }
      except KeyError:
        pass

    plot_data = {
        'cn0_by_sat': self.cn0_by_sat,
        'idle_time_by_node': self.idle_time_by_node
    }
    self.gps = gps.Plots(plot_data, parent=self)

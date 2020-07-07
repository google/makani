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

"""Plots relating to the GPS."""

import types

from makani.analysis.plot.python import mplot
from matplotlib.pyplot import cm
from matplotlib.pyplot import plot
import numpy

MFig = mplot.PlotGroup.MFig  # pylint: disable=invalid-name

# Name of the satellite system based on bits 19-16 on the channel tracking
# status message, according to
# https://www.novatel.com/assets/Documents/Manuals/om-20000129.pdf#page=590
_SATELLITE_SYSTEM = {
    0: 'GPS',
    1: 'GLONASS',
    2: 'SBAS',
    3: 'Galileo',
    4: 'BeiDou',
    5: 'QZSS',
    6: 'Reserved',
    7: 'Other'
}

# Signal type on bits 25-21 on the channel tracking status message, according to
# https://www.novatel.com/assets/Documents/Manuals/om-20000129.pdf#page=590
_SIGNAL_TYPE = {
    'GPS': {
        0: 'L1 C/A',
        5: 'L2 P',
        9: 'L2 P codeless',
        14: 'L5 Q',
        17: 'L2 C'
    },
    'GLONASS': {
        0: 'L1 C/A',
        1: 'L2 C/A',
        5: 'L2 P'
    },
    'Galileo': {
        2: 'E1C',
        12: 'E5a Q',
        17: 'E5b Q',
        20: 'AltBOC Q'
    },
    'QZSS': {
        0: 'L1 C/A',
        14: 'L5Q',
        17: 'L2C'
    },
    'SBAS': {
        0: 'L1 C/A',
        6: 'L5I'
    },
    'BeiDou': {
        0: 'B1 with D1 data',
        1: 'B2 with D1 data',
        4: 'B1 with D2 data',
        5: 'B2 with D2 data'
    },
    'Other': {
        19: 'L-Band'
    }
}


class Plots(mplot.PlotGroup):
  """Plots of the GPS."""

  def __init__(self, *args, **kwargs):
    super(Plots, self).__init__(*args, **kwargs)

    # Create a color map and index the satellite names so that both plots share
    # the same color for each satellite.
    self._all_satellites = set()
    for node in args[0]['cn0_by_sat'].keys():
      self._all_satellites.update(args[0]['cn0_by_sat'][node])
    self._color_map = cm.get_cmap(name='nipy_spectral',
                                  lut=len(self._all_satellites))
    self._all_satellites_list = list(self._all_satellites)
    self._all_satellites_index_map = {
        self._all_satellites_list[i]: i
        for i in range(len(self._all_satellites_list))}

    # If the timestamp array is provided, use it for the x-axis, otherwise, use
    # the index.
    timestamp = None
    if 'timestamp' in args[0]['cn0_by_sat']:
      timestamp = args[0]['cn0_by_sat']['timestamp']
    xlabel = 'Time [UTC]' if timestamp is not None else 'Time [samples]'
    nodes = args[0]['cn0_by_sat'].keys()
    if 'timestamp' in nodes:
      nodes.remove('timestamp')
    # Dynamically create the PlotCarrierToNoiseDensityRatio functions for the
    # given nodes provided in the `cn0_by_sat map`. The function name is set to
    # 'PlotCarrierToNoiseDensityRatio<node name>'. For example, if the nodes
    # provided are 'FcA' and 'GpsBaseStation', then this class will contain two
    # plotting methods: PlotCarrierToNoiseDensityRatioFcA() and
    # PlotCarrierToNoiseDensityRatioGpsBaseStation().
    for node in nodes:
      setattr(
          self,
          # Set the name of the method.
          'PlotCarrierToNoiseDensityRatio' + node,
          # types.MethodType binds the function to the name as a callable
          # function.
          types.MethodType(
              # MFig(args)(function) applies the decorator to the function.
              MFig(
                  title='Carrier to noise density ratio ' + node,
                  ylabel='C/No [dB-Hz]',
                  xlabel=xlabel
              )(self._get_plot_carrier_to_noise_density_ratio_function(
                  node, timestamp=timestamp)),
              self,
              Plots
          ))

  @MFig(title='Idle time', ylabel='Idle time [percent]', xlabel='Time [s]')
  def PlotIdleTime(self, plot_data):
    for node in plot_data['idle_time_by_node']:
      plot(plot_data['idle_time_by_node'][node]['timestamp'],
           plot_data['idle_time_by_node'][node]['idle_time'], label=node)

    first_node = plot_data['idle_time_by_node'].keys()[0]
    plot(plot_data['idle_time_by_node'][first_node]['timestamp'],
         15.0 * numpy.ones(
             plot_data['idle_time_by_node'][first_node]['idle_time'].shape),
         'k--', label='danger zone', color='r')

  def _get_plot_carrier_to_noise_density_ratio_function(
      self, node, timestamp=None):
    """Returns the PlotCarrierToNoiseDensityRatio for a given node.

    Args:
      node: String with the name of the node, which must be a valid key in the
        cn0_by_sat map.
      timestamp: A numpy.array of numpy.object with the datetime.datetime used
        for the x-axis. If none, the sample index is used for the x-axis.
    """

    def plot_carrier_to_noise_density_ratio(self, plot_data):
      for sat_name in sorted(plot_data['cn0_by_sat'][node].keys()):
        if timestamp is None:
          plot(
              plot_data['cn0_by_sat'][node][sat_name],
              label=sat_name,
              color=self._color_map(self._all_satellites_index_map[sat_name]))
        else:
          plot(
              plot_data['cn0_by_sat']['timestamp'],
              plot_data['cn0_by_sat'][node][sat_name],
              label=sat_name,
              color=self._color_map(self._all_satellites_index_map[sat_name]))
    return plot_carrier_to_noise_density_ratio


def get_cn0_by_satellite(novatel_obs, satellite_names=None):
  """Returns a map of C/No by satellite name.

  This function generates a map for each satellite present in the data with its
  respective carrier to noise density ratio (C/No = 10[log_10(S/N_0)] (db-Hz)).
  If a data entry does not contain a measurement for a satellite that was
  previously detected, a `numpy.nan` value is assigned.

  Args:
    novatel_obs: Log of NovAtelObservations data.
    satellite_names: A set where the satellites names found are added to.

  Returns:
    A map of satellite names to C/No data.
  """
  prn = novatel_obs['range']['prn']
  num_obs = novatel_obs['range']['num_obs']
  cn0 = novatel_obs['range']['cn0']
  status_bits = novatel_obs['range']['status_bits']

  cn0_by_satellite = {}
  for i, cn0_value in enumerate(cn0):
    for obs_idx in range(num_obs[i]):
      satellite_name = _get_satellite_name(prn[i, obs_idx],
                                           status_bits[i, obs_idx])
      if satellite_name is not None:
        if satellite_names is not None:
          satellite_names.add(satellite_name)
        if satellite_name not in cn0_by_satellite:
          # TODO: Consider adding a parameter to use 0 instead of
          # numpy.nan in case we want to see when exactly the cn0 drops.
          cn0_by_satellite[satellite_name] = numpy.full_like(cn0[:, 0],
                                                             numpy.nan)
        if cn0_value[obs_idx] > 0:
          cn0_by_satellite[satellite_name].put(i, cn0_value[obs_idx])
  return cn0_by_satellite


def _get_satellite_name(prn, status_bits):
  """Build a satellite name with the system name, PRN/slot, and signal type.

  Args:
    prn: The satellite PRN number of range measurement.
    status_bits: The channel tracking status of the NovAtelObservations range
        message.

  Returns:
    A string with the full satellite name, or None if the values in the status
    bits are not supported.
  """
  if prn == 0 or status_bits == 0:
    return None

  satellite_system_code = (status_bits >> 16) & 0x7  # Extract bits 19-16.
  if satellite_system_code not in _SATELLITE_SYSTEM:
    print 'Unsupported satellite system: {0:b}'.format(satellite_system_code)
    satellite_system_name = 'UNKNOWN_SATELLITE'
  else:
    satellite_system_name = _SATELLITE_SYSTEM[satellite_system_code]

  signal_type_code = (status_bits >> 21) & 0x1F  # Extract bits 25-21.
  if signal_type_code not in _SIGNAL_TYPE[satellite_system_name]:
    print 'Unsupported signal type: {0:b}'.format(signal_type_code)
    signal_type_name = 'UNKNOWN_SIGNAL_TYPE'
  else:
    signal_type_name = _SIGNAL_TYPE[satellite_system_name][signal_type_code]

  return '{0} #{1}: {2}'.format(satellite_system_name, prn, signal_type_name)

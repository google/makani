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

"""Plots relating to the controller's outputs."""

from makani.analysis.plot.python import mplot
from makani.avionics.common import plc_messages
from makani.control import system_types
from makani.lib.python import c_helpers
from matplotlib.pyplot import fill_between
from matplotlib.pyplot import plot
from matplotlib.pyplot import yticks
import numpy as np

MFig = mplot.PlotGroup.MFig  # pylint: disable=invalid-name

_MOTORS_HELPER = c_helpers.EnumHelper('Motor', system_types)
_FLAPS_HELPER = c_helpers.EnumHelper('Flap', system_types)
_GROUND_STATION_MODE_HELPER = c_helpers.EnumHelper('GroundStationMode',
                                                   plc_messages)

_COLORS = [
    '#777777',
    '#0000ff',
    '#ff0000',
    '#ff77ff',
    '#ffff00',
    '#000077',
    '#770000',
    '#000000',
]


class Plots(mplot.PlotGroup):
  """Plots of the controller's outputs."""

  @MFig(title='Thrust', ylabel='Thrust [N]', xlabel='Time [s]')
  def PlotThrust(self, co, c, s):
    plot(c['time'], c['thrust_moment_avail']['thrust'], '-',
         label='Thrust Avail')
    plot(c['time'], c['thrust_moment']['thrust'], 'k:',
         label='Thrust Cmd')
    if s is not None:
      plot(s['time'], s['wing']['fm_rotors']['force']['x'], 'r',
           label='Sim Thrust')

  @MFig(title='Moments', ylabel='Moment [N-m]', xlabel='Time [s]')
  def PlotMoment(self, co, c, s):
    dims = ['x', 'y', 'z']
    for i, d in enumerate(dims):
      plot(c['time'], c['thrust_moment']['moment'][d], '--',
           label='Moment Cmd ' + d, color=_COLORS[i])
      plot(c['time'], c['thrust_moment_avail']['moment'][d], '-',
           label='Moment Avail ' + d, color=_COLORS[i])
      if s is not None:
        plot(s['time'], s['wing']['fm_rotors']['moment'][d], '-.',
             label='Sim ' + d, color=_COLORS[i])

  @MFig(title='Flaps', ylabel='Deflection [deg]', xlabel='Time [s]')
  def PlotFlaps(self, co, c, s, flap=None):
    if flap is None:
      flaps = _FLAPS_HELPER.Values()
    else:
      flaps = [flap]
    for i in flaps:
      plot(c['time'], np.rad2deg(co['flaps'][:, i]),
           color=_COLORS[i], linestyle=':',
           label='%s cmd' % _FLAPS_HELPER.ShortName(i))
      plot(c['time'], np.rad2deg(c['control_input']['flaps'][:, i]),
           color=_COLORS[i],
           label='%s' % _FLAPS_HELPER.ShortName(i))

  @MFig(title='Motors', ylabel='Motor Speeds [rad/s]', xlabel='Time [s]')
  def PlotMotors(self, co, c, s, motor=None):
    if motor is None:
      motors = _MOTORS_HELPER.Values()
    else:
      motors = [motor]
    for i in motors:
      # TODO: Expand plot to include torque and lower speed.
      plot(c['time'], co['motor_speed_upper_limit'][:, i],
           color=_COLORS[i], linestyle='-.',
           label='%s cmd' % _MOTORS_HELPER.ShortName(i))
      plot(c['time'], c['control_input']['rotors'][:, i],
           color=_COLORS[i],
           label='%s' % _MOTORS_HELPER.ShortName(i))

  @MFig(title='Winch Velocity Command', ylabel='[m/s]', xlabel='Time [s]')
  def PlotWinchVelocity(self, co, c, s):
    plot(c['time'], co['winch_vel_cmd'])

  @MFig(title='Detwist Command', ylabel='Detwist pos [deg]', xlabel='Time [s]')
  def PlotDetwistCmd(self, co, c, s):
    plot(c['time'], np.rad2deg(co['detwist_cmd']), label='Cmd')
    if s is not None:
      plot(s['time'], np.rad2deg(s['gs02']['detwist_angle']), '-.',
           label='Sim')

  @MFig(title='Blinky Light', ylabel='Is it on? [bool]', xlabel='Time [s]')
  def PlotBlinkyLight(self, co, c, s):
    plot(c['time'], co['light'])

  @MFig(title='Ground Station Mode Request', ylabel='GS mode [#]',
        xlabel='Time [s]')
  def PlotGroundStationMode(self, co, c, s):
    plot(c['time'], co['gs_mode_request'], label='Cmd')
    if s is not None:
      plot(s['time'], s['gs02']['mode'], '-.', label='Sim')
    yticks(_GROUND_STATION_MODE_HELPER.Values(),
           _GROUND_STATION_MODE_HELPER.ShortNames())

  @MFig(title='Ground Station Targetting Data', ylabel='Azimuth [deg]',
        xlabel='Time [s]')
  def PlotGroundStationTargettingData(self, co, c, s):
    gs_target_azi_deg = np.rad2deg(co['gs_azi_cmd']['target'])
    gs_target_azi_deadzone_deg = np.rad2deg(co['gs_azi_cmd']['dead_zone'])
    plot(c['time'], gs_target_azi_deg, label='target azi (cmd)')
    fill_between(c['time'],
                 gs_target_azi_deg - gs_target_azi_deadzone_deg,
                 gs_target_azi_deg + gs_target_azi_deadzone_deg,
                 alpha=0.2, label='target azi deadzone')
    plot(s['time'], np.rad2deg(s['gs02']['azimuth']))

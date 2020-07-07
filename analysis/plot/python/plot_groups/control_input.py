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

"""Plots relating to the controller's inputs."""
import collections

from makani.analysis.control import geometry
from makani.analysis.plot.python import mplot
from makani.avionics.common import plc_messages
from makani.control import control_types
from makani.lib.python import c_helpers
from matplotlib.pyplot import bar
from matplotlib.pyplot import legend
from matplotlib.pyplot import plot
from matplotlib.pyplot import ylim
from matplotlib.pyplot import yticks
import numpy as np

MFig = mplot.PlotGroup.MFig  # pylint: disable=invalid-name

_GPS_SOLUTION_TYPE_HELPER = c_helpers.EnumHelper(
    'GpsSolutionType', control_types)

_LOADCELL_SENSOR_TYPE_HELPER = c_helpers.EnumHelper(
    'LoadcellSensor', control_types)

_GROUND_STATION_MODE_HELPER = c_helpers.EnumHelper(
    'GroundStationMode', plc_messages)

_WING_GPS_HELPER = c_helpers.EnumHelper(
    'WingGpsReceiver', control_types)


class Plots(mplot.PlotGroup):
  """Plots of the controller's inputs."""

  def _PlotGpsPositionEcef(self, time, pos_ecef, pos_sigma_ecef, label):
    dims = ['x', 'y', 'z']
    pos_offset = {d: np.median(pos_ecef[d]) for d in dims}
    pos = {d: pos_ecef[d] - pos_offset[d] for d in dims}
    labels = ['%s %s ECEF -(%g)' % (label, d, pos_offset[d]) for d in dims]
    mplot.PlotVec3(time, pos, labels=labels)
    mplot.PlotVec3(time, {d: pos[d] + pos_sigma_ecef[d] for d in dims},
                   linestyle=':', labels=[None for d in dims])
    mplot.PlotVec3(time, {d: pos[d] - pos_sigma_ecef[d] for d in dims},
                   linestyle=':', labels=[None for d in dims])

  def _PlotGpsVelocityEcef(self, time, vel_ecef, vel_sigma_ecef, label):
    dims = ['x', 'y', 'z']
    vel = {d: vel_ecef[d] for d in dims}
    labels = ['%s %s ECEF' % (label, d) for d in dims]
    mplot.PlotVec3(time, vel, labels=labels)
    mplot.PlotVec3(time, {d: vel[d] + vel_sigma_ecef[d] for d in dims},
                   linestyle=':', labels=[None for d in dims])
    mplot.PlotVec3(time, {d: vel[d] - vel_sigma_ecef[d] for d in dims},
                   linestyle=':', labels=[None for d in dims])

  @MFig(title='Loadcells', ylabel='Force [N]', xlabel='Time [s]')
  def PlotLoadcells(self, ci, c, s):
    for i, label in enumerate(_LOADCELL_SENSOR_TYPE_HELPER.ShortNames()):
      plot(c['time'], ci['loadcells'][:, i], label=label)
    legend()

  @MFig(title='Differential Pressures', ylabel='Pressure [Pa]',
        xlabel='Time [s]')
  def PlotDiffPressures(self, ci, c, s):
    pressure_names = ['alpha', 'beta', 'dyn']
    for p, color in zip(pressure_names, ['b', 'g', 'r']):
      plot(c['time'], ci['pitots']['diff'][p + '_press'][:, 0], color,
           label='high speed ' + p)
      plot(c['time'], ci['pitots']['diff'][p + '_press'][:, 1], color + '--',
           label='low speed ' + p)

  @MFig(title='Static Pressure', ylabel='Pressure [Pa]', xlabel='Time [s]')
  def PlotBaroPressure(self, ci, c, s):
    plot(c['time'], ci['pitots']['stat_press'][:, 0], label='HighSpeed')
    plot(c['time'], ci['pitots']['stat_press'][:, 1], label='LowSpeed')

  @MFig(title='Position Sigma', ylabel='Position Sigma [m]', xlabel='Time [s]')
  def PlotWingGpsPositionSigmas(self, ci, c, s):
    for i in range(control_types.kNumWingGpsReceivers):
      mplot.PlotVec3(c['time'], ci['wing_gps']['pos_sigma'][:, i],
                     label=_WING_GPS_HELPER.ShortName(i))

  @MFig(title='Velocity Sigma', ylabel='Velocity Sigma [m/s]',
        xlabel='Time [s]')
  def PlotWingGpsVelocitySigmas(self, ci, c, s):
    for i in range(control_types.kNumWingGpsReceivers):
      mplot.PlotVec3(c['time'], ci['wing_gps']['vel_sigma'][:, i],
                     label=_WING_GPS_HELPER.ShortName(i))

  @MFig(title='Wing GPS A Pos.', ylabel='Position ECEF [m]', xlabel='Time [s]')
  def PlotWingGpsAPositionEcef(self, ci, c, s):
    self._PlotGpsPositionEcef(c['time'], ci['wing_gps']['pos'][:, 0],
                              ci['wing_gps']['pos_sigma'][:, 0], 'Wing GPS A')

  @MFig(title='Wing GPS B Pos.', ylabel='Position ECEF [m]', xlabel='Time [s]')
  def PlotWingGpsBPositionEcef(self, ci, c, s):
    self._PlotGpsPositionEcef(c['time'], ci['wing_gps']['pos'][:, 1],
                              ci['wing_gps']['pos_sigma'][:, 1], 'Wing GPS B')

  @MFig(title='Wing GPS A Vel.', ylabel='Velocity ECEF [m/s]',
        xlabel='Time [s]')
  def PlotWingGpsAVelocityEcef(self, ci, c, s):
    self._PlotGpsVelocityEcef(c['time'], ci['wing_gps']['vel'][:, 0],
                              ci['wing_gps']['vel_sigma'][:, 0], 'Wing GPS A')

  @MFig(title='Wing GPS B Vel.', ylabel='Velocity ECEF [m/s]',
        xlabel='Time [s]')
  def PlotWingGpsBVelocityEcef(self, ci, c, s):
    self._PlotGpsVelocityEcef(c['time'], ci['wing_gps']['vel'][:, 1],
                              ci['wing_gps']['vel_sigma'][:, 1], 'Wing GPS B')

  @MFig(title='Wing GPS Solution Type', ylabel='Solution Type',
        xlabel='Time [s]')
  def PlotWingGpsSolutionType(self, ci, c, s):
    for i in range(control_types.kNumWingGpsReceivers):
      plot(c['time'], ci['wing_gps']['pos_sol_type'][:, i] + 0.05 * i,
           'C%d-' % i, label=_WING_GPS_HELPER.ShortName(i) + ' Pos Type')
      plot(c['time'], ci['wing_gps']['vel_sol_type'][:, i] + 0.05 * i,
           'C%d:' % i, label=_WING_GPS_HELPER.ShortName(i) + ' Vel Type')
    yticks(_GPS_SOLUTION_TYPE_HELPER.Values(),
           _GPS_SOLUTION_TYPE_HELPER.ShortNames())

  @MFig(title='Wing GPS Solution Type Histogram', pedantic=False,
        xticks=[val + 0.5 for val in _GPS_SOLUTION_TYPE_HELPER.Values()],
        xticklabels=_GPS_SOLUTION_TYPE_HELPER.ShortNames(),
        xticklabels_orientation='vertical',
        axvline_values=_GPS_SOLUTION_TYPE_HELPER.Values(),
        collect_labels=True, axgrid=False)
  def PlotWingGpsSolutionTypeHistogram(self, ci, c, s):
    solutions_set = set()
    counts = {}
    for i in range(control_types.kNumWingGpsReceivers):
      gps_name = _WING_GPS_HELPER.ShortName(i)
      counts[gps_name] = {
          'pos': collections.Counter(ci['wing_gps']['pos_sol_type'][:, i]),
          'vel': collections.Counter(ci['wing_gps']['vel_sol_type'][:, i]),
      }
      solutions_set.update(counts[gps_name]['pos'].keys())
      solutions_set.update(counts[gps_name]['vel'].keys())
    width = 1.0 / (2.0 * len(solutions_set))
    for x, gps_name in enumerate(sorted(counts)):
      for gps_receiver_idx, count in counts[gps_name]['pos'].items():
        bar(width / 2.0 + gps_receiver_idx + x * (width*2 + 0.02), count,
            width=width, color='C%d'%x, label=gps_name+' Pos Type')
      for gps_receiver_idx, count in counts[gps_name]['vel'].items():
        bar(width / 2.0 + gps_receiver_idx + x * (width*2 + 0.02) + width,
            count, width=width, color='C%d'%x, label=gps_name+' Vel Type',
            edgecolor='black', ls='dashed')

  @MFig(title='Joystick Throttle', ylabel='Throttle [#]', xlabel='Time [s]')
  def PlotJoystickThrottle(self, ci, c, s):
    plot(c['time'], ci['joystick']['throttle'],
         label='Joystick Throttle')

  # perch
  @MFig(title='Winch Position', ylabel='Winch Pos [m]', xlabel='Time [s]')
  def PlotWinchPosition(self, ci, c, s):
    plot(c['time'], ci['perch']['winch_pos'], label='Winch Position')

  @MFig(title='Perch Heading', ylabel='Perch Heading [deg]', xlabel='Time [s]')
  def PlotPerchHeading(self, ci, c, s):
    plot(c['time'], np.rad2deg(ci['perch']['perch_heading']),
         label='Perch Heading')

  @MFig(title='Perch Azimuth', ylabel='Perch Azimuth [deg]', xlabel='Time [s]')
  def PlotPerchAzimuth(self, ci, c, s):
    plot(c['time'], np.rad2deg(ci['perch']['perch_azi']),
         label='Perch Azimuth')
    if s is not None:
      plot(s['time'], np.rad2deg(s['gs02']['azimuth']),
           label='Gs02 Azimuth [sim]')

  @MFig(title='Platform Attitude', ylabel='Angles [deg]', xlabel='Time [s]')
  def PlotPlatformAttitude(self, ci, c, s):
    # Plot the perch_heading first because it is noiser than the
    # ground estimator's filtered version.
    plot(c['time'], np.rad2deg(ci['perch']['perch_heading']),
         label='perch_heading')

    # The following operation takes a few seconds.
    dcm_g2p = ci['ground_estimate']['dcm_g2p']
    eulers = np.array([geometry.DcmToAngle(np.matrix(m['d'])) for m in dcm_g2p])

    plot(c['time'], np.rad2deg(eulers[:, 2]), label='roll (ground estimator)')
    plot(c['time'], np.rad2deg(eulers[:, 1]), label='pitch (ground estimator)')
    plot(c['time'], np.rad2deg(eulers[:, 0]), label='yaw (ground estimator)')

    # Note: Perch azi will only agree with the above if vessel heading
    # is zero (for example in the onshore case at Parker Ranch).
    plot(c['time'], np.rad2deg(ci['perch']['perch_azi']),
         label='perch_azi', linestyle='--')

  @MFig(title='Levelwind Ele', ylabel='Angle [rad]', xlabel='Time [s]')
  def PlotLevelwindEle(self, ci, c, s):
    plot(c['time'], ci['perch']['levelwind_ele'],
         label='Levelwind Ele')

  # gs_sensors
  @MFig(title='Ground Station Mode', ylabel='Mode [enum]', xlabel='Time [s]')
  def PlotGroundStationMode(self, ci, c, s):
    plot(c['time'], ci['gs_sensors']['mode'])
    yticks(_GROUND_STATION_MODE_HELPER.Values(),
           _GROUND_STATION_MODE_HELPER.ShortNames())

  @MFig(title='Ground Station Transform Stage', ylabel='Stage [#]',
        xlabel='Time [s]')
  def PlotGroundStationTransformStage(self, ci, c, s):
    plot(c['time'], ci['gs_sensors']['transform_stage'])

  @MFig(title='Proximity Sensor', ylabel='Proximity [bool]', xlabel='Time [s]')
  def PlotProximitySensor(self, ci, c, s):
    plot(c['time'], ci['gs_sensors']['proximity'], label='proximity')

  @MFig(title='GSG', ylabel='Angle [deg]', xlabel='Time [s]')
  def PlotGsg(self, ci, c, s):
    plot(c['time'], np.rad2deg(ci['gsg']['azi']), label='Gsg Azi')
    plot(c['time'], np.rad2deg(ci['gsg']['ele']), label='Gsg Ele')

  @MFig(title='GSG Yoke Angle', ylabel='Angle [deg]', xlabel='Time [s]')
  def PlotGsgYoke(self, ci, c, s):
    plot(c['time'], np.rad2deg(ci['gsg']['azi']), label='Gsg Yoke')
    ii = (ci['gs_sensors']['mode'] ==
          _GROUND_STATION_MODE_HELPER.Value('HighTension'))
    # TODO: Put these limits into the configuration system.
    plot(c['time'][ii], -73.0 * np.ones(c['time'][ii].shape), 'k--')
    plot(c['time'][ii], 73.0 * np.ones(c['time'][ii].shape), 'k--')
    plot(c['time'][ii], -65.0 * np.ones(c['time'][ii].shape), 'k--', alpha=0.5)
    plot(c['time'][ii], 65.0 * np.ones(c['time'][ii].shape), 'k--', alpha=0.5)
    ylim([-90.0, 90.0])

  @MFig(title='GSG Termination Angle', ylabel='Angle [deg]', xlabel='Time [s]')
  def PlotGsgTermination(self, ci, c, s):
    plot(c['time'], np.rad2deg(ci['gsg']['ele']), label='Gsg Termination')
    ii = (ci['gs_sensors']['mode'] ==
          _GROUND_STATION_MODE_HELPER.Value('HighTension'))
    # TODO: Put these limits into the configuration system.
    plot(c['time'][ii], -35.0 * np.ones(c['time'][ii].shape), 'k--')
    plot(c['time'][ii], 35.0 * np.ones(c['time'][ii].shape), 'k--')
    plot(c['time'][ii], -31.0 * np.ones(c['time'][ii].shape), 'k--', alpha=0.5)
    plot(c['time'][ii], 31.0 * np.ones(c['time'][ii].shape), 'k--', alpha=0.5)
    ylim([-90.0, 90.0])

  @MFig(title='Ground Estimate Position', ylabel='m', xlabel='Time [s]')
  def PlotGroundEstimatePosition(self, ci, c, s):
    mplot.PlotVec3(c['time'], ci['ground_estimate']['Xg'],
                   label='vessel_pos_g [estimate]')
    if s is not None:
      # Note: These can only be expected to agree if the simulator and
      # vessel estimator agree on the origin of the g-frame. That's a
      # good reason to standardize the ned origin and express these in the
      # ned frame.
      mplot.PlotVec3(s['time'], s['buoy']['Xg'],
                     label='vessel_pos_g [sim]',
                     linestyle='--')

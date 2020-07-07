#!/usr/bin/python
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

"""Generates plots from regression output."""

import importlib
import os
import shutil
import sys

import gflags
import h5py
import makani
from makani.control import common as control_common
from makani.control import control_types
from makani.lib.python import c_helpers
import matplotlib
import numpy
from scipy import stats

matplotlib.use('Agg')
pylab = importlib.import_module('pylab')

gflags.DEFINE_string('old_log', None, 'Path to old log file.')
gflags.DEFINE_string('new_log', None, 'Path to new log file.')
gflags.DEFINE_string('output_dir', None, 'Output directory.')

gflags.MarkFlagAsRequired('new_log')
gflags.MarkFlagAsRequired('old_log')
gflags.MarkFlagAsRequired('output_dir')

FLAGS = gflags.FLAGS
_STYLE_OLD_CMD = 'C0--'
_STYLE_NEW_CMD = 'C1--'
_STYLE_OLD_ACTUAL = 'C0-'
_STYLE_NEW_ACTUAL = 'C1-'


class RegressionPlotter(object):
  """Generates plots from regression data."""

  def __init__(self, old_log, new_log):
    self.old_sim_telem = (old_log['messages']['kAioNodeSimulator']
                          ['kMessageTypeSimTelemetry']['message'])
    self.new_sim_telem = (new_log['messages']['kAioNodeSimulator']
                          ['kMessageTypeSimTelemetry']['message'])

    self.old_control_telem = (old_log['messages']['kAioNodeControllerA']
                              ['kMessageTypeControlDebug']['message'])
    self.new_control_telem = (new_log['messages']['kAioNodeControllerA']
                              ['kMessageTypeControlDebug']['message'])

    self.old_control_time = self.old_control_telem['time']
    self.new_control_time = self.new_control_telem['time']
    self.old_sim_time = self.old_sim_telem['time']
    self.new_sim_time = self.new_sim_telem['time']

    # If the timestamps of the old and new telemetries disagree, then data in
    # difference plots will be interpolated and restricted to the intersection
    # of the two time intervals.
    self._interpolate_control_diffs, self._control_diff_time = (
        self._InitDiffInterpolation(self.old_control_time,
                                    self.new_control_time))
    self._interpolate_sim_diffs, self._sim_diff_time = (
        self._InitDiffInterpolation(self.old_sim_time,
                                    self.new_sim_time))

  def _InitDiffInterpolation(self, old_time, new_time):
    """Initializes interpolation for difference plots.

    Args:
      old_time: Array of old time values.
      new_time: Array of new time values.

    Returns:
      (needs_interpolation, merged_time_values): A Boolean indicating whether
          interpolation is needed, and an array of merged time values.
    """

    if len(old_time) == len(new_time) and (old_time == new_time).all():
      return False, old_time
    else:
      t_lower = max(min(old_time), min(new_time))
      t_upper = min(max(old_time), max(new_time))

      # Sample times are arbitrarily chosen to come from new_time.
      merged_time = new_time[numpy.logical_and(
          new_time >= t_lower, new_time <= t_upper)]

      return True, merged_time

  def Retrieve(self, telem, reference):
    value = telem
    for field in reference.split('.'):
      value = value[field]
    return value

  def RetrieveControlOld(self, reference):
    return self.Retrieve(self.old_control_telem, reference)

  def RetrieveControlNew(self, reference):
    return self.Retrieve(self.new_control_telem, reference)

  def PlotControlOldData(self, data, *plot_args, **plot_kwargs):
    pylab.plot(self.old_control_time, data,
               *plot_args, **plot_kwargs)

  def PlotControlNewData(self, data, *plot_args, **plot_kwargs):
    pylab.plot(self.new_control_time, data,
               *plot_args, **plot_kwargs)

  def PlotSimOld(self, reference, *plot_args, **plot_kwargs):
    pylab.plot(self.old_sim_time,
               self.Retrieve(self.old_sim_telem, reference),
               *plot_args, **plot_kwargs)

  def PlotSimNew(self, reference, *plot_args, **plot_kwargs):
    pylab.plot(self.new_sim_time,
               self.Retrieve(self.new_sim_telem, reference),
               *plot_args, **plot_kwargs)

  def PlotControlOld(self, reference, *plot_args, **plot_kwargs):
    pylab.plot(self.old_control_time,
               self.Retrieve(self.old_control_telem, reference),
               *plot_args, **plot_kwargs)

  def PlotControlNew(self, reference, *plot_args, **plot_kwargs):
    pylab.plot(self.new_control_time,
               self.Retrieve(self.new_control_telem, reference),
               *plot_args, **plot_kwargs)

  def PlotSimDifference(self, reference, *plot_args, **plot_kwargs):
    old_data = self.Retrieve(self.old_sim_telem, reference)
    new_data = self.Retrieve(self.new_sim_telem, reference)
    if self._interpolate_sim_diffs:
      old_data = numpy.interp(self._sim_diff_time, self.old_sim_time, old_data)
      new_data = numpy.interp(self._sim_diff_time, self.new_sim_time, new_data)

    pylab.plot(self._sim_diff_time, old_data - new_data, *plot_args,
               **plot_kwargs)

  def PlotControlDifference(self, reference, *plot_args, **plot_kwargs):
    old_data = self.Retrieve(self.old_control_telem, reference)
    new_data = self.Retrieve(self.new_control_telem, reference)
    if self._interpolate_control:
      old_data = numpy.interp(self._control_diff_time, self.old_control_time,
                              old_data)
      new_data = numpy.interp(self._control_diff_time, self.new_control_time,
                              new_data)

    pylab.plot(self._control_diff_time, old_data - new_data, *plot_args,
               **plot_kwargs)


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  if not os.path.exists(FLAGS.output_dir):
    os.makedirs(FLAGS.output_dir)
  shutil.copyfile(os.path.join(makani.HOME,
                               'analysis/util/regression_dashboard.html'),
                  os.path.join(FLAGS.output_dir, 'index.html'))

  plotter = RegressionPlotter(h5py.File(FLAGS.old_log, 'r'),
                              h5py.File(FLAGS.new_log, 'r'))

  flight_modes = c_helpers.EnumHelper('FlightMode', control_types)
  fm = plotter.Retrieve(plotter.new_control_telem, 'flight_mode')
  fm_crosswind = flight_modes.Value('CrosswindNormal')
  fm_crosswind_idx = numpy.where(fm == fm_crosswind)[0]
  if fm_crosswind_idx.size > 0:
    xlim_crosswind = (plotter.new_control_time[fm_crosswind_idx[0]],
                      plotter.new_control_time[fm_crosswind_idx[-1]])
  else:
    xlim_crosswind = None
  fm_transin_idx = numpy.where(fm == flight_modes.Value('TransIn'))[0]

  # Plot the components of wing position for both logs.
  pylab.figure(figsize=(16, 9))
  pylab.title('Comparison of wing position, ground frame')
  plotter.PlotSimOld('wing.Xg.x', 'C3--', alpha=0.5, label='x (old)')
  plotter.PlotSimNew('wing.Xg.x', 'C3-', label='x (new)')
  plotter.PlotSimOld('wing.Xg.y', 'C1--', alpha=0.5, label='y (old)')
  plotter.PlotSimNew('wing.Xg.y', 'C1-', label='y (new)')
  plotter.PlotSimOld('wing.Xg.z', 'C2--', alpha=0.5, label='z (old)')
  plotter.PlotSimNew('wing.Xg.z', 'C2-', label='z (new)')
  pylab.ylabel('Position [m]')
  pylab.xlabel('Time [s]')
  pylab.legend(loc='lower left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/wing_position_compare.svg')
  pylab.close()

  # Generate a plot with the difference in each component of wing position.
  pylab.figure(figsize=(16, 9))
  pylab.title('Difference in wing position, ground frame')
  plotter.PlotSimDifference('wing.Xg.x', 'C3-', label='x')
  plotter.PlotSimDifference('wing.Xg.y', 'C1-', label='y')
  plotter.PlotSimDifference('wing.Xg.z', 'C2-', label='z')
  pylab.xlabel('Time [s]')
  pylab.ylabel('Position difference [m]')
  pylab.legend(loc='lower left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/wing_position_diff.svg')
  pylab.close()

  pylab.figure(figsize=(16, 9))
  pylab.title('Thrust Cmd')
  plotter.PlotControlOld('thrust_moment.thrust', 'r-', label='thrust cmd (old)')
  plotter.PlotControlNew('thrust_moment.thrust', 'b-', label='thrust cmd (new)')
  plotter.PlotControlOld('thrust_moment_avail.thrust', 'r--',
                         label='thrust avail (old)')
  plotter.PlotControlNew('thrust_moment_avail.thrust', 'b--',
                         label='thrust avail (new)')
  plotter.PlotSimOld('wing.fm_rotors.force.x', 'r:',
                     label='thrust actual (old)')
  plotter.PlotSimNew('wing.fm_rotors.force.x', 'b:',
                     label='thrust actual (new)')
  pylab.xlabel('Time [s]')
  pylab.ylabel('Thrust [N]')
  pylab.legend(loc='lower left')
  pylab.grid()
  fm_accel_idx = numpy.where(fm == flight_modes.Value('HoverAccel'))[0]
  if fm_accel_idx.size > 0:
    t_accel = plotter.new_control_time[fm_accel_idx[0]]
    pylab.xlim((t_accel - 5.0, t_accel + 5.0))
    pylab.title('Thrust cmd at transition to HoverAccel')
    pylab.savefig(FLAGS.output_dir + '/accel_thrust_cmd.svg')
  fm_transout_idx = numpy.where(fm == flight_modes.Value('HoverTransOut'))[0]
  if fm_transout_idx.size > 0:
    t_transout = plotter.new_control_time[fm_transout_idx[0]]
    pylab.xlim((t_transout - 5.0, t_transout + 5.0))
    pylab.title('Thrust cmd at transition to HoverTransOut')
    pylab.savefig(FLAGS.output_dir + '/transout_thrust_cmd.svg')
  fm_transin_idx = numpy.where(fm == flight_modes.Value('TransIn'))[0]
  if fm_transin_idx.size > 0:
    t_transin = plotter.new_control_time[fm_transin_idx[0]]
    pylab.xlim((t_transin - 5.0, t_transin + 5.0))
    pylab.title('Thrust cmd at transition to TransIn')
    pylab.savefig(FLAGS.output_dir + '/transin_thrust_cmd.svg')
  pylab.close()

  pylab.figure(figsize=(16, 9))
  pylab.title('Power')
  plotter.PlotSimOld('power_sys.P_elec', 'r-', label='power (old)')
  plotter.PlotSimNew('power_sys.P_elec', 'b-', label='power (new)')
  pylab.xlabel('Time [s]')
  pylab.ylabel('Power Generated [W]')
  pylab.legend(loc='lower left')
  pylab.grid()
  if fm_accel_idx.size > 0:
    pylab.xlim((t_accel - 5.0, t_accel + 5.0))
    pylab.title('Electrical power at transition to HoverAccel')
    pylab.savefig(FLAGS.output_dir + '/accel_power.svg')
  if fm_transout_idx.size > 0:
    pylab.xlim((t_transout - 5.0, t_transout + 5.0))
    pylab.title('Electrical power at transition to HoverTransOut')
    pylab.savefig(FLAGS.output_dir + '/transout_power.svg')
  if fm_transin_idx.size > 0:
    pylab.xlim((t_transin - 5.0, t_transin + 5.0))
    pylab.title('Electrical power at transition to TransIn')
    pylab.savefig(FLAGS.output_dir + '/transin_power.svg')
  pylab.close()

  pylab.figure(figsize=(16, 9))
  pylab.title('Airspeed Estimate')
  plotter.PlotControlNew('state_est.Vb.x', 'r-', label='Vb.x (new)')
  apparent_wind = plotter.RetrieveControlNew('state_est.apparent_wind.vector.x')
  pylab.plot(plotter.new_control_time, -apparent_wind, 'm-',
             label='apparent wind.vector.x (new)')
  plotter.PlotControlNew('state_est.apparent_wind.sph_f.v', 'b-',
                         label='apparent wind.sph_f.v (new)')
  pylab.xlabel('Time [s]')
  pylab.ylabel('Speed [m/s]')
  pylab.legend(loc='lower left')
  pylab.grid()
  if fm_accel_idx.size > 0:
    pylab.xlim((t_accel - 5.0, t_accel + 5.0))
    pylab.title('Airspeed estimate at transition to HoverAccel')
    pylab.savefig(FLAGS.output_dir + '/accel_airspeed.svg')
  if fm_transout_idx.size > 0:
    pylab.xlim((t_transout - 5.0, t_transout + 5.0))
    pylab.title('Airspeed estimate at transition to HoverTransOut')
    pylab.savefig(FLAGS.output_dir + '/transout_airspeed.svg')
  if fm_transin_idx.size > 0:
    pylab.xlim((t_transin - 5.0, t_transin + 5.0))
    pylab.title('Airspeed estimate at transition to TransIn')
    pylab.savefig(FLAGS.output_dir + '/transin_airspeed.svg')
  pylab.close()

  # Plot aero angles.
  pylab.figure(figsize=(16, 9))
  pylab.title('Crosswind angle of attack [sim]')
  plotter.PlotSimOld('wing.apparent_wind_b.alpha', _STYLE_OLD_ACTUAL,
                     label='alpha (old)')
  plotter.PlotSimNew('wing.apparent_wind_b.alpha', _STYLE_NEW_ACTUAL,
                     label='alpha (new)')
  plotter.PlotControlOld('crosswind.alpha_cmd', _STYLE_OLD_CMD,
                         label='alpha cmd (old)')
  plotter.PlotControlNew('crosswind.alpha_cmd', _STYLE_NEW_CMD,
                         label='alpha cmd (new)')
  pylab.xlim(xlim_crosswind)
  pylab.xlabel('Time [s]')
  pylab.ylabel('Angle [rad]')
  pylab.ylim([0.0, 0.1])
  pylab.legend(loc='lower left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/wing_alpha.svg')
  pylab.close()

  pylab.figure(figsize=(16, 9))
  pylab.title('Crosswind side-slip [sim]')
  plotter.PlotSimOld('wing.apparent_wind_b.beta', _STYLE_OLD_ACTUAL,
                     label='beta (old)')
  plotter.PlotSimNew('wing.apparent_wind_b.beta', _STYLE_NEW_ACTUAL,
                     label='beta (new)')
  plotter.PlotControlOld('crosswind.beta_cmd', _STYLE_OLD_CMD,
                         label='beta cmd (old)')
  plotter.PlotControlNew('crosswind.beta_cmd', _STYLE_NEW_CMD,
                         label='beta cmd (new)')
  pylab.xlim(xlim_crosswind)
  pylab.xlabel('Time [s]')
  pylab.ylabel('Angle [rad]')
  pylab.ylim([-0.15, 0.05])
  pylab.legend(loc='lower left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/wing_beta.svg')
  pylab.close()

  # Plot flight modes.
  pylab.figure(figsize=(16, 9))
  pylab.title('Sequenced flight modes')

  seq_to_flight_mode = []
  flight_mode = flight_modes.Value('PilotHover')
  while flight_mode not in seq_to_flight_mode:
    seq_to_flight_mode.append(flight_mode)
    flight_mode = control_common.GetNextFlightMode(flight_mode)
  # We use `argsort` to compute the flight-mode-to-sequence mapping given the
  # sequence of flight modes. This approach works only because the flight modes
  # according to GetNextFlightMode are continuous integers starting from 0.
  # And it should include all flight modes for the purpose of plotting.
  assert len(seq_to_flight_mode) == control_types.kNumFlightModes
  flight_mode_to_seq = numpy.argsort(seq_to_flight_mode)
  old_seq_flight_mode = flight_mode_to_seq[
      plotter.RetrieveControlOld('flight_mode')]
  new_seq_flight_mode = flight_mode_to_seq[
      plotter.RetrieveControlNew('flight_mode')]
  plotter.PlotControlOldData(old_seq_flight_mode, _STYLE_OLD_ACTUAL, alpha=0.5,
                             label='old')
  plotter.PlotControlNewData(new_seq_flight_mode, _STYLE_NEW_ACTUAL,
                             label='new')

  seq_names = [flight_modes.ShortName(fm) for fm in seq_to_flight_mode]
  pylab.yticks(range(len(seq_names)), seq_names)
  pylab.xlabel('Time [s]')
  pylab.legend(loc='upper left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/flight_modes.svg')
  pylab.close()

  # Plot ground sensor wind measurement for wind speed reference.
  pylab.figure(figsize=(16, 9))
  pylab.title('Wind speed observed at the ground station.')
  plotter.PlotControlOld('state_est.wind_g.speed_f', _STYLE_OLD_CMD,
                         label='old')
  plotter.PlotControlNew('state_est.wind_g.speed_f', _STYLE_NEW_CMD,
                         label='new')
  pylab.xlim(xlim_crosswind)
  pylab.xlabel('Time [s]')
  pylab.ylabel('Wind speed [m/s]')
  pylab.legend(loc='lower left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/ground_wind_speed.svg')
  pylab.close()

  pylab.figure(figsize=(16, 9))
  pylab.title('Wind direction observed at the ground station.')
  plotter.PlotControlOld('state_est.wind_g.dir_f', _STYLE_OLD_CMD,
                         label='old')
  plotter.PlotControlNew('state_est.wind_g.dir_f', _STYLE_NEW_CMD,
                         label='new')
  pylab.xlim(xlim_crosswind)
  pylab.xlabel('Time [s]')
  pylab.ylabel('Wind direction [rad]')
  pylab.legend(loc='lower left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/ground_wind_direction.svg')
  pylab.close()

  # Plot crosswind radius error.
  def CalcCurrentPosRadiusError(telemetry):
    current_pos_cw = plotter.Retrieve(telemetry, 'crosswind.current_pos_cw')
    radius_cmd = plotter.Retrieve(telemetry, 'crosswind.path_radius_target')
    return radius_cmd - numpy.hypot(current_pos_cw['x'], current_pos_cw['y'])
  new_radius_error = CalcCurrentPosRadiusError(plotter.new_control_telem)
  old_radius_error = CalcCurrentPosRadiusError(plotter.old_control_telem)

  pylab.figure(figsize=(16, 9))
  pylab.title('Crosswind path radius error')
  pylab.plot(plotter.old_control_time, old_radius_error, _STYLE_OLD_ACTUAL,
             label='radius error [m] (old)')
  pylab.plot(plotter.new_control_time, new_radius_error, _STYLE_NEW_ACTUAL,
             label='radius error [m] (new)')
  pylab.xlim(xlim_crosswind)
  pylab.xlabel('Time [s]')
  pylab.ylabel('Radius Error [m]')
  pylab.legend(loc='lower left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/crosswind_radius_error.svg')
  pylab.close()

  pylab.figure(figsize=(16, 9))
  pylab.title('Crosswind airspeed')
  plotter.PlotControlOld('crosswind.airspeed_cmd', _STYLE_OLD_CMD,
                         label='airspeed_cmd old')
  plotter.PlotControlNew('crosswind.airspeed_cmd', _STYLE_NEW_CMD,
                         label='airspeed_cmd new')
  plotter.PlotControlOld('state_est.apparent_wind.sph.v', _STYLE_OLD_ACTUAL,
                         label='airspeed old')
  plotter.PlotControlNew('state_est.apparent_wind.sph.v', _STYLE_NEW_ACTUAL,
                         label='airspeed new')
  pylab.xlim(xlim_crosswind)
  pylab.xlabel('Time [s]')
  pylab.ylabel('Airspeed [m/s]')
  pylab.legend(loc='lower left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/crosswind_airspeed.svg')
  pylab.close()

  airspeed_error_new = (
      plotter.RetrieveControlNew('crosswind.airspeed_cmd') -
      plotter.RetrieveControlNew('state_est.apparent_wind.sph.v'))
  airspeed_error_old = (
      plotter.RetrieveControlOld('crosswind.airspeed_cmd') -
      plotter.RetrieveControlOld('state_est.apparent_wind.sph.v'))
  pylab.figure(figsize=(16, 9))
  pylab.title('Crosswind airspeed error')
  pylab.plot(plotter.old_control_time, airspeed_error_old, _STYLE_OLD_ACTUAL,
             label='airspeed error (old)')
  pylab.plot(plotter.new_control_time, airspeed_error_new, _STYLE_NEW_ACTUAL,
             label='airspeed error (new)')
  pylab.xlim(xlim_crosswind)
  pylab.ylim((-10.0, 10.0))
  pylab.xlabel('Time [s]')
  pylab.ylabel('Airspeed error [m/s]')
  pylab.legend()
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/crosswind_airspeed_error.svg')
  pylab.close()

  pylab.figure(figsize=(16, 9))
  pylab.title('Crosswind Thrust')
  plotter.PlotControlOld('thrust_moment.thrust', _STYLE_OLD_CMD,
                         label='thrust_cmd old')
  plotter.PlotControlNew('thrust_moment.thrust', _STYLE_NEW_CMD,
                         label='thrust cmd new')
  plotter.PlotControlOld('thrust_moment_avail.thrust', _STYLE_OLD_ACTUAL,
                         label='thrust avail old')
  plotter.PlotControlNew('thrust_moment_avail.thrust', _STYLE_NEW_ACTUAL,
                         label='thrust avail new')
  pylab.xlim(xlim_crosswind)
  pylab.xlabel('Time [s]')
  pylab.ylabel('Thrust [N]')
  pylab.legend(loc='lower left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/crosswind_thrust.svg')
  pylab.close()

  pylab.figure(figsize=(16, 9))
  pylab.title('Crosswind Thrust Cmd Components')
  plotter.PlotControlNew('crosswind.thrust_ff', label='thrust_ff')
  plotter.PlotControlNew('crosswind.thrust_fb', label='thrust_fb')
  plotter.PlotControlNew('crosswind.int_thrust', label='int thrust')
  pylab.xlim(xlim_crosswind)
  pylab.xlabel('Time [s]')
  pylab.ylabel('Thrust [N]')
  pylab.legend(loc='lower left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/crosswind_thrust2.svg')
  pylab.close()

  # Plot hover Y position
  def VecGToAziDeg(v):
    return numpy.rad2deg(numpy.arctan2(v['y'], v['x']))
  def VecGToRadius(v):
    return numpy.hypot(v['x'], v['y'])
  def VecGToZCoord(v):
    return v['z']

  raw_pos_cmd_old = plotter.RetrieveControlOld('hover.raw_wing_pos_g_cmd')
  raw_pos_cmd_new = plotter.RetrieveControlNew('hover.raw_wing_pos_g_cmd')

  pos_cmd_old = plotter.RetrieveControlOld('hover.wing_pos_g_cmd')
  pos_cmd_new = plotter.RetrieveControlNew('hover.wing_pos_g_cmd')

  pos_old = plotter.RetrieveControlOld('state_est.Xg')
  pos_new = plotter.RetrieveControlNew('state_est.Xg')

  for params in [
      {
          'title': 'Hover Azimuth',
          'filename': 'hover_azimuth.svg',
          'f': VecGToAziDeg,
          'ylabel': 'Azimuth [deg]'},
      {
          'title': 'Hover Radius',
          'filename': 'hover_radius.svg',
          'f': VecGToRadius,
          'ylabel': 'Radius [m]'},
      {
          'title': 'Hover Pos Z',
          'filename': 'hover_pos_z.svg',
          'f': VecGToZCoord,
          'ylabel': 'Z [m]'},
      ]:
    pylab.figure(figsize=(16, 9))
    pylab.title(params['title'])

    pylab.plot(plotter.old_control_time, params['f'](raw_pos_cmd_old), 'C0:',
               label='raw cmd (old)')
    pylab.plot(plotter.new_control_time, params['f'](raw_pos_cmd_new), 'C1:',
               label='raw cmd (new)')

    pylab.plot(plotter.old_control_time, params['f'](pos_cmd_old),
               _STYLE_OLD_CMD, label='cmd (old)')
    pylab.plot(plotter.new_control_time, params['f'](pos_cmd_new),
               _STYLE_NEW_CMD, label='cmd (new)')

    pylab.plot(plotter.old_control_time, params['f'](pos_old),
               _STYLE_OLD_ACTUAL, label='pos (old)')
    pylab.plot(plotter.new_control_time, params['f'](pos_new),
               _STYLE_NEW_ACTUAL, label='pos (new)')

    pylab.ylabel(params['ylabel'])
    pylab.xlabel('Time [s]')
    pylab.legend(loc='upper left')
    pylab.grid()
    pylab.savefig(FLAGS.output_dir + '/' + params['filename'])
    pylab.close()

  # Plot hover Y velocity
  pylab.figure(figsize=(16, 9))
  pylab.title('Hover Y velocity')
  plotter.PlotControlOld('hover.wing_vel_g_cmd.y', _STYLE_OLD_CMD,
                         label='cmd (old)')
  plotter.PlotControlNew('hover.wing_vel_g_cmd.y', _STYLE_NEW_CMD,
                         label='cmd (new)')
  plotter.PlotControlOld('state_est.Vg.y', _STYLE_OLD_ACTUAL, label='vel (old)')
  plotter.PlotControlNew('state_est.Vg.y', _STYLE_NEW_ACTUAL, label='vel (new)')
  pylab.ylabel('Velocity [m/s]')
  pylab.xlabel('Time [s]')
  pylab.legend(loc='upper left')
  pylab.grid()
  pylab.ylim(-10, 10)
  pylab.savefig(FLAGS.output_dir + '/hover_vel_y.svg')
  pylab.close()

  # Plot hover yaw stuff.
  pylab.figure(figsize=(16, 9))
  pylab.title('Hover yaw')
  plotter.PlotControlOld('hover.angles_cmd.z', _STYLE_OLD_CMD,
                         label='yaw cmd (old)')
  plotter.PlotControlOld('hover.angles.z', _STYLE_OLD_ACTUAL, label='yaw (old)')
  plotter.PlotControlNew('hover.angles_cmd.z', _STYLE_NEW_CMD,
                         label='yaw cmd (new)')
  plotter.PlotControlNew('hover.angles.z', _STYLE_NEW_ACTUAL, label='yaw (new)')
  pylab.ylabel('Angle [rad]')
  pylab.xlabel('Time [s]')
  pylab.legend(loc='upper left')
  pylab.grid()
  pylab.ylim(-0.5, 0.5)
  pylab.savefig(FLAGS.output_dir + '/hover_angles_z.svg')
  pylab.close()

  # Plot hover Z velocity
  pylab.figure(figsize=(16, 9))
  pylab.title('Hover Z velocity')
  plotter.PlotControlOld('hover.wing_vel_g_cmd.z', _STYLE_OLD_CMD,
                         label='cmd (old)')
  plotter.PlotControlNew('hover.wing_vel_g_cmd.z', _STYLE_NEW_CMD,
                         label='cmd (new)')
  plotter.PlotControlOld('state_est.Vg.z', _STYLE_OLD_ACTUAL, label='vel (old)')
  plotter.PlotControlNew('state_est.Vg.z', _STYLE_NEW_ACTUAL, label='vel (new)')
  pylab.ylabel('Velocity [m/s]')
  pylab.xlabel('Time [s]')
  pylab.legend(loc='upper left')
  pylab.grid()
  pylab.ylim(5, -5)
  pylab.savefig(FLAGS.output_dir + '/hover_vel_z.svg')
  pylab.close()

  # Plot hover thrust
  pylab.figure(figsize=(16, 9))
  pylab.title('Hover thrust')
  plotter.PlotControlOld('thrust_moment.thrust', _STYLE_OLD_CMD,
                         label='thrust cmd (old)')
  plotter.PlotControlOld('thrust_moment_avail.thrust', _STYLE_OLD_ACTUAL,
                         label='thrust avail (old)')
  plotter.PlotControlNew('thrust_moment.thrust', _STYLE_NEW_CMD,
                         label='thrust cmd (new)')
  plotter.PlotControlNew('thrust_moment_avail.thrust', _STYLE_NEW_ACTUAL,
                         label='thrust avail (new)')
  pylab.ylabel('Thrust [N]')
  pylab.xlabel('Time [s]')
  pylab.legend(loc='upper left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/hover_thrust.svg')
  pylab.close()

  pylab.figure(figsize=(16, 9))
  pylab.title('Hover yaw moment')
  plotter.PlotControlOld('thrust_moment.moment.z', _STYLE_OLD_CMD,
                         label='yaw moment cmd (old)')
  plotter.PlotControlOld('thrust_moment_avail.moment.z', _STYLE_OLD_ACTUAL,
                         label='yaw moment avail (old)')
  plotter.PlotControlNew('thrust_moment.moment.z', _STYLE_NEW_CMD,
                         label='yaw moment cmd (new)')
  plotter.PlotControlNew('thrust_moment_avail.moment.z', _STYLE_NEW_ACTUAL,
                         label='yaw moment avail (new)')
  pylab.ylabel('Torque [Nm]')
  pylab.xlabel('Time [s]')
  pylab.legend(loc='upper left')
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/hover_yaw_moment.svg')
  pylab.close()

  # Plot tension during hover
  pylab.figure(figsize=(16, 9))
  pylab.title('Hover tension control')
  plotter.PlotControlOld('state_est.tether_force_b.tension_f',
                         _STYLE_OLD_ACTUAL, label='tension [old]')
  plotter.PlotControlNew('state_est.tether_force_b.tension_f',
                         _STYLE_NEW_ACTUAL, label='tension [new]')
  plotter.PlotControlOld('hover.tension_cmd', _STYLE_OLD_CMD,
                         label='tension_cmd [old]')
  plotter.PlotControlNew('hover.tension_cmd', _STYLE_NEW_CMD,
                         label='tension_cmd [new]')
  pylab.ylabel('Tension [N]')
  pylab.xlabel('Time [s]')
  pylab.legend()
  pylab.grid()
  pylab.ylim((-100.0, 25e3))
  pylab.savefig(FLAGS.output_dir + '/hover_tension.svg')
  pylab.close()

  # Plot kite elevation during hover.
  pylab.figure(figsize=(16, 9))
  pylab.title('Kite elevation')
  plotter.PlotControlOld('hover.elevation_cmd', _STYLE_OLD_CMD,
                         label='cmd [old]')
  plotter.PlotControlNew('hover.elevation_cmd', _STYLE_NEW_CMD,
                         label='cmd [new]')
  plotter.PlotControlOld('hover.elevation_fb', label='fb [old]')
  plotter.PlotControlNew('hover.elevation_fb', label='fb [new]')
  pylab.ylabel('Elevation [rad]')
  pylab.xlabel('Time [s]')
  pylab.legend()
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/hover_elevation.svg')
  pylab.close()

  # Plot tether elevation during hover.
  pylab.figure(figsize=(16, 9))
  pylab.title('Tether elevation')
  plotter.PlotSimOld('tether.Xv_start_elevation', _STYLE_OLD_ACTUAL,
                     label='tether elevation [old]')
  plotter.PlotSimNew('tether.Xv_start_elevation', _STYLE_NEW_ACTUAL,
                     label='tether elevation [new]')
  plotter.PlotControlOld('hover.tether_elevation_cmd',
                         _STYLE_OLD_CMD, label='cmd [old]')
  plotter.PlotControlNew('hover.tether_elevation_cmd',
                         _STYLE_NEW_CMD, label='cmd [new]')
  pylab.ylabel('Elevation [rad]')
  pylab.xlabel('Time [s]')
  pylab.legend()
  pylab.grid()
  pylab.ylim((-0.2, 0.4))
  pylab.savefig(FLAGS.output_dir + '/hover_tether_elevation.svg')
  pylab.close()

  # Plot aero force estimates.
  for axis in ['x', 'y', 'z']:
    pylab.figure(figsize=(16, 9))
    pylab.title('Aero forces and estimates (body ' + axis + ')')
    plotter.PlotSimNew('wing.fm_aero.force.' + axis, label=axis)
    plotter.PlotControlNew('crosswind.aero_force_b.' + axis,
                           label=axis+' (est)')
    pylab.ylabel('Force [N]')
    pylab.xlabel('time [s]')
    pylab.legend(loc='upper left')
    pylab.xlim(xlim_crosswind)
    pylab.grid()
    pylab.savefig(FLAGS.output_dir + '/aero_force_est_' + axis + '.svg')
    pylab.close()

  # Plot GS 02 azimuth error
  pylab.figure(figsize=(16, 9))
  pylab.title('Gs02 azi error')
  plotter.PlotSimOld('gs02.a_error', _STYLE_OLD_ACTUAL, label='old')
  plotter.PlotSimNew('gs02.a_error', _STYLE_NEW_ACTUAL, label='new')
  pylab.xlabel('Time [s]')
  pylab.ylabel('Angle [rad]')
  pylab.legend()
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/gs02_azi_error.svg')
  pylab.close()

  # Plot simulator integration time.
  pylab.figure(figsize=(16, 9))
  pylab.title('Integration timing')
  plotter.PlotSimOld('integration_usec', 'C0.', label='old')
  plotter.PlotSimNew('integration_usec', 'C1.', label='new')
  mu_and_sigma_old = stats.norm.fit(plotter.old_sim_telem['integration_usec'])
  mu_and_sigma_new = stats.norm.fit(plotter.new_sim_telem['integration_usec'])
  pylab.title('    '.join([
      r'$\mu_{old}$=%.0f $\mu$s, $\sigma_{old}$=%.0f $\mu$s' % mu_and_sigma_old,
      r'$\mu_{new}$=%.0f $\mu$s, $\sigma_{new}$=%.0f $\mu$s' % mu_and_sigma_new
  ]))
  pylab.xlabel('Time [s]')
  pylab.ylabel(r'Integration time [$\mu$s]')
  pylab.legend(loc='upper left')
  pylab.grid()
  # PNG is better than SVG for point clouds.
  pylab.savefig(FLAGS.output_dir + '/integration_timing.png')
  pylab.close()

  # Plot controller loop time.
  pylab.figure(figsize=(16, 9))
  pylab.title('Controller timing')
  plotter.PlotControlOld('loop_usec', 'C0.', label='old')
  plotter.PlotControlNew('loop_usec', 'C1.', label='new')
  mu_and_sigma_old = stats.norm.fit(plotter.old_control_telem['loop_usec'])
  mu_and_sigma_new = stats.norm.fit(plotter.new_control_telem['loop_usec'])
  pylab.title('    '.join([
      r'$\mu_{old}$=%.0f $\mu$s, $\sigma_{old}$=%.0f $\mu$s' % mu_and_sigma_old,
      r'$\mu_{new}$=%.0f $\mu$s, $\sigma_{new}$=%.0f $\mu$s' % mu_and_sigma_new
  ]))
  pylab.xlabel('Time [s]')
  pylab.ylabel('Time [usec]')
  pylab.legend(loc='upper left')
  pylab.ylim([0, 500])
  pylab.grid()
  pylab.savefig(FLAGS.output_dir + '/control_timing.png')
  pylab.close()


if __name__ == '__main__':
  main(sys.argv)

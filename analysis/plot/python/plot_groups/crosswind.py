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

"""Plots relating to the crosswind controller."""

from makani.analysis.plot.python import mplot
from makani.control import control_types
from matplotlib.pyplot import plot
from matplotlib.pyplot import xlim
import numpy as np

MFig = mplot.PlotGroup.MFig  # pylint: disable=invalid-name


class Plots(mplot.PlotGroup):
  """Plots of the crosswind controller."""

  def SetXlimCrosswind(self):
    c = self.data[1]
    xlim(c['time'][c['flight_mode']
                   == control_types.kFlightModeCrosswindNormal][[0, -1]])

  @MFig(title='Crosswind Alpha', ylabel='alpha [degrees]', xlabel='Time [s]')
  def PlotAlpha(self, cw, c, s):
    aw = c['state_est']['apparent_wind']
    plot(c['time'], np.rad2deg(cw['alpha_cmd']), 'b--', label='alpha_cmd')
    plot(c['time'], np.rad2deg(aw['sph_f']['alpha']), 'b-', label='alpha_f')
    plot(c['time'], np.rad2deg(aw['sph']['alpha']), 'c-', label='alpha')

  @MFig(title='Crosswind Beta', ylabel='beta [degrees]', xlabel='Time [s]')
  def PlotBeta(self, cw, c, s):
    aw = c['state_est']['apparent_wind']
    plot(c['time'], np.rad2deg(cw['beta_cmd']), 'b--', label='beta_cmd')
    plot(c['time'], np.rad2deg(aw['sph_f']['beta']), 'b-', label='beta_f')
    plot(c['time'], np.rad2deg(aw['sph']['beta']), 'c-', label='beta')

  @MFig(title='Crosswind Airspeed', ylabel='Speed [m/s]', xlabel='Time [s]')
  def PlotAirspeed(self, cw, c, s):
    aw = c['state_est']['apparent_wind']
    plot(c['time'], cw['airspeed_cmd'], 'b--', label='airspeed_cmd')
    plot(c['time'], aw['sph']['v'], 'b-', label='airspeed')
    plot(c['time'], aw['sph_f']['v'], 'c-', label='airspeed_f')

  @MFig(title='Crosswind Crosstrack Integrator', ylabel='curvature [1/m]',
        xlabel='Time [s]')
  def PlotCrosstrackIntegrator(self, cw, c, s):
    plot(c['time'], cw['int_crosstrack'], 'b--', label='int_crosstrack')

  @MFig(title='Crosswind Inner-Loop Integrators', ylabel='rad*s',
        xlabel='Time [s]')
  def PlotCrosswindIntegrators(self, cw, c, s):
    plot(c['time'], cw['int_beta_error'], label='int_beta_error')
    plot(c['time'], cw['int_tether_roll_error'], label='int_tether_roll_error')

  @MFig(title='Crosswind Actuator Integrators', ylabel='rad*s',
        xlabel='Time [s]')
  def PlotCrosswindActuatorIntegrators(self, cw, c, s):
    plot(c['time'], cw['int_aileron'], label='int_aileron')
    plot(c['time'], cw['int_elevator'], label='int_elevator')
    plot(c['time'], cw['int_rudder'], label='int_rudder')

  @MFig(title='Crosswind Path Radius', ylabel='radius [m]', xlabel='Time [s]')
  def PlotPathRadius(self, cw, c, s):
    crosswind_path_radius = (
        cw['current_pos_cw']['x']**2.0 +
        cw['current_pos_cw']['y']**2.0)**0.5
    plot(c['time'], cw['path_radius_target'], 'b--', label='path_radius_target')
    plot(c['time'], crosswind_path_radius, 'b-', label='path_radius')

  @MFig(title='Crosswind Position', ylabel='[m]', xlabel='Time [s]')
  def PlotPosition(self, cw, c, s):
    plot(c['time'], cw['current_pos_cw']['x'], label='current_pos_cw.x')
    plot(c['time'], cw['current_pos_cw']['y'], label='current_pos_cw.y')

  @MFig(title='Crosswind Loop Angle', ylabel='loop angle [deg]',
        xlabel='Time [s]')
  def PlotLoopAngle(self, cw, c, s):
    plot(c['time'], np.rad2deg(cw['loop_angle']))

  @MFig(title='Thrust', ylabel='Thrust [N]', xlabel='Time [s]')
  def PlotThrust(self, cw, c, s):
    plot(c['time'], cw['thrust_ff'], label='thrust FF')
    plot(c['time'], cw['thrust_fb'], label='thrust FB')
    plot(c['time'], cw['int_thrust'], label='int_thrust')
    plot(c['time'], cw['thrust_ff'] + cw['thrust_fb'], label='thrust FF + FB')
    plot(c['time'], c['thrust_moment']['thrust'], label='thrust_moment.thrust')
    plot(c['time'], c['thrust_moment_avail']['thrust'],
         label='thrust_moment_avail.thrust')

  @MFig(title='Tether Roll Angle', ylabel='Tether Roll [deg]',
        xlabel='Time [s]')
  def PlotTetherRoll(self, cw, c, s):
    plot(c['time'], np.rad2deg(cw['tether_roll_cmd']),
         'b--', label='tether_roll_cmd')
    plot(c['time'], np.rad2deg(c['state_est']['tether_force_b']['sph']['roll']),
         'b-', label='tether_roll')

  @MFig(title='Curvature', ylabel='Curvature [1/m]',
        xlabel='time [s]')
  def PlotCurvature(self, cw, c, s):
    plot(c['time'], cw['k_geom_curr'], 'b-', label='k_geom_curr')
    plot(c['time'], cw['k_geom_cmd'], 'b--', label='k_geom_cmd')
    plot(c['time'], cw['k_aero_curr'], 'g-', label='k_aero_curr')
    plot(c['time'], cw['k_aero_cmd'], 'g--', label='k_aero_cmd')

  @MFig(title='Body Rates', ylabel='rad/s', xlabel='time [s]')
  def PlotBodyRates(self, cw, c, s):
    mplot.PlotVec3(c['time'], cw['pqr_cmd'], linestyle='--', label='pqr_cmd')
    mplot.PlotVec3(c['time'], c['state_est']['pqr_f'], label='pqr_f')

  @MFig(title='Deltas', ylabel='rad', xlabel='Time [s]')
  def PlotDeltas(self, cw, c, s):
    for (index, field) in enumerate(
        ['aileron', 'inboard_flap', 'midboard_flap',
         'outboard_flap', 'elevator', 'rudder']):
      plot(c['time'], c['deltas'][field], 'C%d-' % index,
           label='%s cmd' % field)
      plot(c['time'], c['deltas_avail'][field], 'C%d--' % index,
           label='%s avail' % field)

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

"""Plots relating to trans-in."""

from makani.analysis.plot.python import mplot
from makani.control import control_types
from makani.lib.python import c_helpers
from matplotlib.pyplot import plot
from matplotlib.pyplot import xlim
from matplotlib.pyplot import ylim
import numpy as np

MFig = mplot.PlotGroup.MFig  # pylint: disable=invalid-name

_TRANS_IN_GATE_HELPER = c_helpers.EnumHelper('TransInGate', control_types)


class Plots(mplot.PlotGroup):
  """Trans-in plots."""

  def SetXlimTransIn(self):
    c = self.data[1]
    xlim(c['time'][c['flight_mode']
                   == control_types.kFlightModeTransIn][[0, -1]])

  @MFig(title='Gates', ylabel='Gate Label', xlabel='Time [s]')
  def PlotGates(self, ti, c, s):
    mplot.PlotBitMask(
        c['time'],
        c['flight_mode_gates'][:, control_types.kFlightModeTransIn],
        _TRANS_IN_GATE_HELPER)

  @MFig(title='Radial Velocity', ylabel='Radial Velocity [m/s]',
        xlabel='Time [s]')
  def PlotRadialVelocity(self, ti, c, s):
    plot(c['time'], c['trans_in']['radial_vel_ti_cmd'], 'b:', label='Cmd')
    plot(c['time'], c['trans_in']['radial_vel_ti'], 'b', label='Est')

  @MFig(title='Airspeed', ylabel='Airspeed [m/s]', xlabel='Time [s]')
  def PlotAirspeed(self, ti, c, s):
    plot(c['time'], c['estimator']['apparent_wind_pitot']['v'],
         label='Pitot Estimate')
    plot(c['time'], c['state_est']['apparent_wind']['sph_f']['v'],
         label='Estimate')
    if 'airspeed_cmd' in ti.dtype.names:
      plot(c['time'], ti['airspeed_cmd'], label='Cmd')
    if 'airspeed' in ti.dtype.names:
      plot(c['time'], ti['airspeed'], label='Ti')

    plot(c['time'], (c['state_est']['Vg_f']['x']**2.0
                     + c['state_est']['Vg_f']['y']**2.0
                     + c['state_est']['Vg_f']['z']**2.0)**0.5,
         'r:', label='Kite Speed')
    if s is not None:
      plot(s['time'], s['wing']['apparent_wind_b']['v'], 'b:', label='Sim')
      plot(s['time'], s['wing']['wind_g']['z'], label='Updraft')

  @MFig(title='Aerodynamic Angles', ylabel='Angle [deg]', xlabel='Time [s]')
  def PlotAeroAngles(self, ti, c, s):
    plot(c['time'], np.rad2deg(ti['angle_of_attack_cmd']), 'b:',
         label='AOA Cmd')
    plot(c['time'],
         np.rad2deg(ti['eulers_ti2b']['y'] - ti['aero_climb_angle']),
         'r-.', label='Pitch-Gamma')
    plot(c['time'],
         np.rad2deg(c['state_est']['apparent_wind']['sph_f']['alpha']), 'b',
         label='Est AOA')
    if 'int_angle_of_attack' in ti.dtype.names:
      plot(c['time'],
           np.rad2deg(ti['int_angle_of_attack']), 'b-.', label='Int AOA')

    plot(c['time'],
         np.rad2deg(c['state_est']['apparent_wind']['sph_f']['beta']),
         'g', label='Est AOS')
    if s is not None:
      plot(s['time'], np.rad2deg(s['wing']['apparent_wind_b']['alpha']),
           'b--', label='Sim AOA')
      plot(s['time'], np.rad2deg(s['wing']['apparent_wind_b']['beta']),
           'g--', label='Sim AOS')
    ylim([-10.0, 10.0])

  @MFig(title='Tension', ylabel='Tension [N]', xlabel='Time [s]')
  def PlotTension(self, ti, c, s):
    if 'tension_cmd' in ti.dtype.names:
      plot(c['time'], ti['tension_cmd'], label='Tension Cmd')
    plot(c['time'], c['state_est']['tether_force_b']['vector_f']['z'],
         label='Tension f')

  @MFig(title='Apparent Wind', ylabel='Speed [m/s]', xlabel='Time [s]')
  def PlotApparentWind(self, ti, c, s):
    plot(c['time'], c['state_est']['apparent_wind']['sph']['v'], 'b',
         label='airspeed')
    plot(c['time'],
         np.rad2deg(c['state_est']['apparent_wind']['sph']['alpha']),
         'g', label='AOA')
    plot(c['time'],
         np.rad2deg(c['state_est']['apparent_wind']['sph']['beta']),
         'r', label='AOS')
    if s is not None:
      plot(s['time'], s['wing']['apparent_wind_b']['v'], 'b:',
           label='sim airspeed')
      plot(s['time'], np.rad2deg(s['wing']['apparent_wind_b']['alpha']),
           'g:', label='sim AOA')
      plot(s['time'], np.rad2deg(s['wing']['apparent_wind_b']['beta']),
           'r:', label='sim AOS')

  @MFig(title='Aero Climb Angle', ylabel='Angle [deg]', xlabel='Time [s]')
  def PlotAeroClimbAngle(self, ti, c, s):
    climb_angle = np.arctan2(-c['state_est']['Vg']['z'],
                             np.hypot(c['state_est']['Vg']['x'],
                                      c['state_est']['Vg']['y']))
    elevation_angle_g = np.arctan2(-c['state_est']['Xg']['z'],
                                   np.hypot(c['state_est']['Xg']['x'],
                                            c['state_est']['Xg']['y']))
    plot(c['time'], np.rad2deg(ti['aero_climb_angle']), 'b-',
         label='Aero Climb Est')
    if 'aero_climb_angle_cmd' in ti.dtype.names:
      plot(c['time'], np.rad2deg(ti['aero_climb_angle_cmd']), 'b:',
           label='Aero Climb Cmd')
    plot(c['time'], np.rad2deg(climb_angle), 'k', label='Climb Angle Est')
    plot(c['time'], 90.0 - np.rad2deg(elevation_angle_g), 'g-.',
         label='Tangent')
    if s is not None:
      plot(s['time'], np.rad2deg(np.arcsin(
          -(s['wing']['Vg']['z'] - s['wing']['wind_g']['z']) /
          np.hypot(s['wing']['Vg']['z'] - s['wing']['wind_g']['z'],
                   np.hypot(s['wing']['Vg']['x'] - s['wing']['wind_g']['x'],
                            s['wing']['Vg']['y'] - s['wing']['wind_g']['y'])))),
           'r', label='Sim Aero Climb Angle')

  @MFig(title='Radial Position', ylabel='Radial Position [m]',
        xlabel='Time [s]')
  def PlotRadialPosition(self, ti, c, s):
    radial_pos_g = np.hypot(c['state_est']['Xg']['x'],
                            np.hypot(c['state_est']['Xg']['y'],
                                     c['state_est']['Xg']['z']))
    radial_pos_ti = np.hypot(ti['wing_pos_ti']['x'],
                             ti['wing_pos_ti']['z'])

    plot(c['time'], 434.0 + 5.0 * np.ones(c['time'].shape), 'g', label='Cmd')
    plot(c['time'], radial_pos_g, 'g', label='G')
    plot(c['time'], radial_pos_ti, 'r', label='Ti')

  @MFig(title='Lateral Position', ylabel='Position [m]', xlabel='Time [s]')
  def PlotLateralPosition(self, ti, c, s):
    if 'wing_pos_ti_y_cmd' in ti.dtype.names:
      plot(c['time'], ti['wing_pos_ti_y_cmd'], 'b:', label='Cmd')
    else:
      plot(c['time'], np.zeros(ti['wing_pos_ti']['y'].shape), 'b:', label='Cmd')
    plot(c['time'], ti['wing_pos_ti']['y'], 'b', label='Est')

  @MFig(title='Lateral Velocity', ylabel='Velocity [m/s]', xlabel='Time [s]')
  def PlotLateralVelocity(self, ti, c, s):
    if 'wing_vel_ti_y_cmd' in ti.dtype.names:
      plot(c['time'], ti['wing_vel_ti_y_cmd'], 'b:', label='Cmd')
    else:
      plot(c['time'], np.zeros(ti['wing_vel_ti']['y'].shape), 'b:', label='Cmd')
    plot(c['time'], ti['wing_vel_ti']['y'], 'b', label='Est')

  @MFig(title='Euler Angle Errors', ylabel='Angle Error [deg]',
        xlabel='Time [s]')
  def PlotEulerAngleError(self, ti, c, s):
    mplot.PlotVec3(c['time'], ti['eulers_ti2b'], scale=180.0 / np.pi,
                   labels=['Roll', 'Pitch', 'Yaw'])
    mplot.PlotVec3(c['time'], ti['eulers_ti2cmd'], scale=180.0 / np.pi,
                   labels=['Roll Cmd', 'Pitch Cmd', 'Yaw Cmd'], linestyle=':')

  @MFig(title='Attitude Error', ylabel='Error [deg]', xlabel='Time [s]')
  def PlotAttitudeError(self, ti, c, s):
    labels = ['Roll Error', 'Pitch Error', 'Yaw Error']
    mplot.PlotVec3(c['time'], ti['axis_b2cmd'], scale=180.0 / np.pi,
                   labels=labels)

  @MFig(title='Body Rates', ylabel='Rotation Rate [rad/s]', xlabel='Time [s]')
  def PlotBodyRates(self, ti, c, s):
    labels = ['Roll Rate', 'Pitch Rate', 'Yaw Rate']
    mplot.PlotVec3(c['time'], c['state_est']['pqr_f'], labels=labels)
    mplot.PlotVec3(c['time'], ti['pqr_cmd'], labels=labels, linestyle=':')
    plot(c['time'], ti['pitch_rate_b_cmd'], 'k:', label='Pitch Rate Cmd')

  @MFig(title='Aileron Deflections', ylabel='Angle [deg]',
        xlabel='Time [s]')
  def PlotAilerons(self, ti, c, s):
    indices = [(0, 'A1'), (1, 'A2'), (4, 'A7'), (5, 'A8')]
    mplot.PlotComponents(c['time'], c['control_output']['flaps'],
                         [(i, l + ' Cmd') for i, l in indices],
                         linestyle=':', scale=180.0 / np.pi)
    mplot.PlotComponents(c['time'], c['control_input']['flaps'],
                         [(i, l + ' Response') for i, l in indices],
                         scale=180.0 / np.pi)

  @MFig(title='Flap Deflections', ylabel='Angle [deg]',
        xlabel='Time [s]')
  def PlotFlaps(self, ti, c, s):
    indices = [(2, 'A3'), (3, 'A4')]
    mplot.PlotComponents(c['time'], c['control_output']['flaps'],
                         [(i, l + ' Cmd') for i, l in indices],
                         linestyle=':', scale=180.0 / np.pi)
    mplot.PlotComponents(c['time'], c['control_input']['flaps'],
                         [(i, l + ' Response') for i, l in indices],
                         scale=180.0 / np.pi)

  @MFig(title='Elevator Deflection', ylabel='Angle [deg]', xlabel='Time [s]')
  def PlotElevator(self, ti, c, s):
    plot(c['time'], np.rad2deg(c['control_output']['flaps'][:, 6]), ':',
         label='Command')
    plot(c['time'], np.rad2deg(c['control_input']['flaps'][:, 6]),
         label='Response')
    if s is not None:
      plot(s['time'], 10.0-np.rad2deg(s['wing']['apparent_wind_b']['alpha']),
           'k:', label='Stall')
      plot(s['time'], -10.0-np.rad2deg(s['wing']['apparent_wind_b']['alpha']),
           'k:', label='Stall')

  @MFig(title='Rudder Deflection', ylabel='Rudder Angle [deg]',
        xlabel='Time [s]')
  def PlotRudder(self, ti, c, s):
    plot(c['time'], np.rad2deg(c['control_output']['flaps'][:, 7]), ':',
         label='Rudder Command')
    plot(c['time'], np.rad2deg(c['control_input']['flaps'][:, 7]),
         label='Rudder Response')

  @MFig(title='Motor Moments', ylabel='Moment [N-m]', xlabel='Time [s]')
  def PlotMotorMoments(self, ti, c, s):
    mplot.PlotVec3(c['time'], c['thrust_moment_avail']['moment'],
                   labels=['Roll Avail', 'Pitch Avail', 'Yaw Avail'])
    mplot.PlotVec3(c['time'], c['thrust_moment']['moment'], linestyle=':',
                   labels=['Roll Cmd', 'Pitch Cmd', 'Yaw Cmd'])

  @MFig(title='CL', ylabel='CL [#]', xlabel='Time [s]')
  def PlotCL(self, ti, c, s):
    plot(c['time'], ti['CL_cmd'], label='Command')
    if s is not None:
      plot(s['time'], s['wing']['CL'], ':', label='Sim')

  @MFig(title='Thrust', ylabel='Thrust [N]', xlabel='Time [s]')
  def PlotThrust(self, ti, c, s):
    plot(c['time'], c['thrust_moment']['thrust'], 'b:',
         label='Thrust Cmd')
    plot(c['time'], c['thrust_moment_avail']['thrust'], 'b',
         label='Thrust Avail')
    plot(c['time'], ti['int_airspeed_thrust_cmd'], 'g',
         label='Thrust Int.')

    if s is not None:
      plot(s['time'], s['wing']['fm_rotors']['force']['x'],
           'r', label='Sim')

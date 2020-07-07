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

"""Plots relating to the hover."""

from makani.analysis.plot.python import mplot
from makani.avionics.common import plc_messages
from makani.control import control_types
from makani.lib.python import c_helpers
from makani.lib.python.h5_utils import numpy_utils
from matplotlib.pyplot import gca
from matplotlib.pyplot import plot
from matplotlib.pyplot import xlim
from matplotlib.pyplot import ylim
import numpy as np

MFig = mplot.PlotGroup.MFig  # pylint: disable=invalid-name

_HOVER_ACCEL_GATE_HELPER = c_helpers.EnumHelper(
    'HoverAccelGate', control_types)

_HOVER_TRANS_OUT_GATE_HELPER = c_helpers.EnumHelper(
    'CrosswindHoverTransOutGate', control_types)

_GROUND_STATION_MODE_HELPER = c_helpers.EnumHelper(
    'GroundStationMode', plc_messages)


class Plots(mplot.PlotGroup):
  """Plots of the hover."""

  def SetXlimAccel(self):
    c = self.data[1]
    xlim(c['time'][c['flight_mode']
                   == control_types.kFlightModeHoverAccel][[0, -1]])

  @MFig(title='Accel Gates', ylabel='Gate Label', xlabel='Time [s]')
  def PlotAccelGates(self, h, c, s, params):
    mplot.PlotBitMask(
        c['time'],
        c['flight_mode_gates'][:, control_types.kFlightModeHoverAccel],
        _HOVER_ACCEL_GATE_HELPER)

  @MFig(title='Trans Out Gates', ylabel='Gate Label', xlabel='Time [s]')
  def PlotTransOutGates(self, h, c, s, params):
    mplot.PlotBitMask(
        c['time'],
        c['flight_mode_gates'][:, control_types.kFlightModeHoverTransOut],
        _HOVER_TRANS_OUT_GATE_HELPER)

  @MFig(title='Hover Angles', ylabel='Angles [rad]', xlabel='Time [s]')
  def PlotHoverAngles(self, h, c, s, params):
    plot(c['time'], h['angles']['x'], 'b', label='Roll')
    plot(c['time'], h['angles']['y'], 'g', label='Pitch')
    plot(c['time'], h['angles']['z'], 'r', label='Yaw')
    plot(c['time'], h['angles_cmd']['x'], 'b:', label='Roll Cmd')
    plot(c['time'], h['angles_cmd']['y'], 'g:', label='Pitch Cmd')
    plot(c['time'], h['angles_cmd']['z'], 'r:', label='Yaw Cmd')

  @MFig(title='Hover Int Angles', ylabel='Angles [rad]', xlabel='Time [s]')
  def PlotHoverIntAngles(self, h, c, s, params):
    plot(c['time'], h['int_angles']['x'], 'b', label='x')
    plot(c['time'], h['int_angles']['y'], 'g', label='y')
    plot(c['time'], h['int_angles']['z'], 'r', label='z')

  @MFig(title='Hover Int Moment', ylabel='Moment [N-m]', xlabel='Time [s]')
  def PlotHoverIntMoment(self, h, c, s, params):
    plot(c['time'], h['int_moment']['x'], 'b', label='x')
    plot(c['time'], h['int_moment']['y'], 'g', label='y')
    plot(c['time'], h['int_moment']['z'], 'r', label='z')

  @MFig(title='Hover Tension', ylabel='Tension [N]', xlabel='Time [s]')
  def PlotHoverTension(self, h, c, s, params):
    plot(c['time'], h['tension_cmd'], ':', label='Cmd')
    plot(c['time'], c['state_est']['tether_force_b']['tension_f'], '-',
         label='Est')
    if s is not None:
      plot(s['time'], s['wing']['tether_force_b']['tension'], '-.',
           label='Sim')

  @MFig(title='Horizontal Tension', ylabel='Tension [N]', xlabel='Time [s]')
  def PlotHoverHorizontalTension(self, h, c, s, params):
    plot(c['time'], h['horizontal_tension_cmd'], ':', label='Cmd')
    plot(c['time'], h['horizontal_tension'], '-', label='Est')

  @MFig(title='Hover Position', ylabel='Pos [m]', xlabel='Time [s]')
  def PlotHoverPosition(self, h, c, s, params, dim=None):
    if dim is None:
      mplot.PlotVec3(c['time'], h['raw_wing_pos_g_cmd'], label='raw cmd',
                     linestyle=':')
      mplot.PlotVec3(c['time'], h['wing_pos_g_cmd'], label='cmd',
                     linestyle='--')
      mplot.PlotVec3(c['time'], c['state_est']['Xg'], label='pos',
                     linestyle='-')
    else:
      plot(c['time'], h['raw_wing_pos_g_cmd'][dim], label=dim + ' raw cmd',
           linestyle=':')
      plot(c['time'], h['wing_pos_g_cmd'][dim], label=dim + ' cmd',
           linestyle='--')
      plot(c['time'], c['state_est']['Xg'][dim], label=dim + ' pos',
           linestyle='-')

  @MFig(title='Hover Azimuth', ylabel='Azi [deg]', xlabel='Time [s]')
  def PlotHoverPathAzimuth(self, h, c, s, params):
    def Vec3ToAzimuthDeg(v):
      return np.rad2deg(np.arctan2(v['y'], v['x']))
    plot(c['time'], Vec3ToAzimuthDeg(h['raw_wing_pos_g_cmd']), label='raw',
         linestyle=':')
    plot(c['time'], Vec3ToAzimuthDeg(h['wing_pos_g_cmd']), label='cmd',
         linestyle='--')
    plot(c['time'], Vec3ToAzimuthDeg(c['state_est']['Xg']), label='pos',
         linestyle='-')

  @MFig(title='Hover Radius', ylabel='Radius [m]', xlabel='Time [s]')
  def PlotHoverPathRadius(self, h, c, s, params):
    def Vec3ToRadius(v):
      return np.hypot(v['y'], v['x'])
    plot(c['time'], Vec3ToRadius(h['raw_wing_pos_g_cmd']), label='raw',
         linestyle=':')
    plot(c['time'], Vec3ToRadius(h['wing_pos_g_cmd']), label='cmd',
         linestyle='--')
    plot(c['time'], Vec3ToRadius(c['state_est']['Xg']), label='pos',
         linestyle='-')

  @MFig(title='Hover Radial Velocity',
        ylabel='Radial Velocity [m/s] (outward = positive)',
        xlabel='Time [s]')
  def PlotHoverRadialVelocity(self, h, c, s, params):
    wing_pos_g = numpy_utils.Vec3ToArray(c['state_est']['Xg'])

    # Develop a unit vector in the radial direction.
    wing_pos_g_radial_hat = np.copy(wing_pos_g)
    wing_pos_g_radial_hat[:, 2] = 0.0
    wing_pos_g_radial_norm = np.linalg.norm(wing_pos_g_radial_hat, axis=1)
    wing_pos_g_radial_hat /= wing_pos_g_radial_norm[:, np.newaxis]

    wing_vel_g = numpy_utils.Vec3ToArray(c['state_est']['Vg_f'])
    wing_vel_g_radial = (wing_vel_g *  wing_pos_g_radial_hat).sum(axis=1)

    wing_vel_g_cmd = numpy_utils.Vec3ToArray(h['wing_vel_g_cmd'])
    wing_vel_g_cmd_radial = (wing_vel_g_cmd * wing_pos_g_radial_hat).sum(axis=1)

    plot(c['time'], wing_vel_g_radial, '-', label='radial velocity')
    plot(c['time'], wing_vel_g_cmd_radial, '--', label='radial velocity cmd')

  @MFig(title='Hover Velocity', ylabel='Vel [m/s]', xlabel='Time [s]')
  def PlotHoverVelocity(self, h, c, s, params, dim=None):
    if dim is None:
      mplot.PlotVec3(c['time'], h['wing_vel_g_cmd'], label='cmd',
                     linestyle='--')
      mplot.PlotVec3(c['time'], c['state_est']['Vg_f'], label='vel',
                     linestyle='-')
      if s is not None:
        mplot.PlotVec3(s['time'], s['wing']['Vg'], label='vel (sim)')
    else:
      plot(c['time'], h['wing_vel_g_cmd'][dim], label=dim + ' cmd',
           linestyle='--')
      plot(c['time'], c['state_est']['Vg_f'][dim], label=dim + ' vel',
           linestyle='-')
      if s is not None:
        plot(s['time'], s['wing']['Vg'][dim], label=dim + ' sim')

  @MFig(title='Hover Position Error (body coords)', ylabel='Pos [m]',
        xlabel='Time [s]')
  def PlotHoverPositionError(self, h, c, s, params):
    mplot.PlotVec3(c['time'], h['wing_pos_b_error'], label='b err',
                   linestyle='-')

  @MFig(title='Hover Velocity Error (body coords)', ylabel='Pos [m]',
        xlabel='Time [s]')
  def PlotHoverVelocityError(self, h, c, s, params):
    mplot.PlotVec3(c['time'], h['wing_vel_b_error'], label='b err',
                   linestyle='-')

  @MFig(title='Hover Altitude', ylabel='Pos Z [m]', xlabel='Time [s]')
  def PlotHoverAltitude(self, h, c, s, params):
    plot(c['time'], h['raw_wing_pos_g_cmd']['z'], label='raw cmd',
         linestyle=':')
    plot(c['time'], h['wing_pos_g_cmd']['z'], label='cmd', linestyle='--')
    plot(c['time'], c['state_est']['Xg']['z'], label='est')
    if s is not None:
      plot(s['time'], s['wing']['Xg']['z'], label='sim')
    gca().invert_yaxis()

  @MFig(title='Hover Altitude Error', ylabel='Pos Z [m]', xlabel='Time [s]')
  def PlotHoverAltitudeError(self, h, c, s, params):
    plot(c['time'], h['raw_wing_pos_g_cmd']['z'] - c['state_est']['Xg']['z'],
         label='raw', linestyle=':')
    plot(c['time'], h['wing_pos_g_cmd']['z'] - c['state_est']['Xg']['z'],
         label='error')
    ylim((-2.0, 2.0))
    gca().invert_yaxis()

  @MFig(title='Hover Thrust', ylabel='force [N]', xlabel='Time [s]')
  def PlotHoverThrust(self, h, c, s, params):
    plot(c['time'], h['thrust_ff'], label='feed-forward')
    plot(c['time'], h['thrust_fb'], label='feedback')
    plot(c['time'], h['int_thrust'], label='integrator')
    plot(c['time'], h['int_boost'], label='boost')
    plot(c['time'], c['thrust_moment']['thrust'], label='thrust_cmd')
    plot(c['time'], c['thrust_moment_avail']['thrust'], label='thrust avail')

  @MFig(title='Gain Ramp', ylabel='scale [#]', xlabel='Time [s]')
  def PlotGainRamp(self, h, c, s, params):
    plot(c['time'], h['gain_ramp_scale'])

  @MFig(title='Tether Elevation', ylabel='[deg]', xlabel='Time [s]')
  def PlotHoverTetherElevation(self, h, c, s, params):
    tether_ele_g = c['state_est']['tether_ground_angles']['elevation_g']
    tether_ele_p = c['state_est']['tether_ground_angles']['elevation_p']
    # Set the tether elevation to NaN where the valid flag is not set.
    tether_ele_g[np.logical_not(
        c['state_est']['tether_ground_angles']['elevation_valid']
        )] = float('nan')
    tether_ele_p[np.logical_not(
        c['state_est']['tether_ground_angles']['elevation_valid']
        )] = float('nan')
    tether_ele_cmd = h['tether_elevation_cmd']

    ii_reel = (c['state_est']['gs_mode'] ==
               _GROUND_STATION_MODE_HELPER.Value('Reel'))
    ii_transform = (c['state_est']['gs_mode'] ==
                    _GROUND_STATION_MODE_HELPER.Value('Transform'))

    tether_ele_limits = {}
    for severity in ['very_low', 'low', 'high', 'very_high']:
      tether_ele_limits[severity] = np.nan * np.zeros(tether_ele_cmd.shape)

    # TODO: Move the following limits to the configuration system.
    tether_ele_limits['very_low'][ii_reel] = np.deg2rad(-2.0)
    tether_ele_limits['low'][ii_reel] = np.deg2rad(1.0)
    tether_ele_limits['high'][ii_reel] = np.deg2rad(12.0)
    tether_ele_limits['very_high'][ii_reel] = np.deg2rad(18.0)

    nan = np.nan
    limits_by_stage_deg = [
        {'very_low': nan, 'low': nan, 'high': nan, 'very_high': nan},  # 0
        {'very_low': 3.7, 'low': 4.7, 'high': 8.7, 'very_high': 9.7},  # 1
        {'very_low': 3.7, 'low': 4.7, 'high': 8.7, 'very_high': 9.7},  # 2
        {'very_low': 3.7, 'low': 4.7, 'high': 12., 'very_high': 18.},  # 3
        {'very_low': -2., 'low': 1.0, 'high': 12., 'very_high': 18.},  # 4
        ]

    for stage in range(5):
      for severity in ['very_low', 'low', 'high', 'very_high']:
        ii_transform_and_stage = np.logical_and(
            ii_transform,
            c['state_est']['gs_transform_stage'] == stage)
        tether_ele_limits[severity][ii_transform_and_stage] = (
            np.deg2rad(limits_by_stage_deg[stage][severity]))

    plot(c['time'], np.rad2deg(tether_ele_g), label='Est g')
    plot(c['time'], np.rad2deg(tether_ele_p), label='Est p')
    plot(c['time'], np.rad2deg(tether_ele_cmd), label='Cmd')
    if s is not None:
      plot(s['time'], np.rad2deg(s['tether']['Xv_start_elevation']), '--',
           label='Sim')
    plot(c['time'], np.rad2deg(tether_ele_limits['low']), 'y')
    plot(c['time'], np.rad2deg(tether_ele_limits['high']), 'y')
    plot(c['time'], np.rad2deg(tether_ele_limits['very_low']), 'r')
    plot(c['time'], np.rad2deg(tether_ele_limits['very_high']), 'r')

  @MFig(title='Hover Elevation', ylabel='[deg]', xlabel='Time [s]')
  def PlotHoverElevation(self, h, c, s, params):
    plot(c['time'], -np.rad2deg(
        np.arctan2(c['state_est']['Xg']['z'],
                   np.hypot(c['state_est']['Xg']['x'],
                            c['state_est']['Xg']['y']))),
         label='est')
    # Select flight modes with a valid elevation command.
    ii = np.isin(c['flight_mode'],
                 [control_types.kFlightModeHoverPayOut,
                  control_types.kFlightModeHoverReelIn,
                  control_types.kFlightModeHoverPrepTransformGsUp,
                  control_types.kFlightModeHoverPrepTransformGsDown,
                  control_types.kFlightModeHoverTransformGsUp,
                  control_types.kFlightModeHoverTransformGsDown])

    elevation_cmd = h['elevation_cmd']
    elevation_min = h['elevation_min']
    elevation_max = h['elevation_max']
    elevation_cmd[np.logical_not(ii)] = float('nan')
    elevation_min[np.logical_not(ii)] = float('nan')
    elevation_max[np.logical_not(ii)] = float('nan')
    plot(c['time'], np.rad2deg(elevation_min), 'k--', label='min')
    plot(c['time'], np.rad2deg(elevation_max), 'k--', label='max')
    plot(c['time'], np.rad2deg(elevation_cmd), label='cmd')
    plot(c['time'], np.rad2deg(h['elevation_ff']), label='ff')
    plot(c['time'], np.rad2deg(h['elevation_fb']), label='fb')

  @MFig(title='Hover Tension Control', ylabel='[deg]', xlabel='Time [s]')
  def PlotHoverTensionControl(self, h, c, s, params):
    plot(c['time'], np.rad2deg(h['pitch_ff']), label='pitch_ff')
    plot(c['time'], np.rad2deg(h['pitch_fb']), label='pitch_fb')
    plot(c['time'], np.rad2deg(h['int_pitch']), label='int_pitch')
    plot(c['time'], np.rad2deg(h['pitch_cmd']), label='pitch_cmd')
    plot(c['time'], np.rad2deg(h['angles_cmd']['y']), label='angles_cmd.y')
    plot(c['time'], np.rad2deg(h['angles']['y']), label='angles.y')

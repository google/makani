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

"""Plots relating to the estimator."""

from makani.analysis.plot.python import mplot
from makani.avionics.common import plc_messages
from makani.control import control_types
from makani.lib.python import c_helpers
from makani.lib.python.h5_utils import numpy_utils
from matplotlib.pyplot import plot
from matplotlib.pyplot import yticks
import numpy as np
from scipy import interpolate

MFig = mplot.PlotGroup.MFig  # pylint: disable=invalid-name

_WING_GPS_RECEIVER_HELPER = c_helpers.EnumHelper(
    'WingGpsReceiver', control_types)

_GROUND_STATION_MODE_HELPER = c_helpers.EnumHelper(
    'GroundStationMode', plc_messages)


def _QuatToVec(q):
  dims = ['q0', 'q1', 'q2', 'q3']
  return np.array([q[d] for d in dims])


class Plots(mplot.PlotGroup):
  """Plots of the estimator."""

  @MFig(title='Filtered Velocity', ylabel='Velocity [m/s]', xlabel='Time [s]')
  def PlotFilteredVelocity(self, e, c, s, params):
    mplot.PlotVec3(c['time'], c['state_est']['Vg'], label='Vg', linestyle='-')
    mplot.PlotVec3(c['time'], c['state_est']['Vg_f'], label='Vg_f',
                   linestyle='-.')
    if s is not None:
      mplot.PlotVec3(s['time'], s['wing']['Vg'], label='sim', linestyle=':')

  @MFig(title='Acc Norm f', ylabel='Acc. [m/s^2]', xlabel='Time [s]')
  def PlotAccNormF(self, e, c, s, params, imu_index=0):
    plot(c['time'], c['state_est']['acc_norm_f'])

  @MFig(title='Gyros', ylabel='Rate [rad/s]', xlabel='Time [s]')
  def PlotGyros(self, e, c, s, params, imu_index=0):
    mplot.PlotVec3(c['time'], c['control_input']['imus']['gyro'][:, imu_index])

  @MFig(title='Filtered Body Rates', ylabel='Rate [rad/s]', xlabel='Time [s]')
  def PlotBodyRates(self, e, c, s, params):
    mplot.PlotVec3(c['time'], c['state_est']['pqr_f'])

  @MFig(title='Attitude Error', ylabel='Error [deg]', xlabel='Time [s]')
  def PlotAttitudeError(self, e, c, s, params):
    for imu_index in range(3):
      if s is not None:
        dims = ['q0', 'q1', 'q2', 'q3']
        q_s = {d: np.zeros(c['time'].shape) for d in dims}
        for d in dims:
          q_s[d] = interpolate.interp1d(s['time'], s['wing']['q'][d],
                                        bounds_error=False)(c['time'])
        q_s = _QuatToVec(q_s)
        if 'q_g2b' in e.dtype.names:
          q_c = e['q_g2b'][:, imu_index]
          q_c = _QuatToVec(q_c)
          plot(c['time'], np.rad2deg(
              np.arccos(1.0 - 2.0 * (1.0 - np.sum(q_c * q_s, axis=0)**2.0))),
               label='Imu %d' % imu_index)
        if 'mahony_states' in e.dtype.names:
          q_c = e['mahony_states']['q'][:, imu_index]
          q_c = _QuatToVec(q_c)
          plot(c['time'], np.rad2deg(
              np.arccos(1.0 - 2.0 * (1.0 - np.sum(q_c * q_s, axis=0)**2.0))),
               label='Imu %d' % imu_index)

  @MFig(title='Gyro Biases', ylabel='Biases [rad/s]', xlabel='Time [s]')
  def PlotGyroBiases(self, e, c, s, params, imu_index=0):
    mplot.PlotVec3(c['time'], e['gyro_biases'][:, imu_index],
                   label='IMU %d' % imu_index)
    if s is not None:
      mplot.PlotVec3(s['time'], s['imus']['gyro_bias_b'][:, imu_index],
                     linestyle=':')

  @MFig(title='Acc Biases', ylabel='Biases [m/s^1]', xlabel='Time [s]')
  def PlotAccBiases(self, e, c, s, params, imu_index=0):
    mplot.PlotVec3(c['time'], e['acc_b_estimates'][:, imu_index],
                   label='IMU %d' % imu_index)

  @MFig(title='Air Speed', ylabel='Speed [m/s]', xlabel='Time [s]')
  def PlotAirspeed(self, e, c, s, params):
    plot(c['time'], c['state_est']['apparent_wind']['sph']['v'], 'b',
         label='est')
    plot(c['time'], c['state_est']['apparent_wind']['sph_f']['v'], 'g',
         label='filt')
    if s is not None:
      plot(s['time'], s['wing']['apparent_wind_b']['v'], 'b:', label='sim')

  @MFig(title='Magnetometer', ylabel='Field [Gauss]', xlabel='Time [s]')
  def PlotMagnetometer(self, e, c, s, params):
    mplot.PlotVec3(c['time'], c['control_input']['imus']['mag'][:, 0],
                   linestyle='-', label='A')
    mplot.PlotVec3(c['time'], c['control_input']['imus']['mag'][:, 1],
                   linestyle=':', label='B')
    mplot.PlotVec3(c['time'], c['control_input']['imus']['mag'][:, 2],
                   linestyle='-.', label='C')

  @MFig(title='Specific Force', ylabel='Specific Force [m/s^2]',
        xlabel='Time [s]')
  def PlotAccelerometer(self, e, c, s, params):
    mplot.PlotVec3(c['time'], c['control_input']['imus']['acc'][:, 0],
                   linestyle='-', label='A')
    mplot.PlotVec3(c['time'], c['control_input']['imus']['acc'][:, 1],
                   linestyle=':', label='B')
    mplot.PlotVec3(c['time'], c['control_input']['imus']['acc'][:, 2],
                   linestyle='-.', label='C')

  @MFig(title='Accel.', ylabel='Specific Force [m/s^2]', xlabel='Time [s]')
  def PlotSpecificForce(self, e, c, s, params):
    mplot.PlotVec3(c['time'], c['estimator']['acc_b_estimates'][:, 0],
                   linestyle='-', label='A')
    mplot.PlotVec3(c['time'], c['estimator']['acc_b_estimates'][:, 1],
                   linestyle=':', label='B')
    mplot.PlotVec3(c['time'], c['estimator']['acc_b_estimates'][:, 2],
                   linestyle='-.', label='C')

  @MFig(title='Magnetometer Diff', ylabel='Field [Gauss]', xlabel='Time [s]')
  def PlotMagnetometerDiff(self, e, c, s, params, dimension='x'):
    plot(c['time'],
         c['control_input']['imus']['mag'][dimension][:, 0]
         - c['control_input']['imus']['mag'][dimension][:, 1], 'b',
         label='A-B ' + dimension)
    plot(c['time'],
         c['control_input']['imus']['mag'][dimension][:, 1]
         - c['control_input']['imus']['mag'][dimension][:, 2], 'g',
         label='B-C ' + dimension)
    plot(c['time'],
         c['control_input']['imus']['mag'][dimension][:, 2]
         - c['control_input']['imus']['mag'][dimension][:, 0], 'r',
         label='C-A ' + dimension)

  @MFig(title='Current GPS', ylabel='GPS Receiver', xlabel='Time [s]')
  def PlotGpsReceiver(self, e, c, s, params):
    plot(c['time'], e['current_gps_receiver'], label='current_receiver')
    yticks(_WING_GPS_RECEIVER_HELPER.Values(),
           _WING_GPS_RECEIVER_HELPER.ShortNames())

  def _PlotGpsPositionEcefChannel(self, c, d):
    sigma = c['control_input']['wing_gps']['pos_sigma'][d]
    wing_gps_pos = np.array(c['control_input']['wing_gps']['pos'][d])
    wing_gps_pos[wing_gps_pos == 0] = float('nan')
    plot(c['time'], wing_gps_pos[:, 0], 'b', label='0:%s ECEF' % d)
    plot(c['time'], wing_gps_pos[:, 0] + sigma[:, 0], 'b:')
    plot(c['time'], wing_gps_pos[:, 0] - sigma[:, 0], 'b:')
    plot(c['time'], wing_gps_pos[:, 1], 'g', label='1:%s ECEF' % d)
    plot(c['time'], wing_gps_pos[:, 1] + sigma[:, 1], 'g:')
    plot(c['time'], wing_gps_pos[:, 1] - sigma[:, 1], 'g:')

  @MFig(title='GPS Position ECEF', ylabel='Position [m]', xlabel='Time [s]')
  def PlotGpsPositionEcefX(self, e, c, s, params):
    self._PlotGpsPositionEcefChannel(c, 'x')

  @MFig(title='GPS Position ECEF', ylabel='Position [m]', xlabel='Time [s]')
  def PlotGpsPositionEcefY(self, e, c, s, params):
    self._PlotGpsPositionEcefChannel(c, 'y')

  @MFig(title='GPS Position ECEF', ylabel='Position [m]', xlabel='Time [s]')
  def PlotGpsPositionEcefZ(self, e, c, s, params):
    self._PlotGpsPositionEcefChannel(c, 'z')

  @MFig(title='Kite Velocity Sigma', ylabel='Sigma Velocity [m/s]',
        xlabel='Time [s]')
  def PlotVelocitySigmas(self, e, c, s, params, plot_glas=True):
    if 'cov_vel_g' in e.dtype.names:
      plot(c['time'], e['cov_vel_g']['x']**0.5, 'b', label='Vg_x est')
      plot(c['time'], e['cov_vel_g']['y']**0.5, 'g', label='Vg_y est')
      plot(c['time'], e['cov_vel_g']['z']**0.5, 'r', label='Vg_z est')

    if 'gps' in e.dtype.names:
      aux_indices = np.argwhere(e['current_gps_receiver'] == 1)
      vg = e['gps']['sigma_Vg'][:, 0]
      vg[aux_indices] = e['gps']['sigma_Vg'][aux_indices, 1]
      plot(c['time'], vg['x'], 'b-.', label='Vg_x gps')
      plot(c['time'], vg['y'], 'g-.', label='Vg_y gps')
      plot(c['time'], vg['z'], 'r-.', label='Vg_z gps')

  @MFig(title='Kite Position Sigma', ylabel='Sigma Position [m]',
        xlabel='Time [s]')
  def PlotPositionSigmas(self, e, c, s, params, plot_glas=True):
    if 'cov_vel_g' in e.dtype.names:
      plot(c['time'], e['cov_pos_g']['x']**0.5, 'b', label='Xg_x est')
      plot(c['time'], e['cov_pos_g']['y']**0.5, 'g', label='Xg_y est')
      plot(c['time'], e['cov_pos_g']['z']**0.5, 'r', label='Xg_z est')

    if 'gps' in e.dtype.names:
      aux_indices = np.argwhere(e['current_gps_receiver'] == 1)
      xg = e['gps']['sigma_Xg'][:, 0]
      xg[aux_indices] = e['gps']['sigma_Xg'][aux_indices, 1]
      plot(c['time'], xg['x'], 'b-.', label='Xg_x gps')
      plot(c['time'], xg['y'], 'g-.', label='Xg_y gps')
      plot(c['time'], xg['z'], 'r-.', label='Xg_z gps')

  @MFig(title='Kite Velocity', ylabel='Velocity [m/s]', xlabel='Time [s]')
  def PlotVelocity(self, e, c, s, params, plot_glas=True):
    plot(c['time'], c['state_est']['Vg']['x'], 'b', label='Vg_x est')
    plot(c['time'], c['state_est']['Vg']['y'], 'g', label='Vg_y est')
    plot(c['time'], c['state_est']['Vg']['z'], 'r', label='Vg_z est')
    if 'Vg_gps' in e.dtype.names:
      plot(c['time'], e['Vg_gps']['x'], 'b-.', label='Vg_x gps')
      plot(c['time'], e['Vg_gps']['y'], 'g-.', label='Vg_y gps')
      plot(c['time'], e['Vg_gps']['z'], 'r-.', label='Vg_z gps')

    if 'gps' in e.dtype.names:
      aux_indices = np.argwhere(e['current_gps_receiver'] == 1)
      vg = e['gps']['Vg'][:, 0]
      vg[aux_indices] = e['gps']['Vg'][aux_indices, 1]
      plot(c['time'], vg['x'], 'b-.', label='Vg_x gps')
      plot(c['time'], vg['y'], 'g-.', label='Vg_y gps')
      plot(c['time'], vg['z'], 'r-.', label='Vg_z gps')
    if plot_glas and 'Vg_glas' in e.dtype.names:
      plot(c['time'], e['Vg_glas']['x'], 'b:', label='Vg_x glas')
      plot(c['time'], e['Vg_glas']['y'], 'g:', label='Vg_y glas')
      plot(c['time'], e['Vg_glas']['z'], 'r:', label='Vg_z glas')

    if s is not None:
      plot(s['time'], s['wing']['Vg']['x'], 'b-o', label='Vg_x sim')
      plot(s['time'], s['wing']['Vg']['y'], 'g-o', label='Vg_y sim')
      plot(s['time'], s['wing']['Vg']['z'], 'r-o', label='Vg_z sim')

  @MFig(title='Payout', ylabel='Payout [m]', xlabel='Time [s]')
  def PlotPayout(self, e, c, s, params):
    plot(c['time'], c['state_est']['winch']['payout'], label='Payout')

  @MFig(title='Tension', ylabel='Tension [N]', xlabel='Time [s]')
  def PlotTension(self, e, c, s, params):
    plot(c['time'], c['state_est']['tether_force_b']['sph']['tension'],
         label='Tension est')

  @MFig(title='Tether Angles', ylabel='Angles [deg]', xlabel='Time [s]')
  def PlotTetherAngles(self, e, c, s, params):
    plot(c['time'],
         np.rad2deg(c['state_est']['tether_force_b']['sph']['roll']),
         label='Tether Roll')
    plot(c['time'],
         np.rad2deg(c['state_est']['tether_force_b']['sph']['pitch']),
         label='Tether Pitch')

  @MFig(title='Kite Position', ylabel='Position [m]', xlabel='Time [s]')
  def PlotPosition(self, e, c, s, params, plot_glas=True):
    for (d, clr) in [('x', 'b'), ('y', 'g'), ('z', 'r')]:
      plot(c['time'], c['state_est']['Xg'][d], clr, label='Xg_%s est' % d)
      plot(c['time'],
           c['state_est']['Xg'][d] + c['estimator']['cov_pos_g'][d]**0.5,
           clr+':', label='Xg_%s est' % d)
      plot(c['time'],
           c['state_est']['Xg'][d] - c['estimator']['cov_pos_g'][d]**0.5,
           clr+':', label='Xg_%s est' % d)
      plot(c['time'], e['gps']['Xg'][d][:], clr+'--', label='Xg_%s gps' % d)
      plot(c['time'], e['gps']['Xg'][d][:]
           + e['gps']['sigma_Xg'][d][:], clr+':', label='Xg_%s gps' % d)
      plot(c['time'], e['gps']['Xg'][d][:]
           - e['gps']['sigma_Xg'][d][:], clr+':', label='Xg_%s gps' % d)

      if plot_glas:
        plot(c['time'], e['glas']['Xg'][d][:], clr+'-.', label='Xg_%s glas' % d)
        plot(c['time'], e['glas']['Xg'][d][:]
             + e['glas']['sigma_Xg'][d][:], clr+':', label='Xg_%s glas' % d)
        plot(c['time'], e['glas']['Xg'][d][:]
             - e['glas']['sigma_Xg'][d][:], clr+':', label='Xg_%s glas' % d)

    clr = 'r'  # z-color from above loop
    plot(c['time'], e['baro']['Xg_z'], clr+'-*', label='Xg_z baro')
    if s is not None:
      plot(s['time'], s['wing']['Xg']['x'], 'b-o', label='Xg_x sim')
      plot(s['time'], s['wing']['Xg']['y'], 'g-o', label='Xg_y sim')
      plot(s['time'], s['wing']['Xg']['z'], 'r-o', label='Xg_z sim')

  @MFig(title='GSG Biases', ylabel='Angles [deg]', xlabel='Time [s]')
  def PlotGsgBias(self, e, c, s, params):
    plot(c['time'], np.rad2deg(e['gsg_bias']['azi']), 'b', label='Azi Bias')
    plot(c['time'], np.rad2deg(e['gsg_bias']['ele']), 'g', label='Ele Bias')

  @MFig(title='GPS Bias', ylabel='Position [m]', xlabel='Time [s]')
  def PlotGpsBias(self, e, c, s, params):
    mplot.PlotVec3(c['time'], e['Xg_gps_biases'][:, 0], label='GPS A bias')
    mplot.PlotVec3(c['time'], e['Xg_gps_biases'][:, 1], label='GPS B bias')

  @MFig(title='Wind Speed', ylabel='Wind Speed [m/s]', xlabel='Time [s]')
  def PlotWindSpeed(self, e, c, s, params):
    if s is not None:
      wind_g = s['wind_sensor']['wind_g']
      plot(s['time'], numpy_utils.Vec3Norm(wind_g), 'C1--',
           label='wind speed at wind sensor [sim]')
      wind_g = s['wing']['wind_g']
      plot(s['time'], numpy_utils.Vec3Norm(wind_g), 'C2:',
           label='wind speed at kite [sim]')
    # Plot the estimated wind speed last so that it will be on top.
    plot(c['time'], c['state_est']['wind_g']['speed_f'], 'C0-',
         linewidth=2, label='wind speed [est]')

  @MFig(title='Kite Azimuth', ylabel='Azimuth [deg]', xlabel='Time [s]')
  def PlotKiteAzimuth(self, e, c, s, params):
    xg = c['state_est']['Xg']
    plot(c['time'], np.rad2deg(np.arctan2(xg['y'], xg['x'])), 'b')

  @MFig(title='Wind Direction (FROM)', ylabel='Direction [deg]',
        xlabel='Time [s]')
  def PlotWindDir(self, e, c, s, params):
    if s is not None:
      wind_g = s['wind_sensor']['wind_g']
      plot(s['time'],
           np.rad2deg(np.arctan2(-wind_g['y'], -wind_g['x'])), 'C1--',
           label='wind direction at wind sensor [sim]')
      wind_g = s['wing']['wind_g']
      plot(s['time'],
           np.rad2deg(np.arctan2(-wind_g['y'], -wind_g['x'])), 'C1--',
           label='wind direction at kite [sim]')
    # The estimator's "dir_f" is the TO direction. Here we convert to a
    # FROM direction.
    dir_f = np.rad2deg(c['state_est']['wind_g']['dir_f']) + 180.0
    dir_f[dir_f > 360.0] -= 360.0
    # Plot the estimated wind speed last so that it will be on top.
    plot(c['time'], dir_f, 'C0-',
         linewidth=2, label='wind direction [est]')

  @MFig(title='Tether Elevation', ylabel='[deg]', xlabel='Time [s]')
  def PlotTetherElevation(self, e, c, s, params):
    elevation = c['state_est']['tether_ground_angles']['elevation']
    elevation[np.logical_not(
        c['state_est']['tether_ground_angles']['elevation_valid']
    )] = float('nan')
    plot(c['time'], np.rad2deg(elevation), label='Est')
    if s is not None:
      plot(s['time'], np.rad2deg(s['tether']['Xv_start_elevation']), '--',
           label='Sim')

  @MFig(title='Ground Station Mode', ylabel='Mode [enum]', xlabel='Time [s]')
  def PlotGroundStationMode(self, e, c, s, params):
    plot(c['time'], c['control_input']['gs_sensors']['mode'], label='ci')
    plot(c['time'], c['state_est']['gs_mode'], label='est')
    if s is not None:
      plot(s['time'], s['gs02']['mode'], '-.', label='Sim')
    yticks(_GROUND_STATION_MODE_HELPER.Values(),
           _GROUND_STATION_MODE_HELPER.ShortNames())

  @MFig(title='Ground Station Transform Stage', ylabel='Stage [#]',
        xlabel='Time [s]')
  def PlotGroundStationTransformStage(self, e, c, s, params):
    plot(c['time'], c['control_input']['gs_sensors']['transform_stage'],
         label='ci')
    plot(c['time'], c['state_est']['gs_transform_stage'], label='est')
    if s is not None:
      # This value is not yet in simulator telemetry.
      pass

  # TODO: Create separate 'simulator' plot group.
  @MFig(title='Moments', ylabel='Nm', xlabel='Time [s]')
  def PlotKiteMoments(self, e, c, s, params, axis='y'):
    for what in ['aero', 'gravity', 'tether', 'rotors',
                 'disturb', 'blown_wing', 'total']:
      plot(s['time'], s['wing']['fm_'+what]['moment'][axis], label='fm_'+what)

  @MFig(title='Kite Azimuth and Elevation', ylabel='Angle [deg]',
        xlabel='Time [s]')
  def PlotKiteAzimuthAndElevation(self, e, c, s, params):
    wing_pos_g = s['wing']['Xg']
    plot(s['time'], np.rad2deg(np.arctan2(wing_pos_g['y'], wing_pos_g['x'])),
         label='kite azimuth')
    plot(s['time'], np.rad2deg(np.arctan2(-wing_pos_g['z'],
                                          np.hypot(wing_pos_g['x'],
                                                   wing_pos_g['y']))),
         label='kite elevation')

  @MFig(title='Air Density (Measured at Ground Station)',
        ylabel='Density [kg/m^3]', xlabel='Time [s]')
  def PlotDensity(self, e, c, s, params):
    plot(c['time'], c['state_est']['rho'], label='state_est.rho')
    plot(c['time'],
         np.full_like(c['time'],
                      params['system_params']['phys']['rho']),
         label='hard-coded value')

  @MFig(title='Tether Anchor Point', ylabel='[m]', xlabel='Time [s]')
  def PlotTetherAnchorPoint(self, e, c, s, params):
    mplot.PlotVec3(c['time'], c['state_est']['tether_anchor']['pos_g'],
                   label='pos_g [est]', linestyle='-')
    mplot.PlotVec3(c['time'], c['state_est']['tether_anchor']['pos_g_f'],
                   label='pos_g_f [est]', linestyle='--')

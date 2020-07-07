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

"""Plots relating to the ground estimator."""

from makani.analysis.plot.python import mplot
from makani.lib.python.h5_utils import numpy_utils
from matplotlib.pyplot import plot
import numpy as np

MFig = mplot.PlotGroup.MFig  # pylint: disable=invalid-name


class Plots(mplot.PlotGroup):
  """Plots of the ground estimator."""

  @MFig(title='Vessel Position', ylabel='[m]', xlabel='Time [s]')
  def PlotVesselPosition(self, g, s, params):
    mplot.PlotVec3(g['estimate']['time'],
                   g['estimate']['Xg'], label='[est]', linestyle='--')
    mplot.PlotVec3(g['estimate']['time'],
                   g['estimator']['gps']['Xg'], label='GPS (adjusted)',
                   linestyle=':')
    if s is not None:
      mplot.PlotVec3(s['time'], s['buoy']['Xg'], label='[sim]', linestyle='-')

  @MFig(title='Yaw Rate', ylabel='[deg/s]', xlabel='Time [s]')
  def PlotPlatformYawRate(self, g, s, params):
    plot(g['estimate']['time'],
         np.rad2deg(g['input']['gps_compass']['heading_rate']),
         label='GPS compass heading rate')
    plot(g['estimate']['time'], np.rad2deg(g['input']['imu']['gyro']['z']),
         label='IMU gyro yaw rate')
    plot(g['estimate']['time'], np.rad2deg(g['estimate']['pqr']['z']),
         label='Yaw rate [estimate]')

  # Plots for ground estimator input messages.
  @MFig(title='GS GPS position ECEF', ylabel='[m]', xlabel='Time [s]')
  def PlotGsGpsPositionEcef(self, g, s, params):
    mplot.PlotVec3(g['estimate']['time'], g['input']['gs_gps']['pos'])

  @MFig(title='GS GPS position sigmas', ylabel='[m]', xlabel='Time [s]')
  def PlotGsGpsPositionSigmas(self, g, s, params):
    mplot.PlotVec3(g['estimate']['time'], g['input']['gs_gps']['pos_sigma'])

  @MFig(title='GS GPS compass angles', ylabel='[deg]', xlabel='Time [s]')
  def PlotGsGpsCompass(self, g, s, params):
    plot(g['estimate']['time'],
         np.rad2deg(g['input']['gps_compass']['heading']),
         label='heading')
    plot(g['estimate']['time'],
         np.rad2deg(g['input']['gps_compass']['pitch']),
         label='pitch')

  @MFig(title='Accelerometer', ylabel='m/s^2', xlabel='Time [s]')
  def PlotAccelerometer(self, g, s, params):
    mplot.PlotVec3(g['estimate']['time'], g['input']['imu']['acc'],
                   label='accelerometer')

  @MFig(title='Gyro', ylabel='rad/sec', xlabel='Time [s]')
  def PlotGyro(self, g, s, params):
    mplot.PlotVec3(g['estimate']['time'], g['input']['imu']['gyro'],
                   label='gyro')

  @MFig(title='Magnetometer', ylabel='gauss', xlabel='Time [s]')
  def PlotMagnetometer(self, g, s, params):
    mplot.PlotVec3(g['estimate']['time'], g['input']['imu']['mag'],
                   label='mag')
    plot(g['estimate']['time'],
         numpy_utils.Vec3Norm(g['input']['imu']['mag']),
         'k--', label='mag norm')

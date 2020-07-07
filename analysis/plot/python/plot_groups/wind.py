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

"""Plots relating to the wind."""

from makani.analysis.plot.python import mplot
from makani.lib.python.h5_utils import numpy_utils
from matplotlib.pyplot import plot
from matplotlib.pyplot import ylabel
import pandas as pd

MFig = mplot.PlotGroup.MFig  # pylint: disable=invalid-name


class Plots(mplot.PlotGroup):
  """Plots of the wind data."""

  @MFig(title='Wind speed at ground level', ylabel='Speed [m/s]',
        xlabel='Time [s]')
  def PlotWindSpeedAtGround(self, c):
    wind_g_mag = numpy_utils.Vec3Norm(c['state_est']['wind_g']['vector'])
    plot(c['time'], wind_g_mag)

  @MFig(title='Wind speed aloft', ylabel='Speed [m/s]', xlabel='Time [s]')
  def PlotWindSpeedAloft(self, c):
    wind_aloft_g_mag = numpy_utils.Vec3Norm(
        c['state_est']['wind_aloft_g']['vector'])
    plot(c['time'], wind_aloft_g_mag)

  @MFig(title='Turbulence Intensity', xlabel='Time [s]', pedantic=False)
  def PlotTurbulenceIntensity(self, c, window_minutes=5):
    assert isinstance(window_minutes, int)
    assert window_minutes >= 1
    df = pd.DataFrame(index=pd.to_timedelta(c['time'], unit='s'))
    df['wind_g_mag'] = numpy_utils.Vec3Norm(c['state_est']['wind_g']['vector'])
    df_1s = df.resample('1s', label='right').agg(['mean', 'std'])
    df_1s_nt = df_1s.xs('mean', level=1, axis='columns').rolling(
        window='%dT' % window_minutes,
        min_periods=window_minutes * 60).agg(['mean', 'std']).shift(
            periods=(-window_minutes * 60) // 2, freq='s')

    df_1s_nt_ti = (df_1s_nt[('wind_g_mag', 'std')] /
                   df_1s_nt[('wind_g_mag', 'mean')])

    ylabel('TI (over %d-minute rolling calculations)' % window_minutes)
    plot(df_1s_nt_ti.keys().total_seconds(), df_1s_nt_ti)

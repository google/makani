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

"""Layout to monitor motors."""

from makani.avionics.common import gps_receiver
from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import control
from makani.gs.monitor2.apps.plugins.indicators import estimator
from makani.gs.monitor2.apps.plugins.indicators import gps
from makani.gs.monitor2.apps.plugins.indicators import ground_station
from makani.gs.monitor2.apps.plugins.layouts import gps_util


class EstimatorLayout(base.BaseLayout):
  """Layout to monitor estimators."""

  _NAME = 'Estimator'
  _DESIRED_VIEW_COLS = 12
  _MODE = common.FULL_COMMS_MODE
  _ORDER_HORIZONTALLY = False

  def Initialize(self):

    self._AddIndicators('Network', [
        control.FlightModeIndicator(self._MODE),
        control.ControlTimeIndicator(self._MODE),
        # {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateThrottle},
        control.HoverAnglesChart(self._MODE),
    ], {'cols': 3})

    self._AddBreak()

    self._AddIndicators('Imus', [
        estimator.EstimatorGyroBiasChart(self._MODE),
        estimator.EstimatorGyroBiasIndicator(self._MODE),
        # {col_0, INDICATOR_TYPE, (UpdateFuncType)&UpdateEstimatorAccBDiff},
        estimator.EstimatorGyroDiffIndicator(self._MODE),
        estimator.EstimatorMagnetometerDiffIndicator(self._MODE),
        estimator.EstimatorAttitudeDiffChart(self._MODE),
        control.FdDisabledIndicator(),
        control.FdAllActiveIndicator(),
        # {col_1, INDICATOR_CHART_TYPE,
        #  (UpdateFuncType)&UpdateEstimatorTetherForce},
        control.WingPosChart(self._MODE),
        # {col_1, INDICATOR_TYPE,
        #  (UpdateFuncType)&UpdateEstimatorCurrentGpsReceiver},
    ], {'cols': 3})

    self._AddBreak()

    self._AddIndicators('Gs GPS (NovAtel)', [
        gps.NovAtelSigmasIndicator('GpsBaseStation'),
        gps.GsNovAtelPosVelTypeIndicator('GpsBaseStation'),
        gps.NovAtelCn0Indicator('GpsBaseStation'),
    ], properties={'cols': 3})

    wing_gps_indicators = []
    for gps_type, fc_name in gps_util.GpsSelector():
      if gps_type == gps_receiver.GpsReceiverType.NOV_ATEL.value:
        wing_gps_indicators += [
            gps.NovAtelSigmasIndicator(fc_name, name='Sigmas (%s)' % fc_name),
            gps.WingNovAtelPosVelTypeIndicator(
                fc_name, name='Pos./Vel. Type (%s)' % fc_name),
            gps.NovAtelCn0Indicator(fc_name, name='Cn0 (%s)' % fc_name),
        ]
      elif gps_type == gps_receiver.GpsReceiverType.SEPTENTRIO.value:
        wing_gps_indicators += [
            gps.SeptentrioSigmasIndicator(fc_name,
                                          name='Sigmas (%s)' % fc_name),
            gps.SeptentrioModeIndicator(
                fc_name, name='Mode (%s)' % fc_name),
            gps.SeptentrioCn0Indicator(fc_name, name='Cn0 (%s)' % fc_name),
        ]
      else:
        raise ValueError('Invalid GPS type: %d.' % gps_type)

    self._AddIndicators('Wing GPS', wing_gps_indicators + [
        # {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateEstimatorGpsDiff},
        # {col_1, INDICATOR_TYPE, (UpdateFuncType)&UpdateEstimatorGsPosEcef},
        # {col_1, INDICATOR_CHART_TYPE, (UpdateFuncType)&UpdateGsGpsToWingDist},
    ], properties={'cols': 3})

    self._AddIndicators('Platform', [
        # {col_2, INDICATOR_TYPE, (UpdateFuncType)&UpdateEstimatorGsgBias},
        # {col_2, XY_PLOT_TYPE, (UpdateFuncType)&UpdatePerchPosition},
        ground_station.PerchAzimuthIndicator(),
    ], {'cols': 3})

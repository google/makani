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

"""Layout to monitor ground station status."""

from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.plugins.indicators import aio_comms
from makani.gs.monitor2.apps.plugins.indicators import control
from makani.gs.monitor2.apps.plugins.indicators import gps
from makani.gs.monitor2.apps.plugins.indicators import ground_station


class GsLayout(base.BaseLayout):
  """The ground station layout."""

  _NAME = 'Ground Station'
  _DESIRED_VIEW_COLS = 2
  _GPS_NODE = 'GpsBaseStation'
  _ORDER_HORIZONTALLY = False

  def Initialize(self):

    self._AddIndicators('AIO Update', [
        aio_comms.GsCoreSwitchAioUpdateIndicator(),
        aio_comms.GsGpsAioUpdateIndicator(),
        aio_comms.PlatformSensorsAioUpdateIndicator(),
    ])

    self._AddIndicators('Wind', [
        ground_station.WindIndicator(),
        ground_station.WindSensorSpeedIndicator(),
        ground_station.WindSensorStatusIndicator(),
        control.WindStateEstIndicator(),
        ground_station.WeatherSensorIndicator(),
        ground_station.AirDensityIndicator(),
    ])

    self._AddIndicators('GPS', [
        gps.NovAtelNumSatsIndicator(self._GPS_NODE),
        gps.NovAtelCn0Indicator(self._GPS_NODE),
        gps.NovAtelSigmasIndicator(self._GPS_NODE),
        gps.CompassHeadingIndicator(self._GPS_NODE),
    ])

    self._AddIndicators('Winch PLC', [
        ground_station.PerchAzimuthIndicator(),
        ground_station.GsgAzimuthIndicator(['A']),
        ground_station.GsgElevationIndicator(),
        ground_station.PlcStatusIndicator(),
        # TODO: The following indicators will be removed in the future
        # when we test the top head for China Lake.
        ground_station.LevelwindElevationIndicator(),
        ground_station.WinchArmedIndicator(),
        ground_station.DrumStateIndicator(),
        ground_station.WinchProximityIndicator(),
    ])

    self._AddBreak()

    self._AddIndicators('PLC', [
        ground_station.DetwistArmedIndicator(),
        ground_station.DetwistStatusIndicator(),
        ground_station.DetwistTemperatureIndicator(),
        ground_station.DetwistStatusInfoIndicator(),
        ground_station.Ground480VIndicator(),
    ])

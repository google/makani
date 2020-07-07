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
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import avionics
from makani.gs.monitor2.apps.plugins.indicators import control
from makani.gs.monitor2.apps.plugins.indicators import gps
from makani.gs.monitor2.apps.plugins.indicators import ground_station


class Gs02Layout(base.BaseLayout):
  """The ground station layout."""

  _NAME = 'Ground Station 02'
  _DESIRED_VIEW_COLS = 4
  _GPS_NODE = 'GpsBaseStation'
  _ORDER_HORIZONTALLY = False
  _MODE = common.SPARSE_COMMS_MODE

  def Initialize(self):

    self._AddIndicators('General', [
        control.FlightModeIndicator(self._MODE),
        control.FlightModeGatesIndicator(self._MODE),
        ground_station.GsModeIndicator(),
        ground_station.Ground480VIndicator(),
        ground_station.EStopStatusIndicator(),
        ground_station.GsCommsIndicator(),
        ground_station.GsArmingStatusIndicator(),
        ground_station.GsProxSensorsIndicator(),
        ground_station.GsWingProximityIndicator(),
        ground_station.GsTetherEngagementIndicator(),
        ground_station.TetherUpPlatformSensorsStatusIndicator('A'),
        ground_station.TetherUpPlatformSensorsStatusIndicator('B'),
        ground_station.TetherUpDrumSensorsStatusIndicator('A'),
        ground_station.TetherUpDrumSensorsStatusIndicator('B'),
        ground_station.TetherUpGsStatusIndicator(),
        ground_station.DetwistKiteLoopErrorIndicator(),
    ])

    self._AddIndicators('Wind', [
        ground_station.WindIndicator(),
        ground_station.WindSensorStatusIndicator(),
        ground_station.WeatherSensorIndicator(),
        ground_station.AirDensityIndicator(),
    ])

    self._AddIndicators('GPS', [
        gps.NovAtelNumSatsIndicator(self._GPS_NODE),
        gps.NovAtelCn0Indicator(self._GPS_NODE),
        gps.NovAtelSigmasIndicator(self._GPS_NODE),
    ])

    self._AddIndicators('Compass', [
        gps.CompassHeadingIndicator(self._GPS_NODE),
        gps.CompassNumSatsIndicator(self._GPS_NODE),
        gps.CompassSigmasIndicator(self._GPS_NODE),
        gps.CompassSolTypeIndicator(self._GPS_NODE),
    ])

    self._AddIndicators('Temperatures', [
        avionics.MaxBoardTemperature(
            'Gs02 Max Board Temp',
            ['DrumSensorsMonitor', 'PlatformSensorsMonitor', 'CoreSwitchStatus',
             'GpsStatus', 'RecorderStatus', 'GroundStationPlcMonitorStatus'],
            nodes=['DrumSensorsA', 'DrumSensorsB', 'PlatformSensorsA',
                   'PlatformSensorsB', 'CsGsA', 'CsGsB', 'GpsBaseStation',
                   'RecorderTms570Platform', 'PlcGs02']),
        ground_station.MaxServoTempIndicator(),
    ])

    self._AddBreak()

    self._AddIndicators('Azimuth', [
        ground_station.GsOrientationWindow(),
        ground_station.AzimuthErrorChart(),
        ground_station.GsAzimuthVelocityChart(),
    ])

    self._AddIndicators('Consistency', [
        ground_station.PerchAziConsistentIndicator(),
        ground_station.DetwistConsistentIndicator(),
        ground_station.WinchConsistentIndicator(),
        ground_station.LvlShuttleConsistentIndicator(),
        ground_station.LvlShoulderConsistentIndicator(),
        ground_station.LvlWristConsistentIndicator(),
        ground_station.GsgYokeConsistentIndicator(),
        ground_station.GsgTermConsistentIndicator(),
    ])

    self._AddIndicators('Vessel', [
        ground_station.VesselVelocityChart(),
        ground_station.VesselOrientationChart(),
    ])

    self._AddBreak()

    self._AddIndicators('Drum', [
        ground_station.DrumOrientationWindow(),
        control.PayoutIndicator(),
        ground_station.GsWinchPositionChart(),
        ground_station.GsWinchVelocityChart(),
        ground_station.GsLevelwindPositionChart(),
    ])

    self._AddBreak()

    self._AddIndicators('Detwist', [
        ground_station.DetwistErrorChart(),
        ground_station.GsDetwistPositionChart(),
    ])

    self._AddIndicators('Tether Elevation Angle', [
        ground_station.TetherAngleChart(),
        ground_station.GsgAngleCrosswindDictChart(['A', 'B']),
    ])

    self._AddIndicators('Debug', [
        ground_station.GsTorqueChart(),
    ])

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

"""Layout to monitor flight status through ControlTelemetry."""

from makani.avionics.common import gps_receiver
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import avionics
from makani.gs.monitor2.apps.plugins.indicators import batt
from makani.gs.monitor2.apps.plugins.indicators import gps
from makani.gs.monitor2.apps.plugins.layouts import flight_template
from makani.gs.monitor2.apps.plugins.layouts import gps_util


class DebugFlightLayout(flight_template.FlightLayout):
  """The debug flight layout."""

  _NAME = 'Debug (Flight)'
  _MODE = common.FULL_COMMS_MODE

  def Initialize(self):
    super(DebugFlightLayout, self).Initialize()

    self._AddIndicators('Avionics', [
        avionics.MaxBoardTemperature(
            'Avionics Max Board Temp',
            ['BatteryStatus', 'CoreSwitchStatus', 'FlightComputerSensor',
             'GpsStatus', 'Loadcell', 'RecorderStatus', 'ServoStatus']),
        avionics.MaxBoardTemperature(
            'Groundvionics Max Board Temp',
            ['DrumSensorsMonitor', 'GroundStationPlcMonitorStatus',
             'JoystickMonitorStatus', 'PlatformSensorsMonitor']),
        batt.LvBusVoltageIndicator(),
        batt.StateOfChargeIndicator(['A', 'B'], 'State of Charge'),
    ])

    wing_gps_indicators = []
    for gps_type, fc_name in gps_util.GpsSelector():
      if gps_type == gps_receiver.GpsReceiverType.NOV_ATEL.value:
        wing_gps_indicators += [
            gps.NovAtelSigmasIndicator(fc_name, name='%s Sigmas' % fc_name),
            gps.WingNovAtelPosVelTypeIndicator(
                fc_name, name='%s Sol. Type' % fc_name),
            gps.NovAtelCn0Indicator(fc_name, name='%s Cn0' % fc_name),
        ]
      elif gps_type == gps_receiver.GpsReceiverType.SEPTENTRIO.value:
        wing_gps_indicators += [
            gps.SeptentrioSigmasIndicator(fc_name,
                                          name='%s Sigmas' % fc_name),
            gps.SeptentrioModeIndicator(
                fc_name, name='%s Mode' % fc_name),
            gps.SeptentrioCn0Indicator(fc_name, name='%s Cn0' % fc_name),
        ]
      else:
        raise ValueError('Invalid GPS type: %d.' % gps_type)

    self._AddIndicators('Wing GPS', wing_gps_indicators)

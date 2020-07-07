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

"""Layout to monitor flight status  through TetherDown and TetherUp."""

from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import gps
from makani.gs.monitor2.apps.plugins.indicators import node_status
from makani.gs.monitor2.apps.plugins.layouts import flight_template


class FlightLayout(flight_template.FlightLayout):
  """The flight layout."""

  _NAME = 'Flight'
  _MODE = common.SPARSE_COMMS_MODE

  def Initialize(self):
    super(FlightLayout, self).Initialize()

    self._AddIndicators('Avionics', [
        node_status.TetherMaxBoardTemperature(
            'Max Board Temp', [[0, 85]], [[0, 100]], 30),
        node_status.TetherMaxBoardHumidity(
            'Board Humidity', [[0, 80]], [[0, 90]], 30),
        node_status.TetherNodeNetworkSummary('Network'),
        node_status.TetherNodeSelfTestSummary('Self Test'),
        # Error reporting for Mvlv is disabled for SN1/Gin because of sensor
        # noise.
        node_status.TetherNodeErrorSummary('Error'),
        node_status.TetherNodeWarningSummary('Warning'),
    ])

    self._AddIndicators('Wing GPS', [
        gps.TetherDownGpsIndicator(wing_gps_receiver='Crosswind'),
        gps.TetherDownGpsIndicator(wing_gps_receiver='Hover'),
        gps.TetherDownGpsIndicator(wing_gps_receiver='Port'),
        gps.TetherDownGpsIndicator(wing_gps_receiver='Star'),
    ])

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

from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import node_status

_WING_TMS570_NODES = common.WingTms570Nodes()


class StatusLayout(base.BaseLayout):
  """Layout to monitor motors."""

  _NAME = 'Status'
  _DESIRED_VIEW_COLS = 12

  def Initialize(self):

    self._AddIndicators('Network', [
        node_status.TetherNodeNetworkIndicator(
            node_name, node_name,
            node_name not in common.NETWORK_STATUS_NODES_TO_EXCLUDE)
        for node_name in _WING_TMS570_NODES
    ], {'cols': 3})

    self._AddIndicators('Failures', [
        node_status.TetherNodeFailureIndicator(node_name, node_name)
        for node_name in _WING_TMS570_NODES
    ], {'cols': 2})

    self._AddIndicators('Power', [
        node_status.TetherNodePowerIndicator(node_name, node_name)
        for node_name in _WING_TMS570_NODES
    ], {'cols': 2})

    self._AddIndicators('Temperature [C]', [
        node_status.TetherNodeTempIndicator(node_name, node_name)
        for node_name in _WING_TMS570_NODES
    ], {'cols': 2})

    self._AddIndicators('Humidity', [
        node_status.TetherNodeHumidityIndicator(node_name, node_name)
        for node_name in _WING_TMS570_NODES
    ], {'cols': 2})

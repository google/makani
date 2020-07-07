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

"""Layout to monitor low voltage bus boards' status."""

from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.plugins.indicators import batt
from makani.gs.monitor2.apps.plugins.indicators import mvlv


class LvLayout(base.BaseLayout):
  """The flight layout."""

  _NAME = 'LV Monitor'

  def Initialize(self):

    self._AddIndicators('Batts', [
        batt.BattConnectedIndicator(['A', 'B'], 'Battery Status'),
        batt.BattAnalogIndicator(),
        batt.BattChargerIndicator('Charger Output'),
        batt.BattMcp342xIndicator(),
        batt.BalancerIndicator(['A', 'B']),
        batt.BattErrorWarningIndicator(['A', 'B'], 'Errors and Warnings'),
    ])
    self._UpdateProperties('Batts', {'cols': 3})

    self._AddIndicators('MVLV', [
        mvlv.MvlvStatusIndicator('MVLV Status'),
        mvlv.MvlvAnalogIndicator(),
        mvlv.MvlvMcp342xIndicator(),
        mvlv.MvlvErrorWarningIndicator('Errors and Warnings'),
    ])
    self._UpdateProperties('MVLV', {'cols': 3})

    # TODO: Summary indicators for LV bus voltage, temperatures, faults.

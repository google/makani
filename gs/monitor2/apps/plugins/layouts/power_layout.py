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

"""Layout to monitor ground power status."""

from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.plugins.indicators import ground_power


class PowerLayout(base.BaseLayout):
  """The ground station layout."""

  _NAME = 'Ground Power'
  _DESIRED_VIEW_COLS = 12
  _ORDER_HORIZONTALLY = True

  def Initialize(self):

    self._AddIndicators('Summary', [
        ground_power.SummaryDCVoltageIndicator(show_label=True),
        ground_power.SummaryDCCurrentIndicator(show_label=False),
        ground_power.SummaryFaultInductorIndicator(show_label=False),
        ground_power.SummaryMeanCommonModeVoltsIndicator(show_label=False),
        ground_power.SummaryInstCommonModeVoltsIndicator(show_label=False),
        ground_power.SummaryCBAirTempIndicator(show_label=False),
        ground_power.SummaryTransformerTempIndicator(show_label=False),
        ground_power.SummaryHeatSink1T1Indicator(show_label=False),
        ground_power.SummaryHeatSink1T2Indicator(show_label=False),
        ground_power.SummaryFaultIndicator(
            'Fault 1', word_index=0, show_label=False),
        ground_power.SummaryFaultIndicator(
            'Fault 2', word_index=1, show_label=False),
        ground_power.SummaryFaultIndicator(
            'Fault 3', word_index=2, show_label=False),
        ground_power.SummaryFaultIndicator(
            'Fault 4', word_index=3, show_label=False),
        ground_power.SummaryFaultIndicator(
            'Fault 5', word_index=4, show_label=False),
        ground_power.SummaryFaultIndicator(
            'Fault 6', word_index=5, show_label=False),
        ground_power.SummaryFaultIndicator(
            'Fault 7', word_index=6, show_label=False),
        ground_power.SummaryFaultIndicator(
            'Fault 8', word_index=7, show_label=False),
    ], properties={'cols': 10})

    self._AddBreak()

    self._AddIndicators('Voltage', [
        ground_power.VoltageIndicator([0, 1, 2]),
        ground_power.VoltageIndicator([3, 4, 5]),
    ], properties={'cols': 5})

    self._AddIndicators('Current', [
        ground_power.CurrentIndicator([0, 1, 2]),
        ground_power.CurrentIndicator([3, 4, 5]),
    ], properties={'cols': 5})

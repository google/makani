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
from makani.gs.monitor2.apps.plugins.indicators import motor


class MotorLayout(base.BaseLayout):
  """Layout to monitor motors."""

  _NAME = 'Motors'
  _DESIRED_VIEW_COLS = 2
  _MODE = common.FULL_COMMS_MODE

  def Initialize(self):
    # Define how motors should be grouped, each group renders a chart.
    self._motor_groups = {
        'PTO/SBO': ['Pto', 'Sbo'],
        'PBO/STO': ['Pbo', 'Sto'],
        'PTI/SBI': ['Pti', 'Sbi'],
        'PBI/STI': ['Pbi', 'Sti'],
    }

    self._AddIndicators('Status', [
        motor.AioUpdateIndicator(self._MODE),
        motor.ArmedIndicator(self._MODE),
        motor.ErrorIndicator(self._MODE),
        motor.WarningIndicator(self._MODE),
        motor.MotorBusVoltageIndicator(self._MODE),
    ])

    self._AddIndicators('Temperatures', [
        motor.BoardTemperatureIndicator(self._MODE),
        motor.HeatPlateTemperatureIndicator(self._MODE),
        motor.StatorCoreTemperatureIndicator(self._MODE),
        motor.WindingTemperatureIndicator(self._MODE),
    ])

    self._AddIndicators('Warning and error printout', [
        motor.MotorFlagNameIndicator(self._MODE, 'Warnings'),
        motor.MotorFlagNameIndicator(self._MODE, 'Errors'),
    ])

    self._UpdateProperties('Warning and error printout', {'cols': 2})

    self._AddIndicators('Speed', [
        motor.SpeedStackingChart(self._MODE, group_name, motor_short_names)
        for group_name, motor_short_names in self._motor_groups.iteritems()])

    self._UpdateProperties('Speed', {'cols': 2})

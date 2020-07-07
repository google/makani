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

import copy

from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import motor


class DynoLayout(base.BaseLayout):
  """Layout to monitor dynos."""

  _NAME = 'Dyno'
  _DESIRED_VIEW_COLS = 6
  _DEFAULT_FONT_SIZE = 10
  _MODE = common.FULL_COMMS_MODE

  def _GetMotorGroups(self):
    """Define how motors should be grouped, each group renders a chart."""
    return {
        'PTO/SBO': ['Pto', 'Sbo'],
        'PBO/STO': ['Pbo', 'Sto'],
        'PTI/SBI': ['Pti', 'Sbi'],
        'PBI/STI': ['Pbi', 'Sti'],
    }

  def Initialize(self):

    yasa_common_args = {
        'aio_node_prefix': 'Motor',
    }

    protean_common_args = {
        'aio_node_prefix': 'DynoMotor',
    }

    motor_rows = [
        ('Top', ['Pto', 'Pti', 'Sti', 'Sto']),
        ('Bottom', ['Pbo', 'Pbi', 'Sbi', 'Sbo']),
    ]

    brands = ['Yasa', 'Protean']

    self._motor_groups = self._GetMotorGroups()

    for row_name, motors in motor_rows:
      for pos, common_args in enumerate(
          [yasa_common_args, protean_common_args]):
        brand = brands[pos]
        kwargs = copy.copy(common_args)
        kwargs.update({'motor_labels': motors})

        self._AddIndicators('%s Status (%s)' % (brand, row_name), [
            motor.AioUpdateIndicator(self._MODE, show_label=True, **kwargs),
            motor.ArmedIndicator(self._MODE, show_label=False, **kwargs),
            motor.ErrorIndicator(self._MODE, show_label=False, **kwargs),
            motor.WarningIndicator(self._MODE, show_label=False, **kwargs),
        ])

        self._AddIndicators('%s Temperatures (%s)' % (brand, row_name), [
            motor.BoardTemperatureIndicator(
                self._MODE, show_label=True, **kwargs),
            motor.HeatPlateTemperatureIndicator(
                self._MODE, show_label=False, **kwargs),
        ])

        self._AddIndicators('%s Temperatures (%s) ' % (brand, row_name), [
            motor.StatorCoreTemperatureIndicator(
                self._MODE, show_label=True, **kwargs),
            motor.WindingTemperatureIndicator(
                self._MODE, show_label=False, **kwargs),
        ])

    for pos, common_args in enumerate([yasa_common_args, protean_common_args]):
      brand = brands[pos]

      self._AddIndicators('%s Bus Voltage' % brand, [
          motor.MotorBusVoltageStackingChart(
              self._MODE, group_name, motor_short_names, **common_args)
          for group_name, motor_short_names in self._motor_groups.iteritems()])

      self._AddIndicators('%s Bus Current' % brand, [
          motor.MotorBusCurrentStackingChart(
              self._MODE, group_name, motor_short_names, **common_args)
          for group_name, motor_short_names in self._motor_groups.iteritems()])

      self._AddIndicators('%s Drive Voltage' % brand, [
          motor.DriveVoltageStackingChart(
              self._MODE, group_name, motor_short_names, **common_args)
          for group_name, motor_short_names in self._motor_groups.iteritems()])

    for pos, common_args in enumerate([yasa_common_args, protean_common_args]):
      brand = brands[pos]

      self._AddIndicators('%s Torque Cmd' % brand, [
          motor.TorqueCmdStackingChart(
              self._MODE, group_name, motor_short_names, **common_args)
          for group_name, motor_short_names in self._motor_groups.iteritems()])

      self._AddIndicators('%s Speed' % brand, [
          motor.SpeedStackingChart(
              self._MODE, group_name, motor_short_names, **common_args)
          for group_name, motor_short_names in self._motor_groups.iteritems()])

      self._AddIndicators('%s Iq' % brand, [
          motor.IqStackingChart(
              self._MODE, group_name, motor_short_names, **common_args)
          for group_name, motor_short_names in self._motor_groups.iteritems()])

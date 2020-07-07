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

"""Classes to perform checks on flight computers."""

from makani.analysis.checks import base_check
from makani.analysis.checks.collection import avionics_checks
from makani.avionics.firmware.monitors import fc_types
from makani.avionics.network import aio_node
from makani.lib.python import c_helpers

_FC_LABELS_HELPER = c_helpers.EnumHelper('FcLabel', aio_node,
                                         prefix='kAioNodeFc')
_FC_ANALOG_VOLTAGE_HELPER = c_helpers.EnumHelper('FcAnalogVoltage', fc_types)
_FC_INA219_MONITOR_HELPER = c_helpers.EnumHelper('FcIna219Monitor', fc_types)


class FcMonAnalogChecker(avionics_checks.BaseVoltageChecker):
  """The monitor to check flight computer voltage."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, fc_short_name, **base_kwargs):
    """Initialize the voltage checker for a given flight computer.

    Args:
      for_log: True if this check is performed over a log. False if it is for
          realtime AIO messages.
      fc_short_name: Short name of a Flight Computer.
      **base_kwargs: Args for the base class, including name, enum_names,
          verbose_names, normal_ranges, and warning_ranges.
    """

    super(FcMonAnalogChecker, self).__init__(
        for_log, 'FlightComputerSensor', 'Fc' + fc_short_name,
        'fc_mon.analog_data', 'fc_mon.analog_populated',
        _FC_ANALOG_VOLTAGE_HELPER, **base_kwargs)


class FcMonBusVoltageChecker(avionics_checks.BaseVoltageChecker):
  """The monitor to check ina219 values."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, fc_short_name, **base_kwargs):

    super(FcMonBusVoltageChecker, self).__init__(
        for_log, 'FlightComputerSensor', 'Fc' + fc_short_name,
        'fc_mon.ina219_data[:].voltage', 'fc_mon.ina219_populated',
        _FC_INA219_MONITOR_HELPER, **base_kwargs)


class FcMonBusCurrentChecker(avionics_checks.BaseSensorGroupChecker):
  """The monitor to check ina219 values."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, fc_short_name, **base_kwargs):

    super(FcMonBusCurrentChecker, self).__init__(
        for_log, 'FlightComputerSensor', 'Fc' + fc_short_name,
        'fc_mon.ina219_data[:].current', 'fc_mon.ina219_populated',
        _FC_INA219_MONITOR_HELPER, **base_kwargs)


class FcChecks(avionics_checks.BaseAvionicsChecks):
  """A checklist containing checks to perform for all flight computers."""

  def __init__(self, for_log):
    super(FcChecks, self).__init__(
        for_log, 'FlightComputerSensor', _FC_LABELS_HELPER, 'Fc')

    self._items_to_check += [
        FcMonAnalogChecker(for_log, s, name='Fc%s.FcMon.Analog.Voltage' % s)
        for s in _FC_LABELS_HELPER.ShortNames()
    ] + [
        FcMonBusVoltageChecker(for_log, s, name='Fc%s.FcMon.BusVoltage' % s)
        for s in _FC_LABELS_HELPER.ShortNames()
    ] + [
        FcMonBusCurrentChecker(for_log, s, name='Fc%s.FcMon.BusCurrent' % s)
        for s in _FC_LABELS_HELPER.ShortNames()
    ]

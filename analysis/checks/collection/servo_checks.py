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

"""Classes to perform checks on servos."""

from makani.analysis.checks import base_check
from makani.analysis.checks.collection import avionics_checks
from makani.avionics.firmware.monitors import servo_types
from makani.avionics.network import aio_node
from makani.lib.python import c_helpers

_SERVO_LABELS_HELPER = c_helpers.EnumHelper('ServoLabel', aio_node,
                                            prefix='kAioNodeServo')
_SERVO_ANALOG_VOLTAGE_HELPER = c_helpers.EnumHelper('ServoAnalogVoltage',
                                                    servo_types)
_SERVO_MCP9800_MONITOR_HELPER = c_helpers.EnumHelper('ServoMcp9800',
                                                     servo_types)
_SERVO_MONITOR_ERROR_HELPER = c_helpers.EnumHelper('ServoMonitorError',
                                                   servo_types)
_SERVO_MONITOR_WARNING_HELPER = c_helpers.EnumHelper('ServoMonitorWarning',
                                                     servo_types)


class ServoMonAnalogChecker(avionics_checks.BaseVoltageChecker):
  """The monitor to check servo voltage."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, servo_short_name, enum_names=None, **base_kwargs):
    """Initialize the voltage checker for a given servo.

    Args:
      for_log: True if this check is performed over a log. False if it is for
          realtime AIO messages.
      servo_short_name: Short name of a servo.
      enum_names: A list of short names of device enum types to check.
      **base_kwargs: Args for the base class, including name,
          verbose_names, normal_ranges, and warning_ranges.
    """
    # Remove ClampResistor which is only used for debugging.
    if enum_names is None:
      enum_names = [x for x in _SERVO_ANALOG_VOLTAGE_HELPER.ShortNames()
                    if x != 'ClampResistor']

    super(ServoMonAnalogChecker, self).__init__(
        for_log, 'ServoStatus', 'Servo' + servo_short_name,
        'servo_mon.analog_data', 'servo_mon.analog_populated',
        _SERVO_ANALOG_VOLTAGE_HELPER, enum_names=enum_names, **base_kwargs)


class ServoMonTemperatureChecker(avionics_checks.BaseSensorGroupChecker):
  """The monitor to check servo temperature."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, servo_short_name, **base_kwargs):

    super(ServoMonTemperatureChecker, self).__init__(
        for_log, 'ServoStatus', 'Servo' + servo_short_name,
        'servo_mon.mcp9800_data', 'servo_mon.mcp9800_populated',
        _SERVO_MCP9800_MONITOR_HELPER, **base_kwargs)


class ServoErrorCheck(base_check.BitmaskErrorCheck):
  """Class to check for servo_mon.flags for errors."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, servos=None, errors=None, name=None):
    """Initialize this class.

    Args:
      for_log: True if we check against logs, False for AIO messages.
      servos: A list of servo short names to check, defaults to checking all.
          e.g. ['E2', 'R1', 'A8']
      errors: A list of errors to check for, defaults to checking all.
          e.g. ['kServoMonitorWarning12v', kServoMonitorWarningLvA']
      name: Name of the check.
    """
    if servos is None:
      servo_labels = _SERVO_LABELS_HELPER.ShortNames()
    else:
      servo_labels = servos

    sources = ['Servo'+s for s in servo_labels]
    error_if_fail = True

    super(ServoErrorCheck, self).__init__(for_log, 'ServoStatus', sources,
                                          'servo_mon.flags.error',
                                          _SERVO_MONITOR_ERROR_HELPER,
                                          error_if_fail, errors, name)


class ServoWarningCheck(base_check.BitmaskErrorCheck):
  """Class to check for servo_mon.flags for warnings."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, servos=None, errors=None, name=None):
    """See ServoErrorCheck above for documentation."""
    if servos is None:
      servo_labels = _SERVO_LABELS_HELPER.ShortNames()
    else:
      servo_labels = servos

    sources = ['Servo'+s for s in servo_labels]
    error_if_fail = False

    super(ServoWarningCheck, self).__init__(for_log, 'ServoStatus', sources,
                                            'servo_mon.flags.warning',
                                            _SERVO_MONITOR_WARNING_HELPER,
                                            error_if_fail, errors, name)


class ServoChecks(avionics_checks.BaseAvionicsChecks):
  """A checklist containing checks to perform for all servos."""

  def __init__(self, for_log):
    super(ServoChecks, self).__init__(
        for_log, 'ServoStatus', _SERVO_LABELS_HELPER, 'Servo')

    self._items_to_check += [
        ServoMonAnalogChecker(for_log, s,
                              name='Servo%s.ServoMon.AnalogVoltage' % s)
        for s in _SERVO_LABELS_HELPER.ShortNames()
    ] + [
        ServoMonTemperatureChecker(
            for_log, s, name='Servo%s.ServoMon.Temperature' % s)
        for s in _SERVO_LABELS_HELPER.ShortNames()
    ]

    self._items_to_check += [
        ServoErrorCheck(for_log, errors=[error],
                        name='Servo Error (%s)' % error)
        for error in _SERVO_MONITOR_ERROR_HELPER.ShortNames()
    ] + [
        ServoWarningCheck(for_log, errors=[error],
                          name='Servo Warning (%s)' % error)
        for error in _SERVO_MONITOR_WARNING_HELPER.ShortNames()
    ]

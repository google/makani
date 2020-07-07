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

""""Monitor indicators from the ground station."""

import collections
import operator

from makani.analysis.checks import avionics_util
from makani.analysis.checks import check_range
from makani.analysis.control import flap_limits
from makani.avionics.common import pack_avionics_messages
from makani.avionics.common import servo_types as servo_common
from makani.avionics.firmware.monitors import servo_types
from makani.avionics.network import aio_labels
from makani.control import control_types
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import avionics
from makani.lib.python import c_helpers
from makani.lib.python import struct_tree

import numpy

_SERVO_WARNING_HELPER = c_helpers.EnumHelper('ServoWarning', servo_common)
_SERVO_ERROR_HELPER = c_helpers.EnumHelper('ServoError', servo_common)

_SERVO_STATUS_HELPER = c_helpers.EnumHelper('ServoStatus', servo_common)

_SERVO_LABELS_HELPER = c_helpers.EnumHelper('ServoLabel', aio_labels,
                                            prefix='kServo')

_SERVO_ANALOG_VOLTAGE_HELPER = c_helpers.EnumHelper('ServoAnalogVoltage',
                                                    servo_types)

_SERVO_MON_WARNING_HELPER = c_helpers.EnumHelper('ServoMonitorWarning',
                                                 servo_types)
_SERVO_MON_ERROR_HELPER = c_helpers.EnumHelper('ServoMonitorError',
                                               servo_types)
_ACTUATOR_STATE_HELPER = c_helpers.EnumHelper('ActuatorState',
                                              pack_avionics_messages,
                                              exclude='ActuatorStateCommand')


class BaseServoIndicator(avionics.BaseActuatorIndicator):
  """Base class with utilities shared by servo indicators."""

  def __init__(self, mode, label, precision,
               servo_labels=_SERVO_LABELS_HELPER.ShortNames(),
               show_label=True):
    super(BaseServoIndicator, self).__init__(
        mode, label, precision, servo_labels, 'Servo',
        _SERVO_LABELS_HELPER, common.MAX_NO_UPDATE_COUNT_SERVO_STATUS,
        full_comms_message_type='ServoStatus',
        tether_attribute='servo_statuses', show_label=show_label)


class BaseArmedIndicator(BaseServoIndicator):
  """Base indicator for servos' armed status."""

  def _GetSingleValue(self, arg_idx, *args):
    """Obtain a single value for one servo, invoked within _GetAvailableValues.

    Args:
      arg_idx: The index referring to the n-th servo.
      *args: The list of attributes to the indicator. The attributes vary
          in different modes. For FULL_COMMS_MODE, it is the list
          of ServoStatus messages for each servo, so args[arg_idx] refers to
          the servo's message struct. For SPARSE_COMMS_MODE, it is
          [TetherDown.servo_statuses, valid, timestamp_sec], so
          args[0][`EnumValue(A2)`] refers to the state of servo A2.

    Returns:
      The servo status of the n-th servo.
    """
    if self._mode == common.FULL_COMMS_MODE:
      if struct_tree.IsValidElement(args[arg_idx]):
        return args[arg_idx].flags.status
      else:
        return None
    elif self._mode == common.SPARSE_COMMS_MODE:
      return self._GetTetherValue(args[0], self._node_labels[arg_idx], 'state')
    else:
      assert False

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    """Get the armed information of all servos.

    Args:
      *args: The list of attributes to the indicator. The attributes vary
          in different modes. For FULL_COMMS_MODE, it is the list
          of ServoStatus messages for each servo, so args[arg_idx] refers to
          the servo's message struct. For SPARSE_COMMS_MODE, it is
          [TetherDown.servo_statuses, valid, timestamp_sec], so
          args[0][`EnumValue(A2)`] refers to the state of servo A2.

    Returns:
      The text and stoplight to show.
    """

    servo_status = self._GetAvailableValues(*args)

    if self._mode == common.FULL_COMMS_MODE:
      status_helper = _SERVO_STATUS_HELPER
      expecting = ['Armed']
    elif self._mode == common.SPARSE_COMMS_MODE:
      status_helper = _ACTUATOR_STATE_HELPER
      expecting = ['Armed', 'Running']
    else:
      assert False

    return self._CheckStatusFlags(servo_status, status_helper, expecting,
                                  stoplights.STOPLIGHT_ERROR)


class BaseR22TemperatureIndicator(BaseServoIndicator):
  """Base indicator for servos' R22 temperatures."""

  def __init__(self, *args, **kwargs):
    super(BaseR22TemperatureIndicator, self).__init__(*args, show_label=False,
                                                      **kwargs)
    self._normal_ranges = check_range.BuildRanges([[None, 65]])
    self._warning_ranges = check_range.BuildRanges([[None, 75]])

  def _GetSingleValue(self, arg_idx, *args):
    if self._mode == common.FULL_COMMS_MODE:
      if struct_tree.IsValidElement(args[arg_idx]):
        return args[arg_idx].r22.temperature
      else:
        return None
    elif self._mode == common.SPARSE_COMMS_MODE:
      return self._GetTetherValue(
          args[0], self._node_labels[arg_idx], 'r22_temp')
    else:
      assert False

  @indicator.ReturnIfInputInvalid('', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    temperatures, stoplight = self._GetFieldInfo(
        self._normal_ranges, self._warning_ranges, None, *args)
    return self._DictToString(temperatures), stoplight


class BaseLvBusIndicator(indicator.BaseIndicator):
  """The base class for low voltage bus indicators."""

  _voltage_names = ['LvA', 'LvB']

  def __init__(self, servos, name):
    self._short_names = servos
    super(BaseLvBusIndicator, self).__init__(name)

  def _GatherVoltageData(self, messages):
    """Gather voltage data from the messages."""
    voltages = collections.defaultdict(dict)
    any_value = False
    warning = False
    errors = []

    for servo in self._short_names:
      if 'ServoStatus.Servo' + servo not in messages:
        continue
      any_value = True
      populated = messages[
          'ServoStatus.Servo%s.servo_mon.analog_populated' % servo]
      for voltage_name in self._voltage_names:
        # Guard against bad voltage names.
        if voltage_name not in _SERVO_ANALOG_VOLTAGE_HELPER:
          errors.append('Servo %s: Invalid voltage (%s)' %
                        (servo, voltage_name))
          continue

        index = _SERVO_ANALOG_VOLTAGE_HELPER.Value(voltage_name)
        if not avionics_util.TestMask(populated, index):
          continue

        voltages[voltage_name][servo] = messages[
            'ServoStatus.Servo%s.servo_mon.analog_data[%d]' % (servo, index)]

        warning |= avionics_util.CheckWarning(
            messages['ServoStatus.Servo%s.servo_mon.flags' % servo],
            _SERVO_MON_WARNING_HELPER.Value(voltage_name))

    if errors:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif not any_value:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif warning:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    return voltages, stoplight, errors

  def Filter(self, messages):
    if not messages:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE

    voltages, stoplight, errors = self._GatherVoltageData(messages)

    results = ['    '  + ' '.join(v.rjust(4) for v in self._voltage_names)]
    for servo in self._short_names:
      servo_text = '%s:' % servo
      for voltage_name in self._voltage_names:
        if voltage_name in voltages and servo in voltages[voltage_name]:
          servo_text += ' %5.1f' % voltages[voltage_name][servo]
        else:
          servo_text += ' --'.rjust(6)
      results.append(servo_text)
    return '\n'.join(errors + results), stoplight


class ArmedTailIndicator(BaseArmedIndicator):

  def __init__(self, mode):
    super(ArmedTailIndicator, self).__init__(
        mode, 'Tail Armed', 0, ['E1', 'E2', 'R1', 'R2'])


class ArmedPortIndicator(BaseArmedIndicator):

  def __init__(self, mode):
    super(ArmedPortIndicator, self).__init__(
        mode, 'Port Armed', 0, ['A1', 'A2', 'A4'])


class ArmedStarboardIndicator(BaseArmedIndicator):

  def __init__(self, mode):
    super(ArmedStarboardIndicator, self).__init__(
        mode, 'Starboard Armed', 0, ['A5', 'A7', 'A8'])


class R22TemperatureTailIndicator(BaseR22TemperatureIndicator):

  def __init__(self, mode):
    super(R22TemperatureTailIndicator, self).__init__(
        mode, 'Tail R22 Temp', 0, ['E1', 'E2', 'R1', 'R2'])


class R22TemperaturePortIndicator(BaseR22TemperatureIndicator):

  def __init__(self, mode):
    super(R22TemperaturePortIndicator, self).__init__(
        mode, 'Port R22 Temp', 0, ['A1', 'A2', 'A4'])


class R22TemperatureStarboardIndicator(BaseR22TemperatureIndicator):

  def __init__(self, mode):
    super(R22TemperatureStarboardIndicator, self).__init__(
        mode, 'Star R22 Temp', 0, ['A5', 'A7', 'A8'])


class LvBusTailIndicator(BaseLvBusIndicator):

  def __init__(self):
    super(LvBusTailIndicator, self).__init__(
        ['E1', 'E2', 'R1', 'R2'], 'Tail Bus [V]')


class LvBusPortIndicator(BaseLvBusIndicator):

  def __init__(self):
    super(LvBusPortIndicator, self).__init__(
        ['A1', 'A2', 'A4'], 'Port Bus [V]')


class LvBusStarboardIndicator(BaseLvBusIndicator):

  def __init__(self):
    super(LvBusStarboardIndicator, self).__init__(
        ['A5', 'A7', 'A8'], 'Starboard Bus [V]')


class BasePosChart(avionics.ActuatorCmdDictChart):
  """The indicator to show servo position angles."""

  def __init__(self, mode, name, servo_labels, show_cmd=True, **base_kwargs):
    super(BasePosChart, self).__init__(
        mode, name, servo_labels, 'Servo',
        _SERVO_LABELS_HELPER, common.MAX_NO_UPDATE_COUNT_SERVO_STATUS,
        show_cmd=show_cmd, full_comms_message_type='ServoStatus',
        tether_attribute='servo_statuses', precision=0, **base_kwargs)

  def _GetValuePerNode(self, arg_idx, *args):
    if self._mode == common.FULL_COMMS_MODE:
      return (numpy.rad2deg(args[arg_idx].angle_estimate)
              if struct_tree.IsValidElement(args[arg_idx]) else None)
    elif self._mode == common.SPARSE_COMMS_MODE:
      rad = self._GetTetherValue(args[0], self._node_labels[arg_idx], 'angle')
      return numpy.rad2deg(rad) if rad is not None else None
    else:
      assert False

  def _GetCmdValue(self, servo, controller_command):
    servo_idx = _SERVO_LABELS_HELPER.Value(servo)
    return numpy.rad2deg(controller_command.servo_angle[servo_idx])


class RudPosChart(BasePosChart):

  def __init__(self, mode, **widget_kwargs):
    nodes = ['R1', 'R2']
    super(RudPosChart, self).__init__(
        mode, 'Rud Pos [&deg;]', nodes, show_cmd=True, **widget_kwargs)

    limits = flap_limits.FlapsToServos(
        flap_limits.GetControlCrosswindLimits())['R1']
    limits = numpy.rad2deg(limits).tolist()
    self._SetLimits({
        self._ObservationLabel(n): (
            check_range.Interval(limits, inclusiveness=(False, False)),
            check_range.AllInclusiveRange())
        for n in nodes
    }, [control_types.kFlightModeCrosswindNormal,
        control_types.kFlightModeCrosswindPrepTransOut])


class ElePosChart(BasePosChart):

  def __init__(self, mode, **widget_kwargs):
    nodes = ['E1', 'E2']
    super(ElePosChart, self).__init__(
        mode, 'Ele Pos [&deg;]', nodes, show_cmd=True, **widget_kwargs)

    limits = flap_limits.FlapsToServos(
        flap_limits.GetControlCrosswindLimits())['E1']
    limits = numpy.rad2deg(limits).tolist()
    self._SetLimits({
        self._ObservationLabel(n): (
            check_range.Interval(limits, inclusiveness=(False, False)),
            check_range.AllInclusiveRange())
        for n in nodes
    }, [control_types.kFlightModeCrosswindNormal,
        control_types.kFlightModeCrosswindPrepTransOut])


class PortPosChart(BasePosChart):

  def __init__(self, mode, **widget_kwargs):
    super(PortPosChart, self).__init__(
        mode, 'Port Ail Pos [&deg;]', ['A1', 'A2', 'A4'], show_cmd=True,
        **widget_kwargs)

    self._SetLimits({
        self._ObservationLabel(n): (
            check_range.Interval(
                numpy.rad2deg(flap_limits.FlapsToServos(
                    flap_limits.GetControlCrosswindLimits())[n]).tolist(),
                inclusiveness=(False, False)),
            check_range.AllInclusiveRange())
        for n in ['A1', 'A2']
    }, [control_types.kFlightModeCrosswindNormal,
        control_types.kFlightModeCrosswindPrepTransOut])


class StarboardPosChart(BasePosChart):

  def __init__(self, mode, **widget_kwargs):
    super(StarboardPosChart, self).__init__(
        mode, 'Star Ail Pos [&deg;]', ['A5', 'A7', 'A8'], show_cmd=True,
        **widget_kwargs)

    self._SetLimits({
        self._ObservationLabel(n): (
            check_range.Interval(
                numpy.rad2deg(flap_limits.FlapsToServos(
                    flap_limits.GetControlCrosswindLimits())[n]).tolist(),
                inclusiveness=(False, False)),
            check_range.AllInclusiveRange())
        for n in ['A7', 'A8']
    }, [control_types.kFlightModeCrosswindNormal,
        control_types.kFlightModeCrosswindPrepTransOut])


class LvBusSummaryIndicator(BaseLvBusIndicator):
  """The summary class for low voltage bus indicators."""

  _voltage_names = ['LvA', 'LvB']

  def __init__(self):
    super(LvBusSummaryIndicator, self).__init__(
        _SERVO_LABELS_HELPER.ShortNames(), 'Servo LV Bus [V]')

  def Filter(self, messages):
    if not messages:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE

    all_voltages, stoplight, errors = self._GatherVoltageData(messages)

    all_stats = {}
    for voltage_name in self._voltage_names:
      voltages = all_voltages[voltage_name]
      sorted_pairs = sorted(voltages.items(), key=operator.itemgetter(1))
      num_units = len(voltages)

      all_stats[voltage_name] = {
          'min': sorted_pairs[0] if voltages else None,
          'max': sorted_pairs[-1] if voltages else None,
          'median': sorted_pairs[num_units / 2] if voltages else None,
      }

    delimiter = ' '
    results = [' '.rjust(7)  + delimiter +
               delimiter.join(v.rjust(8) for v in self._voltage_names)]
    for metric in ['min', 'max', 'median']:
      text = metric.rjust(7)
      for voltage_name in self._voltage_names:
        stats = all_stats[voltage_name]
        text += delimiter
        if stats[metric] is not None:
          if isinstance(stats[metric], tuple):
            text += '{: 2.1f}({:2})'.format(
                stats[metric][1], stats[metric][0])
          else:
            text += '{: 7.1f}'.format(stats[metric])
        else:
          text += '--'.rjust(8)
      results.append(text)
    return '\n'.join(errors + results), stoplight


class StatusIndicator(BaseServoIndicator):
  """Summary servo status."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, **format_kwargs):
    super(StatusIndicator, self).__init__(mode, 'Servo Status', 0)
    self._format_kwargs = format_kwargs

  def _GetSingleValue(self, arg_idx, *args):
    if self._mode == common.FULL_COMMS_MODE:
      if struct_tree.IsValidElement(args[arg_idx]):
        return [args[arg_idx].flags, args[arg_idx].servo_mon.flags]
      else:
        return None
    elif self._mode == common.SPARSE_COMMS_MODE:
      return self._GetTetherValue(
          args[0], self._node_labels[arg_idx], 'state')
    else:
      assert False

  @indicator.ReturnIfInputInvalid('', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *attributes):
    any_warning_or_error = False
    warnings = collections.defaultdict(list)
    errors = collections.defaultdict(list)
    report_by_servo = collections.defaultdict(list)
    any_servo = False

    reports = self._GetAvailableValues(*attributes)

    for servo in _SERVO_LABELS_HELPER.ShortNames():
      if servo not in reports or reports[servo] is None:
        continue
      if self._mode == common.FULL_COMMS_MODE:
        flags, mon_flags = reports[servo]
        any_servo = True
        if common.CheckFlags(servo, report_by_servo, warnings, errors, flags,
                             _SERVO_WARNING_HELPER, _SERVO_ERROR_HELPER):
          any_warning_or_error = True

        if common.CheckFlags(
            servo, report_by_servo, warnings, errors, mon_flags,
            _SERVO_MON_WARNING_HELPER, _SERVO_MON_ERROR_HELPER):
          any_warning_or_error = True
      elif self._mode == common.SPARSE_COMMS_MODE:
        any_servo = True
        if reports[servo] & _ACTUATOR_STATE_HELPER.Value('Error'):
          any_warning_or_error = True
          report_by_servo[servo].append(('ERROR', 'status'))
          errors['status'].append(servo)

    return common.SummarizeWarningsAndErrors(
        any_servo, report_by_servo, warnings, errors, any_warning_or_error,
        **self._format_kwargs)

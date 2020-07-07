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

"""Indicators for motors."""

import collections
from datetime import datetime
import math
import string

from makani.analysis.checks import check_range
from makani.avionics.common import motor_thermal_types
from makani.avionics.common import pack_avionics_messages
from makani.avionics.motor.firmware import flags
from makani.avionics.network import aio_labels
from makani.gs.monitor import monitor_params
from makani.gs.monitor2.apps.layout import checks
from makani.gs.monitor2.apps.layout import indicator as base
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import avionics
from makani.lib.python import c_helpers
from makani.lib.python import struct_tree

import numpy

_ACTUATOR_STATE_HELPER = c_helpers.EnumHelper(
    'ActuatorState', pack_avionics_messages, exclude='ActuatorStateCommand')
_TETHER_MOTOR_CONTROLLER_TEMP_HELPER = c_helpers.EnumHelper(
    'TetherMotorControllerTemp', pack_avionics_messages)
_TETHER_MOTOR_TEMP_HELPER = c_helpers.EnumHelper(
    'TetherMotorTemp', pack_avionics_messages)

_MONITOR_PARAMS = monitor_params.GetMonitorParams().contents

# Minimum bus voltage for enabling common mode and chassis voltage checks.
_MOTOR_ISO_ENABLE_THRESHOLD = 250.0
_MOTOR_STATUS_HELPER = c_helpers.EnumHelper('MotorStatus', flags)
_MOTOR_WARNING_HELPER = c_helpers.EnumHelper('MotorWarning', flags)
_MOTOR_ERROR_HELPER = c_helpers.EnumHelper('MotorError', flags)
_MOTOR_LABELS_HELPER = c_helpers.EnumHelper('MotorLabel', aio_labels,
                                            prefix='kMotor')
_THERMAL_CHANNEL_HELPER = c_helpers.EnumHelper('MotorThermalChannel',
                                               motor_thermal_types)

# Pairs of stacked motors, in the ascending order by chassis voltage.
_MOTOR_STACKS = [
    ['Sbo', 'Pto'],
    ['Pbo', 'Sto'],
    ['Sbi', 'Pti'],
    ['Pbi', 'Sti'],
]


class BaseMotorIndicator(avionics.BaseActuatorIndicator):
  """Base class with utilities shared by motor indicators."""

  def __init__(self, mode, label, precision,
               motor_labels=_MOTOR_LABELS_HELPER.ShortNames(),
               aio_node_prefix='Motor',
               full_comms_message_type='MotorStatus',
               show_label=True):
    super(BaseMotorIndicator, self).__init__(
        mode, label, precision, motor_labels, aio_node_prefix,
        _MOTOR_LABELS_HELPER, common.MAX_NO_UPDATE_COUNT_MOTOR_STATUS,
        full_comms_message_type=full_comms_message_type,
        tether_attribute='motor_statuses', show_label=show_label)


class AioUpdateIndicator(BaseMotorIndicator):
  """The indicator to show motors' AIO update status."""

  def __init__(self, mode, **kwargs):
    super(AioUpdateIndicator, self).__init__(mode, 'AIO Update', 0, **kwargs)

  def _IsTetherValid(self, motor_status, motor):
    no_update_count = motor_status[
        '[%d].no_update_count' % self._node_label_helper.Value(motor)]
    return no_update_count <= self._max_no_update_count

  def _Filter(self, *args):
    """Get the AIO update information of all motors."""
    if self._mode == common.FULL_COMMS_MODE:
      motor_updated = {motor for idx, motor in enumerate(self._node_labels)
                       if struct_tree.IsValidElement(args[idx])}
    elif self._mode == common.SPARSE_COMMS_MODE:
      # TODO: Use the no_update_count.
      # Check the TetherDown valid bit.
      if args[1]:
        motor_updated = {
            motor for motor in self._node_labels
            if self._IsTetherValid(args[0], motor)}
      else:
        motor_updated = set()
    else:
      assert False

    # {MotorName: True if the motor is updated.}
    aio_update = {key: 1 if key in motor_updated else 0
                  for key in self._node_labels}

    aio_stoplight = checks.CheckForSize(
        motor_updated, len(self._node_labels),
        equal_flag=stoplights.STOPLIGHT_NORMAL,
        unequal_flag=stoplights.STOPLIGHT_ERROR,
        unexpectedly_empty_flag=stoplights.STOPLIGHT_ERROR)

    return self._DictToString(aio_update), aio_stoplight


class MotorBusVoltageIndicator(BaseMotorIndicator):
  """The indicator to show motors' bus voltages."""

  _NORMAL_RANGES = check_range.Interval([
      _MONITOR_PARAMS.power.v_bus.low,
      _MONITOR_PARAMS.power.v_bus.high,
  ])
  _WARNING_RANGES = check_range.Interval([
      _MONITOR_PARAMS.power.v_bus.very_low,
      _MONITOR_PARAMS.power.v_bus.very_high,
  ])

  def __init__(self, mode, **kwargs):
    super(MotorBusVoltageIndicator, self).__init__(
        mode, 'Bus Voltage [V]', 0, **kwargs)

  def _GetSingleValue(self, node_idx, *attributes):
    if self._mode == common.FULL_COMMS_MODE:
      # Check whether message is available.
      if struct_tree.IsValidElement(attributes[node_idx]):
        return attributes[node_idx].bus_voltage
      else:
        return None
    elif self._mode == common.SPARSE_COMMS_MODE:
      return self._GetTetherValue(
          attributes[0], self._node_labels[node_idx], 'bus_voltage')
    else:
      assert False

  @base.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    """Get the bus voltages of all motors."""
    bus_voltage = self._GetAvailableValues(*args)
    stoplight = stoplights.STOPLIGHT_NORMAL
    for value in bus_voltage.values():
      stoplight = stoplights.MostSevereStoplight(
          stoplights.SetByRanges(
              value, self._NORMAL_RANGES, self._WARNING_RANGES),
          stoplight)
    return self._DictToString(bus_voltage), stoplight


class MotorFlagNameIndicator(BaseMotorIndicator):
  """Indicator to print enum names of red/yellow status flags present."""

  def __init__(self, mode, flag_type, **kwargs):
    self._flag_type = flag_type
    if flag_type == 'Warnings':
      self._enum_helper = _MOTOR_WARNING_HELPER
      self._name = 'Motor warnings'
      self._flags_active_stoplight = stoplights.STOPLIGHT_WARNING
    elif flag_type == 'Errors':
      self._enum_helper = _MOTOR_ERROR_HELPER
      self._name = 'Motor errors'
      self._flags_active_stoplight = stoplights.STOPLIGHT_ERROR
    else:
      assert False

    super(MotorFlagNameIndicator, self).__init__(
        mode, self._name, 0, **kwargs)

  def _GetFlagNames(self, current_flag):
    """Return enum names of any red/yellow status flags present."""
    flags_present = []
    for flag_name in self._enum_helper.Names():
      if current_flag & self._enum_helper.Value(flag_name):
        flags_present.append(flag_name)
    return flags_present

  def _GetSingleValue(self, node_idx, *attributes):
    """Get status flag from MotorStatusMessage for a given node ID."""
    if self._mode == common.FULL_COMMS_MODE:
      # Check whether message is available.
      if struct_tree.IsValidElement(attributes[node_idx]):
        if self._flag_type == 'Warnings':
          return attributes[node_idx].motor_warning
        elif self._flag_type == 'Errors':
          return attributes[node_idx].motor_error
        else:
          assert False  # Only allowed types are Warnings and Errors.
      else:
        return None  # No messages available.
    elif self._mode == common.SPARSE_COMMS_MODE:
      # TetherDown doesn't keep info about which flags are present.
      return None
    else:
      assert False

  @base.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    """Get the status flag of all motors."""
    current_flags = self._GetAvailableValues(*args)
    active_flag_names = {}
    stoplight = stoplights.STOPLIGHT_NORMAL
    for motor in current_flags:
      active_flag_names[motor] = self._GetFlagNames(current_flags[motor])
      if active_flag_names[motor]:  # Status flags activated.
        stoplight = self._flags_active_stoplight

    return '\n'.join([motor + ': ' + ', '.join(active_flag_names[motor])
                      for motor in active_flag_names]), stoplight


class MotorVoltageIndicator(BaseMotorIndicator):
  """The indicator to show motors' chassis isolation status."""

  def __init__(self, chars_per_line, **kwargs):
    super(MotorVoltageIndicator, self).__init__(
        common.FULL_COMMS_MODE, 'Isolation [V]', 1, **kwargs)
    self._chars_per_line = chars_per_line

  def _GetSingleValue(self, node_idx, *attributes):
    assert self._mode == common.FULL_COMMS_MODE
    # Check whether message is available.
    if struct_tree.IsValidElement(attributes[node_idx]):
      return {
          'cm': attributes[node_idx].cm_voltage,
          'bus': attributes[node_idx].bus_voltage,
          'chassis': attributes[node_idx].chassis_voltage,
      }
    else:
      return None

  def _TargetChassisVoltage(self, stack_pair_idx, total_tether_voltage):
    baseline_voltage = -total_tether_voltage * 0.375
    return baseline_voltage + total_tether_voltage / 4.0 * stack_pair_idx

  @base.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    info = self._GetAvailableValues(*args)
    cm_voltage_warning_motors = []
    cm_voltage_error_motors = []
    chassis_voltage_warning_motors = []
    chassis_voltage_error_motors = []
    num_levels_online = 0
    total_stack_voltage = 0.0
    bus_voltage_max = 0.0

    for pair in _MOTOR_STACKS:
      # Number of peer motors available in a stack pair.
      num_peers_online = 0
      level_voltage_sum = 0.0
      # We compute total stack voltage by adding up average voltages per pair.
      for key in pair:
        if key in info:
          num_peers_online += 1
          voltages = info[key]
          cm_voltage = voltages['cm']
          bus_voltage = voltages['bus']

          if bus_voltage > bus_voltage_max:
            bus_voltage_max = bus_voltage

          level_voltage_sum += bus_voltage
          state = common.IsInTargetRange(bus_voltage * 0.5, bus_voltage * 0.5,
                                         0.25, 0.5, cm_voltage)
          if state == 1:  # Warning
            cm_voltage_warning_motors.append(key)
          elif state == 2:  # Error
            cm_voltage_error_motors.append(key)

      if num_peers_online:
        num_levels_online += 1
        total_stack_voltage += level_voltage_sum / float(num_peers_online)

    total_stack_voltage *= 4.0 / float(num_levels_online)

    if bus_voltage_max < _MOTOR_ISO_ENABLE_THRESHOLD:
      return 'Bus voltage too low', stoplights.STOPLIGHT_WARNING

    for idx, pair in enumerate(_MOTOR_STACKS):
      for motor in pair:
        if motor in info:
          state = common.IsInTargetRange(
              self._TargetChassisVoltage(idx, total_stack_voltage),
              info[motor]['bus'] * 0.5, 0.25, 0.5, info[motor]['chassis'])
          if state == 1:  # Warning
            chassis_voltage_warning_motors.append(motor)
          elif state == 2:  # Error
            chassis_voltage_error_motors.append(motor)

    errors = []
    for motor in cm_voltage_error_motors:
      errors.append('%s (CM)' % motor)
    for motor in chassis_voltage_error_motors:
      errors.append('%s (Chassis)' % motor)

    if errors:
      stoplight = stoplights.STOPLIGHT_ERROR
      text = common.TimeCycle(errors, self._chars_per_line)
    else:
      warnings = []
      for motor in cm_voltage_warning_motors:
        warnings.append('%s (CM)' % motor)
      for motor in chassis_voltage_warning_motors:
        warnings.append('%s (Chassis)' % motor)

      if warnings:
        stoplight = stoplights.STOPLIGHT_WARNING
        text = common.TimeCycle(warnings, self._chars_per_line)
      else:
        stoplight = stoplights.STOPLIGHT_NORMAL
        text = 'Normal'
    return text, stoplight


class MotorLVInputIndicator(BaseMotorIndicator):
  """The indicator to show motors' LV-12V input status."""

  _PRI_NORMAL_RANGES = check_range.Interval([
      _MONITOR_PARAMS.power.v_12v_pri.low,
      _MONITOR_PARAMS.power.v_12v_pri.high,
  ])
  _PRI_WARNING_RANGES = check_range.Interval([
      _MONITOR_PARAMS.power.v_12v_pri.very_low,
      _MONITOR_PARAMS.power.v_12v_pri.very_high,
  ])
  _AUX_NORMAL_RANGES = check_range.Interval([
      _MONITOR_PARAMS.power.v_12v_aux.low,
      _MONITOR_PARAMS.power.v_12v_aux.high,
  ])
  _AUX_WARNING_RANGES = check_range.Interval([
      _MONITOR_PARAMS.power.v_12v_aux.very_low,
      _MONITOR_PARAMS.power.v_12v_aux.very_high,
  ])

  def __init__(self, chars_per_line, **kwargs):
    super(MotorLVInputIndicator, self).__init__(
        common.FULL_COMMS_MODE, 'Motor LV-12V', 1, **kwargs)
    self._chars_per_line = chars_per_line

  def _GetSingleValue(self, node_idx, *attributes):
    assert self._mode == common.FULL_COMMS_MODE
    # Check whether message is available.
    if struct_tree.IsValidElement(attributes[node_idx]):
      return {
          'primary': attributes[node_idx].v_supply_primary,
          'auxiliary': attributes[node_idx].v_supply_auxiliary,
          'diode_ored': attributes[node_idx].motor_mon.ina219_data[0].voltage,
      }
    else:
      return None

  @base.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    lv12v = self._GetAvailableValues(*args)
    primary_failed = []
    primary_warning = []
    auxiliary_failed = []
    auxiliary_warning = []
    pri_aux_swapped = []
    pri_aux_warning = []
    diode_open = []

    for motor, value in lv12v.iteritems():
      primary = value['primary']
      auxiliary = value['auxiliary']
      diode_ored = value['diode_ored']

      state = stoplights.SetByRanges(
          primary, self._PRI_NORMAL_RANGES, self._PRI_WARNING_RANGES)
      if state == stoplights.STOPLIGHT_WARNING:
        primary_warning.append(motor)
      elif state == stoplights.STOPLIGHT_ERROR:
        primary_failed.append(motor)

      state = stoplights.SetByRanges(
          auxiliary, self._AUX_NORMAL_RANGES, self._AUX_WARNING_RANGES)
      if state == stoplights.STOPLIGHT_WARNING:
        auxiliary_warning.append(motor)
      elif state == stoplights.STOPLIGHT_ERROR:
        auxiliary_failed.append(motor)

      # Verify primary voltage (12.6V) is greater than auxiliary (12.0V).
      voltage_delta = primary - auxiliary
      if voltage_delta > 0.3:
        # It's possible to check that the primary 12V OR'ing diode has not
        # failed open.
        if primary - diode_ored > 0.75:
          diode_open.append(motor)
      elif voltage_delta > -0.3:
        # Small primary-auxiliary voltage deltas can indicate various problems:
        # wrong voltage setpoints, short between pri-aux, shorted diode, etc.
        pri_aux_warning.append(motor)
      else:
        pri_aux_swapped.append(motor)
        # It's possible to check that the auxiliary 12V OR'ing diode has not
        # failed open.
        if auxiliary - diode_ored > 0.75:
          diode_open.append(motor)

    errors = []
    for motor in primary_failed:
      errors.append(
          '%s (12V_A: %5.2fV)' % (motor, lv12v[motor]['primary']))
    for motor in auxiliary_failed:
      errors.append(
          '%s (12V_B: %5.2fV)' % (motor, lv12v[motor]['auxiliary']))

    if errors:
      stoplight = stoplights.STOPLIGHT_ERROR
      text = common.TimeCycle(errors, self._chars_per_line)
    else:
      warnings = []
      for motor in primary_warning:
        warnings.append(
            '%s (12V_A: %5.2fV)' % (motor, lv12v[motor]['primary']))
      for motor in auxiliary_warning:
        warnings.append(
            '%s (12V_B: %5.2fV)' % (motor, lv12v[motor]['auxiliary']))
      for motor in pri_aux_warning:
        warnings.append(
            '%s (A(%5.2fV) ~ B(%5.2fV))' %
            (motor, lv12v[motor]['primary'], lv12v[motor]['auxiliary']))
      for motor in pri_aux_swapped:
        warnings.append(
            '%s (A(%5.2fV) < B(%5.2fV))' %
            (motor, lv12v[motor]['primary'], lv12v[motor]['auxiliary']))
      for motor in diode_open:
        warnings.append('%s (OR\'ing diode failed open)' % motor)

      if warnings:
        stoplight = stoplights.STOPLIGHT_WARNING
        text = common.TimeCycle(warnings, self._chars_per_line)
      else:
        stoplight = stoplights.STOPLIGHT_NORMAL
        text = 'Normal'
    return text, stoplight


class ArmedIndicator(BaseMotorIndicator):
  """The indicator to show motors' armed status."""

  def __init__(self, mode, **kwargs):
    super(ArmedIndicator, self).__init__(mode, 'Armed', 0, **kwargs)

  def _GetSingleValue(self, node_idx, *attributes):
    if self._mode == common.FULL_COMMS_MODE:
      if struct_tree.IsValidElement(attributes[node_idx]):
        return attributes[node_idx].motor_status
      else:
        return None
    elif self._mode == common.SPARSE_COMMS_MODE:
      return self._GetTetherValue(
          attributes[0], self._node_labels[node_idx], 'state')
    else:
      assert False

  @base.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    """Get the armed information of all motors."""
    motor_status = self._GetAvailableValues(*args)

    if self._mode == common.FULL_COMMS_MODE:
      status_helper = _MOTOR_STATUS_HELPER
    elif self._mode == common.SPARSE_COMMS_MODE:
      status_helper = _ACTUATOR_STATE_HELPER
    else:
      assert False

    return self._CheckStatusFlags(
        motor_status, status_helper, ['Armed', 'Running'],
        stoplights.STOPLIGHT_ERROR)


class BaseFlagIndicator(BaseMotorIndicator):
  """The indicator to show motor status."""

  def __init__(self, mode, name, full_comms_field, sparse_comms_field,
               **kwargs):
    super(BaseFlagIndicator, self).__init__(mode, name, 0, **kwargs)
    self._full_comms_field = full_comms_field
    self._sparse_comms_field = sparse_comms_field

  @base.ReturnIfInputInvalid('', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    """Get the error information of all motors."""

    active_flags = self._GetAvailableValues(*args)

    if active_flags:
      flagged_motors = {motor for motor, value in active_flags.iteritems()
                        if value}
      # Normal if there is no motor error.
      stoplight = checks.CheckForSize(
          flagged_motors, 0,
          equal_flag=stoplights.STOPLIGHT_NORMAL,
          unequal_flag=stoplights.STOPLIGHT_WARNING)
    else:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE

    return self._DictToString(active_flags), stoplight

  def _GetSingleValue(self, node_idx, *attributes):
    if self._mode == common.FULL_COMMS_MODE:
      if struct_tree.IsValidElement(attributes[node_idx]):
        return self._GetCTypeFieldByString(
            attributes[node_idx], self._full_comms_field) != 0
      else:
        return None
    elif self._mode == common.SPARSE_COMMS_MODE:
      aio_label = self._node_labels[node_idx]
      if not self._IsTetherUpdated(attributes[0], aio_label):
        return None
      error = getattr(attributes[0][self._node_label_helper.Value(aio_label)],
                      self._sparse_comms_field)
      return error
    else:
      assert False


class ErrorIndicator(BaseFlagIndicator):
  """The indicator to show motor errors."""

  def __init__(self, mode, **kwargs):
    super(ErrorIndicator, self).__init__(
        mode, 'Motor Errors', 'motor_error', 'error', **kwargs)


class WarningIndicator(BaseFlagIndicator):
  """The indicator to show motor errors."""

  def __init__(self, mode, **kwargs):
    super(WarningIndicator, self).__init__(
        mode, 'Motor Warnings', 'motor_warning', 'warning', **kwargs)


class BaseTemperatureIndicator(BaseMotorIndicator):
  """The indicator to show motor temperatures."""

  # A temporary fix to detect whether values are stuck for a long period,
  # indicating a bad reading.
  _MAX_VALUE_STUCK_SECONDS = 10
  _MAX_STUCKABLE_TEMPERATURE = 5.0

  def __init__(self, mode, name, normal_ranges, warning_ranges, error_ranges,
               full_thermal_types, sparse_thermal_types, is_controller,
               reducer=max, **kwargs):
    self._use_median = False
    super(BaseTemperatureIndicator, self).__init__(mode, name, 1, **kwargs)
    self._normal_ranges = (None if normal_ranges is None
                           else check_range.BuildRanges(normal_ranges))
    self._warning_ranges = (None if warning_ranges is None
                            else check_range.BuildRanges(warning_ranges))
    self._error_ranges = (None if error_ranges is None
                          else check_range.BuildRanges(error_ranges))
    self._full_thermal_types = full_thermal_types
    self._sparse_thermal_types = sparse_thermal_types
    self._is_controller = is_controller
    self._last_values = collections.defaultdict(lambda: None)
    self._last_update = collections.defaultdict(lambda: None)
    self._reducer = reducer

  def _GetState(self, status, aio_label):
    if status is None:
      return None
    node_id = self._node_label_helper.Value(aio_label)
    node_status = status[node_id]
    no_update_count = node_status.no_update_count
    return node_status if no_update_count <= self._max_no_update_count else None

  def _GetSingleValue(self, node_idx, *attributes):

    def GetChannelTemperature(motor_state):
      if motor_state is not None:
        return motor_state.controller_temps[
            _TETHER_MOTOR_CONTROLLER_TEMP_HELPER.Value(channel)]
      else:
        return None

    if self._mode == common.FULL_COMMS_MODE:
      if struct_tree.IsValidElement(attributes[node_idx]):
        values = []
        temps = attributes[node_idx].temps
        for ch in self._full_thermal_types:
          values.append(temps[_THERMAL_CHANNEL_HELPER.Value(ch)])
        return None if not values else self._reducer(values)
      else:
        return None
    elif self._mode == common.SPARSE_COMMS_MODE:
      aio_label = self._node_labels[node_idx]
      if self._is_controller:
        values = [
            GetChannelTemperature(self._GetState(attributes[0], aio_label))
            for channel in self._sparse_thermal_types]
        return self._reducer(values)
      else:
        motor_temps = self._GetTetherValue(
            attributes[0], aio_label, 'motor_temps')
        if motor_temps:
          values = [motor_temps[_TETHER_MOTOR_TEMP_HELPER.Value(channel)]
                    for channel in self._sparse_thermal_types]
          return self._reducer(values)
        else:
          return None
    else:
      assert False

  @base.ReturnIfInputInvalid('', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    temperatures, stoplight = self._GetFieldInfo(
        self._normal_ranges, self._warning_ranges, self._error_ranges, *args)

    is_value_stuck = False
    current_time = datetime.now()
    is_over_temp = False

    for node_idx, motor in enumerate(self._node_labels):
      if motor not in temperatures:
        continue
      temperature = temperatures[motor]
      if temperature != self._last_values[motor]:
        self._last_values[motor] = temperature
        self._last_update[motor] = current_time
      else:
        if self._mode == common.SPARSE_COMMS_MODE:
          enable_stuck_value_check = (
              temperature <= self._MAX_STUCKABLE_TEMPERATURE)
        elif self._mode == common.FULL_COMMS_MODE:
          warning = args[node_idx].motor_warning
          enable_stuck_value_check = (
              warning & _MOTOR_WARNING_HELPER.Value('TempReadErrors'))
          if not is_over_temp:
            for ch in self._full_thermal_types:
              if warning & _MOTOR_WARNING_HELPER.Value('OverTemp' + ch):
                is_over_temp = True
                stoplight = stoplights.MostSevereStoplight(
                    stoplight, stoplights.STOPLIGHT_WARNING)
                break
        else:
          assert False

        if (enable_stuck_value_check and
            (current_time - self._last_update[motor]).total_seconds()
            > self._MAX_VALUE_STUCK_SECONDS):
          is_value_stuck = True

    output = self._DictToString(temperatures)
    if is_value_stuck:
      output += ' [Stuck]'
      stoplight = stoplights.MostSevereStoplight(
          stoplight, stoplights.STOPLIGHT_WARNING)
    return output, stoplight


class BoardTemperatureIndicator(BaseTemperatureIndicator):
  """The indicator to show motors' board temperatures."""

  def __init__(self, mode, **kwargs):
    limits = _MONITOR_PARAMS.thermal.motor_controller_board
    super(BoardTemperatureIndicator, self).__init__(
        mode, 'Board [&deg;C]',
        [[limits.low, limits.high]],
        [[limits.very_low, limits.very_high]],
        None, ['Board'], ['Board'], True, **kwargs)


class CapacitorTemperatureIndicator(BaseTemperatureIndicator):
  """The indicator to show motors' capacitor temperatures."""

  def __init__(self, mode, **kwargs):
    limits = _MONITOR_PARAMS.thermal.motor_controller_capacitor
    super(CapacitorTemperatureIndicator, self).__init__(
        mode, 'Capacitor [&deg;C]',
        [[limits.low, limits.high]],
        [[limits.very_low, limits.very_high]],
        None, ['Capacitor'], ['Capacitor'], True,
        **kwargs)


class HeatPlateTemperatureIndicator(BaseTemperatureIndicator):
  """The indicator to show motors' heatplate temperatures."""

  def __init__(self, mode, **kwargs):
    limits = _MONITOR_PARAMS.thermal.motor_controller_heat_plate
    super(HeatPlateTemperatureIndicator, self).__init__(
        mode, 'Heat Plates [&deg;C]',
        [[limits.low, limits.high]],
        [[limits.very_low, limits.very_high]],
        None, ['HeatPlate1', 'HeatPlate2'], ['HeatPlate'], True, **kwargs)


class ModuleTemperatureIndicator(BaseTemperatureIndicator):
  """The indicator to show motors' module temperatures."""

  def __init__(self, **kwargs):
    limits = _MONITOR_PARAMS.thermal.motor_controller_module
    super(ModuleTemperatureIndicator, self).__init__(
        common.FULL_COMMS_MODE, 'Module [&deg;C]',
        [[limits.low, limits.high]],
        [[limits.very_low, limits.very_high]],
        None, ['Ht3000A', 'Ht3000B', 'Ht3000C'], [], True,
        reducer=lambda values: numpy.median(numpy.asarray(values)),
        **kwargs)


class RotorTemperatureIndicator(BaseTemperatureIndicator):
  """The indicator to show motors' board temperatures."""

  def __init__(self, mode, **kwargs):
    limits = _MONITOR_PARAMS.thermal.motor_rotor
    super(RotorTemperatureIndicator, self).__init__(
        mode, 'Rotor [&deg;C]',
        [[limits.low, limits.high]],
        [[limits.very_low, limits.very_high]],
        None, ['Rotor'], ['Rotor'], False, **kwargs)


class StatorCoreTemperatureIndicator(BaseTemperatureIndicator):
  """The indicator to show motors' stator core temperatures."""

  def __init__(self, mode, **kwargs):
    limits = _MONITOR_PARAMS.thermal.motor_stator_core
    super(StatorCoreTemperatureIndicator, self).__init__(
        mode, 'Stator Core [&deg;C]',
        [[limits.low, limits.high]],
        [[limits.very_low, limits.very_high]],
        [[None, 1200]],
        ['StatorCore'], ['StatorCore'], False, **kwargs)


class WindingTemperatureIndicator(BaseTemperatureIndicator):
  """The indicator to show motors' winding temperatures."""

  def __init__(self, mode, **kwargs):
    limits = _MONITOR_PARAMS.thermal.motor_stator_winding
    super(WindingTemperatureIndicator, self).__init__(
        mode, 'Winding [&deg;C]',
        [[limits.low, limits.high]],
        [[limits.very_low, limits.very_high]],
        [[None, 1200]],
        ['StatorCoil'], ['StatorCoil'], False, **kwargs)


class SpeedChart(avionics.ActuatorCmdDictChart):
  """The indicator to show motor speeds."""

  def __init__(self, mode, name, motor_labels, show_cmd=False,
               aio_node_prefix='Motor', full_comms_message_type='MotorStatus',
               **widget_kwargs):
    super(SpeedChart, self).__init__(
        mode, name, motor_labels, aio_node_prefix,
        _MOTOR_LABELS_HELPER, common.MAX_NO_UPDATE_COUNT_MOTOR_STATUS,
        full_comms_message_type, show_cmd=show_cmd,
        tether_attribute='motor_statuses', precision=0, **widget_kwargs)

  def _GetStoplight(self, values, any_values, missing_values, flight_mode):
    stoplight = super(SpeedChart, self)._GetStoplight(
        values, any_values, missing_values, flight_mode)

    speeds = [abs(s) for s in values.values() if isinstance(s, float)]
    if not speeds:
      return stoplight

    max_speed = max(speeds)
    speed_limits = self._GetLimits(_MONITOR_PARAMS.rotors.speed)
    stoplight = stoplights.MostSevereStoplight(
        stoplights.SetByRanges(
            max_speed, speed_limits['normal'], speed_limits['warning']),
        stoplight)
    return stoplight

  def _GetValuePerNode(self, idx, *args):
    if self._mode == common.FULL_COMMS_MODE:
      return (args[idx].omega if struct_tree.IsValidElement(args[idx])
              else None)
    elif self._mode == common.SPARSE_COMMS_MODE:
      return self._GetTetherValue(args[0], self._node_labels[idx], 'speed')
    else:
      assert False

  def _GetCmdValue(self, motor, controller_command):
    return controller_command.motor_speed_upper_limit[
        _MOTOR_LABELS_HELPER.Value(motor)]


class BaseStackingChart(avionics.ActuatorDictChart):
  """The indicator to show motor stacking information."""

  def __init__(self, mode, name, motor_labels, aio_node_prefix='Motor',
               full_comms_message_type='MotorStatus',
               tether_attribute='motor_statuses', **widget_kwargs):

    fields, label_templates = self._FieldsAndLabelsByMode(mode)
    full_labels = []
    for short_name in motor_labels:
      for index, field in enumerate(fields):
        if label_templates:
          template = string.Template(label_templates[index])
          full_labels.append(template.substitute({'tag': short_name}))
        else:
          full_labels.append(field)

    self._fields = fields

    super(BaseStackingChart, self).__init__(
        mode, name, motor_labels, full_labels, aio_node_prefix,
        _MOTOR_LABELS_HELPER, common.MAX_NO_UPDATE_COUNT_MOTOR_STATUS,
        full_comms_message_type, tether_attribute, **widget_kwargs)

  def _FieldsAndLabelsByMode(self, mode):
    raise NotImplementedError

  @base.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    timestamps = {}
    values = {}
    has_valid_values = False
    has_invalid_values = False

    label_idx = 0
    for idx in range(len(self._node_labels)):
      for field in self._fields:
        label = self._labels[label_idx]
        timestamps[label] = self._GetTimestamp(idx, *args)
        values[label] = self._GetSingleValue(idx, field, *args)
        label_idx += 1
        if values[label] is None:
          has_invalid_values = True
        else:
          has_valid_values = True

    if not has_valid_values:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif has_invalid_values:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    return timestamps, values, stoplight

  def _GetSingleValue(self, node_idx, field, *attributes):
    if self._mode == common.FULL_COMMS_MODE:
      if struct_tree.IsValidElement(attributes[node_idx]):
        return self._GetCTypeFieldByString(attributes[node_idx], field)
      else:
        return None
    elif self._mode == common.SPARSE_COMMS_MODE:
      return self._GetTetherValue(
          attributes[0], self._node_labels[node_idx], field)
    else:
      assert False


class SpeedStackingChart(BaseStackingChart):
  """The indicator to show speed of stacked motors."""

  def _FieldsAndLabelsByMode(self, mode):
    if mode == common.FULL_COMMS_MODE:
      return (
          ['omega', 'omega_upper_limit', 'omega_lower_limit'],
          ['$tag', '$tag.high', '$tag.low']
      )
    elif mode == common.SPARSE_COMMS_MODE:
      return ['speed'], ['$tag']
    else:
      assert False


class IqStackingChart(BaseStackingChart):
  """The indicator to show Iq of stacked motors."""

  def _FieldsAndLabelsByMode(self, mode):
    if mode == common.FULL_COMMS_MODE:
      return (
          ['iq', 'iq_cmd'],
          ['$tag.iq', '$tag.cmd'],
      )
    elif mode == common.SPARSE_COMMS_MODE:
      return ['iq'], ['$tag']
    else:
      assert False


class IdStackingChart(BaseStackingChart):
  """The indicator to show Id of stacked motors."""

  def _FieldsAndLabelsByMode(self, mode):
    if mode == common.FULL_COMMS_MODE:
      return (
          ['id', 'iq_cmd'],
          ['$tag.id', '$tag.cmd'],
      )
    elif mode == common.SPARSE_COMMS_MODE:
      return ['id'], ['$tag']
    else:
      assert False


class DriveVoltageStackingChart(BaseStackingChart):
  """The indicator to show drive voltages of stacked motors."""

  def _FieldsAndLabelsByMode(self, mode):
    if mode == common.FULL_COMMS_MODE:
      return (
          ['vd', 'vq', 'vref'],
          ['$tag.vd', '$tag.vq', '$tag.ref'],
      )
    else:
      assert False

  def _GetSingleValue(self, node_idx, field, *attributes):
    if field != 'vref':
      return super(DriveVoltageStackingChart, self)._GetSingleValue(
          node_idx, field, *attributes)
    else:
      vd, vq = [
          super(DriveVoltageStackingChart, self)._GetSingleValue(
              node_idx, field, *attributes) for field in ('vd', 'vq')]
      if vd is None or vq is None:
        return None
      else:
        return math.sqrt(vd ** 2.0 + vq ** 2.0)


class MotorBusVoltageStackingChart(BaseStackingChart):
  """The indicator to show bus voltages of stacked motors."""

  def _FieldsAndLabelsByMode(self, mode):
    if mode in [common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE]:
      return ['bus_voltage'], ['$tag']
    else:
      assert False


class MotorBusCurrentStackingChart(BaseStackingChart):
  """The indicator to show bus currents of stacked motors."""

  def _FieldsAndLabelsByMode(self, mode):
    if mode in [common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE]:
      return ['bus_current'], ['$tag']
    else:
      assert False


class TorqueCmdStackingChart(BaseStackingChart):
  """The indicator to show torque commands of stacked motors."""

  def _FieldsAndLabelsByMode(self, mode):
    if mode == common.FULL_COMMS_MODE:
      return ['torque_est', 'torque_cmd'], ['$tag.est', '$tag.cmd']
    else:
      assert False

  def _GetSingleValue(self, node_idx, field, *attributes):
    if field != 'torque_est':
      return super(TorqueCmdStackingChart, self)._GetSingleValue(
          node_idx, field, *attributes)
    else:
      iq = super(TorqueCmdStackingChart, self)._GetSingleValue(
          node_idx, 'iq', *attributes)
      if iq is None:
        return None
      else:
        # This only applies to Protean motors.
        return 3.0  / 2 * 32 * 0.06203 * iq


class StackBusPower(object):
  """A class to compute the stack bus power. Generated power is positive."""

  _V_NORMAL_RANGES = check_range.Interval(
      [_MONITOR_PARAMS.power.v_bus.low * 4,
       _MONITOR_PARAMS.power.v_bus.high * 4])

  _V_WARNING_RANGES = check_range.Interval(
      [_MONITOR_PARAMS.power.v_bus.very_low * 4,
       _MONITOR_PARAMS.power.v_bus.very_high * 4])

  def __init__(self, mode, max_no_update_count):
    self._mode = mode
    self._max_no_update_count = max_no_update_count
    motor_labels = _MOTOR_LABELS_HELPER.ShortNames()
    self._label_index = dict(zip(motor_labels, range(len(motor_labels))))
    self._power_label = 'Power[kW]'
    self._voltage_label = 'Voltage[V]'
    self._current_label = 'Current[A]'

  def _IsTetherValueValid(self, status, tetherdown_valid, aio_label):
    idx = _MOTOR_LABELS_HELPER.Value(aio_label)
    return (tetherdown_valid and
            status[idx].no_update_count <=
            self._max_no_update_count)

  def _IsValueAvailable(self, label, *args):
    if self._mode == common.FULL_COMMS_MODE:
      return struct_tree.IsValidElement(args[self._label_index[label]])
    elif self._mode == common.SPARSE_COMMS_MODE:
      return self._IsTetherValueValid(args[0], args[1], label)
    else:
      assert False

  def _GetField(self, field, label, *args):
    if self._mode == common.FULL_COMMS_MODE:
      return getattr(args[self._label_index[label]], field)
    elif self._mode == common.SPARSE_COMMS_MODE:
      return getattr(args[0][_MOTOR_LABELS_HELPER.Value(label)], field)
    else:
      assert None

  def Filter(self, *args):
    """Compute the stack bus power according to motor status (tether or not)."""
    v_stack = 0.0
    i_stack = 0.0
    num_part_levels = 0
    num_full_levels = 0
    for stack in _MOTOR_STACKS:
      label_a = stack[0]
      label_b = stack[1]
      updated_a = self._IsValueAvailable(label_a, *args)
      updated_b = self._IsValueAvailable(label_b, *args)
      if updated_a and updated_b:
        v_stack += 0.5 * (self._GetField('bus_voltage', label_a, *args) +
                          self._GetField('bus_voltage', label_b, *args))
        i_stack += (self._GetField('bus_current', label_a, *args) +
                    self._GetField('bus_current', label_b, *args))
        num_part_levels += 1
        num_full_levels += 1
      elif updated_a:
        v_stack += self._GetField('bus_voltage', label_a, *args)
        num_part_levels += 1
      elif updated_b:
        v_stack += self._GetField('bus_voltage', label_b, *args)
        num_part_levels += 1

    if num_full_levels > 0:
      i_stack /= num_full_levels
      v_stack *= float(len(_MOTOR_STACKS)) / num_part_levels
      # Negate so that positive means generated power.
      p_stack_kw = - i_stack * v_stack * 0.001
      suggested_stoplight = stoplights.SetByRanges(
          v_stack, self._V_NORMAL_RANGES, self._V_WARNING_RANGES)
      availability_stoplight = (
          stoplights.STOPLIGHT_NORMAL
          if num_full_levels == len(_MOTOR_STACKS)
          else stoplights.STOPLIGHT_WARNING)
      stoplight = stoplights.MostSevereStoplight(
          suggested_stoplight, availability_stoplight)
    else:
      p_stack_kw = '--'
      i_stack = '--'
      v_stack = '--'
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE

    values = {
        self._power_label: p_stack_kw,
        self._voltage_label: v_stack,
        self._current_label: i_stack,
    }

    return values, stoplight

  def PowerLabel(self):
    return self._power_label

  def VoltageLabel(self):
    return self._voltage_label

  def CurrentLabel(self):
    return self._current_label


class StackBusPowerChart(avionics.ActuatorDictChart):
  """The indicator to show bus power."""

  def __init__(self, mode, name, **widget_kwargs):
    self._executor = StackBusPower(
        mode, common.MAX_NO_UPDATE_COUNT_MOTOR_STATUS)
    motor_labels = _MOTOR_LABELS_HELPER.ShortNames()
    self._power_history = []
    self._last_power_per_loop_kw = None
    self._best_power_per_loop_kw = None
    self._total_power_all_loops_kw = 0.0
    self._last_loop_count = -1
    # The starting loop count to measure average power per loop.
    self._base_loop_count = -1

    super(StackBusPowerChart, self).__init__(
        mode, name, motor_labels,
        [self._executor.PowerLabel(), self._executor.VoltageLabel(),
         self._executor.CurrentLabel(), self._LastLoopPowerLabel(),
         self._BestLoopPowerLabel(), self._AvgLoopPowerLabel()], 'Motor',
        _MOTOR_LABELS_HELPER, common.MAX_NO_UPDATE_COUNT_MOTOR_STATUS,
        full_comms_message_type='MotorStatus',
        tether_attribute='motor_statuses', precision=1,
        chart_keys=[self._executor.PowerLabel()], **widget_kwargs)

  def _LastLoopPowerLabel(self):
    return 'Last kW/Loop'

  def _BestLoopPowerLabel(self):
    return 'Best kW/Loop'

  def _AvgLoopPowerLabel(self):
    return 'Avg kW/Loop'

  def _GetMessageAttributes(self):
    attributes = super(StackBusPowerChart, self)._GetMessageAttributes()
    attributes.append(('filtered', 'loop_count', 'total_rev_count'))
    return attributes

  @base.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    values, stoplight = self._executor.Filter(*args)

    new_loop_count = args[-1]

    # Update the records for power per loop.
    # TODO: Ideally this should be done at high frequency filter.
    if new_loop_count != self._last_loop_count:
      self._last_loop_count = new_loop_count
      self._last_power_per_loop_kw = (
          numpy.average(self._power_history) if len(self._power_history)
          else None)
      if self._last_power_per_loop_kw is not None:
        if self._base_loop_count < 0:
          # The starting loop count is the one before the current loop.
          self._base_loop_count = new_loop_count - 1
        self._total_power_all_loops_kw += self._last_power_per_loop_kw
        if self._best_power_per_loop_kw is None:
          self._best_power_per_loop_kw = self._last_power_per_loop_kw
        else:
          self._best_power_per_loop_kw = max(self._best_power_per_loop_kw,
                                             self._last_power_per_loop_kw)
      self._power_history = []

    current_power = values[self._executor.PowerLabel()]
    if isinstance(current_power, float):
      self._power_history.append(current_power)

    values[self._LastLoopPowerLabel()] = (
        self._last_power_per_loop_kw if self._last_power_per_loop_kw is not None
        else '--')
    values[self._BestLoopPowerLabel()] = (
        self._best_power_per_loop_kw if self._best_power_per_loop_kw is not None
        else '--')
    values[self._AvgLoopPowerLabel()] = (
        self._total_power_all_loops_kw /
        (new_loop_count - self._base_loop_count) if self._base_loop_count >= 0
        else '--')

    current_time = (
        (datetime.now() - datetime.utcfromtimestamp(0)).total_seconds())
    timestamps = {
        self._executor.PowerLabel(): current_time,
        self._executor.VoltageLabel(): current_time,
        self._executor.CurrentLabel(): current_time,
        self._LastLoopPowerLabel(): current_time,
        self._BestLoopPowerLabel(): current_time,
        self._AvgLoopPowerLabel(): current_time,
    }

    return timestamps, values, stoplight


class StackBusPowerIndicator(BaseMotorIndicator):
  """The indicator to show motors' bus power."""

  def __init__(self, mode, label, **kwargs):
    super(StackBusPowerIndicator, self).__init__(
        mode, label, 1, show_label=False, **kwargs)
    self._executor = StackBusPower(
        mode, common.MAX_NO_UPDATE_COUNT_MOTOR_STATUS)

  @base.ReturnIfInputInvalid('--\n\n\n\n', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    values, stoplight = self._executor.Filter(*args)
    lines = []
    for key in [self._executor.PowerLabel(), self._executor.VoltageLabel(),
                self._executor.CurrentLabel()]:
      if isinstance(values[key], (str, unicode)):
        value = '{:7}'.format(values[key])
      else:
        value = '{: 7.1f}'.format(values[key])
      lines.append('{:12}: '.format(key) + value)
    return '\n'.join(lines), stoplight

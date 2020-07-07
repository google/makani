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

"""Monitor indicators related to avionics."""
import collections
import copy
import ctypes

from makani.analysis.checks import avionics_util
from makani.avionics.common import motor_thermal_types
from makani.avionics.common import pack_avionics_messages as avionics_messages
from makani.avionics.common import pitot_cover_types
from makani.avionics.firmware.monitors import aio_types
from makani.avionics.firmware.monitors import cs_types
from makani.avionics.firmware.monitors import loadcell_types
from makani.avionics.network import aio_labels
from makani.avionics.network import network_config
from makani.control import sensor_util
from makani.control import system_params
from makani.gs.monitor import monitor_params
from makani.gs.monitor2.apps.layout import checks
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.layout import widgets
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.project import settings
from makani.lib.python import c_helpers
from makani.lib.python import ctype_util
from makani.lib.python import struct_tree
import numpy as np


_AIO_SI7021_HELPER = c_helpers.EnumHelper('AioSi7021Monitor', aio_types)
_SERVO_LABEL_HELPER = c_helpers.EnumHelper('ServoLabel', aio_labels,
                                           prefix='kServo')
_CS_LABEL_HELPER = c_helpers.EnumHelper('CoreSwitchLabel', aio_labels,
                                        prefix='kCoreSwitch')
_CS_SI7021_HELPER = c_helpers.EnumHelper('CsSi7021Monitor', cs_types)
_FLIGHT_COMPUTER_HELPER = c_helpers.EnumHelper('FlightComputer', aio_labels)
_BRIDLE_JUNC_WARNING_HELPER = c_helpers.EnumHelper('BridleJuncWarning',
                                                   loadcell_types)
_MOTOR_LABELS_HELPER = c_helpers.EnumHelper('MotorLabel', aio_labels,
                                            prefix='kMotor')
_PITOT_COVER_STATUS_HELPER = c_helpers.EnumHelper('PitotCoverStatus',
                                                  pitot_cover_types)
_MONITOR_PARAMS = monitor_params.GetMonitorParams().contents
_SYSTEM_PARAMS = system_params.GetSystemParams().contents

_MON_BY_MESSAGE_TYPE = {
    'BatteryStatus': 'aio_mon',
    'CoreSwitchStatus': 'cs_mon',
    'DrumSensorsMonitor': 'aio_mon',
    'FlightComputerSensor': 'aio_mon',
    'GpsStatus': 'aio_mon',
    'GroundStationPlcMonitorStatus': 'aio_mon',
    'JoystickMonitorStatus': 'aio_mon',
    'Loadcell': 'aio_mon',
    'PlatformSensorsMonitor': 'aio_mon',
    'RecorderStatus': 'aio_mon',
    'ServoStatus': 'aio_mon',
}

_SOURCES_BY_MESSAGE_TYPE = {}
for m in network_config.NetworkConfig(settings.NETWORK_YAML).all_messages:
  _SOURCES_BY_MESSAGE_TYPE[m.name] = [
      s.camel_name for s in m.all_senders
  ]

_DEFAULT_TEMPERATURE_LIMITS = _MONITOR_PARAMS.thermal.aiomon_default


def WindWsToWindG(velocity, pqr, dcm_g2p):
  """Convert wind measurement from wind sensor to ground frame."""
  wind_g = sensor_util.Vec3()
  wind_ws = sensor_util.Vec3()
  wind_ws.x = velocity[0]
  wind_ws.y = velocity[1]
  wind_ws.z = velocity[2]
  wind_sensor_params = ctypes.cast(
      ctypes.pointer(_SYSTEM_PARAMS.wind_sensor),
      ctypes.POINTER(sensor_util.WindSensorParams))

  vessel_vel_g = sensor_util.kVec3Zero

  sensor_util.WindWsToWindG(
      ctypes.pointer(wind_ws),
      ctype_util.CastPointer(dcm_g2p, sensor_util.Mat3),
      ctype_util.CastPointer(pqr, sensor_util.Vec3),
      vessel_vel_g, wind_sensor_params, ctypes.pointer(wind_g))
  return wind_g


def _GetMonSi7021Helper(mon_field):
  if mon_field == 'aio_mon':
    return _AIO_SI7021_HELPER
  elif mon_field == 'cs_mon':
    return _CS_SI7021_HELPER
  else:
    raise ValueError('Invalid monitor field "%s".' % mon_field)


class MaxBoardTemperature(indicator.BaseIndicator):
  """Indicator that aggregates board temperatures."""

  def __init__(self, name, message_types, nodes='All'):
    super(MaxBoardTemperature, self).__init__('%s [&deg;C]' % name)
    self._message_types = message_types
    self._nodes = nodes

  def Filter(self, messages):
    if not messages:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE

    max_temp = float('-inf')
    max_board = None

    for message_type in self._message_types:
      if message_type == 'MotorStatus':
        board_temp_idx = motor_thermal_types.kMotorThermalChannelBoard
        for motor_name in _SOURCES_BY_MESSAGE_TYPE[message_type]:
          if self._nodes != 'All' and motor_name not in self._nodes:
            continue
          message = messages.Subtree('%s.%s' % (message_type, motor_name))
          if not message:
            continue
          message = message.Data()
          temp = message.temps[board_temp_idx]
          if temp > max_temp:
            max_temp = temp
            max_board = motor_name
      else:
        for source in _SOURCES_BY_MESSAGE_TYPE[message_type]:
          if self._nodes != 'All' and source not in self._nodes:
            continue
          message = messages.Subtree('%s.%s' % (message_type, source))
          if not message:
            continue
          message = message.Data()
          mon_field = _MON_BY_MESSAGE_TYPE[message_type]
          populated = getattr(message, mon_field).si7021_populated
          for device in _GetMonSi7021Helper(mon_field).Values():
            if not avionics_util.IsDevicePopulated(populated, device):
              continue
            temp = getattr(message, mon_field).si7021_data[device].temperature
            if temp > max_temp:
              max_temp = temp
              max_board = source

    if max_board is None:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE
    elif max_temp > _DEFAULT_TEMPERATURE_LIMITS.very_high:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif max_temp > _DEFAULT_TEMPERATURE_LIMITS.high:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    return '{}: {:3.2f}'.format(max_board, max_temp), stoplight


def _GetActuatorMessageAttributes(mode, node_labels, node_prefix,
                                  full_comms_message_type, tether_attribute):
  """Get message snapshot indices as inputs to actuator indicators."""

  if mode == common.SPARSE_COMMS_MODE:
    return [
        ('filtered', 'merge_tether_down', 'tether_down.' + tether_attribute),
        ('filtered', 'merge_tether_down', 'valid'),
        ('filtered', 'merge_tether_down', 'timestamp_sec'),
    ]
  elif mode == common.FULL_COMMS_MODE:
    # `node_prefix` can be 'Motor', and `node_label` can be 'Pbo'.
    return [(full_comms_message_type, node_prefix + node_label)
            for node_label in node_labels]
  else:
    assert False


def _GetActuatorAndControllerMessageAttributes(
    mode, node_labels, node_prefix, full_comms_message_type, tether_attribute):
  """Get message indices as inputs to actuator indicators showing commands."""

  attributes = _GetActuatorMessageAttributes(
      mode, node_labels, node_prefix, full_comms_message_type, tether_attribute)
  return attributes + [
      ('ControllerCommand', None),
      ('filtered', 'merge_tether_down', 'valid'),
      ('filtered', 'merge_tether_down',
       'tether_down.control_telemetry.flight_mode')]


def _IsActuatorMessageAnyValid(mode, node_labels, node_label_helper,
                               max_no_update_count, *attributes):
  """Check if any actuator message is available."""
  if mode == common.SPARSE_COMMS_MODE:
    # Check the `valid` variable for TetherDown.
    if attributes[1]:
      for label in node_labels:
        idx = node_label_helper.Value(label)
        if attributes[0][idx].no_update_count <= max_no_update_count:
          return True
    return False
  elif mode == common.FULL_COMMS_MODE:
    for attribute in attributes:
      if struct_tree.IsValidElement(attribute):
        return True
    return False
  else:
    assert False


def _IsActuatorMessageAllValid(mode, node_labels, node_label_helper,
                               max_no_update_count, *attributes):
  """Check if all actuator message are available."""
  if mode == common.SPARSE_COMMS_MODE:
    # Check the `valid` variable for TetherDown.
    if attributes[1]:
      for label in node_labels:
        idx = node_label_helper.Value(label)
        if attributes[0][idx].no_update_count > max_no_update_count:
          return False
      return True
    return False
  elif mode == common.FULL_COMMS_MODE:
    for attribute in attributes:
      if not struct_tree.IsValidElement(attribute):
        return False
    return True
  else:
    assert False


def _IsTetherDownActuatorUpdated(
    status, aio_label, node_label_helper, max_no_update_count):
  node_id = node_label_helper.Value(aio_label)
  if not struct_tree.IsValidElement(status):
    return False
  no_update_count = status[node_id].no_update_count
  return no_update_count <= max_no_update_count


def _GetTetherDownActuatorStateField(
    status, aio_label, node_label_helper, field, max_no_update_count):
  if status is None:
    return None
  node_id = node_label_helper.Value(aio_label)
  node_status = status[node_id]
  no_update_count = node_status.no_update_count
  return (getattr(node_status, field)
          if no_update_count <= max_no_update_count else None)


class BaseActuatorIndicator(indicator.MultiModeIndicator):
  """Base class with utilities shared by actuator indicators."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, label, precision, node_labels, aio_node_prefix,
               node_label_helper, max_no_update_count,
               full_comms_message_type, tether_attribute, show_label):
    self._precision = precision
    self._node_labels = node_labels
    self._node_label_helper = node_label_helper
    self._max_no_update_count = max_no_update_count
    self._aio_node_prefix = aio_node_prefix
    self._show_label = show_label
    self._full_comms_message_type = full_comms_message_type
    self._tether_attribute = tether_attribute
    super(BaseActuatorIndicator, self).__init__(mode, label)

  def _GetMessageAttributes(self):
    return _GetActuatorMessageAttributes(
        self._mode, self._node_labels, self._aio_node_prefix,
        self._full_comms_message_type, self._tether_attribute)

  def _IsTetherValueValid(self, status, tetherdown_valid, aio_label):
    idx = self._node_label_helper.Value(aio_label)
    return (tetherdown_valid and
            status[idx].no_update_count <=
            self._max_no_update_count)

  def _IsValidInput(self, *attributes):
    return _IsActuatorMessageAnyValid(
        self._mode, self._node_labels, self._node_label_helper,
        self._max_no_update_count, *attributes)

  def _IsTetherUpdated(self, status, aio_label):
    return _IsTetherDownActuatorUpdated(
        status, aio_label, self._node_label_helper, self._max_no_update_count)

  def _GetTetherValue(self, status, aio_label, field):
    return _GetTetherDownActuatorStateField(
        status, aio_label, self._node_label_helper,
        field, self._max_no_update_count)

  def _GetSingleValue(self, node_idx, *attributes):
    """Get a field corresponding to the `node_idx` node from the attributes.

    In FULL_COMMS_MODE, attributes[node_idx] correspond to the target node.
    In SPARSE_COMMS_MODE, value has to be fetched from attributes[0] (the
    merged TetherDown message.)

    Args:
      node_idx: The index of the node in self._node_labels.
      *attributes: The list of attributes passed to _Filter.

    Returns:
      A value for the corresponding node.
    """
    raise NotImplementedError

  def _GetAvailableValues(self, *args):
    """Helper function to retrieve the same field from multiple AIO nodes."""
    values = {}
    for idx, aio_label in enumerate(self._node_labels):
      value = self._GetSingleValue(idx, *args)
      if value is not None:
        values[aio_label] = value
    return values

  def _DictToString(self, value_dict, str_length=5):
    """Convert a dict of values to a string as the indicator output.

    Args:
      value_dict: The dict of values to show.
      str_length: The number of characters for each item to cover.

    Returns:
      The string output.
    """

    def FormatValue(v, value_format, str_length):
      if isinstance(v, (int, float)):
        return value_format % v
      else:
        return str(v).rjust(str_length)

    text = []
    blank = '--'.rjust(str_length)

    if self._show_label:
      text.append(' '.join(k.rjust(str_length) for k in self._node_labels))

    if not self._precision:
      value_format = '%% %dd' % str_length
    else:
      value_format = '%% %d.%df' % (str_length, self._precision)

    text.append(' '.join(
        [FormatValue(value_dict[k], value_format, str_length)
         if k in value_dict else blank for k in self._node_labels]))

    return '\n'.join(text)

  def _CheckStatusFlags(self, raw_node_status, status_helper,
                        expected_statuses, failed_stoplight):
    """Check status flags per node and set stoplight accordingly.

    Normal if all nodes have the expected statuses, warning otherwise.

    Args:
      raw_node_status: The raw status values by node ({<node>: <flag>}).
      status_helper: The enum helper for the status bits.
      expected_statuses: Names of the expected status.
      failed_stoplight: The stoplight to show when the check fails.

    Returns:
      The string showing the node statuses, and the stoplight.
    """

    if self._mode == common.FULL_COMMS_MODE:
      filtered_nodes = checks.GetActuatorsWithStatus(
          raw_node_status, status_helper, expected_statuses)
      node_status = {key: 1 if key in filtered_nodes else 0
                     for key, value in raw_node_status.iteritems()}
    elif self._mode == common.SPARSE_COMMS_MODE:
      target_values = [status_helper.Value(x) for x in expected_statuses]
      filtered_nodes = {node for node, value in raw_node_status.iteritems()
                        if value in target_values}
      node_status = {key: status_helper.ShortName(value)
                     for key, value in raw_node_status.iteritems()}
    else:
      assert False

    stoplight = checks.CheckForSize(
        filtered_nodes, len(node_status),
        equal_flag=stoplights.STOPLIGHT_NORMAL,
        unequal_flag=failed_stoplight)

    return self._DictToString(node_status), stoplight

  def _GetFieldInfo(self, normal_ranges, warning_ranges, error_ranges,
                    *message_attributes):
    """Get node values and set stoplight according to thresholds.

    Args:
      normal_ranges: A makani/analysis/checks/check_range.py:BaseRange object.
          Values falling within it are normal.
      warning_ranges: Values falling out of previous ranges and within this
          ranges raise warnings. If None, error ranges will be checked.
      error_ranges: Values falling out of previous ranges and within this
          ranges raise errors. Values falling out of this ranges are
          invalid/unavailable. If None, then values ending up here will always
          raise errors.
      *message_attributes: The list of message attributes.

    Returns:
      The dict of values, and the resulting stoplight.
    """

    # Get available values in a dict {source: value}.
    values = self._GetAvailableValues(*message_attributes)

    assert normal_ranges is not None

    any_valid = False
    all_valid = True
    error = False
    warning = False
    if values:
      for key, value in values.iteritems():
        if value is None:
          all_valid = False
          values[key] = '--'
          continue
        if value in normal_ranges:
          any_valid = True
          continue
        elif warning_ranges is not None and value in warning_ranges:
          # If warning ranges is None, we test against error ranges.
          any_valid = True
          warning = True
          continue
        elif error_ranges is None or value in error_ranges:
          # If error_ranges is None, set the stoplight to error.
          any_valid = True
          error = True
          continue
        else:
          # Treat it as an invalid value.
          all_valid = False
          values[key] = '--'
          continue

      if not any_valid:
        stoplight = stoplights.STOPLIGHT_UNAVAILABLE
      elif error:
        stoplight = stoplights.STOPLIGHT_ERROR
      elif warning:
        stoplight = stoplights.STOPLIGHT_WARNING
      elif not all_valid:
        stoplight = stoplights.STOPLIGHT_WARNING
      else:
        stoplight = stoplights.STOPLIGHT_NORMAL
    else:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE

    return values, stoplight


class ActuatorDictChart(indicator.MultiModeDictChart):
  """Base dict chart class for actuators."""

  @indicator.RegisterModes(common.FULL_COMMS_MODE, common.SPARSE_COMMS_MODE)
  def __init__(self, mode, name, node_labels, labels, aio_node_prefix,
               node_label_helper, max_no_update_count,
               full_comms_message_type, tether_attribute, **widget_kwargs):
    """Initialize the object.

    Args:
      mode: Typical indicator mode, e.g, common.FULL_COMMS_MODE.
      name: Name of the indicator.
      node_labels: Labels of the actuator.
          E.g, ['Pbi', 'Sto'] for motor, ['A1', 'A8'] for servo.
      labels: A list of keys labeling the data sequences to show.
      aio_node_prefix: Prefix for the aio node.
          E.g, 'Motor' for motors, 'Servo' for servos.
      node_label_helper: The enum helper for node labels.
      max_no_update_count: The max value for the TetherDown no_update_count
          to indicate freshness.
      full_comms_message_type: The message type to use in FULL_COMMS_MODE.
      tether_attribute: The TetherDown attribute to use in SPARSE_COMMS_MODE.
      **widget_kwargs: The kwargs for the widget.
    """
    self._node_labels = node_labels
    self._aio_node_prefix = aio_node_prefix
    self._node_label_helper = node_label_helper
    self._max_no_update_count = max_no_update_count
    self._full_comms_message_type = full_comms_message_type
    self._tether_attribute = tether_attribute
    super(ActuatorDictChart, self).__init__(mode, labels, name, **widget_kwargs)

  def _GetMessageAttributes(self):
    return _GetActuatorMessageAttributes(
        self._mode, self._node_labels, self._aio_node_prefix,
        self._full_comms_message_type, self._tether_attribute)

  def _IsValidInput(self, *attributes):
    return _IsActuatorMessageAnyValid(
        self._mode, self._node_labels, self._node_label_helper,
        self._max_no_update_count, *attributes)

  def _IsTetherUpdated(self, status, aio_label):
    return _IsTetherDownActuatorUpdated(
        status, aio_label, self._node_label_helper, self._max_no_update_count)

  def _GetTetherValue(self, status, aio_label, field):
    return _GetTetherDownActuatorStateField(
        status, aio_label, self._node_label_helper,
        field, self._max_no_update_count)

  def _GetTimestamp(self, idx, *args):
    if self._mode == common.FULL_COMMS_MODE:
      return args[idx].capture_info['timestamp'] if args[idx] else None
    elif self._mode == common.SPARSE_COMMS_MODE:
      # args[2] is the timestamp for TetherDown and args[1] is its validity.
      return args[2] if args[1] else None
    else:
      assert False


class ActuatorCmdDictChart(ActuatorDictChart):
  """Base dict chart class for actuators and their controller commands."""

  _OBSERVATION_KEY = 'obs'
  _COMMAND_KEY = 'cmd'

  def __init__(self, mode, name, node_labels, aio_node_prefix,
               node_label_helper, max_no_update_count,
               full_comms_message_type, tether_attribute,
               show_cmd, **widget_kwargs):
    """Initialize the class and populate labels according to show_cmd."""
    self._show_cmd = show_cmd
    self._limits = {}

    if show_cmd:
      labels = []
      for node in node_labels:
        labels.append(self._ObservationLabel(node))
        labels.append(self._CommandLabel(node))

      widget_kwargs = copy.copy(widget_kwargs)
      widget_kwargs['heading'] = '      %s    %s' % (
          self._OBSERVATION_KEY, self._COMMAND_KEY)
      secondary_keys = [self._OBSERVATION_KEY, self._COMMAND_KEY]
      widget_kwargs['secondary_keys'] = secondary_keys
      chart_keys = []
      for node in node_labels:
        chart_keys.append(self._ObservationLabel(node))
      widget_kwargs['chart_keys'] = chart_keys
    else:
      labels = node_labels

    super(ActuatorCmdDictChart, self).__init__(
        mode, name, node_labels, labels, aio_node_prefix,
        node_label_helper, max_no_update_count,
        full_comms_message_type, tether_attribute,
        keys=node_labels, **widget_kwargs)

  def _SetLimits(self, limits, flight_modes):
    """Set limits to check as {key: (normal_ranges, warning_ranges)}."""
    for flight_mode in flight_modes:
      if flight_mode not in self._limits:
        self._limits[flight_mode] = {}
      self._limits[flight_mode].update(limits)

  def _GetMessageAttributes(self):
    return _GetActuatorAndControllerMessageAttributes(
        self._mode, self._node_labels, self._aio_node_prefix,
        self._full_comms_message_type, self._tether_attribute)

  def _IsValidInput(self, *attributes):
    return _IsActuatorMessageAnyValid(
        self._mode, self._node_labels, self._node_label_helper,
        self._max_no_update_count, *attributes)

  def _ObservationLabel(self, node):
    if self._show_cmd:
      return node + '.' + self._OBSERVATION_KEY
    else:
      return node

  def _CommandLabel(self, node):
    return node + '.' + self._COMMAND_KEY

  def _GetCmdValue(self, node, controller_command):
    raise NotImplementedError

  @indicator.ReturnIfInputInvalid(None, None, stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *args):
    values = {}
    timestamps = {}
    missing_values = False
    any_values = False
    controller_command = args[-3]
    # args[-2] is the valid bit. args[-1] is the flight mode field.
    flight_mode = args[-1] if args[-2] else None
    for idx, node in enumerate(self._node_labels):
      key = self._ObservationLabel(node)
      values[key] = self._GetValuePerNode(idx, *args)
      timestamps[key] = self._GetTimestamp(idx, *args)

      if values[key] is None:
        missing_values = True
      else:
        any_values = True

      if controller_command:
        key = self._CommandLabel(node)
        values[key] = self._GetCmdValue(node, controller_command)
        if values[key] is None:
          missing_values = True
        timestamps[key] = controller_command.capture_info['timestamp']

    stoplight = self._GetStoplight(
        values, any_values, missing_values, flight_mode)
    return timestamps, values, stoplight

  def _GetStoplight(self, values, any_values, missing_values, flight_mode):
    """Get the stoplight according to the values.

    Args:
      values: The values to determine stoplights. It is used in overrides of
          this method.
      any_values: If False, then the stoplight should be grey.
      missing_values: If True, then the stoplight should not be green.
      flight_mode: The current flight mode.

    Returns:
      The stoplight corresponding to the value and conditions.
    """
    if not any_values:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif missing_values:
      stoplight = stoplights.STOPLIGHT_ERROR
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    if values and flight_mode in self._limits:
      range_stoplight = stoplights.STOPLIGHT_NORMAL
      for key, (normal_ranges, warning_ranges) in self._limits[
          flight_mode].iteritems():
        range_stoplight = stoplights.MostSevereStoplight(
            range_stoplight,
            stoplights.SetByRanges(
                values[key], normal_ranges, warning_ranges))
      stoplight = stoplights.MostSevereStoplight(stoplight, range_stoplight)
    return stoplight


class AioMonTemperatureIndicator(indicator.BaseIndicator):
  """Indicator to show individual temperature monitor readings."""

  def __init__(self, name, message_type, sources=None, source_prefix=''):
    super(AioMonTemperatureIndicator, self).__init__('%s [&deg;C]' % name)
    self._message_type = message_type
    self._precision = 1
    self._sources = (
        [source_prefix + source for source in sources] if sources
        else _SOURCES_BY_MESSAGE_TYPE[message_type])
    self._source_prefix = source_prefix

  def _GetKeyName(self, source, device, device_helper):
    start_idx = (
        len(self._source_prefix)
        if source.startswith(self._source_prefix) else 0)
    if len(device_helper) == 1:
      return source[start_idx:]
    else:
      return '%s.%s' % (source[start_idx:], device_helper.ShortName(device))

  def Filter(self, messages):
    timestamps = {}
    results = {}
    any_value = False
    warning = False
    error = False
    for source in self._sources:
      message = messages.Subtree('%s.%s' % (self._message_type, source))
      if not message:
        continue
      message = message.Data()
      mon_field = _MON_BY_MESSAGE_TYPE[self._message_type]
      populated = getattr(message, mon_field).si7021_populated
      device_helper = _GetMonSi7021Helper(mon_field)
      for device in device_helper.Values():
        if not avionics_util.IsDevicePopulated(populated, device):
          continue
        any_value = True
        temp = getattr(message, mon_field).si7021_data[device].temperature
        time = message.capture_info['timestamp']
        key = self._GetKeyName(source, device, device_helper)
        results[key] = temp
        timestamps[key] = time
        if (temp > _DEFAULT_TEMPERATURE_LIMITS.very_high or
            temp < _DEFAULT_TEMPERATURE_LIMITS.very_low):
          error = True
        elif (temp > _DEFAULT_TEMPERATURE_LIMITS.high or
              temp < _DEFAULT_TEMPERATURE_LIMITS.low):
          warning = True

    if not any_value:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif error:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif warning:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL
    return timestamps, results, stoplight

  def Plot(self, timestamps, results, stoplight):
    keys = []
    for source in self._sources:
      mon_field = _MON_BY_MESSAGE_TYPE[self._message_type]
      device_helper = _GetMonSi7021Helper(mon_field)
      for device in device_helper.ShortNames():
        keys.append(self._GetKeyName(source, device, device_helper))

    return widgets.DictTrailsWidget(
        self._label, timestamps, results, stoplight, self._precision, keys)


class BaseConfigLimitIndicator(indicator.BaseAttributeIndicator):
  """A generic class to check a message field agains config limits."""

  def __init__(self, name, message_type, aio_node, field, limits, text_format):
    """Initialize the indicator.

    Args:
      name: Label to display as the indicator's title.
      message_type: The short name of the message type.
      aio_node: The short name of the AIO node as source of the message. Note
          that a message may have multiple sources. If any source is fine, then
          put it as None.
      field: The string index into the message's StructTree presentation.
      limits: The limits from the config, containing bounds such as [very_low,
          low, high, very_high]. E.g.
          monitor_params.GetMonitorParams().contents.thermal.detwist
      text_format: The text format to show with the field value. E.g. '%2.1f C'.
    """
    super(BaseConfigLimitIndicator, self).__init__(
        [(message_type, aio_node)], name)
    self._field = field
    self._text_format = text_format
    self._limits = limits

  def _Filter(self, message):
    if not struct_tree.IsValidElement(message):
      text = '    --'
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    else:
      value_ranges = self._GetLimits(self._limits)
      value = self._GetCTypeFieldByString(message, self._field)
      text = self._text_format % value
      stoplight = stoplights.SetByRanges(
          value, value_ranges['normal'],
          value_ranges['warning'])
    return text, stoplight


class BridleJunctionLoadcellIndicator(indicator.BaseIndicator):
  """Indicator for the bridle junction loadcell."""
  # TODO: These could be separate indicators; and it might be
  # nice to directly compare the estimator's tether roll indication to
  # the value from this encoder. Tension is more interesting during
  # flight operations.

  def Filter(self, messages):
    if not messages:
      return '--\n', stoplights.STOPLIGHT_UNAVAILABLE

    read_timeout_warning = False
    any_available = False
    load = messages['Loadcell.LoadcellPortB.bridle_junc.junc_load']
    angle = messages['Loadcell.LoadcellPortB.bridle_junc.junc_angle']
    warning = messages['Loadcell.LoadcellPortB.bridle_junc.flags.warning']

    if load is None:
      return '--\n', stoplights.STOPLIGHT_UNAVAILABLE

    # Display text.
    load_pin_text = 'Tension:    '
    encoder_text = 'Roll angle: '
    # Warn operator if load pin comms freeze; otherwise display load data.
    if warning & _BRIDLE_JUNC_WARNING_HELPER.Value('LoadPinReadTimeout'):
      load_pin_text += 'Read timeout\n'
      read_timeout_warning = True
    else:
      load_pin_text += ('{tension:7.3f} kN\n').format(tension=load * 1e-3)
      any_available = True
    # Warn operator if encoder comms freeze; otherwise display angle data.
    if warning & _BRIDLE_JUNC_WARNING_HELPER.Value('EncoderReadTimeout'):
      encoder_text += 'Read timeout'
      read_timeout_warning = True
    else:
      encoder_text += ('{roll_deg:3.0f} deg').format(
          roll_deg=np.rad2deg(angle))
      any_available = True

    text = load_pin_text + encoder_text
    if not any_available:
      return text, stoplights.STOPLIGHT_UNAVAILABLE
    elif read_timeout_warning:
      return text, stoplights.STOPLIGHT_WARNING
    else:
      return text, stoplights.STOPLIGHT_NORMAL


class FpvIndicator(indicator.MultiModeIndicator):
  """Indicator to warn an enabled FPV camera which may interfere with GPS."""

  def __init__(self, mode, fc_shortnames, name=None):
    self._fc_shortnames = fc_shortnames
    self._fc_values = [_FLIGHT_COMPUTER_HELPER.Value(fc)
                       for fc in fc_shortnames]
    super(FpvIndicator, self).__init__(mode, name)

  def _GetMessageAttributes(self):
    if self._mode == common.SPARSE_COMMS_MODE:
      return [
          ('filtered', 'merge_tether_down',
           'tether_down.flight_computers[%d]' % fc) for fc in self._fc_values
      ] + [('filtered', 'merge_tether_down', 'valid')]
    elif self._mode == common.FULL_COMMS_MODE:
      return [('FlightComputerSensor', 'Fc' + fc, 'flags.warning')
              for fc in self._fc_shortnames]
    else:
      assert False

  def _IsValidInput(self, *attributes):
    """Tests if the inputs are valid.

    Args:
      *attributes: List of attributes according to the fields returned by
          _GetMessageAttributes. Note the list differs according to the
          communication mode.

    Returns:
      True if the inputs are valid.
    """
    if self._mode == common.SPARSE_COMMS_MODE:
      return attributes[-1]
    elif self._mode == common.FULL_COMMS_MODE:
      for attribute in attributes:
        if attribute is not None:
          return True
      return False
    else:
      assert False

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *attributes):
    any_enabled_fpv = False
    if self._mode == common.SPARSE_COMMS_MODE:
      # Iterate through all flags, excluding the valid bit.
      for attrib in attributes[:-1]:
        if attrib.flags & avionics_messages.kTetherFlightComputerFlagFpvEnabled:
          any_enabled_fpv = True
          break
    elif self._mode == common.FULL_COMMS_MODE:
      for attrib in attributes:
        if (attrib and
            attrib & avionics_messages.kFlightComputerWarningFpvEnabled):
          any_enabled_fpv = True
          break
    else:
      assert False

    if any_enabled_fpv:
      return 'On', stoplights.STOPLIGHT_WARNING
    else:
      return 'Off', stoplights.STOPLIGHT_NORMAL


class PitotCoverIndicator(indicator.MultiModeIndicator):
  """Indicator for the pitot cover status."""

  def __init__(self, mode, fc_shortnames, name=None):
    self._fc_shortnames = fc_shortnames
    self._fc_values = [_FLIGHT_COMPUTER_HELPER.Value(fc)
                       for fc in fc_shortnames]
    super(PitotCoverIndicator, self).__init__(mode, name)

  def _GetMessageAttributes(self):
    if self._mode == common.SPARSE_COMMS_MODE:
      return []
    elif self._mode == common.FULL_COMMS_MODE:
      return [('FlightComputerSensor', 'Fc' + fc, 'pitot_cover_status')
              for fc in self._fc_shortnames]
    else:
      assert False

  def _IsValidInput(self, *attributes):
    """Tests if the inputs are valid.

    Args:
      *attributes: List of attributes according to the fields returned by
          _GetMessageAttributes. Note the list differs according to the
          communication mode.

    Returns:
      True if the inputs are valid.
    """
    if self._mode == common.SPARSE_COMMS_MODE:
      return False
    elif self._mode == common.FULL_COMMS_MODE:
      for attribute in attributes:
        if attribute is not None:
          return True
      return False
    else:
      assert False

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, *attributes):
    pitot_cover_status = pitot_cover_types.kPitotCoverStatusUnknown
    if self._mode == common.SPARSE_COMMS_MODE:
      pass
    elif self._mode == common.FULL_COMMS_MODE:
      for attrib in attributes:
        # This assumes there is only one Fc reporting pitot cover status.
        if (attrib and
            attrib != pitot_cover_types.kPitotCoverStatusUnknown):
          pitot_cover_status = attrib
          break
    else:
      assert False

    stoplight = stoplights.STOPLIGHT_WARNING
    if pitot_cover_status == pitot_cover_types.kPitotCoverStatusOpened:
      stoplight = stoplights.STOPLIGHT_NORMAL
    elif pitot_cover_status in [pitot_cover_types.kPitotCoverStatusClosing,
                                pitot_cover_types.kPitotCoverStatusClosed]:
      stoplight = stoplights.STOPLIGHT_ERROR

    return _PITOT_COVER_STATUS_HELPER.ShortName(pitot_cover_status), stoplight


def GetMonitorFields(message, monitor_field, monitor_type, monitor_helper,
                     warning_helper, error_helper, monitor_names, aio_node,
                     read_error_name):
  """Extract avionics monitor data fields."""
  monitor = getattr(message, monitor_field)
  populated = getattr(monitor, monitor_type + '_populated')
  warning = False
  error = False
  unresponsive_device_warning = False
  error_messages = []
  monitors = {}

  if read_error_name and read_error_name in warning_helper:
    unresponsive_device_warning = avionics_util.CheckWarning(
        monitor.flags, warning_helper.Value(read_error_name))
    warning |= unresponsive_device_warning

  for monitor_name in monitor_names:
    # Guard against bad voltage names.
    if monitor_name not in monitor_helper:
      error_messages.append('%sInvalid (%s)' %
                            (aio_node + ': ' if aio_node else '',
                             monitor_name))
      continue

    index = monitor_helper.Value(monitor_name)
    if not avionics_util.TestMask(populated, index):
      continue

    monitor_data = getattr(monitor, monitor_type + '_data')
    monitors[monitor_name] = monitor_data[index]

    if monitor_name in warning_helper:
      warning |= avionics_util.CheckWarning(
          monitor.flags,
          warning_helper.Value(monitor_name))

    if monitor_name in error_helper:
      error |= avionics_util.CheckError(
          monitor.flags,
          error_helper.Value(monitor_name))
  return monitors, warning, error, error_messages, unresponsive_device_warning


class BaseMonitorIndicator(indicator.BaseIndicator):
  """The base class for hardware monitor indicators."""

  def __init__(self, message_type, monitor_field, short_names, name,
               error_helper, warning_helper, monitor_helper, monitor_type,
               monitor_names, read_error_name):
    self._message_type = message_type        # 'BatteryStatus'
    self._monitor_field = monitor_field      # "batt_mon"
    self._short_names = short_names          # ["BattA", "BattB"]
    self._error_helper = error_helper        # BATT_MON_ERROR_HELPER
    self._warning_helper = warning_helper    # BATT_MON_WARNING_HELPER
    self._monitor_helper = monitor_helper    # BATT_MCP342X_MONITOR_HELPER
    self._monitor_type = monitor_type        # "mcp342x
    self._monitor_names = monitor_names      # ['HeatPlate1', 'HeatPlate2',...]
    self._read_error_name = read_error_name  # 'TempReadErrors'
    self._unresponsive_device_warning = False
    super(BaseMonitorIndicator, self).__init__(name)

  def _GetMonitors(self, messages):
    monitors = collections.defaultdict(dict)
    any_value = False
    warning = False
    error = False
    self._unresponsive_device_warning = False
    error_messages = []

    for short_name in self._short_names:
      if self._message_type + '.' + short_name not in messages:
        continue
      any_value = True
      message = messages['%s.%s' % (self._message_type, short_name)]
      (fields, warning_per_node, error_per_node, error_messages_per_node,
       unresponsive_device_warning_per_node) = (
           GetMonitorFields(message, self._monitor_field, self._monitor_type,
                            self._monitor_helper, self._warning_helper,
                            self._error_helper, self._monitor_names,
                            short_name, self._read_error_name))
      for monitor_name, value in fields.iteritems():
        monitors[monitor_name][short_name] = value
      warning |= warning_per_node
      error |= error_per_node
      error_messages += error_messages_per_node
      self._unresponsive_device_warning |= unresponsive_device_warning_per_node
    return monitors, any_value, error_messages, error, warning

  def Filter(self, messages):
    if not messages:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE

    monitors, any_value, error_messages, error, warning = self._GetMonitors(
        messages)

    if error_messages or error:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif not any_value:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif warning:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    results = [' ' * 10  + ' '.join(v.rjust(6) for v in self._monitor_names)]
    for short_name in self._short_names:
      text = '{:9}'.format(short_name)
      for monitor_name in self._monitor_names:
        column_width = max(7, len(monitor_name) + 1)
        if monitor_name in monitors and short_name in monitors[monitor_name]:
          text += '{:{width}.1f}'.format(monitors[monitor_name][short_name],
                                         width=column_width)
        else:
          text += '--'.rjust(column_width)
      # If there's a warning for missing chips, report this condition.
      if self._unresponsive_device_warning:
        text += ' (WARNING: DEVICE UNRESPONSIVE)'
      results.append(text)
    return '\n'.join(error_messages + results), stoplight

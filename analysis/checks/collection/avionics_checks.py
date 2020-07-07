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

"""Classes to perform checks on avionics sensors."""

from makani.analysis.checks import avionics_util
from makani.analysis.checks import base_check
from makani.analysis.checks import check_range
from makani.avionics.firmware.monitors import aio_types
from makani.lib.python import c_helpers

_AIOMON_WARNING_HELPER = c_helpers.EnumHelper('AioMonitorWarning', aio_types,
                                              prefix='kAioMonitorWarning')


class BaseSensorGroupChecker(base_check.BaseCheckItem):
  """The base class for checking values of avionics sensor outputs."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, node,
               data_field, populated_field, index_enum_helper,
               enum_names=None, verbose_names=None,
               **base_kwargs):
    """Initialize the checker for a given unit.

    Args:
      for_log: True if this check is performed over a log. False if it is for
          realtime AIO messages.
      message_type: Name of the message type.
      node: Short name of the AIO node.
      data_field: Path to the data field.
      populated_field: Path to the populated field.
      index_enum_helper: An EnumHelper for the device enumerators.
      enum_names: A list of short names of device enum types to check.
      verbose_names: A list of names to display, each corresponding to one
          enum_name.
      **base_kwargs: Args for the base class, including
          normal_ranges and warning_ranges.
    """
    self._message_type = message_type
    self._aio_node = node
    self._data_field = data_field
    self._populated_field = populated_field
    if not enum_names:
      enum_names = index_enum_helper.ShortNames()
    if not verbose_names:
      verbose_names = enum_names
    assert len(enum_names) == len(verbose_names)
    device_ids = [index_enum_helper.Value(x) for x in enum_names]
    self._devices = dict(zip(device_ids, verbose_names))
    super(BaseSensorGroupChecker, self).__init__(for_log, **base_kwargs)

  def _RegisterInputs(self):
    """Register what fields will be used to calculate the check results."""
    return [
        self._Arg(self._message_type, self._aio_node, self._data_field),
        self._Arg(self._message_type, self._aio_node, self._populated_field),
    ]

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, data, populated):
    """Calculate check results using the list of registered inputs."""
    if self._for_log:
      # Assume the `populated` field stays the same during a log.
      populated = populated[0]

    for device_id in self._devices:
      if avionics_util.IsDevicePopulated(populated, device_id):
        self._CheckOneDevice(device_id, data)

  def _GetDeviceData(self, data, device_id):
    if self._for_log:
      # Convert each value in the log sequence.
      return data[:, device_id]
    else:
      return data[device_id]

  def _CheckOneDevice(self, device_id, data):
    """Check a single device.

    Args:
      device_id: The enum value of the device.
      data: Data to check.
    """

    label = (('%s [%s]' % (self._name, self._devices[device_id]))
             if self._name else self._devices[device_id])

    values = self._GetDeviceData(data, device_id)
    # Check the values, using `ranges` for both normal and warning ranges
    # so that if a value falls out of `ranges`, an error is reported.
    self._CheckByRange(label, values, self._normal_ranges, self._warning_ranges)


class BaseVoltageChecker(BaseSensorGroupChecker):
  """The monitor to check analog values."""

  _EXCLUDED_VOLTAGE_TYPES = {'ClampResistor'}

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, node, data_field,
               populated_field, index_enum_helper, **base_kwargs):

    super(BaseVoltageChecker, self).__init__(
        for_log, message_type, node, data_field, populated_field,
        index_enum_helper, **base_kwargs)

  def _CheckOneDevice(self, device_id, data):
    device_name = self._devices[device_id]
    if device_name in self._EXCLUDED_VOLTAGE_TYPES:
      return

    label = (('%s [%s]' % (self._name, device_name))
             if self._name else self._devices[device_id])

    normal_ranges = (avionics_util.VoltageRange(device_name)
                     if self._normal_ranges is None else self._normal_ranges)
    warning_ranges = (avionics_util.VoltageRange(device_name)
                      if self._warning_ranges is None else self._warning_ranges)

    values = self._GetDeviceData(data, device_id)
    self._CheckByRange(label, values, normal_ranges, warning_ranges)


class AioMonAnalogChecker(BaseVoltageChecker):
  """The monitor to check Aio node analog voltage."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, node, **base_kwargs):

    super(AioMonAnalogChecker, self).__init__(
        for_log, message_type, node,
        'aio_mon.analog_data', 'aio_mon.analog_populated',
        c_helpers.EnumHelper('AioAnalogVoltage', aio_types), **base_kwargs)


class AioMonTemperatureChecker(BaseSensorGroupChecker):
  """The monitor to check Aio node temperature."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, node, **base_kwargs):
    super(AioMonTemperatureChecker, self).__init__(
        for_log, message_type, node,
        'aio_mon.si7021_data[:].temperature', 'aio_mon.si7021_populated',
        c_helpers.EnumHelper('AioSi7021Monitor', aio_types), **base_kwargs)


class AioMonHumidityChecker(BaseSensorGroupChecker):
  """The monitor to check Aio node humidity."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, node, **base_kwargs):

    super(AioMonHumidityChecker, self).__init__(
        for_log, message_type, node,
        'aio_mon.si7021_data[:].rel_humidity', 'aio_mon.si7021_populated',
        c_helpers.EnumHelper('AioSi7021Monitor', aio_types), **base_kwargs)


class AioMonBusVoltageChecker(BaseVoltageChecker):
  """The monitor to check Aio node voltage."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, node, **base_kwargs):
    super(AioMonBusVoltageChecker, self).__init__(
        for_log, message_type, node,
        'aio_mon.ina219_data[:].voltage', 'aio_mon.ina219_populated',
        c_helpers.EnumHelper('AioIna219Monitor', aio_types), **base_kwargs)


class AioMonBusCurrentChecker(BaseSensorGroupChecker):
  """The monitor to check Aio node current."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, node, **base_kwargs):
    super(AioMonBusCurrentChecker, self).__init__(
        for_log, message_type, node,
        'aio_mon.ina219_data[:].current', 'aio_mon.ina219_populated',
        c_helpers.EnumHelper('AioIna219Monitor', aio_types), **base_kwargs)


class AioMonWarningCheck(base_check.BitmaskErrorCheck):
  """Class to check for aio_mon.flags for warnings."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, message_type, sources, errors=None, name=None):
    """Initialize this class.

    Args:
      for_log: True if we check against logs, False for AIO messages.
      message_type: String representing the message type, e.g. 'ServoStatus'
      sources: List of aio nodes to use as message sources.
      errors: A list of errors to check for, defaults to checking all. e.g.
          ['12v', LvA']
      name: The name of the check.
    """
    error_if_fail = False

    super(AioMonWarningCheck, self).__init__(for_log, message_type, sources,
                                             'aio_mon.flags.warning',
                                             _AIOMON_WARNING_HELPER,
                                             error_if_fail, errors, name)


class BaseAvionicsChecks(base_check.ListOfChecks):
  """A checklist containing checks to perform for AIO modules."""

  def __init__(self, for_log, message_type, aio_node_label_helper,
               aio_node_prefix=''):

    aio_node_labels = [aio_node_prefix + s
                       for s in aio_node_label_helper.ShortNames()]

    self._items_to_check = [
        AioMonAnalogChecker(for_log, message_type, node,
                            name='%s.AioMon.AnalogVoltage' % node)
        for node in aio_node_labels
    ] + [
        AioMonTemperatureChecker(
            for_log, message_type, node,
            name='%s.AioMon.Temperature' % node,
            normal_ranges=check_range.Interval([0, 65]),
            warning_ranges=check_range.Interval([0, 85]))
        for node in aio_node_labels
    ] + [
        AioMonHumidityChecker(for_log, message_type, node,
                              name='%s.AioMon.Humidity' % node)
        for node in aio_node_labels
    ] + [
        AioMonBusVoltageChecker(for_log, message_type, node,
                                name='%s.AioMon.BusVoltage' % node)
        for node in aio_node_labels
    ] + [
        AioMonBusCurrentChecker(for_log, message_type, node,
                                name='%s.AioMon.BusCurrent' % node)
        for node in aio_node_labels
    ]

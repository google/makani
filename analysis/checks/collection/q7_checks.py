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

"""Classes to perform checks on Q7SlowStatusMessages."""

from makani.analysis.checks import base_check
from makani.avionics.common import pack_avionics_messages
from makani.lib.python import c_helpers


class Q7CpuLoadCheck(base_check.BaseCheckItem):
  """Check CPU load.
  """

  @base_check.RegisterSpecs
  def __init__(self, for_log, node):
    """Initialize the checks.

    Args:
      for_log: True if this check is used for log analysis. Otherwise, it is
          for realtime AIO monitoring.
      node: The name of the node sending the message.
    """
    self.node = node
    super(Q7CpuLoadCheck, self).__init__(for_log,
                                         name='Q7CpuLoadCheck.' + node)

  def _RegisterInputs(self):
    return [self._Arg('Q7SlowStatus', self.node, 'sys_info.num_cpus'),
            self._Arg('Q7SlowStatus', self.node, 'sys_info.load_averages[:]')]

  def _CheckSingleLoad(self, num_cpus, loads, index, minutes):
    if self._for_log:
      # Just in case the data ever drops out, use our best number.
      num_cpus = max(num_cpus)
    load = loads[:, index]
    load_per_cpu = load / float(num_cpus)
    name = '%s.%s (%d-minute)' % (self.node, 'CpuLoadCheck', minutes)
    self._CheckByRange(name, load_per_cpu, [[0, 1.6]], [[0, 2]])

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, num_cpus, loads):
    """Do the checking.

    Args:
      num_cpus: The number of logical CPUs on the Q7, as scalar or numpy.array.
      loads: SysInfo.load_averages, as float[3] or numpy.array(float[3])
    """
    self._CheckSingleLoad(num_cpus, loads, 0, 1)
    self._CheckSingleLoad(num_cpus, loads, 1, 5)
    self._CheckSingleLoad(num_cpus, loads, 2, 15)


class Q7TemperatureCheck(base_check.BaseCheckItem):
  """Check CPU and SSD temperatures.
  """

  _temperature_info_flag_helper = c_helpers.EnumHelper('TemperatureInfoFlag',
                                                       pack_avionics_messages)

  @base_check.RegisterSpecs
  def __init__(self, for_log, node):
    """Initialize the checks.

    Args:
      for_log: True if this check is used for log analysis. Otherwise, it is
          for realtime AIO monitoring.
      node: The name of the node sending the message.
    """
    self.node = node
    super(Q7TemperatureCheck, self).__init__(for_log, [[0, 70]], [[-40, 85]],
                                             name='Q7TemperatureCheck.' + node)

  def _RegisterInputs(self):
    """Indicates what fields are needed for the check.

    Returns:
      A list of arguments.  Each argument contains three components: message
      type, aio node (short name), and message field. The message field is a
      string that indexes into the message struct. It can refer to arrays and
      dicts.  E.g. "fc_mon.analog_data[2]".  In addition, "timestamp" will refer
      to the derived field "capture_header.tv_sec + capture_header.tv_usec *
      1e-6".
    """
    #                |  Message  |  Aio node  | Message |
    #                |   type    | Short name | field   |
    return [self._Arg('Q7SlowStatus', self.node, 'temperature_info.flags'),
            self._Arg('Q7SlowStatus', self.node, 'temperature_info.cpu_zone_0'),
            self._Arg('Q7SlowStatus', self.node, 'temperature_info.cpu_zone_1'),
            self._Arg('Q7SlowStatus', self.node, 'temperature_info.ssd')]

  def _Check(self, flags, cpu_zone_0, cpu_zone_1, ssd):
    """Do the checking.

    Each argument comes in either as a scalar or as a numpy.array of such
    scalars.

    Args:
      flags: Q7SlowStatusMessage.temperature_info.flags
      cpu_zone_0: Q7SlowStatusMessage.temperature_info.cpu_zone_0
      cpu_zone_1: Q7SlowStatusMessage.temperature_info.cpu_zone_1
      ssd: Q7SlowStatusMessage.temperature_info.ssd
    """

    def GetMask(bit_name):
      return self._temperature_info_flag_helper.Value(bit_name)

    def Filter(for_log, flag_field, data_field):
      """Skip checking any data for which the flag bit isn't set.

      Args:
        for_log: True if this check is used for log analysis. Otherwise, it is
            for realtime AIO monitoring.
        flag_field: A flag or array of flags indicating presence of data.
        data_field: The data to use, if the flag is set.
      Returns:
        If the flag isn't set, a single value will get converted to None;
        an array of values will be filtered, and a shorter array returned, that
        includes only the values corresponding to indices where the flag was
        set.
      """
      if flag_field is None or data_field is None:
        return None
      if for_log:
        return data_field[flag_field & mask]
      elif mask & flag_field == 0:
        return None
      return data_field

    if flags is None:
      return

    tuples = [('CpuZone0', cpu_zone_0),
              ('CpuZone1', cpu_zone_1),
              ('Ssd', ssd)]
    # Check each temperature field for which data is present.
    for t in tuples:
      mask = GetMask('%sValid' % t[0])
      temperature_data = t[1]
      temperature_name = '%s.%sTemperature' % (self.node, t[0])
      # Controller Q7s don't yet report temperatures, but when they do, they
      # won't have SSDs.
      if 'Recorder' not in self.node and 'Ssd' in temperature_name:
        continue
      flag_value = flags & mask != 0
      # Warn if we're not getting temperature info.
      self._CheckByRange(temperature_name, flag_value,
                         [[True, True]], [[False, True]])

      temperature_data = Filter(self._for_log, flag_value, temperature_data)
      if temperature_data is not None:
        self._CheckByRange(temperature_name, temperature_data,
                           self._normal_ranges, self._warning_ranges)


class Q7Checks(base_check.ListOfChecks):

  def __init__(self, for_log):
    self._items_to_check = [
        Q7TemperatureCheck(for_log, 'RecorderQ7Wing'),
        Q7TemperatureCheck(for_log, 'RecorderQ7Platform'),
        Q7CpuLoadCheck(for_log, 'RecorderQ7Wing'),
        Q7CpuLoadCheck(for_log, 'RecorderQ7Platform')]

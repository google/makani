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

"""Classes to check for humidity and temperature."""

from makani.analysis.checks import base_check
from makani.analysis.checks import check_range
from makani.analysis.checks.collection import avionics_checks
from makani.avionics.network import network_config


class AioMonChecks(base_check.ListOfChecks):
  """The class to check all AIO module housekeeping sensors.

  These include humidity, temperature, voltages, and currents.
  """

  def __init__(self, for_log):

    self._items_to_check = []

    net_conf = network_config.NetworkConfig()
    node_messages = {'batt': 'BatteryStatus',
                     'drum': 'DrumSensorsMonitor',
                     'flight_computer': 'FlightComputerSensor',
                     'gps': 'GpsStatus',
                     'plc_gs02': 'GroundStationPlcMonitorStatus',
                     'joystick': 'JoystickMonitorStatus',
                     'loadcell_node': 'Loadcell',
                     'mvlv': 'MvlvStatus',
                     'platform': 'PlatformSensorsMonitor',
                     'recorder_tms570': 'RecorderStatus',
                     'servo': 'ServoStatus',
                     'short_stack': 'ShortStackStatus'}

    for label, message_type in node_messages.iteritems():
      nodes = [node.camel_name for node in net_conf.GetAioNodesByLabel(label)]
      # TODO: Specify thresholds for different boards once thermo
      # characterization is completed.
      self._items_to_check += [
          avionics_checks.AioMonTemperatureChecker(
              for_log, message_type, node,
              name='%s.AioMon.Temperature' % node,
              normal_ranges=check_range.Interval([0, 75]),
              warning_ranges=check_range.Interval([0, 85]))
          for node in nodes]

      self._items_to_check += [
          avionics_checks.AioMonHumidityChecker(
              for_log, message_type, node,
              name='%s.AioMon.Humidity' % node,
              normal_ranges=check_range.Interval([0, 40]),
              warning_ranges=check_range.Interval([0, 70]))
          for node in nodes]

      self._items_to_check += [
          avionics_checks.AioMonBusVoltageChecker(
              for_log, message_type, node,
              name='%s.AioMon.BusVoltage' % node)
          for node in nodes]

      self._items_to_check += [
          avionics_checks.AioMonBusCurrentChecker(
              for_log, message_type, node,
              name='%s.AioMon.BusCurrent' % node)
          for node in nodes]

      self._items_to_check += [
          avionics_checks.AioMonWarningCheck(
              for_log, message_type, node,
              name='%s.AioMon.Warnings' % node)
          for node in nodes]

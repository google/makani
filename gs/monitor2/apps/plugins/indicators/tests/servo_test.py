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

"""Test for web monitor indicators."""

import datetime
import unittest

from makani.avionics.common import pack_avionics_messages
from makani.avionics.common import servo_types
from makani.avionics.firmware.monitors import servo_types as servo_mon_types
from makani.avionics.network import aio_labels
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import servo
from makani.gs.monitor2.apps.receiver import test_util
from makani.lib.python import c_helpers
import mock

SERVO_WARNING_HELPER = c_helpers.EnumHelper('ServoWarning',
                                            servo_types)
SERVO_ERROR_HELPER = c_helpers.EnumHelper('ServoError',
                                          servo_types)
SERVO_MON_WARNING_HELPER = c_helpers.EnumHelper('ServoMonitorWarning',
                                                servo_mon_types)
SERVO_MON_ERROR_HELPER = c_helpers.EnumHelper('ServoMonitorError',
                                              servo_mon_types)
SERVO_LABELS_HELPER = c_helpers.EnumHelper('ServoLabel', aio_labels,
                                           prefix='kServo')


class _FakeDateTime(object):

  def __init__(self, year, month, day, hour, minute, second):
    self._time = datetime.datetime(
        year=year, month=month, day=day,
        hour=hour, minute=minute, second=second)

  def now(self):  # pylint: disable=invalid-name
    return self._time


class TestIndicators(unittest.TestCase):

  # pylint: disable=invalid-name
  def _AddFaults(self, faulty_servos, warning, error, is_servo_mon):
    for short_name in faulty_servos:
      if is_servo_mon:
        self._servo_statuses['Servo' + short_name].servo_mon.flags.warning |= (
            SERVO_MON_WARNING_HELPER.Value(warning))
        self._servo_statuses['Servo' + short_name].servo_mon.flags.error |= (
            SERVO_MON_ERROR_HELPER.Value(error))
      else:
        self._servo_statuses['Servo' + short_name].flags.warning |= (
            SERVO_WARNING_HELPER.Value(warning))
        self._servo_statuses['Servo' + short_name].flags.error |= (
            SERVO_ERROR_HELPER.Value(error))

  def setUp(self):
    self._messages = test_util.SynthesizeMessages(['ServoStatus'], 0)
    # Convert data from ctypes object to Python dicts.
    data = self._messages.Data(convert_to_basic_types=False)
    # Create a new StructTree referencing the data so that we can change
    self._servo_statuses = data['ServoStatus']
    self._tetherdown_servo_statuses = (
        data['filtered'].merge_tether_down.tether_down.servo_statuses)

  def _TestStatusOutput(self, indicator, warning, error, is_servo_mon):
    # Sort results by Servo if there are fewer faulty servos than
    # warning/error categories.
    self._AddFaults(['A4'], warning, error, is_servo_mon)
    text, stoplight = indicator.Filter(self._messages)
    self.assertEqual(stoplight, stoplights.STOPLIGHT_ERROR)
    self.assertEqual(text, 'A4: (WARNING)%s,(ERROR)%s' % (warning, error))

    # Sort results by warning/error categories if there are fewer
    # warning/error categories than faulty servos.
    self._AddFaults(['A4', 'R1', 'E2'], warning, error, is_servo_mon)
    text, stoplight = indicator.Filter(self._messages)
    self.assertEqual(stoplight, stoplights.STOPLIGHT_ERROR)
    self.assertEqual(
        text, 'WARNING %s: A4,E2,R1\nERROR %s: A4,E2,R1' % (warning, error))

    # Add two more faulty servos. Start time cycling.
    self._AddFaults(['A1', 'A5'], warning, error, is_servo_mon)
    fake_datetime = _FakeDateTime(2016, 1, 1, 12, 59, 11)
    with mock.patch('datetime.datetime', fake_datetime):
      text, stoplight = indicator.Filter(self._messages)
      self.assertEqual(text, 'A1:\n[ERROR]\n%s' % error)
    fake_datetime = _FakeDateTime(2016, 1, 1, 12, 59, 12)
    with mock.patch('datetime.datetime', fake_datetime):
      text, stoplight = indicator.Filter(self._messages)
      self.assertEqual(text, 'A4:\n[WARNING]\n%s' % warning)

  def testServoMonStatusIndicator(self):

    indicator = servo.StatusIndicator(common.FULL_COMMS_MODE,
                                      max_chars_per_line=100,
                                      force_time_cycle=False)
    text, stoplight = indicator.Filter(self._messages)
    self.assertEqual(stoplight, stoplights.STOPLIGHT_NORMAL)
    self.assertEqual(text, 'Normal')

    warning = SERVO_MON_WARNING_HELPER.ShortNames()[0]
    error = SERVO_MON_ERROR_HELPER.ShortNames()[0]
    self._TestStatusOutput(indicator, warning, error, True)

  def testStatusIndicator(self):

    indicator = servo.StatusIndicator(common.FULL_COMMS_MODE,
                                      max_chars_per_line=100,
                                      force_time_cycle=False)
    text, stoplight = indicator.Filter(self._messages)
    self.assertEqual(stoplight, stoplights.STOPLIGHT_NORMAL)
    self.assertEqual(text, 'Normal')

    warning = SERVO_WARNING_HELPER.ShortNames()[0]
    error = SERVO_ERROR_HELPER.ShortNames()[0]
    self._TestStatusOutput(indicator, warning, error, False)

  def testTetherDownStatusIndicator(self):
    faulty_servos = ['A1', 'A7']
    for short_name in faulty_servos:
      servo_id = SERVO_LABELS_HELPER.Value(short_name)
      self._tetherdown_servo_statuses[servo_id].state = (
          pack_avionics_messages.kActuatorStateError)

    indicator = servo.StatusIndicator(common.SPARSE_COMMS_MODE)
    text, stoplight = indicator.Filter(self._messages)
    self.assertEqual(stoplight, stoplights.STOPLIGHT_ERROR)
    self.assertIn(text, ['A1:\n[ERROR]\nstatus', 'A7:\n[ERROR]\nstatus'])

if __name__ == '__main__':
  unittest.main()

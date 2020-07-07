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

"""Tests for makani.avionics.motor.motor_client.

This module uses snake case for new function names so that test functions can be
consistent with cmd.Cmd methods without offending Lint.
"""

import copy
import re
import socket
import tempfile
import textwrap
import threading
import time
import unittest

from makani.avionics.common import actuator_types
from makani.avionics.common import cmd_client
from makani.avionics.common import pack_avionics_messages
from makani.avionics.motor import motor_client
from makani.avionics.motor.firmware import flags
from makani.avionics.network import aio_node
from makani.lib.python import test_util
import numpy

_TIMEOUT = 0.01
_EPS32 = numpy.finfo(numpy.float32).eps


class MulticastListener(cmd_client.AioThread):

  def __init__(self, set_state_callback, command_callback, param_callback):
    super(MulticastListener, self).__init__(
        ['kMessageTypeControllerCommand', 'kMessageTypeDynoCommand',
         'kMessageTypeMotorSetParam', 'kMessageTypeDynoMotorSetParam',
         'kMessageTypeMotorSetState', 'kMessageTypeDynoMotorSetState',
         'kMessageTypeMotorGetParam', 'kMessageTypeDynoMotorGetParam'],
        allowed_sources=['kAioNodeControllerA', 'kAioNodeOperator'],
        timeout=_TIMEOUT)
    self._set_state_callback = set_state_callback
    self._command_callback = command_callback
    self._param_callback = param_callback

  def _RunOnce(self):
    try:
      _, header, message = self._client.Recv()
      if header.source == aio_node.kAioNodeOperator:
        if isinstance(message, pack_avionics_messages.MotorSetStateMessage):
          self._set_state_callback(message)
        elif (isinstance(message, pack_avionics_messages.MotorSetParamMessage)
              or isinstance(
                  message, pack_avionics_messages.DynoMotorSetParamMessage)):
          self._param_callback(message)
        elif (isinstance(message, pack_avionics_messages.MotorGetParamMessage)
              or isinstance(
                  message, pack_avionics_messages.DynoMotorGetParamMessage)):
          self._param_callback(message)
        elif isinstance(message, pack_avionics_messages.DynoCommandMessage):
          self._command_callback(message)
      elif header.source == aio_node.kAioNodeControllerA:
        if isinstance(message, pack_avionics_messages.ControllerCommandMessage):
          self._command_callback(message)
    except socket.timeout:
      pass


class FakeMotor(cmd_client.AioThread):

  def __init__(self, nickname):
    self._node_string = motor_client.AioNodeNameFromMotorNickname(nickname)
    self._index = motor_client.MOTORS.index(nickname)
    self._bitmask = 1 << self._index

    self._status = pack_avionics_messages.MotorStatusMessage()
    self._status.motor_status = flags.kMotorStatusInit
    self._status_lock = threading.Lock()

    self.running = False
    self.params = {v: 0.0 for v in motor_client.MOTOR_PARAMS.itervalues()}
    self.torque = 0.0
    self.speed_lower = 0.0
    self.speed_upper = 0.0

    super(FakeMotor, self).__init__(['kMessageTypeMotorStatus',
                                     'kMessageTypeMotorAckParam'],
                                    allowed_sources=[self._node_string],
                                    timeout=_TIMEOUT)

    self._multicast_listener = MulticastListener(
        self._HandleMotorSetStateMessage, self._HandleControllerCommandMessage,
        self._HandleParamMessage)

  def GetParam(self, param_name):
    return self.params[motor_client.MOTOR_PARAMS[param_name]]

  def __enter__(self):
    self.start()
    self._multicast_listener.start()
    return self

  def __exit__(self, *args):
    self._multicast_listener.Exit()
    self._multicast_listener.join()
    self.Exit()
    self.join()

  def GetStatus(self):
    with self._status_lock:
      return copy.copy(self._status)

  def GetError(self):
    with self._status_lock:
      return self._status.motor_error

  def SetError(self, error):
    with self._status_lock:
      self._status.motor_error = error
      self._status.motor_status |= flags.kMotorStatusError

  def SetWarning(self, warning):
    with self._status_lock:
      self._status.motor_warning = warning
      self._status.motor_status |= flags.kMotorStatusError

  def ClearError(self):
    with self._status_lock:
      self._status.motor_error = flags.kMotorErrorNone
      self._status.motor_status &= ~flags.kMotorStatusError

  def _HandleMotorSetStateMessage(self, message):
    if (message.selected_motors & self._bitmask
        and message.command == actuator_types.kActuatorStateCommandArm):
      with self._status_lock:
        self._status.motor_status = flags.kMotorStatusArmed

  def _HandleControllerCommandMessage(self, message):
    self.torque = message.motor_torque[self._index]
    self.speed_lower = message.motor_speed_lower_limit[self._index]
    self.speed_upper = message.motor_speed_upper_limit[self._index]

    self.running = bool(message.motor_command & flags.kMotorCommandRun)

    if message.motor_command & flags.kMotorCommandClearError:
      self.ClearError()

    if message.motor_command & flags.kMotorCommandDisarm:
      with self._status_lock:
        self._status.motor_status &= ~flags.kMotorStatusArmed

  def _HandleParamMessage(self, message):
    if message.selected_motors & self._bitmask:
      if isinstance(message, pack_avionics_messages.MotorSetParamMessage):
        self.params[message.id] = message.value
      ack = pack_avionics_messages.MotorAckParamMessage()
      ack.id = message.id
      ack.value = self.params[message.id]
      self._client.Send(ack, 'kMessageTypeMotorAckParam',
                        self._node_string)

  def _RunOnce(self):
    with self._status_lock:
      self._client.Send(self._status, 'kMessageTypeMotorStatus',
                        self._node_string)
    time.sleep(0.1)


class MotorCommandClientTest(unittest.TestCase):

  def setUp(self):
    super(MotorCommandClientTest, self).setUp()
    self.client = motor_client.MotorCommandClient()
    self.stdout = test_util.StdoutPatch()

  def tearDown(self):
    super(MotorCommandClientTest, self).tearDown()
    with self.stdout:
      self.client.onecmd('quit')

  def assert_eventually_true(self, func):
    num_tries = 30
    for i in xrange(num_tries):
      if func():
        return True
      if i < num_tries - 1:
        time.sleep(0.1)

    self.assertTrue(False)  # pylint: disable=redundant-unittest-assert

  def test_do_set_targets(self):
    with self.stdout:
      self.client.onecmd('set_targets SBO')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*SBO.*')
      self.client.onecmd('quit')

    with self.stdout:
      self.client.onecmd('set_targets SBI PTO')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*SBI.*')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*PTO.*')

  def test_do_set_targets_dyno(self):
    with self.stdout:
      self.client.onecmd('set_targets_dyno SBO')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*SBO.*')
      self.client.onecmd('quit')

    with self.stdout:
      self.client.onecmd('set_targets_dyno SBI PTO')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*SBI.*')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*PTO.*')

  def test_do_arm_fail(self):
    with self.stdout, FakeMotor('SBO'):
      self.client.onecmd('arm')
      self.assertRegexpMatches(self.stdout.Read(),
                               '(?s).*Invalid set of targets.*')

    with self.stdout, FakeMotor('SBO'):
      self.client.onecmd('set_targets SBO')
      self.client.onecmd('arm SBO')
      self.assertRegexpMatches(self.stdout.Read(),
                               '(?s).*Wrong number of arguments.*')

  def test_do_arm_succeed(self):
    with self.stdout, FakeMotor('SBO') as motor:
      self.client.onecmd('set_targets SBO')
      self.client.onecmd('arm')
      self.assertEqual(motor.GetStatus().motor_status,
                       flags.kMotorStatusArmed)
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Successfully armed.*')

  def test_do_arm_multiple_motors(self):
    with self.stdout, FakeMotor('SBI') as sbi, FakeMotor('PTO') as pto:
      self.client.onecmd('set_targets SBI PTO')
      self.client.onecmd('arm')
      self.assertEqual(sbi.GetStatus().motor_status,
                       flags.kMotorStatusArmed)
      self.assertEqual(pto.GetStatus().motor_status,
                       flags.kMotorStatusArmed)
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Successfully armed.*')

  def test_do_disarm(self):
    with self.stdout, FakeMotor('SBO') as motor:
      self.client.onecmd('set_targets SBO')
      self.client.onecmd('arm')
      self.client.onecmd('disarm')
      self.assertEqual(motor.GetStatus().motor_status,
                       flags.kMotorStatusInit)

  def test_do_set_param(self):
    with self.stdout, FakeMotor('SBO') as motor:
      self.client.onecmd('set_param SBO i_kp 3.14')
      self.assertRegexpMatches(self.stdout.Read(),
                               '(?s).*Setting i_kp to 3.14 on SBO.*')
      self.assertAlmostEqual(motor.GetParam('i_kp'), 3.14, places=6)

  def test_do_get_param(self):
    with self.stdout, FakeMotor('SBO'):
      self.client.onecmd('get_param SBO i_kp')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*SBO i_kp: 0[^0-9]*')

  def test_do_run_fail(self):
    with self.stdout, FakeMotor('SBO'):
      self.client.onecmd('run 1 s')
      self.assertRegexpMatches(
          self.stdout.Read(), '(?s).*Invalid set of targets.*')

    with self.stdout, FakeMotor('SBO'):
      self.client.onecmd('set_targets SBO')
      self.client.onecmd('run 1 s')
      self.assertRegexpMatches(
          self.stdout.Read(), '(?s).*Invalid(?s).*status.*')

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_do_run_succeed(self):
    with self.stdout, FakeMotor('SBO') as motor:
      self.client.onecmd('set_targets SBO')
      self.client.onecmd('arm')
      self.client.onecmd('run 100 s')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Running.*')
      self.assert_eventually_true(lambda: motor.running)

  def test_do_stop_fail(self):
    with self.stdout, FakeMotor('SBO'):
      self.client.onecmd('stop')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Not running.*')

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_do_stop_succeed(self):
    with self.stdout, FakeMotor('SBO') as motor:
      self.client.onecmd('set_targets SBO')
      self.client.onecmd('arm')
      self.client.onecmd('run 100 s')
      self.assert_eventually_true(lambda: motor.running)
      self.client.onecmd('stop')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Run stopped.*')
      self.assert_eventually_true(lambda: not motor.running)

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_do_set_torque(self):
    with self.stdout, FakeMotor('SBI'), FakeMotor('SBO') as motor:
      self.client.onecmd('set_targets SBI')
      self.client.onecmd('set_targets_dyno SBO')
      self.client.onecmd('set_speed_limits -3.14 3.14')
      self.client.onecmd('set_torque 3.14')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Torque desired.*')
      self.assert_eventually_true(
          lambda: abs(motor.torque - 3.14) / 3.14 < _EPS32)
      self.assert_eventually_true(
          lambda: abs(motor.speed_lower + 3.14) / 3.14 < _EPS32)
      self.assert_eventually_true(
          lambda: abs(motor.speed_upper - 3.14) / 3.14 < _EPS32)

  def test_do_set_torque_fail(self):
    with self.stdout, FakeMotor('SBO'):
      self.client.onecmd('set_torque 3.14')
      self.assertRegexpMatches(
          self.stdout.Read(), 'No dynos selected. Use "set_targets_dyno".')
      self.client.onecmd('set_targets_dyno SBO')
      self.client.onecmd('set_torque abc')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Invalid argument.*')
      self.client.onecmd('set_torque 3.14')
      self.assertRegexpMatches(
          self.stdout.Read(), 'Omega limits not set. Use "set_speed_limits".')

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_do_set_speed_limits(self):
    with self.stdout, FakeMotor('SBO') as motor:
      self.client.onecmd('set_targets_dyno SBO')
      self.client.onecmd('set_speed_limits -3.14 3.14')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Omega limits set.*')
      self.assert_eventually_true(
          lambda: abs(motor.speed_lower + 3.14) / 3.14 < _EPS32)
      self.assert_eventually_true(
          lambda: abs(motor.speed_upper - 3.14) / 3.14 < _EPS32)

  def test_do_set_speed_limits_fail(self):
    with self.stdout, FakeMotor('SBO'):
      self.client.onecmd('set_targets_dyno SBO')
      self.client.onecmd('set_speed_limits abc 20')
      self.assertRegexpMatches(
          self.stdout.Read(), '(?s).*Invalid argument.*')
      self.client.onecmd('set_speed_limits 22 20')
      self.assertRegexpMatches(
          self.stdout.Read(), '(?s).*Invalid(?s).*i.e. min value.*')

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_do_set_omega(self):
    with self.stdout, FakeMotor('SBO') as motor:
      self.client.onecmd('set_targets SBO')
      self.client.onecmd('set_omega 3.14')
      self.assertRegexpMatches(self.stdout.Read(), 'Omega desired: 3.14')
      self.assert_eventually_true(
          lambda: abs(motor.speed_lower - motor.speed_upper) / 3.14 < _EPS32)
      self.assert_eventually_true(
          lambda: abs(motor.speed_lower - 3.14) / 3.14 < _EPS32)
      self.assert_eventually_true(
          lambda: abs(motor.speed_upper - 3.14) / 3.14 < _EPS32)

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_do_ramp_omega(self):
    with self.stdout, FakeMotor('SBO') as motor:
      self.client.onecmd('set_targets SBO')
      self.client.onecmd('ramp_omega 3.14 0.0')
      self.client.onecmd('arm')
      self.client.onecmd('run 5s')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Ramping.*')
      self.assert_eventually_true(
          lambda: abs(motor.speed_lower - 3.14) / 3.14 < _EPS32)
      self.assert_eventually_true(
          lambda: abs(motor.speed_upper - 3.14) / 3.14 < _EPS32)

      self.client.onecmd('ramp_omega 6.28 0.5')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Ramping.*')
      self.assert_eventually_true(
          lambda: abs(motor.speed_lower - 6.28) / 6.28 < _EPS32)
      self.assert_eventually_true(
          lambda: abs(motor.speed_upper - 6.28) / 6.28 < _EPS32)

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_do_set_command_function_python_succeed(self):
    with tempfile.NamedTemporaryFile(suffix='.py') as python_file:
      data_length = 3*len(motor_client.MOTORS)
      python_file.write(textwrap.dedent("""
          t_step = 0.1
          t_end = 1.0
          def Cmd(t):
            command = [0.0] * %d
            command[0] = 3.14
            command[8] = 3.14
            command[16] = 3.14
            return command""" % data_length))
      python_file.flush()

      with self.stdout, FakeMotor('SBO') as motor:
        self.client.onecmd('set_targets SBO')
        self.client.onecmd('set_command_function ' + python_file.name)
        self.assertRegexpMatches(self.stdout.Read(),
                                 r'(?s).*Using %s to generate command '
                                 r'profile.*' % python_file.name)
        self.client.onecmd('arm')
        self.client.onecmd('run 10s')
        self.assert_eventually_true(
            lambda: abs(motor.torque - 3.14) / 3.14 < _EPS32)
        self.assert_eventually_true(
            lambda: abs(motor.speed_lower - 3.14) / 3.14 < _EPS32)
        self.assert_eventually_true(
            lambda: abs(motor.speed_upper - 3.14) / 3.14 < _EPS32)

  def test_do_set_command_function_python_fail(self):
    with tempfile.NamedTemporaryFile(suffix='.py') as python_file:
      python_file.write('this will raise a syntax error')
      python_file.flush()

      with self.stdout:
        self.client.onecmd('set_targets SBO')
        self.client.onecmd('set_command_function ' + python_file.name)
        self.assertRegexpMatches(self.stdout.Read(),
                                 '(?s).*Generation of lookup table from %s '
                                 'failed.*' % python_file.name)

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_do_set_command_function_text_succeed(self):
    with tempfile.NamedTemporaryFile(suffix='.txt') as text_file:
      text_file.write(textwrap.dedent("""
          0.0 3.14 1 1 1 1 1 1 1 3.14 1 1 1 1 1 1 1 3.14 1 1 1 1 1 1 1
          100 3.14 1 1 1 1 1 1 1 3.14 1 1 1 1 1 1 1 3.14 1 1 1 1 1 1 1"""[1:]))
      text_file.flush()

      with self.stdout, FakeMotor('SBO') as motor:
        self.client.onecmd('set_targets SBO')
        self.client.onecmd('set_command_function ' + text_file.name)
        self.assertRegexpMatches(self.stdout.Read(),
                                 r'(?s).*Using interpolated values from %s '
                                 r'for command profile.*' % text_file.name)
        self.client.onecmd('arm')
        self.client.onecmd('run 10s')
        self.assert_eventually_true(
            lambda: abs(motor.torque - 3.14) / 3.14 < _EPS32)

  def test_do_set_command_function_text_fail(self):
    with tempfile.NamedTemporaryFile(suffix='.txt') as text_file:
      text_file.write('numpy.load will raise ValueError')
      text_file.flush()

      with self.stdout:
        self.client.onecmd('set_targets SBO')
        self.client.onecmd('set_command_function ' + text_file.name)
        self.assertRegexpMatches(self.stdout.Read(),
                                 '(?s).*Invalid input text file: %s.*' %
                                 text_file.name)

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_do_set_command_function_limit_fail(self):
    with tempfile.NamedTemporaryFile(suffix='.txt') as text_file:
      text_file.write(textwrap.dedent("""
          0.0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
          100 2000 1 1 1 1 1 1 1 2000 1 1 1 1 1 1 1 2000 1 1 1 1 1 1 1"""[1:]))
      text_file.flush()

      with self.stdout, FakeMotor('SBO') as motor:
        self.client.onecmd('set_command_function ' + text_file.name)
        self.assertRegexpMatches(self.stdout.Read(),
                                 r'(?s).*Extreme(?s).*outside of '
                                 r'limits \[%f, %f\] detected.*'
                                 % (motor_client.TORQUE_MIN_LIMIT,
                                    motor_client.TORQUE_MAX_LIMIT))
        self.client.onecmd('arm')
        self.client.onecmd('run 1s')
        self.assert_eventually_true(
            lambda: abs(motor.torque) < _EPS32)

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_print_new_errors_asynchronously(self):
    regex = re.compile('(?s).*SBO: kMotorErrorOverSpeed'
                       ' | kMotorErrorOverVoltage.*')

    with FakeMotor('SBO') as motor:
      motor.SetError(flags.kMotorErrorOverVoltage
                     | flags.kMotorErrorOverSpeed)
      with self.stdout:
        self.client.onecmd('set_targets SBO')
        self.assert_eventually_true(lambda: regex.match(self.stdout.Read()))
        other_motors = [m for m in motor_client.MOTORS if m != 'SBO']

        # Make sure we only print the motor with an error.
        self.assertFalse(re.match('(?s).*(%s).*' % '|'.join(other_motors),
                                  self.stdout.Read()))

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_print_new_warnings_asynchronously(self):
    regex = re.compile('(?s).*SBO: kMotorWarningOverTempBoard'
                       ' | kMotorWarningOverTempStatorCore.*')

    with FakeMotor('SBO') as motor:
      motor.SetWarning(flags.kMotorWarningOverTempBoard
                       | flags.kMotorWarningOverTempStatorCore)
      with self.stdout:
        self.client.onecmd('set_targets SBO')
        self.assert_eventually_true(lambda: regex.match(self.stdout.Read()))
        other_motors = [m for m in motor_client.MOTORS if m != 'SBO']

        # Make sure we only print the motor with an error.
        self.assertFalse(re.match('(?s).*(%s).*' % '|'.join(other_motors),
                                  self.stdout.Read()))

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_do_get_errors(self):
    regex = re.compile('(?s).*SBO: kMotorErrorOverSpeed'
                       ' | kMotorErrorOverVoltage.*')

    with FakeMotor('SBO') as motor:
      motor.SetError(flags.kMotorErrorOverVoltage
                     | flags.kMotorErrorOverSpeed)
      with self.stdout:
        self.client.onecmd('set_targets SBO')
        self.assert_eventually_true(lambda: regex.match(self.stdout.Read()))
      with self.stdout:  # Reset stdout contents.
        self.client.onecmd('get_errors')
        self.assertRegexpMatches(self.stdout.Read(), regex)

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_do_get_warnings(self):
    regex = re.compile('(?s).*SBO: kMotorWarningOverTempBoard'
                       ' | kMotorWarningOverTempStatorCore.*')

    with FakeMotor('SBO') as motor:
      motor.SetWarning(flags.kMotorWarningOverTempBoard
                       | flags.kMotorWarningOverTempStatorCore)
      with self.stdout:
        self.client.onecmd('set_targets SBO')
        self.assert_eventually_true(lambda: regex.match(self.stdout.Read()))
      with self.stdout:  # Reset stdout contents.
        self.client.onecmd('get_errors')
        self.assertRegexpMatches(self.stdout.Read(), regex)

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_do_clear_errors(self):
    regex = re.compile('(?s).*SBO: kMotorErrorOverVoltage.*')

    with self.stdout, FakeMotor('SBO') as motor:
      motor.SetError(flags.kMotorErrorOverVoltage)
      self.client.onecmd('set_targets SBO')
      self.assert_eventually_true(lambda: regex.match(self.stdout.Read()))
      self.client.onecmd('clear_errors')
      self.assert_eventually_true(
          lambda: motor.GetError() == flags.kMotorErrorNone)

  def test_do_source_fail(self):
    with tempfile.NamedTemporaryFile() as source_file:
      source_file.write(textwrap.dedent("""
          set_targets SBO
          arm
          run 100 s"""[1:]))
      source_file.flush()

      with self.stdout, FakeMotor('SBO'):
        self.client.onecmd('source ' + source_file.name)
        regex = re.compile(
            '(?s).*Only "set_param"-like commands.*')
        self.assertRegexpMatches(self.stdout.Read(), regex)

  def test_do_source_succeed(self):
    with tempfile.NamedTemporaryFile() as source_file:
      source_file.write(textwrap.dedent("""
          set_param SBO i_kp 3.14
          set_param SBO cos_offset 0.2

          # This is a comment.
          set_param SBO iq_lower_limit -1e-3
          set_param SBO iq_upper_limit 245""")[1:])
      source_file.flush()

      with self.stdout, FakeMotor('SBO') as motor:
        self.client.onecmd('source ' + source_file.name)
        self.assertAlmostEqual(motor.GetParam('i_kp'), 3.14, places=6)
        self.assertAlmostEqual(motor.GetParam('cos_offset'), 0.2, places=6)
        self.assertAlmostEqual(motor.GetParam('iq_lower_limit'),
                               -1e-3, places=6)
        self.assertAlmostEqual(motor.GetParam('iq_upper_limit'),
                               245, places=6)

  def test_do_source_track_errors(self):
    with tempfile.NamedTemporaryFile() as source_file:
      source_file.write(textwrap.dedent("""
          set_param SBO i_kp 3.14
          set_param SBO foo 0.2
          set_param SBO omega_kp 0.1
          set_param SBO bar 0.5""")[1:])
      source_file.flush()

      with self.stdout, FakeMotor('SBO'):
        self.client.onecmd('source ' + source_file.name)
        self.assertRegexpMatches(
            self.stdout.Read(),
            '(?s).*Errors encountered.*Line 2.*foo.*Line 4.*bar.*')

  @unittest.skipIf(socket.gethostname().startswith('jenkins-'),
                   'This test is flaky when run on GCE.')
  def test_stop_running_on_error(self):
    regex = re.compile('(?s).*SBO: kMotorErrorOverVoltage.*')

    with FakeMotor('SBO') as motor:
      with self.stdout:
        self.client.onecmd('set_targets SBO')
        self.client.onecmd('arm')
        self.client.onecmd('run 1000 s')

        motor.SetError(flags.kMotorErrorOverVoltage)
        self.assert_eventually_true(lambda: regex.match(self.stdout.Read()))
        self.assert_eventually_true(lambda: not motor.running)


if __name__ == '__main__':
  unittest.main()

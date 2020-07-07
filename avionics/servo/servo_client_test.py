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

"""Tests for makani.avionics.servo.servo_client.

This module uses snake case for new function names so that test functions can be
consistent with cmd.Cmd methods without offending Lint.
"""

import copy
import socket
import tempfile
import textwrap
import threading
import time
import unittest

from makani.avionics.common import actuator_types
from makani.avionics.common import cmd_client
from makani.avionics.common import pack_avionics_messages
from makani.avionics.servo import servo_client
from makani.lib.python import test_util
import numpy

_TIMEOUT = 0.01
_EPS32 = numpy.finfo(numpy.float32).eps


class MulticastListener(cmd_client.AioThread):

  def __init__(self, set_state_callback, command_callback, param_callback):
    super(MulticastListener, self).__init__(
        ['kMessageTypeControllerCommand', 'kMessageTypeServoSetState',
         'kMessageTypeServoSetParam', 'kMessageTypeServoGetParam'],
        allowed_sources=['kAioNodeControllerA', 'kAioNodeOperator'],
        timeout=_TIMEOUT)
    self._set_state_callback = set_state_callback
    self._command_callback = command_callback
    self._param_callback = param_callback

  def _RunOnce(self):
    try:
      _, unused_header, message = self._client.Recv()
      if isinstance(message, pack_avionics_messages.ServoSetStateMessage):
        self._set_state_callback(message)
      elif isinstance(message, pack_avionics_messages.ControllerCommandMessage):
        self._command_callback(message)
      elif isinstance(message, pack_avionics_messages.ServoSetParamMessage):
        self._param_callback(message)
      elif isinstance(message, pack_avionics_messages.ServoGetParamMessage):
        self._param_callback(message)
    except socket.timeout:
      pass


class FakeServo(cmd_client.AioThread):

  def __init__(self, nickname):
    self._node_string = servo_client.AioNodeNameFromServoNickname(nickname)
    self._index = servo_client.servo_label_helper.Value(nickname)
    self._bitmask = 1 << self._index

    self._status = pack_avionics_messages.ServoStatusMessage()
    self._status.state = actuator_types.kActuatorStateReady
    self._status_lock = threading.Lock()

    self.params = {v: 0 for v in servo_client.SERVO_PARAMS}
    self.angle = 0

    super(FakeServo, self).__init__(['kMessageTypeServoStatus',
                                     'kMessageTypeServoAckParam'],
                                    allowed_sources=[self._node_string],
                                    timeout=_TIMEOUT)

    self._multicast_listener = MulticastListener(
        self._HandleServoSetStateMessage, self._HandleControllerCommandMessage,
        self._HandleParamMessage)

  def GetParam(self, param_name):
    return self.params[param_name]

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
      return self._status.servo_error

  def SetError(self, error):
    with self._status_lock:
      self._status.state = actuator_types.kActuatorStateError
      self._status.flags.error |= error

  def ClearError(self):
    with self._status_lock:
      if self._status.state == actuator_types.kActuatorStateError:
        self._status.state = actuator_types.kActuatorStateReady
      self._status.flags.error = 0

  def _HandleServoSetStateMessage(self, message):
    if message.selected_servos & self._bitmask:
      with self._status_lock:
        if self._status.flags.error == 0:
          if (message.state_command
              == actuator_types.kActuatorStateCommandArm):
            self._status.state = actuator_types.kActuatorStateArmed
          elif (message.state_command
                == actuator_types.kActuatorStateCommandDisarm):
            self._status.state = actuator_types.kActuatorStateReady
        if (message.state_command
            == actuator_types.kActuatorStateCommandClearErrors):
          self.ClearError()

  def _HandleControllerCommandMessage(self, message):
    self.angle = message.servo_angle[self._index]
    self._timeout = time.time() + 0.2
    if self._status.state == actuator_types.kActuatorStateArmed:
      with self._status_lock:
        self._status.state = actuator_types.kActuatorStateRunning

  def _HandleParamMessage(self, message):
    if message.selected_servos & self._bitmask:
      param_name = servo_client.r22_param_helper.ShortName(message.param)
      if isinstance(message, pack_avionics_messages.ServoSetParamMessage):
        self.params[param_name] = message.value
      ack = pack_avionics_messages.ServoAckParamMessage()
      ack.param = message.param
      ack.value = self.params[param_name]
      self._client.Send(ack, 'kMessageTypeServoAckParam',
                        self._node_string)

  def _RunOnce(self):
    with self._status_lock:
      if (self._status.state == actuator_types.kActuatorStateRunning
          and self._timeout <= time.time()):
        self._status.state = actuator_types.kActuatorStateReady

      self._client.Send(self._status, 'kMessageTypeServoStatus',
                        self._node_string)

    time.sleep(0.1)


class ServoCommandClientTest(unittest.TestCase):

  def setUp(self):
    self.client = servo_client.ServoCommandClient()
    self.stdout = test_util.StdoutPatch()

  def tearDown(self):
    with self.stdout:
      self.client.onecmd('quit')

  def assert_eventually_true(self, func):
    num_tries = 20
    for i in xrange(num_tries):
      if func():
        return True
      if i < num_tries - 1:
        time.sleep(0.1)

    self.assertTrue(False)  # pylint: disable=redundant-unittest-assert

  def assert_state(self, servo, state):
    self.assert_eventually_true(
        lambda: servo.GetStatus().state == state)

  def test_do_set_targets(self):
    with self.stdout:
      self.client.onecmd('set_targets A2')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*A2.*')
      self.client.onecmd('quit')

    with self.stdout:
      self.client.onecmd('set_targets E1 E2')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*E1.*')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*E2.*')

  def test_do_arm_fail(self):
    with self.stdout, FakeServo('A2'):
      self.client.onecmd('arm')
      self.assertRegexpMatches(self.stdout.Read(),
                               '(?s).*No target servos set.*')

    with self.stdout, FakeServo('A2'):
      self.client.onecmd('set_targets A2')
      self.client.onecmd('arm A2')
      self.assertRegexpMatches(self.stdout.Read(),
                               '(?s).*Wrong number of arguments.*')

  def test_do_arm_succeed(self):
    with self.stdout, FakeServo('A2') as servo:
      self.client.onecmd('set_targets A2')
      self.client.onecmd('arm')
      self.assertEqual(servo.GetStatus().state,
                       actuator_types.kActuatorStateArmed)
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Successfully armed.*')

  def test_do_arm_multiple_servos(self):
    with self.stdout, FakeServo('E1') as e1, FakeServo('E2') as e2:
      self.client.onecmd('set_targets E1 E2')
      self.client.onecmd('arm')
      self.assertEqual(e1.GetStatus().state, actuator_types.kActuatorStateArmed)
      self.assertEqual(e2.GetStatus().state, actuator_types.kActuatorStateArmed)
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Successfully armed.*')

  def test_do_disarm(self):
    with self.stdout, FakeServo('A2') as servo:
      self.client.onecmd('set_targets A2')
      self.client.onecmd('arm')
      self.client.onecmd('disarm')
      self.assert_state(servo, actuator_types.kActuatorStateReady)

  def test_do_set_param(self):
    with self.stdout, FakeServo('A2') as servo:
      self.client.onecmd('set_param A2 ProfileAcceleration 200')
      self.assertRegexpMatches(
          self.stdout.Read(),
          '(?s).*Setting ProfileAcceleration to 200 on A2.*')
      self.assertAlmostEqual(servo.GetParam('ProfileAcceleration'),
                             200, places=6)

  def test_do_get_param(self):
    with self.stdout, FakeServo('A2'):
      self.client.onecmd('get_param A2 ProfileAcceleration')
      self.assertRegexpMatches(self.stdout.Read(),
                               '(?s).*A2 ProfileAcceleration: 0[^0-9]*')

  def test_do_run_fail(self):
    with self.stdout, FakeServo('A2'):
      self.client.onecmd('move 45')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Invalid state.*')

    with self.stdout, FakeServo('A2'):
      self.client.onecmd('set_targets A2')
      self.client.onecmd('move 0')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Invalid state.*')

    with self.stdout, FakeServo('A2'):
      self.client.onecmd('set_targets A2')
      self.client.onecmd('move_pattern 45:1 20:2')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Invalid state.*')

  def test_do_run_succeed(self):
    with self.stdout, FakeServo('A2') as servo:
      self.client.onecmd('set_targets A2')
      self.client.onecmd('arm')
      self.client.onecmd('move 45')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Running.*')
      self.assert_state(servo, actuator_types.kActuatorStateRunning)

  def test_do_move_pattern_succeed(self):
    with self.stdout, FakeServo('A2') as servo:
      self.client.onecmd('set_targets A2')
      self.client.onecmd('arm')
      self.client.onecmd('move_pattern 45:1.5 -20:2')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Running.*')
      self.assert_state(servo, actuator_types.kActuatorStateRunning)

  def test_do_move_pattern_eval_succeed(self):
    with self.stdout, FakeServo('A2') as servo:
      self.client.onecmd('set_targets A2')
      self.client.onecmd('arm')
      self.client.onecmd('move_pattern [(10*i, i) for i in range(5)]')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Running.*')
      self.assert_state(servo, actuator_types.kActuatorStateRunning)

  def test_do_stop_fail(self):
    with self.stdout, FakeServo('A2'):
      self.client.onecmd('stop')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Not running.*')

  def test_do_stop_succeed(self):
    with self.stdout, FakeServo('A2') as servo:
      self.client.onecmd('set_targets A2')
      self.client.onecmd('arm')
      self.client.onecmd('move 45')
      self.assert_state(servo, actuator_types.kActuatorStateRunning)
      self.client.onecmd('stop')
      self.assertRegexpMatches(self.stdout.Read(), '(?s).*Run stopped.*')
      self.assert_state(servo, actuator_types.kActuatorStateReady)

  def test_do_source_fail(self):
    with tempfile.NamedTemporaryFile() as source_file:
      source_file.write(textwrap.dedent("""
          set_targets A2
          arm
          run 100 s"""[1:]))
      source_file.flush()

      with self.stdout, FakeServo('A2'):
        self.client.onecmd('source ' + source_file.name)
        regex = '(?s).*Only "set_param"-like commands.*'
        self.assertRegexpMatches(
            self.stdout.Read(), regex)

  def test_do_source_succeed(self):
    with tempfile.NamedTemporaryFile() as source_file:
      source_file.write(textwrap.dedent("""
          set_param A2 TrajectoryMaxJerk 10000
          set_param A2 ProfileAcceleration 200

          # This is a comment.
          set_param A2 ProfileVelocity 150""")[1:])
      source_file.flush()

      with self.stdout, FakeServo('A2') as servo:
        self.client.onecmd('source ' + source_file.name)
        self.assertAlmostEqual(servo.GetParam('TrajectoryMaxJerk'),
                               10000, places=6)
        self.assertAlmostEqual(servo.GetParam('ProfileAcceleration'),
                               200, places=6)
        self.assertAlmostEqual(servo.GetParam('ProfileVelocity'),
                               150, places=6)

  def test_do_source_track_errors(self):
    with tempfile.NamedTemporaryFile() as source_file:
      source_file.write(textwrap.dedent("""
          set_param A2 TrajectoryMaxJerk 10000
          set_param A2 foo 0.2
          set_param A2 ProfileAcceleration 200
          set_param A2 bar 0.5""")[1:])
      source_file.flush()

      with self.stdout, FakeServo('A2'):
        self.client.onecmd('source ' + source_file.name)
        self.assertRegexpMatches(
            self.stdout.Read(),
            '(?s).*Errors encountered.*Line 2.*foo.*Line 4.*bar.*')

  def test_stop_running_on_error(self):

    with FakeServo('A2') as servo:
      with self.stdout:
        self.client.onecmd('set_targets A2')
        self.client.onecmd('arm')
        self.client.onecmd('move 45')

        servo.SetError(1)
        self.assert_state(servo, actuator_types.kActuatorStateError)


if __name__ == '__main__':
  unittest.main()

#!/usr/bin/python
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

"""Command line client for controlling servos."""

import math
import numbers
import os
import socket
import threading
import time

from makani.avionics.common import actuator_types
from makani.avionics.common import aio
from makani.avionics.common import cmd_client
from makani.avionics.common import pack_avionics_messages
from makani.avionics.common import safety_codes
from makani.avionics.common import servo_types as servo_flags
from makani.avionics.network import aio_labels
from makani.avionics.network import aio_node
from makani.avionics.network import message_type
from makani.avionics.servo.firmware import r22_param
from makani.lib.python import c_helpers

aio_node_helper = c_helpers.EnumHelper('AioNode', aio_node)
r22_param_helper = c_helpers.EnumHelper('R22Parameter', r22_param,
                                        prefix='kR22Param')
servo_label_helper = c_helpers.EnumHelper('ServoLabel', aio_labels,
                                          prefix='kServo')
servo_error_helper = c_helpers.EnumHelper('ServoError', servo_flags)


def BuildServoParamList():
  """Builds a list of servo param names."""
  return r22_param_helper.ShortNames()

# Constants.
SERVOS = servo_label_helper.ShortNames()
OPERATOR = 'kAioNodeOperator'
CONTROLLER = 'kAioNodeControllerA'
SERVO_PARAMS = BuildServoParamList()
ACTUATOR_STATE_NAMES = {val: key for key, val in actuator_types.__dict__.items()
                        if key.startswith('kActuatorState')}


class ServoClientError(cmd_client.WingClientError):
  pass


def ServosAsBits(servo_list):
  """Returns a bitmask describing the servos in `servo_list`."""
  return sum(1 << servo_label_helper.Value(servo) for servo in servo_list)


def AioNodeNameFromServoNickname(servo):
  """Returns AIO node name for the specified servo."""
  for s in SERVOS:
    if s.lower() == servo.lower():
      return 'kAioNodeServo' + s


class ServoCommandClient(cmd_client.WingCommandClient):
  """Command line client for running M600 servos."""

  prompt = '(servo_client) '
  _NUM_RETRIES = 10

  def __init__(self, *args, **kwargs):
    cmd_client.WingCommandClient.__init__(self, *args, **kwargs)

    self._servos_selected = set()
    self._listener = None
    self._runner = None

    self._set_state_aio_client = aio.AioClient(['kMessageTypeServoSetState'],
                                               timeout=0.1)
    self._set_param_aio_client = aio.AioClient(['kMessageTypeServoSetParam'],
                                               timeout=0.1)
    # The long range radio requires at least 2x160 ms for a complete command-
    # response cycle.
    self._ack_param_aio_client = aio.AioClient(['kMessageTypeServoAckParam'],
                                               timeout=0.35)
    self._get_param_aio_client = aio.AioClient(['kMessageTypeServoGetParam'],
                                               timeout=0.1)

  def TryStopThreads(self):
    if self._runner:
      self._runner.TryStop()
    if self._listener:
      self._listener.TryStop()

  def _CheckState(self, valid_states):
    if not self._listener:
      state = actuator_types.kActuatorStateInit
    else:
      state = self._listener.GetMostRestrictiveServoState()

    if state not in valid_states:
      raise ServoClientError('Invalid state: ' + ACTUATOR_STATE_NAMES[state])
    return True

  def _CheckServosSelected(self):
    if not self._servos_selected:
      raise ServoClientError('No target servos set.')
    return True

  @cmd_client.Command()
  def do_set_targets(self, line):  # pylint: disable=invalid-name
    """Sets target servos, e.g. "set_targets A2 A4"."""
    self._CheckState([actuator_types.kActuatorStateInit])
    servos_selected, _ = cmd_client.SelectArgs(
        line.split(), SERVOS, require_some=True, require_all=True,
        select_all=True, require_one=False)

    print 'Servos selected: [%s]' % ', '.join(sorted(list(servos_selected)))
    self._servos_selected = servos_selected

    self.TryStopThreads()
    self._runner = Runner(servos_selected)
    self._listener = Listener(self._servos_selected, self._runner.StopRun)

  def complete_set_targets(self, text, *unused_args):  # pylint: disable=invalid-name
    return self._CompleteArg(text, sorted(SERVOS) + ['All'])

  @cmd_client.Command(num_args=0)
  def do_arm(self, unused_line):  # pylint: disable=invalid-name
    """Arms the selected servos."""
    self._CheckState([actuator_types.kActuatorStateInit])
    self._CheckServosSelected()
    set_state_msg = pack_avionics_messages.ServoSetStateMessage()
    (set_state_msg
     .state_command) = actuator_types.kActuatorStateCommandArm
    (set_state_msg
     .servo_arming_signal) = safety_codes.SERVO_ARMING_SIGNAL
    print 'Arming.'

    for _ in xrange(self._NUM_RETRIES):
      set_state_msg.selected_servos = ServosAsBits(
          self._listener.GetUnarmedServos())
      self._set_state_aio_client.Send(
          set_state_msg, 'kMessageTypeServoSetState', OPERATOR)
      time.sleep(0.1)
      if self._listener.AllServosArmed():
        print 'Successfully armed.'
        return

    raise ServoClientError('Failed to arm.')

  @cmd_client.Command(num_args=0)
  def do_status(self, unused_line):  # pylint: disable=invalid-name
    """Checks the state of the selected servos."""
    self._CheckServosSelected()
    servo_states = self._listener.GetServoStates()
    for servo in sorted(servo_states.keys()):
      # I would like to use EnumHelper here but it has problems when one enum
      # name is the prefix of another name.
      if servo_states[servo] == actuator_types.kActuatorStateInit:
        state_string = 'Init'
      elif servo_states[servo] == actuator_types.kActuatorStateReady:
        state_string = 'Ready'
      elif servo_states[servo] == actuator_types.kActuatorStateArmed:
        state_string = 'Armed'
      elif servo_states[servo] == actuator_types.kActuatorStateRunning:
        state_string = 'Running'
      elif servo_states[servo] == actuator_types.kActuatorStateError:
        state_string = 'Error'
      else:
        state_string = 'Unresponsive'
      print '%s: %s' % (servo, state_string)

  @cmd_client.Command(num_args=3)
  def do_set_param(self, line):  # pylint: disable=invalid-name
    """Sets a param for a specified servo, e.g. "set_param R1 XYZ 150"."""
    self._CheckState([actuator_types.kActuatorStateInit,
                      actuator_types.kActuatorStateError])
    servos, args = cmd_client.SelectArgs(line.split(), SERVOS,
                                         require_some=True, select_all=True)
    param, args = cmd_client.SelectArgs(args, SERVO_PARAMS, require_one=True,
                                        select_all=False)
    try:
      value = int(args[0], 0)
    except ValueError:
      raise ServoClientError('Invalid value: "%s".' % args[0])

    message = pack_avionics_messages.ServoSetParamMessage()
    message.param = r22_param_helper.Value(param)
    message.value = value

    for target in servos:
      print 'Setting %s to %g on %s...' % (param, value, target)
      message.selected_servos = ServosAsBits([target])
      ack_received = False
      for _ in xrange(self._NUM_RETRIES):
        self._set_param_aio_client.Send(
            message, 'kMessageTypeServoSetParam', OPERATOR)
        try:
          _, header, ack = self._ack_param_aio_client.Recv()
          if (header.source == aio_node_helper.Value(
              AioNodeNameFromServoNickname(target))
              and header.type == message_type.kMessageTypeServoAckParam
              and ack.param == message.param):
            if ack.value == message.value:
              ack_received = True
              print '%s %s: %g' % (target, param, ack.value)
              break
            else:
              print 'Got response with incorrect value.'
        except socket.timeout:
          pass

      if not ack_received:
        raise ServoClientError('Failed to get %s from %s; giving up.' %
                               (param, target))

  def complete_set_param(self, text, line, *unused_args):  # pylint: disable=invalid-name
    arg_number = len(line.split())
    if not text:
      arg_number += 1

    if arg_number == 2:
      return self._CompleteArg(text, sorted(SERVOS) + ['All'])
    elif arg_number == 3:
      return self._CompleteArg(text, sorted(SERVO_PARAMS))
    else:
      return []

  @cmd_client.Command()
  def do_get_param(self, line):  # pylint: disable=invalid-name
    """Gets a param value for a servo, e.g. "get_param R1 VelocityVp"."""
    s, args = cmd_client.SelectArgs(line.split(), SERVOS,
                                    require_some=True, select_all=True)
    param, _ = cmd_client.SelectArgs(args, SERVO_PARAMS, require_one=True,
                                     select_all=False)

    msg = pack_avionics_messages.ServoGetParamMessage()
    msg.param = r22_param_helper.Value(param)

    for target in s:
      print 'Getting %s from %s...' % (param, target)
      msg.selected_servos = ServosAsBits([target])
      success = False
      for _ in xrange(self._NUM_RETRIES):
        self._get_param_aio_client.Send(
            msg, 'kMessageTypeServoGetParam', OPERATOR)
        try:
          _, header, ack = self._ack_param_aio_client.Recv()
          if (header.source == aio_node_helper.Value(
              AioNodeNameFromServoNickname(target))
              and header.type == message_type.kMessageTypeServoAckParam
              and ack.param == msg.param):
            print '%s %s: 0x%X' % (target, param, ack.value)
            success = True
            break
        except socket.timeout:
          pass

      if not success:
        raise ServoClientError('Failed to get %s from %s; giving up.' %
                               (param, target))

  complete_get_param = complete_set_param

  @cmd_client.Command(num_args=1)
  def do_move(self, line):  # pylint: disable=invalid-name
    """Moves the selected servos.

    Specify an angle in degrees.  E.g. "move 45" or "move -10.5".

    Args:
      line: Command to this function.
    Raises:
      ServoClientError: If the command is invalid or not currently usable.
    """
    self._CheckState([actuator_types.kActuatorStateArmed,
                      actuator_types.kActuatorStateRunning])
    self._CheckServosSelected()
    try:
      angle = float(line)
    except ValueError:
      raise ServoClientError('Invalid angle: \'%s\'' % line)

    if not self._listener.AllServosArmed():
      raise ServoClientError('Servos not armed.')

    self._runner.StartRun([(angle, 1)])
    print 'Running...'

  @cmd_client.Command()
  def do_move_pattern(self, line):  # pylint: disable=invalid-name
    """Move the selected servos in a specified pattern.

    Provide a sequence of arguments where each argument contains a colon (:).
    The number before the colon should be a destination position, and after the
    colon a time in seconds to hold at that position.  The pattern will repeat
    until another move or stop is issued.  E.g. "move_pattern 45:10 -45:5".

    Alternatively, provide python code which evaluates to a list containing
    tuples of (angle, duration).  This list will be run in sequence as above.

    Args:
      line: Command to this function.
    Raises:
      ServoClientError: If the command is invalid or not currently usable.
    """
    self._CheckState([actuator_types.kActuatorStateArmed,
                      actuator_types.kActuatorStateRunning])
    self._CheckServosSelected()
    pattern = []
    try:
      for token in line.split():
        dest, interval = token.split(':')
        pattern.append((float(dest), float(interval)))
    except ValueError:
      try:
        pattern = eval(line)  # pylint: disable=eval-used
      except:
        raise ServoClientError('Invalid move_pattern specification: \'%s\''
                               % line)
      try:
        for item in pattern:
          a, b = item
          if (not isinstance(a, numbers.Number) or
              not isinstance(b, numbers.Number)):
            raise ServoClientError('Non-numeric value in move pattern.')
      except (TypeError, ValueError):
        raise ServoClientError('The move_pattern specification doesn\'t '
                               'produce a list: \'%s\'' % line)
    if not pattern:
      raise ServoClientError('No arguments specified for move_pattern.')

    self._runner.StartRun(pattern)
    print 'Running...'

  @cmd_client.Command()
  def do_move_file(self, filename):  # pylint: disable=invalid-name
    """Move the selected servos in a specified pattern.

    Provide the filename of a file containing, on each line, one
    timestamp and one angular position, separated by whitespace.
    Note: The last angular position in the file is ignored; instead
    the pattern cycles back to the beginning of the file.

    Args:
      filename: File from which to read a waveform of servo positions.
    Raises:
      ServoClientError: If the command is invalid or not currently usable.

    """
    self._CheckState([actuator_types.kActuatorStateArmed,
                      actuator_types.kActuatorStateRunning])
    self._CheckServosSelected()

    try:
      pattern = []
      with open(filename) as f:
        line = f.readline()
        prev_timestamp, prev_dest = line.split()
        prev_timestamp = float(prev_timestamp)
        prev_dest = float(prev_dest)
        for line in f:
          timestamp, dest = line.split()
          timestamp = float(timestamp)
          dest = float(dest)
          interval = timestamp - prev_timestamp
          if interval <= 0.0:
            raise ServoClientError('All time step intervals must be positive.')
          pattern.append((float(prev_dest), float(interval)))
          prev_timestamp = timestamp
          prev_dest = dest
    except ValueError:
      if not filename:
        raise ServoClientError('No arguments specified for move_pattern.')
      raise ServoClientError('Invalid entry in waveform file: \'%s\'' % line)
    except IOError, e:
      raise ServoClientError('Couldn''t open %s: %s.' % (filename,
                                                         os.strerror(e.errno)))

    self._runner.StartRun(pattern)
    print 'Running...'

  @cmd_client.Command(num_args=0)
  def do_stop(self, unused_line):  # pylint: disable=invalid-name
    """Stops the servos."""
    if not self._runner or not self._runner.IsRunning():
      raise ServoClientError('Not running.')
    else:
      self._runner.StopRun()
      print 'Run stopped.'

  @cmd_client.Command(num_args=0)
  def do_clear_errors(self, unused_line):  # pylint: disable=invalid-name
    self._CheckState([actuator_types.kActuatorStateInit,
                      actuator_types.kActuatorStateError])
    self._CheckServosSelected()

    print 'Clearing errors.'

    set_state_msg = pack_avionics_messages.ServoSetStateMessage()
    (set_state_msg
     .state_command) = actuator_types.kActuatorStateCommandClearErrors

    for _ in xrange(self._NUM_RETRIES):
      set_state_msg.selected_servos = ServosAsBits(self._listener.GetServos())
      self._set_state_aio_client.Send(
          set_state_msg, 'kMessageTypeServoSetState', OPERATOR)
      time.sleep(0.1)
      if self._listener.AllServosErrorFree():
        print 'Successfully cleared errors.'
        return

    raise ServoClientError('Failed to clear errors.')

  @cmd_client.Command(num_args=0)
  def do_disarm(self, unused_line):  # pylint: disable=invalid-name
    """Disarms the servos."""
    self._CheckServosSelected()
    print 'Disarming.'

    set_state_msg = pack_avionics_messages.ServoSetStateMessage()
    (set_state_msg
     .state_command) = actuator_types.kActuatorStateCommandDisarm
    (set_state_msg
     .servo_arming_signal) = safety_codes.SERVO_DISARM_SIGNAL

    for _ in xrange(self._NUM_RETRIES):
      set_state_msg.selected_servos = ServosAsBits(self._listener.GetServos())
      self._set_state_aio_client.Send(
          set_state_msg, 'kMessageTypeServoSetState', OPERATOR)
      time.sleep(0.1)
      if self._listener.AllServosDisarmed():
        print 'Successfully disarmed.'
        return

    raise ServoClientError('Failed to disarm.')

  @cmd_client.Command()
  def do_get_errors(self, unused_line):  # pylint: disable=invalid-name
    self._CheckServosSelected()
    self._listener.PrintErrors()


class Listener(cmd_client.AioThread):
  """Continuously listens to ServoStatusMessages."""

  def __init__(self, servos, error_callback):
    self._servos = servos

    self._errors = {m: 0 for m in SERVOS}
    self._error_lock = threading.Lock()

    self._servo_state = {m: None
                         for m in self._servos}
    self._servo_state_lock = threading.Lock()

    self._servo_sources = {aio.aio_node_helper.Value(
        AioNodeNameFromServoNickname(src)): src for src in self._servos}
    self._error_callback = error_callback

    sources = [AioNodeNameFromServoNickname(s) for s in self._servos]
    super(Listener, self).__init__(['kMessageTypeServoStatus'],
                                   allowed_sources=sources, timeout=0.1)
    self.start()

  def GetServoStates(self):
    """Returns a copy of the servo state dictionary."""
    with self._servo_state_lock:
      return self._servo_state.copy()

  def GetMostRestrictiveServoState(self):
    """Returns the most restrictive state across all selected servos."""
    with self._servo_state_lock:
      servo_states = self._servo_state.values()

    merged_state = set()
    for state in servo_states:
      merged_state.add(state)
    if actuator_types.kActuatorStateRunning in merged_state:
      return actuator_types.kActuatorStateRunning
    elif actuator_types.kActuatorStateArmed in merged_state:
      return actuator_types.kActuatorStateArmed
    elif actuator_types.kActuatorStateError in merged_state:
      return actuator_types.kActuatorStateError

    return actuator_types.kActuatorStateInit

  def AllServosArmed(self):
    with self._servo_state_lock:
      servo_states = self._servo_state.values()
    return all(x == actuator_types.kActuatorStateArmed
               or x == actuator_types.kActuatorStateRunning
               for x in servo_states)

  def AllServosDisarmed(self):
    with self._servo_state_lock:
      servo_states = self._servo_state.values()
    return all(x != actuator_types.kActuatorStateArmed
               and x != actuator_types.kActuatorStateRunning
               for x in servo_states)

  def AllServosErrorFree(self):
    with self._servo_state_lock:
      servo_states = self._servo_state.values()
    return all(x != actuator_types.kActuatorStateError for x in servo_states)

  def GetUnarmedServos(self):
    with self._servo_state_lock:
      return [m for m, s in self._servo_state.iteritems()
              if s != actuator_types.kActuatorStateArmed
              and s != actuator_types.kActuatorStateRunning]

  def GetServos(self):
    return [m for m, _ in self._servo_state.iteritems()]

  def PrintErrors(self):
    with self._error_lock:
      if any(self._errors.itervalues()):
        print 'Errors:'
        for servo, error in self._errors.iteritems():
          if error:
            errors = [name for name in servo_error_helper.ShortNames()
                      if servo_error_helper.Value(name) & error]
            print '%s: %s' % (servo, ' | '.join(errors))
      else:
        print 'No errors.'

  def _RunOnce(self):
    try:
      _, header, msg = self._client.Recv()
      servo = self._servo_sources[header.source]

      with self._error_lock:
        self._errors[servo] = msg.flags.error

      # Invoke error callback after giving up self._error_lock just in case.
      if msg.flags.error:
        self._error_callback()

      # Note servo states.
      with self._servo_state_lock:
        self._servo_state[servo] = msg.state

    except socket.timeout:
      pass


class Runner(cmd_client.AioThread):
  """Continuously sends ControllerCommandMessages."""

  def __init__(self, servos):
    self._servos = servos

    self._command = pack_avionics_messages.ControllerCommandMessage()
    self._command_lock = threading.Lock()
    self._running = False

    self._timeout = 0
    self._index = 0
    self._pattern = [(0., 1.)]

    super(Runner, self).__init__(['kMessageTypeControllerCommand'])
    self.start()

  def IsRunning(self):
    return self._running

  def StartRun(self, pattern):
    self._running = True
    self._pattern = pattern
    self._index = 0
    self._timeout = time.time()

  def StopRun(self):
    if self._running:
      print 'Stopping servos.'
    self._running = False

  def _RunOnce(self):
    """Modifies and sends the ControllerCommandMessage."""
    with self._command_lock:
      if self._running:
        while self._timeout > 0.0 and time.time() >= self._timeout:
          dest, interval = self._pattern[self._index]
          self._index = (self._index + 1) % len(self._pattern)
          self._timeout += interval
          angle = dest * math.pi / 180.0
          for i in range(len(SERVOS)):
            self._command.servo_angle[i] = angle
        self._client.Send(self._command, 'kMessageTypeControllerCommand',
                          CONTROLLER)

    time.sleep(0.0095)


if __name__ == '__main__':
  client = ServoCommandClient()
  try:
    client.cmdloop()
  except BaseException:
    client.TryStopThreads()
    raise

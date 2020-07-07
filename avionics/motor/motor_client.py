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

"""Command line client for controlling motors."""

import collections
import os
import re
import socket
import subprocess
import tempfile
import threading
import time

import makani
from makani.avionics.common import actuator_types
from makani.avionics.common import aio
from makani.avionics.common import cmd_client
from makani.avionics.common import pack_avionics_messages
from makani.avionics.common import safety_codes
from makani.avionics.firmware.params import client as param_client
from makani.avionics.motor.firmware import config_params
from makani.avionics.motor.firmware import flags
from makani.avionics.network import aio_labels
from makani.avionics.network import aio_node
from makani.avionics.network import message_type
from makani.lib.python import c_helpers
import numpy as np
from scipy import interpolate

# TODO: implement NetworkConfig() to replace all these EnumHelper's.
aio_node_helper = c_helpers.EnumHelper('AioNode', aio_node)
motor_label_helper = c_helpers.EnumHelper('MotorLabel', aio_labels,
                                          prefix='kMotor')
motor_error_helper = c_helpers.EnumHelper('MotorError', flags)
motor_warning_helper = c_helpers.EnumHelper('MotorWarning', flags)


def BuildMotorParamDict():
  """Builds a dict mapping motor param names to their indices."""
  # Build up parameter list.
  filename = os.path.join(makani.HOME, 'avionics/motor/firmware/io.c')
  with open(filename) as f:
    f_text = f.read()

  # Get parameter array string.
  re_string = r'static float \*g_mutable_param_addrs\[\] = {\s*^([\s\S]*)^};'
  array_string = re.search(re_string, f_text, re.MULTILINE)

  re_string = r'^ *&[\w\[\]]+.([\w\.\[\]]+)'
  motor_param_keys = re.findall(re_string, array_string.group(0), re.MULTILINE)

  return {key: ind for ind, key in enumerate(motor_param_keys)}


# Constants.
MOTORS = [mot.upper() for mot in motor_label_helper.ShortNames()]
CONTROLLER = 'kAioNodeControllerA'
OPERATOR = 'kAioNodeOperator'
MOTOR_PARAMS = BuildMotorParamDict()
MOTOR_ERROR_NAMES = collections.OrderedDict(
    (error_bitmask, motor_error_helper.Name(error_bitmask))
    for error_bitmask in motor_error_helper.Values()
    if motor_error_helper.Name(error_bitmask) != 'kMotorErrorAll')
MOTOR_WARNING_NAMES = collections.OrderedDict(
    (warning_bitmask, motor_warning_helper.Name(warning_bitmask))
    for warning_bitmask in motor_warning_helper.Values()
    if motor_warning_helper.Name(warning_bitmask) != 'kMotorWarningAll')
MOTOR_STATUS_NAMES = {val: key for key, val in flags.__dict__.items()
                      if key.startswith('kMotorStatus')}
GEN_TABLE_PATH = os.path.join(makani.HOME,
                              'avionics/motor/gen_lookup_table.py')
OMEGA_MIN_LIMIT = -260.0
OMEGA_MAX_LIMIT = 260.0
TORQUE_MIN_LIMIT = -600.0
TORQUE_MAX_LIMIT = 600.0
EPS32 = np.finfo(np.float32).eps


class MotorClientError(cmd_client.WingClientError):
  pass


def MotorsAsBits(motor_list):
  """Returns a bitmask describing the motors in `motor_list`."""
  return sum(1 << motor_label_helper.Value(motor.capitalize())
             for motor in motor_list)


def AioNodeNameFromMotorNickname(motor):
  """Returns AIO node name for the specified motor."""
  return 'kAioNodeMotor' + motor.capitalize()


def AioNodeNameFromDynoNickname(motor):
  """Returns AIO node name for the specified dyno motor."""
  return 'kAioNodeDynoMotor' + motor.capitalize()


def GetMotorErrorNames(error_bitmask):
  """Returns a list of error names corresponding to the specified bitmask."""
  return GetFlagNames(error_bitmask, MOTOR_ERROR_NAMES, 0)


def GetMotorWarningNames(warning_bitmask):
  """Returns a list of warning names corresponding to the specified bitmask."""
  return GetFlagNames(warning_bitmask, MOTOR_WARNING_NAMES, 0)


def GetFlagNames(bitmask, bitmask_dict, default_key=None):
  """Returns a list based on bitmask_dict corresponding to set bits in bitmask.

  Args:
    bitmask: Integer containing a bitmask of desired fields.
    bitmask_dict: Dictionary with power-of-two integer keys and values
        containing names of the corresponding bits.
    default_key: Key to use if bitmask == 0. Set to None to return [].

  Returns:
    A list with the values of bitmask_dict specified by bitmask.
  """
  if bitmask:
    return [name for bit, name in bitmask_dict.iteritems() if bit & bitmask]
  else:
    if default_key is None:
      return []
    else:
      return [bitmask_dict[default_key]]


def GenerateCommandData(args):
  """Generates the data to use for a given speed or torque command.

  Args:
    args: List containing command input file & optional loop parameter.

  Returns:
    data: Numpy array of time, torque and speed limits.
    loop: Boolean of optional loop parameter.

  Raises:
      MotorClientError: An invalid filename or file format was specified.
  """
  cmd_file = args[0]
  if not os.path.isfile(cmd_file):
    raise MotorClientError('Invalid filename: %s' % cmd_file)

  # Handle 1st arg i.e. the command file.
  if cmd_file.endswith(('.py', '.pycmd')):  # Treat as a Python file.
    with tempfile.NamedTemporaryFile() as table_file:
      popen = subprocess.Popen([GEN_TABLE_PATH, '--input_file', cmd_file,
                                '--binary'],
                               stdout=table_file, stderr=subprocess.PIPE)
      _, stderr = popen.communicate()

      if popen.returncode != 0:
        raise MotorClientError('Generation of lookup table from %s failed. '
                               'stderr:\n%s' % (cmd_file, stderr))

      data = np.load(table_file.name)
      print 'Using %s to generate command profile.' % cmd_file

  else:  # Treat as a text file for interpolation.
    try:
      data = np.loadtxt(cmd_file)
    except (IOError, ValueError):
      raise MotorClientError(
          'Invalid input text file: %s. Should contain a table of time, torques'
          'and speed limits with rows of the form:\n\n'
          'time torque1 torque2 ... torque8 omega_lower1 omega_lower2 ...'
          'omega_lower8 omega_upper1 omega_upper2 ... omega_upper8' % cmd_file)
    print 'Using interpolated values from %s for command profile.' % cmd_file

  if data.shape[1] != 25:
    raise MotorClientError(
        'Invalid number of columns in command table. Expected 25, got %d. '
        'Revise input file to generate rows of the form:\n'
        'time torque1 torque2 ... torque8 omega_lower1 omega_lower2 ...'
        'omega_lower8 omega_upper1 omega_upper2 ... omega_upper8'
        % data.shape[1])

  # Handle 2nd arg i.e. the optional parameter to repeat.
  if len(args) == 1:
    loop = False
    print 'Defaulting to \"noloop\".'
  else:
    if args[1] == 'loop':
      loop = True
    elif args[1] == 'noloop':
      loop = False
    else:
      raise MotorClientError('Invalid option: %s. Expecting \"loop\" or '
                             '[default] \"noloop\".' % args[1])

  return data, loop


def CheckCommandLimits(
    cmd_min, cmd_max, cmd_min_limit, cmd_max_limit, cmd_type):
  if cmd_min < cmd_min_limit or cmd_max > cmd_max_limit:
    raise MotorClientError('Extreme %s outside of limits [%f, %f] '
                           'detected. Command not set.' %
                           (cmd_type, cmd_min_limit, cmd_max_limit))
  if cmd_min > cmd_max:
    raise MotorClientError('Invalid %s i.e. min value - %f, is greater '
                           'than max value - %f' % (cmd_type, cmd_min, cmd_max))


class CommandProfile(object):
  """Maintains a lookup table of motor commands while running motors."""

  def __init__(
      self, t, motor_cmd, cmd_min_limit, cmd_max_limit, cmd_type,
      loop_back=False):
    self._loop_back = loop_back
    self._t = t
    self._motor_cmd_func = interpolate.interp1d(self._t, motor_cmd, axis=0)
    cmd_max = np.max(motor_cmd)
    cmd_min = np.min(motor_cmd)
    print ('\nWith {t_start:.2f}s < t < {t_end:.2f}s:'
           '\n min({type}) = {min:f}\n max({type}) = {max:f}\n'.format(
               t_start=t[0], t_end=t[-1], type=cmd_type,
               min=cmd_min, max=cmd_max))
    CheckCommandLimits(cmd_min, cmd_max, cmd_min_limit, cmd_max_limit, cmd_type)

  def __call__(self, t):
    if self._loop_back:
      t = np.mod(t, self._t[-1])
    elif t > self._t[-1]:
      return None
    return list(self._motor_cmd_func(t))


class MotorCommandClient(cmd_client.WingCommandClient):
  """Command line client for running M600 motors."""

  prompt = '(motor_client) '
  _NUM_RETRIES = 10
  _MOTORS = 'motors'
  _DYNOS = 'dynos'

  def __init__(self, *args, **kwargs):
    cmd_client.WingCommandClient.__init__(self, *args, **kwargs)

    self._motors_selected = set()
    self._dynos_selected = set()
    self._spin_dir = {}
    self._motor_runner = Runner(self._motors_selected, self._spin_dir)
    self._dyno_runner = Runner(self._dynos_selected, self._spin_dir,
                               dyno_mode=True)
    self._motor_listener = None
    self._dyno_listener = None
    self._torque = 0.0
    self._omega_lower_limit = 0.0
    self._omega_upper_limit = 0.0

    self._arm_aio_client = aio.AioClient(
        ['kMessageTypeMotorSetState', 'kMessageTypeDynoMotorSetState'],
        timeout=0.1)
    self._set_param_aio_client = aio.AioClient(
        ['kMessageTypeMotorSetParam', 'kMessageTypeDynoMotorSetParam'],
        timeout=0.1)
    # The long range radio requires at least 2x160 ms for a complete command-
    # response cycle.
    self._ack_param_aio_client = aio.AioClient(
        ['kMessageTypeMotorAckParam'], timeout=0.35)
    self._get_param_aio_client = aio.AioClient(
        ['kMessageTypeMotorGetParam', 'kMessageTypeDynoMotorGetParam'],
        timeout=0.1)
    self._param_client = param_client.Client(timeout=0.1)

  def TryStopThreads(self):
    self._motor_runner.TryStop()
    self._dyno_runner.TryStop()
    if self._motor_listener:
      self._motor_listener.TryStop()
    if self._dyno_listener:
      self._dyno_listener.TryStop()

  def _GetListenerAndRunner(self, node_type):
    if node_type == self._MOTORS:
      return self._motor_listener, self._motor_runner
    elif node_type == self._DYNOS:
      return self._dyno_listener, self._dyno_runner
    else:
      raise MotorClientError('Unknown node type.')

  def _CheckStatus(self, valid_statuses, node_type):
    listener, _ = self._GetListenerAndRunner(node_type)

    if not listener:
      status = flags.kMotorStatusInit
    else:
      status = listener.GetMostRestrictiveMotorStatus()

    if status not in valid_statuses:
      raise MotorClientError(
          'Invalid %s status. %s' % (
              node_type.capitalize(), MOTOR_STATUS_NAMES[status]))
    return True

  def _CheckMotorStatus(self, valid_statuses):
    self._CheckStatus(valid_statuses, self._MOTORS)

  def _CheckDynoStatus(self, valid_statuses):
    self._CheckStatus(valid_statuses, self._DYNOS)

  def _CheckTargetsSelected(self):
    if self._motors_selected or self._dynos_selected:
      return True
    else:
      raise MotorClientError('Invalid set of targets. Use either: '
                             '"set_targets" or "set_targets_dyno".')

  def _SetTargets(self, line, node_type):
    """Sets motor or dyno targets.

    Args:
      line: User supplied arguments specifying target motors.
      node_type: String specifying type of targets i.e. 'motors' or 'dynos'.

    Raises:
      MotorClientError: An invalid set of targets was specified.
    """
    targets_selected, _ = cmd_client.SelectArgs(
        line.split(), MOTORS, require_some=True, require_all=True,
        select_all=True, require_one=False)

    if node_type == self._MOTORS:
      self._motors_selected = targets_selected
      motor_params = self._QueryConfig(self._motors_selected, self._MOTORS)
      self._spin_dir = self._GetSpinDir(motor_params)
    elif node_type == self._DYNOS:
      self._dynos_selected = targets_selected
      self._QueryConfig(self._dynos_selected, self._DYNOS)

    self.TryStopThreads()

    if self._motors_selected:
      print 'Motors selected: %s.' % ', '.join(self._motors_selected)
      self._motor_runner = Runner(self._motors_selected, self._spin_dir)
      self._motor_listener = Listener(self._motor_runner.StopRun,
                                      self._motors_selected)
    if self._dynos_selected:
      print 'Dynos selected:  %s.' % ', '.join(self._dynos_selected)
      self._dyno_runner = Runner(self._dynos_selected, self._spin_dir,
                                 dyno_mode=True)
      self._dyno_listener = Listener(self._dyno_runner.StopRun,
                                     self._dynos_selected, dyno_mode=True)

  @cmd_client.Command()
  def do_set_targets(self, line):  # pylint: disable=invalid-name
    """Sets motor targets e.g. "set_targets SBO SBI"."""
    self._CheckMotorStatus([flags.kMotorStatusInit, flags.kMotorStatusError])
    self._SetTargets(line, self._MOTORS)

  @cmd_client.Command()
  def do_set_targets_dyno(self, line):  # pylint: disable=invalid-name
    """Sets dyno targets e.g. "set_targets_dyno SBO SBI"."""
    self._CheckDynoStatus([flags.kMotorStatusInit, flags.kMotorStatusError])
    self._SetTargets(line, self._DYNOS)

  @cmd_client.Command()
  def do_get_targets(self, line):  # pylint: disable=invalid-name
    """Displays selected motor & dyno targets."""
    print 'Current targets.\nMotors: %s.\nDynos: %s.' % (
        ', '.join(self._motors_selected), ', '.join(self._dynos_selected))

  @cmd_client.Command()
  def do_clear_targets(self, line):  # pylint: disable=invalid-name
    """Clears selected motor & dyno targets."""
    old_motors = self._motors_selected.copy()
    old_dynos = self._dynos_selected.copy()
    self.TryStopThreads()
    self._motors_selected = set()
    self._dynos_selected = set()
    self._spin_dir = {}
    self._motor_runner = Runner(self._motors_selected, self._spin_dir)
    self._dyno_runner = Runner(self._dynos_selected, self._spin_dir,
                               dyno_mode=True)
    self._motor_listener = None
    self._dyno_listener = None
    print 'Cleared old targets.\nOld Motors: %s.\nOld Dynos: %s.' % (
        ', '.join(old_motors), ', '.join(old_dynos))

  def complete_set_targets(self, text, *unused_args):  # pylint: disable=invalid-name
    return self._CompleteArg(text, sorted(MOTORS) + ['All'])

  complete_set_targets_dyno = complete_set_targets

  def _GetSpinDir(self, params):
    """Determine the nominal spin direction based off of the motor load type."""
    # List of props that need to spin in the positive x direction / in the
    # negative omega sense.
    # Additional loads are to be added in future commits.
    reversed_loads = [config_params.MotorLoadType.PROP_REV2_POSITIVE_X]
    return {key: -1 if param and param.load_type in reversed_loads else 1
            for key, param in params.iteritems()}

  def _QueryConfig(self, targets, target_type):
    """Test if targets are on the network and query their configurations."""
    params = {}
    for target in targets:
      if target_type == self._DYNOS:
        node = aio_node_helper.Value(AioNodeNameFromDynoNickname(target))
      elif target_type == self._MOTORS:
        node = aio_node_helper.Value(AioNodeNameFromMotorNickname(target))

      section = param_client.SECTION_CONFIG
      try:
        params[target] = self._param_client.GetSection(node, section)
      except socket.timeout:
        params[target] = None
    self._PrintConfig(targets, params)
    return params

  def _PrintConfig(self, motors, params):
    """Print portions of the selected motor config params."""
    load_types = [load_type.CName()[len('kMotorLoadType'):]
                  for load_type in config_params.MotorLoadType.Names()]
    motor_types = [motor_type.CName()[len('kMotorType'):]
                   for motor_type in config_params.MotorType.Names()]

    load_type_max_str_len = max([len(name) for name in load_types])
    motor_type_max_str_len = max([len(name) for name in motor_types])

    for motor in sorted(motors):
      if params[motor] is None:
        print '%s:  unknown' % motor
      else:
        print '{name}:  motor_type: {motor_type} load_type: {load_type}'.format(
            name=motor,
            motor_type=(motor_types[params[motor].motor_type]
                        .ljust(motor_type_max_str_len)),
            load_type=(load_types[params[motor].load_type]
                       .ljust(load_type_max_str_len)))
    print ''

  @cmd_client.Command()
  def do_query_config(self, line):  # pylint: disable=invalid-name
    targets_selected, _ = cmd_client.SelectArgs(
        line.split(), MOTORS, require_some=True, require_all=True,
        select_all=True, require_one=False)
    self._CheckMotorStatus([flags.kMotorStatusInit, flags.kMotorStatusError])
    self._QueryConfig(targets_selected, self._MOTORS)

  @cmd_client.Command()
  def do_query_config_dyno(self, line):  # pylint: disable=invalid-name
    targets_selected, _ = cmd_client.SelectArgs(
        line.split(), MOTORS, require_some=True, require_all=True,
        select_all=True, require_one=False)
    self._CheckDynoStatus([flags.kMotorStatusInit, flags.kMotorStatusError])
    self._QueryConfig(targets_selected, self._DYNOS)

  def _TryArm(self, arm_msg, arm_msg_type, node_type):
    listener, _ = self._GetListenerAndRunner(node_type)
    for _ in xrange(self._NUM_RETRIES):
      self._arm_aio_client.Send(arm_msg, arm_msg_type, OPERATOR)
      time.sleep(0.1)
      if listener.AllMotorsArmed():
        print 'Successfully armed %s.' % node_type
        return
      else:
        raise MotorClientError('Failed to arm %s.' % node_type)

  @cmd_client.Command(num_args=0)
  def do_arm(self, unused_line):  # pylint: disable=invalid-name
    """Arms the selected motors and/or dynos."""
    if self._motors_selected:
      self._CheckMotorStatus([flags.kMotorStatusInit])
    if self._dynos_selected:
      self._CheckDynoStatus([flags.kMotorStatusInit])

    self._CheckTargetsSelected()

    if self._motors_selected:
      motor_arm_msg = pack_avionics_messages.MotorSetStateMessage()
      motor_arm_msg.command = actuator_types.kActuatorStateCommandArm
      motor_arm_msg.command_data = safety_codes.MOTOR_ARMING_SIGNAL
      print 'Arming motors.'
      motor_arm_msg.selected_motors = MotorsAsBits(
          self._motor_listener.GetUnarmedMotors())
      self._TryArm(
          motor_arm_msg, 'kMessageTypeMotorSetState', self._MOTORS)

    if self._dynos_selected:
      dyno_arm_msg = pack_avionics_messages.DynoMotorSetStateMessage()
      dyno_arm_msg.command = actuator_types.kActuatorStateCommandArm
      dyno_arm_msg.command_data = safety_codes.MOTOR_ARMING_SIGNAL
      print 'Arming dynos.'
      dyno_arm_msg.selected_motors = MotorsAsBits(
          self._dyno_listener.GetUnarmedMotors())
      self._TryArm(
          dyno_arm_msg, 'kMessageTypeDynoMotorSetState', self._DYNOS)

  def _SetParam(self, line, message, node_type):  # pylint: disable=invalid-name
    """Sets a param for a specified motor or dyno."""
    targets, args = cmd_client.SelectArgs(
        line.split(), MOTORS, require_some=True, select_all=True)
    param, args = cmd_client.SelectArgs(
        args, MOTOR_PARAMS.keys(), require_one=True, select_all=False)

    if node_type == self._DYNOS:
      targets = ['DYNO_%s' % t.upper() for t in targets]

    try:
      value = float(args[0])
    except ValueError:
      raise MotorClientError('Invalid value: "%s".' % args[0])

    message.id = MOTOR_PARAMS[param]
    message.value = value

    failed_targets = []

    for target in targets:
      print 'Setting %s to %g on %s.' % (param, value, target)

      if target.startswith('DYNO_'):
        message.selected_motors = MotorsAsBits([target[len('DYNO_'):]])
        aio_target = AioNodeNameFromDynoNickname(target[len('DYNO_'):])
        success = self._TrySetParam(
            message, 'kMessageTypeDynoMotorSetParam', param, target, aio_target)
      else:
        message.selected_motors = MotorsAsBits([target])
        aio_target = AioNodeNameFromMotorNickname(target)
        success = self._TrySetParam(
            message, 'kMessageTypeMotorSetParam', param, target, aio_target)

      if not success:
        failed_targets.append(target)

    if failed_targets:
      raise MotorClientError('Failed to verify %s from %s.'
                             % (param, failed_targets))

  def _TrySetParam(self, message, msg_type, param, target, aio_target):
    for _ in xrange(self._NUM_RETRIES):
      self._set_param_aio_client.Send(message, msg_type, OPERATOR)
      for _ in xrange(self._NUM_RETRIES):
        try:
          _, header, ack = self._ack_param_aio_client.Recv()
          if (header.source == aio_node_helper.Value(aio_target)
              and header.type == message_type.kMessageTypeMotorAckParam
              and ack.id == message.id and ack.value == message.value):
            print '%s %s: %g' % (target, param, ack.value)
            return True
        except socket.timeout:
          return False
    return False

  @cmd_client.Command(num_args=3)
  def do_set_param(self, line):  # pylint: disable=invalid-name
    """Sets param for a specified motor, e.g. "set_motor_param SBO Ld 3.14"."""
    self._CheckMotorStatus([flags.kMotorStatusInit, flags.kMotorStatusError])
    message = pack_avionics_messages.MotorSetParamMessage()
    self._SetParam(line, message, self._MOTORS)

  @cmd_client.Command(num_args=3)
  def do_set_param_dyno(self, line):  # pylint: disable=invalid-name
    """Sets param for a specified dyno, e.g. "set_dyno_param SBO Ld 3.14"."""
    self._CheckDynoStatus([flags.kMotorStatusInit, flags.kMotorStatusError])
    message = pack_avionics_messages.DynoMotorSetParamMessage()
    self._SetParam(line, message, self._DYNOS)

  def complete_set_param(self, text, line, *unused_args):  # pylint: disable=invalid-name
    arg_number = len(line.split())
    if not text:
      arg_number += 1

    if arg_number == 2:
      return self._CompleteArg(text, sorted(MOTORS) + ['All'])
    elif arg_number == 3:
      return self._CompleteArg(text, sorted(MOTOR_PARAMS.keys()))
    else:
      return []

  complete_set_param_dyno = complete_set_param

  def _GetParam(self, line, message, node_type):
    targets, args = cmd_client.SelectArgs(
        line.split(), MOTORS, require_some=True, select_all=True)
    param, _ = cmd_client.SelectArgs(
        args, MOTOR_PARAMS.keys(), require_one=True, select_all=False)

    if node_type == self._DYNOS:
      targets = ['DYNO_%s' % t.upper() for t in targets]

    message.id = MOTOR_PARAMS[param]

    failed_targets = []

    for target in targets:
      print 'Getting %s from %s...' % (param, target)
      success = True
      if target.startswith('DYNO_'):
        message.selected_motors = MotorsAsBits([target[len('DYNO_'):]])
        aio_target = AioNodeNameFromDynoNickname(target[len('DYNO_'):])
        success = self._TryGetParam(
            message, 'kMessageTypeDynoMotorGetParam', param, target, aio_target)
      else:
        message.selected_motors = MotorsAsBits([target])
        aio_target = AioNodeNameFromMotorNickname(target)
        success = self._TryGetParam(
            message, 'kMessageTypeMotorGetParam', param, target, aio_target)

      if not success:
        failed_targets.append(target)

    if failed_targets:
      raise MotorClientError('Failed to get %s from %s.'
                             % (param, failed_targets))

  def _TryGetParam(self, message, msg_type, param, target, aio_target):
    for _ in xrange(self._NUM_RETRIES):
      self._get_param_aio_client.Send(message, msg_type, OPERATOR)
      for _ in xrange(self._NUM_RETRIES):
        try:
          _, header, ack = self._ack_param_aio_client.Recv()
          if (header.source == aio_node_helper.Value(aio_target)
              and header.type == message_type.kMessageTypeMotorAckParam
              and ack.id == message.id):
            print '%s %s: %g' % (target, param, ack.value)
            return True
        except socket.timeout:
          return False
    return False

  @cmd_client.Command()
  def do_get_param(self, line):  # pylint: disable=invalid-name
    self._CheckMotorStatus([flags.kMotorStatusInit, flags.kMotorStatusError])
    message = pack_avionics_messages.MotorGetParamMessage()
    self._GetParam(line, message, self._MOTORS)

  @cmd_client.Command()
  def do_get_param_dyno(self, line):  # pylint: disable=invalid-name
    self._CheckDynoStatus([flags.kMotorStatusInit, flags.kMotorStatusError])
    message = pack_avionics_messages.DynoMotorGetParamMessage()
    self._GetParam(line, message, self._DYNOS)

  complete_get_param = complete_set_param

  complete_get_param_dyno = complete_get_param

  @cmd_client.Command()
  def do_run(self, line):  # pylint: disable=invalid-name
    """Runs the selected motors and/or dynos.

    Specify a duration in "s" or "ms".  E.g. "run 10s" or "run 300ms".

    Args:
      line: Command to this function.

    Raises:
      MotorClientError: An invalid duration was specified.
    """
    if self._motors_selected:
      self._CheckMotorStatus([flags.kMotorStatusArmed])
    if self._dynos_selected:
      self._CheckDynoStatus([flags.kMotorStatusArmed])

    self._CheckTargetsSelected()

    if line.endswith('ms'):
      line = line[:-2]
      multiplier = 1e-3
    elif line.endswith('s'):
      line = line[:-1]
      multiplier = 1.0
    else:
      raise MotorClientError('Usage: run {$N {s|ms}}')
    try:
      duration = float(line) * multiplier
    except ValueError:
      raise MotorClientError('Invalid run time: \'%s\'' % line)

    if self._motor_runner.IsRunning() or self._dyno_runner.IsRunning():
      raise MotorClientError('Already running.')

    if self._motors_selected:
      if not self._motor_listener.AllMotorsArmed():
        raise MotorClientError('Motors not armed.')
      self._motor_runner.StartRun(duration)

    if self._dynos_selected:
      if not self._dyno_listener.AllMotorsArmed():
        raise MotorClientError('Dynos not armed.')
      self._dyno_runner.StartRun(duration)

    print 'Running...'

  @cmd_client.Command(num_args=0)
  def do_stop(self, unused_line):  # pylint: disable=invalid-name
    """Stops the motors and/or dynos."""
    if self._motor_runner.IsRunning() or self._dyno_runner.IsRunning():
      self._motor_runner.StopRun()
      self._dyno_runner.StopRun()
    else:
      raise MotorClientError('Not running.')

    print 'Run stopped.'

  def _GetCommandFunction(self, line):
    """Returns a complete command function for each selected motor and/or dyno.

    Args:
      line: Command to this function.

    Raises:
      MotorClientError: Motors and/or dynos are running.

    Returns:
      torque_func: A function that returns torque commands.
      omega_lower_func: A function that returns omega_lower commands.
      omega_upper_func: A function that returns omega_upper commands.
      freeze_command: Specifies if last command should persist on stop.
    """
    if self._motor_runner.IsRunning() or self._dyno_runner.IsRunning():
      raise MotorClientError('Motors and/or dynos are running.')

    args = line.split()

    data, loop = GenerateCommandData(args)
    t = data[:, 0]
    torque_cmd = data[:, 1:9]
    omega_lower_cmd = data[:, 9:17]
    omega_upper_cmd = data[:, 17:25]

    torque_func = CommandProfile(t, torque_cmd, TORQUE_MIN_LIMIT,
                                 TORQUE_MAX_LIMIT, 'torque', loop)
    omega_lower_func = CommandProfile(t, omega_lower_cmd, OMEGA_MIN_LIMIT,
                                      OMEGA_MAX_LIMIT, 'omega', loop)
    omega_upper_func = CommandProfile(t, omega_upper_cmd, OMEGA_MIN_LIMIT,
                                      OMEGA_MAX_LIMIT, 'omega', loop)
    freeze_command = False

    return (torque_func, omega_lower_func, omega_upper_func, freeze_command)

  @cmd_client.Command(num_args=[1, 2])
  def do_set_command_function(self, line):  # pylint: disable=invalid-name, g-doc-args
    # pylint: disable=g-doc-args
    """Sets a command function for motor(s).

    Specify a filename which may be:
      - A Python file (must have .py suffix) corresponding to an input to
        gen_lookup_table.py
      - A text file whose output is a lookup table formatted per the output of
        gen_lookup_table.py.
    """
    self._CheckMotorStatus(
        [flags.kMotorStatusInit, flags.kMotorStatusArmed,
         flags.kMotorStatusError])
    cmd_args = self._GetCommandFunction(line)
    self._motor_runner.SetCommandFunction(*cmd_args)

  @cmd_client.Command(num_args=[1, 2])
  def do_set_command_function_dyno(self, line):  # pylint: disable=invalid-name
    # pylint: disable=g-doc-args
    """Sets a command function for dyno(s).

    Specify a filename which may be:
      - A Python file (must have .py suffix) corresponding to an input to
        gen_lookup_table.py
      - A text file whose output is a lookup table formatted per the output of
        gen_lookup_table.py.
    """
    self._CheckDynoStatus(
        [flags.kMotorStatusInit, flags.kMotorStatusArmed,
         flags.kMotorStatusError])
    cmd_args = self._GetCommandFunction(line)
    self._dyno_runner.SetCommandFunction(*cmd_args)

  def complete_set_motor_command_function(self, _, line, *unused_args):  # pylint: disable=invalid-name
    """Completes arguments for the "set_command_function" command."""
    args = line.split(None, 2)
    if len(args) > 2 or (len(args) == 2 and line.endswith(' ')):
      suggestions = ['noloop', 'loop']
      if len(args) == 3:
        if args[2] in suggestions:
          return []
        suggestions = [x for x in suggestions if x.startswith(args[2])]
    else:
      path = args[1] if len(args) == 2 else ''
      suggestions = cmd_client.CompleteFile(path)
      suggestions = [x for x in suggestions
                     if (x.endswith(('/', '.py', '.pycmd', '.txt', '.dat'))
                         or x.find('.') < 0)]
    return suggestions

  complete_set_dyno_command_function = complete_set_motor_command_function

  @cmd_client.Command(num_args=2)
  def do_set_speed_limits(self, line):  # pylint: disable=invalid-name
    """Sets the speed limits for torque-mode e.g. set_speed_limits 100 200."""
    if not self._dynos_selected:
      raise MotorClientError('No dynos selected. Use "set_targets_dyno".')

    args = line.split()
    try:
      omega_lower = float(args[0])
      omega_upper = float(args[1])
    except ValueError:
      raise MotorClientError('Invalid argument(s): \'{:s}\''.format(line))

    CheckCommandLimits(
        omega_lower, omega_upper, OMEGA_MIN_LIMIT, OMEGA_MAX_LIMIT, 'omega')

    self._omega_lower_limit = omega_lower
    self._omega_upper_limit = omega_upper
    print 'Omega limits set to: %.2f rad/s, %.2f rad/s.' % (
        self._omega_lower_limit, self._omega_upper_limit)

    torque_func = lambda _: self._torque
    omega_lower_func = lambda _: self._omega_lower_limit
    omega_upper_func = lambda _: self._omega_upper_limit
    freeze_command = True

    self._dyno_runner.SetCommandFunction(torque_func, omega_lower_func,
                                         omega_upper_func, freeze_command)

  @cmd_client.Command(num_args=1)
  def do_set_torque(self, line):  # pylint: disable=invalid-name
    """Sets motor torque."""
    if not self._dynos_selected:
      raise MotorClientError('No dynos selected. Use "set_targets_dyno".')

    try:
      torque = float(line)
    except ValueError:
      raise MotorClientError('Invalid argument(s): \'{:s}\''.format(line))

    if self._omega_lower_limit == 0 and self._omega_upper_limit == 0:
      raise MotorClientError('Omega limits not set. Use "set_speed_limits".')

    CheckCommandLimits(
        torque, torque, TORQUE_MIN_LIMIT, TORQUE_MAX_LIMIT, 'torque')

    self._torque = torque

    print 'Torque desired: %.2f Nm. Speed limits: %.2f rad/s, %.2f rad/s.' % (
        torque, self._omega_lower_limit, self._omega_upper_limit)

    torque_func = lambda _: self._torque
    omega_lower_func = lambda _: self._omega_lower_limit
    omega_upper_func = lambda _: self._omega_upper_limit
    freeze_command = True

    self._dyno_runner.SetCommandFunction(torque_func, omega_lower_func,
                                         omega_upper_func, freeze_command)

  @cmd_client.Command(num_args=1)
  def do_set_omega(self, line):  # pylint: disable=invalid-name
    """Sets motor speed."""
    if not self._motors_selected:
      raise MotorClientError('No motors selected. Use "set_targets".')

    try:
      omega = float(line)
    except ValueError:
      raise MotorClientError('Invalid omega: \'{:s}\''.format(line))

    CheckCommandLimits(omega, omega, OMEGA_MIN_LIMIT, OMEGA_MAX_LIMIT, 'omega')

    print 'Omega desired: %s rad/s' % omega

    torque_func = lambda _: 0.0
    omega_lower_func = lambda _: omega
    omega_upper_func = lambda _: omega
    freeze_command = True

    self._motor_runner.SetCommandFunction(torque_func, omega_lower_func,
                                          omega_upper_func, freeze_command)

  def _RampCommand(self, line, cmd_type, runner):
    """Sets a motor speed or torque ramp.

    Args:
      line: Command to this function.
      cmd_type: Torque or Omega command to ramp.
      runner: Runner instance to use for setting command.

    Raises:
      MotorClientError: An invalid parameter was specified.
    """
    args = line.split(None, 2)
    try:
      cmd = float(args[0])
    except ValueError:
      raise MotorClientError('Invalid %s: \'{:s}\''.format(args[0]) % cmd_type)

    if len(args) == 2:
      try:
        dt = self._dt = float(args[1])
      except ValueError:
        raise MotorClientError('Invalid time: \'{:s}\''.format(args[1]))
    else:
      dt = 1.0

    if runner.IsRunning():
      t0 = runner.GetTime()
      motor_cmd = runner.GetCommand()
      cmd0 = motor_cmd[cmd_type]
    else:
      t0 = 0.0
      cmd0 = 0.0

    dcmd_dt = (cmd - cmd0) / dt if abs(dt) > 10.0 * EPS32 else 0.0

    def Ramp(t):
      if t > t0 + dt:
        return cmd
      elif t > t0:
        return dcmd_dt * (t - t0) + cmd0
      else:
        return cmd0

    if cmd_type == 'omega_upper':
      torque_func = lambda _: 0.0
      omega_lower_func = Ramp
      omega_upper_func = Ramp
    elif cmd_type == 'torque':
      torque_func = Ramp
      omega_lower_func = lambda _: self._omega_lower_limit
      omega_upper_func = lambda _: self._omega_upper_limit
    else:
      raise MotorClientError('Invalid command type: %s' % cmd_type)
    freeze_command = True

    runner.SetCommandFunction(
        torque_func, omega_lower_func, omega_upper_func, freeze_command)
    display_cmd = cmd_type.split('_')[0].capitalize()
    print (' Ramping over dt = %4.2f:\n'
           '  %s(t0)      = %4.1f\n'
           '  %s(t0 + dt) = %4.1f' % (dt, display_cmd, cmd0, display_cmd, cmd))

  @cmd_client.Command(num_args=[1, 2])
  def do_ramp_omega(self, line):  # pylint: disable=invalid-name
    # pylint: disable=g-doc-args
    """Sets a motor speed ramp.

    Specify a linear angular rate ramp from the present speed omega0 to a final
    speed omega1 over some time dt (in seconds) with the command:

      ramp_omega [omega1] [dt]

    The second argument is optional. If not specified dt = 1s is assumed.
    """
    self._RampCommand(line, 'omega_upper', self._motor_runner)

  @cmd_client.Command(num_args=[1, 2])
  def do_ramp_torque(self, line):  # pylint: disable=invalid-name
    # pylint: disable=g-doc-args
    """Sets a dyno torque ramp.

    Specify a linear torque ramp from the present torque T0 to a final
    torque T1 over some time dt (in seconds) with the command:

      ramp_torque [T1] [dt]

    The second argument is optional. If not specified dt = 1s is assumed.
    """
    self._RampCommand(line, 'torque', self._dyno_runner)

  @cmd_client.Command(num_args=0)
  def do_clear_errors(self, unused_line):  # pylint: disable=invalid-name
    self._CheckTargetsSelected()
    if self._motors_selected:
      self._CheckMotorStatus([flags.kMotorStatusInit, flags.kMotorStatusError])
      self._motor_listener.ClearErrors()
      self._motor_runner.ClearErrors()
    if self._dynos_selected:
      self._CheckDynoStatus([flags.kMotorStatusInit, flags.kMotorStatusError])
      self._dyno_listener.ClearErrors()
      self._dyno_runner.ClearErrors()

    print 'Errors cleared.'

  def _TryDisarm(self, node_type):
    listener, runner = self._GetListenerAndRunner(node_type)
    for _ in xrange(self._NUM_RETRIES):
      runner.Disarm()
      time.sleep(0.1)
      if listener.AllMotorsDisarmed():
        print 'Successfully disarmed %s.' % node_type
        return

    raise MotorClientError('Failed to disarm %s.' % node_type)

  @cmd_client.Command(num_args=0)
  def do_disarm(self, unused_line):  # pylint: disable=invalid-name
    """Disarms the motors."""
    self._CheckTargetsSelected()
    print 'Disarming.'

    if self._motors_selected:
      self._TryDisarm(self._MOTORS)

    if self._dynos_selected:
      self._TryDisarm(self._DYNOS)

  @cmd_client.Command()
  def do_get_errors(self, unused_line):  # pylint: disable=invalid-name
    self._CheckTargetsSelected()
    if self._motors_selected:
      self._motor_listener.PrintErrors()
    if self._dynos_selected:
      self._dyno_listener.PrintErrors()

  @cmd_client.Command()
  def do_request_control_log(self, unused_line):  # pylint: disable=invalid-name
    self._CheckTargetsSelected()
    if self._motors_selected:
      self._motor_runner.RequestControlLog()
    if self._dynos_selected:
      self._dyno_runner.RequestControlLog()

  @cmd_client.Command()
  def do_request_adc_log(self, unused_line):  # pylint: disable=invalid-name
    self._CheckTargetsSelected()
    if self._motors_selected:
      self._motor_runner.RequestAdcLog()
    if self._dynos_selected:
      self._dyno_runner.RequestAdcLog()


class Listener(cmd_client.AioThread):
  """Continuously listens to MotorStatusMessages."""

  def __init__(self, error_callback, motors, dyno_mode=False):
    self._motors = motors.copy()

    t_now = time.time()
    self._errors = {m: flags.kMotorErrorNone for m in MOTORS}
    self._warnings = {m: flags.kMotorWarningNone for m in MOTORS}
    self._error_lock = threading.Lock()
    self._clear_errors_stop_time = t_now

    self._motor_status = {m: flags.kMotorStatusInit
                          for m in self._motors}
    self._motor_status_lock = threading.Lock()

    self._t_message = {m: t_now for m in self._motors}
    self._t_message_lock = threading.Lock()

    self._dyno_mode = dyno_mode
    if dyno_mode:
      sources = {AioNodeNameFromDynoNickname(m): m for m in self._motors}
    else:
      sources = {AioNodeNameFromMotorNickname(m): m for m in self._motors}
    self._motor_sources = {aio.aio_node_helper.Value(k): sources[k]
                           for k in sources.keys()}
    self._error_callback = error_callback

    super(Listener, self).__init__(['kMessageTypeMotorStatus'],
                                   allowed_sources=sources.keys(), timeout=0.1)
    self.start()

  def ClearErrors(self):
    with self._error_lock:
      for motor in self._errors.keys():
        self._errors[motor] = flags.kMotorErrorNone
        self._warnings[motor] = flags.kMotorWarningNone
    self._clear_errors_stop_time = time.time() + 5*10e-3

  def GetMostRestrictiveMotorStatus(self):
    """Returns the most restrictive status across all motors."""
    with self._motor_status_lock:
      motor_statuses = self._motor_status.values()

    if flags.kMotorStatusRunning in motor_statuses:
      return flags.kMotorStatusRunning
    elif flags.kMotorStatusArmed in motor_statuses:
      return flags.kMotorStatusArmed
    elif flags.kMotorStatusError in motor_statuses:
      return flags.kMotorStatusError

    return flags.kMotorStatusInit

  def AllMotorsArmed(self):
    with self._motor_status_lock:
      motor_statuses = self._motor_status.values()
    return all(x == flags.kMotorStatusArmed for x in motor_statuses)

  def AnyMotorsArmed(self):
    with self._motor_status_lock:
      motor_statuses = self._motor_status.values()
    return any(x == flags.kMotorStatusArmed for x in motor_statuses)

  def AllMotorsDisarmed(self):
    with self._motor_status_lock:
      motor_statuses = self._motor_status.values()
    return all(x != flags.kMotorStatusArmed
               and x != flags.kMotorStatusRunning
               for x in motor_statuses)

  def GetUnarmedMotors(self):
    with self._motor_status_lock:
      return [motor for motor, status in self._motor_status.iteritems()
              if status == flags.kMotorStatusInit]

  def PrintErrors(self):
    with self._error_lock:
      if (any([e != flags.kMotorErrorNone for e in self._errors.itervalues()])
          or any([w != flags.kMotorWarningNone
                  for w in self._warnings.itervalues()])):
        print 'Errors:'
        for motor in MOTORS:
          error = self._errors[motor]
          warning = self._warnings[motor]
          if error != flags.kMotorErrorNone:
            print '%s: %s' % (motor, ' | '.join(GetMotorErrorNames(error)))
            motor = (' ') * len(motor)  # Do no print out the motor name again.
          if warning != flags.kMotorWarningNone:
            print '%s: %s' % (motor, ' | '.join(GetMotorWarningNames(warning)))
      else:
        print 'No errors or warnings.'

  def _RunOnce(self):
    try:
      _, header, msg = self._client.Recv()
      motor = self._motor_sources[header.source]

      t_now = time.time()
      with self._t_message_lock:
        self._t_message[motor] = t_now
        stale = {m: t_now - self._t_message[m] > 0.05 for m in self._motors}

      new_status = False
      execute_callback = False
      with self._error_lock, self._motor_status_lock:
        # New errors.
        if t_now > self._clear_errors_stop_time:
          newline = '\n'
          error_diff = self._errors[motor] ^ msg.motor_error
          if msg.motor_error and error_diff:
            self._errors[motor] |= msg.motor_error
            print ('%sNew motor error(s)   %s: %s' %
                   (newline, motor, ' | '.join(GetMotorErrorNames(error_diff))))
            newline = ''  # Group errors and warning from the same motor.
          warning_diff = self._warnings[motor] ^ msg.motor_warning
          if warning_diff:
            self._warnings[motor] = msg.motor_warning
            if msg.motor_warning & warning_diff:
              print ('%sNew motor warning(s) %s: %s' %
                     (newline, motor,
                      ' | '.join(GetMotorWarningNames(warning_diff
                                                      & msg.motor_warning))))
            else:
              print ('%sCleared motor warning(s) %s: %s' %
                     (newline, motor,
                      ' | '.join(GetMotorWarningNames(warning_diff
                                                      & ~msg.motor_warning))))

        # Change in status.
        if self._motor_status[motor] != msg.motor_status:
          new_status = True
        self._motor_status[motor] = msg.motor_status

        # Invoke error callback after giving up self._error_lock and
        # self._status_lock just in case.
        if (new_status and
            any([e for e in self._errors.values()]) and
            all([self._motor_status[motor] &
                 ~(flags.kMotorStatusRunning | flags.kMotorStatusWindDown) or
                 stale[motor] for motor in self._motors])):
          execute_callback = True
      if execute_callback:
        self._error_callback()

    except socket.timeout:
      pass


class Runner(cmd_client.AioThread):
  """Continuously sends ControllerCommandMessages."""

  def __init__(self, motors, spin_dir, dyno_mode=False):
    self._motors = motors.copy()
    self._spin_dir = [spin_dir.get(motor, 1) for motor in MOTORS]

    self._clear_error_retries = 0
    self._disarm_retries = 0
    self._request_control_log = False
    self._request_adc_log = False

    self._dyno_mode = dyno_mode
    if dyno_mode:
      self._command = pack_avionics_messages.DynoCommandMessage()
    else:
      self._command = pack_avionics_messages.ControllerCommandMessage()
    self._command.motor_command = flags.kMotorCommandNone
    self._command_lock = threading.Lock()
    self._command_function_lock = threading.Lock()

    self._torque_func = lambda _: 0.0
    self._omega_lower_func = lambda _: 0.0
    self._omega_upper_func = lambda _: 0.0

    self._freeze_command = False  # Replace command with a constant on stop.
    self._WriteMotorCommand()

    super(Runner, self).__init__(['kMessageTypeControllerCommand',
                                  'kMessageTypeDynoCommand'])
    self.start()

  def SetCommand(self, command_mask):
    with self._command_lock:
      self._command.motor_command |= command_mask

  def _ClearCommand(self, command_mask):
    with self._command_lock:
      self._command.motor_command &= ~command_mask

  def IsRunning(self):
    return self._command.motor_command & flags.kMotorCommandRun

  def StartRun(self, duration):
    self._start_time = time.time()
    self._stop_time = self._start_time + duration
    self.SetCommand(flags.kMotorCommandRun)

  def StopRun(self):
    if self._freeze_command:
      motor_cmd = self.GetCommand()
      with self._command_function_lock:
        self._torque_func = lambda _: motor_cmd['torque']
        self._omega_lower_func = lambda _: motor_cmd['omega_lower']
        self._omega_upper_func = lambda _: motor_cmd['omega_upper']
    self._ClearCommand(flags.kMotorCommandRun)

  def GetCommand(self):
    """Generates motor commands at the current time.

    Returns:
      motor_cmd: Command to send to motors or dynos at the current time.
    """
    if self.IsRunning():
      curr_time = time.time() - self._start_time
    else:
      curr_time = 0.0

    with self._command_function_lock:
      motor_cmd = {'torque': self._torque_func(curr_time),
                   'omega_lower': self._omega_lower_func(curr_time),
                   'omega_upper': self._omega_upper_func(curr_time)}

    return motor_cmd

  def _CheckCommand(self, cmd_dict):
    for _, val in cmd_dict.iteritems():
      assert isinstance(val, list)
      assert len(val) == len(MOTORS)

  def _WriteMotorCommand(self):
    motor_cmd = self.GetCommand()

    for cmd, val in motor_cmd.iteritems():
      if isinstance(val, int) or isinstance(val, float):
        motor_cmd[cmd] = [val for _ in MOTORS]

    self._CheckCommand(motor_cmd)

    torque = motor_cmd['torque']
    omega_lower = motor_cmd['omega_lower']
    omega_upper = motor_cmd['omega_upper']

    with self._command_lock:
      for i, motor in enumerate(MOTORS):
        spin = self._spin_dir[i]
        if motor in self._motors:
          self._command.motor_torque[i] = torque[i] * spin
          self._command.motor_speed_lower_limit[i] = omega_lower[i] * spin
          self._command.motor_speed_upper_limit[i] = omega_upper[i] * spin
        else:
          self._command.motor_torque[i] = 0.0
          self._command.motor_speed_lower_limit[i] = 0.0
          self._command.motor_speed_upper_limit[i] = 0.0

  def SetCommandFunction(self, torque_func, omega_lower_func,
                         omega_upper_func, freeze_command):
    with self._command_function_lock:
      self._torque_func = torque_func
      self._omega_lower_func = omega_lower_func
      self._omega_upper_func = omega_upper_func
      self._freeze_command = freeze_command
    self._WriteMotorCommand()

  def GetTime(self):
    return time.time() - self._start_time if self.IsRunning() else 0.0

  def ClearErrors(self):
    self.SetCommand(flags.kMotorCommandClearError)
    self._clear_error_retries = 3

  def Disarm(self):
    self.SetCommand(flags.kMotorCommandDisarm)
    self._disarm_retries = 3

  def RequestControlLog(self):
    self._request_control_log = True

  def RequestAdcLog(self):
    self._request_adc_log = True

  def _RunOnce(self):
    """Modifies and sends the ControllerCommandMessage."""

    if self.IsRunning():
      if time.time() > self._stop_time:
        self.StopRun()
        print '\nFinished run.'
      else:
        try:
          self._WriteMotorCommand()
        except AssertionError:
          print ('Warning: Command(t) did not return a scalar or list with '
                 'elements for all motors.')
          self.StopRun()

    if self._clear_error_retries <= 0:
      self._ClearCommand(flags.kMotorCommandClearError)
    else:
      self._clear_error_retries -= 1

    if self._disarm_retries <= 0:
      self._ClearCommand(flags.kMotorCommandDisarm)
    else:
      self._disarm_retries -= 1

    if self._request_control_log:
      self.SetCommand(flags.kMotorCommandSendControlLog)
      self._request_control_log = False
    else:
      self._ClearCommand(flags.kMotorCommandSendControlLog)

    if self._request_adc_log:
      self.SetCommand(flags.kMotorCommandSendAdcLog)
      self._request_adc_log = False
    else:
      self._ClearCommand(flags.kMotorCommandSendAdcLog)

    with self._command_lock:
      if self._dyno_mode:
        self._client.Send(self._command, 'kMessageTypeDynoCommand', OPERATOR)
      else:
        self._client.Send(self._command, 'kMessageTypeControllerCommand',
                          CONTROLLER)

    time.sleep(0.0095)


if __name__ == '__main__':
  client = MotorCommandClient()
  try:
    client.cmdloop()
  except BaseException:
    client.TryStopThreads()
    raise

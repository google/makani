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

"""Command line client for controlling ground power."""

import os
import re
import socket
import threading
import time

import makani
from makani.avionics.common import aio
from makani.avionics.common import cmd_client
from makani.avionics.common import pack_avionics_messages
from makani.avionics.ground_power.q7 import flags
from makani.avionics.ground_power.q7 import inverter_types
from makani.avionics.network import message_type
from makani.gs.monitor2.apps.plugins.indicators import ground_power_faults


def BuildInverterDict(file_path, search_string):
  """Build inverter dictionary from constants in inverter_types.h."""
  filename = os.path.join(makani.HOME, file_path)
  with open(filename) as f:
    f_text = f.read()

  # Get parameter array string.
  re_string = r'typedef enum {\s*^([\s\S]*)^} '+search_string
  array_string = re.search(re_string, f_text, re.MULTILINE)

  if 'GroundPowerCommand' in search_string:
    re_string = search_string+r'(\S+)'
    inverter_keys = re.findall(re_string, array_string.group(1),
                               re.MULTILINE)
    return {key: ind for ind, key in enumerate(inverter_keys)}

  elif 'InverterRegister' in search_string:
    re_string = search_string+r'(\S+)(\W+)(\d+)'
    inverter_keys = re.findall(re_string, array_string.group(1),
                               re.MULTILINE)
    return {k: int(v) for (k, _, v) in inverter_keys}

  else:
    return {}


def BuildSetParamDict(inverter_params):
  set_param_dict = {}
  for param in inverter_params.keys():
    if 'Set' in param:
      set_param_dict[param.replace('Set', '')] = inverter_params[param]
  return set_param_dict


def GetStrandedArgs(data):
  """Get arguments for SetParam absorbed by cmd_client.SelectArgs()."""
  try:
    stringlit = re.findall(STRING_LITERAL, data)
    args = data.split(stringlit[0])
    value = int(args[1])
  except:
    raise GroundPowerClientError(
        'Invalid or no argument supplied for parameter value.')
  return value


# Constants.
# TODO: Use h2py architecture to import these enums.
INVERTER_COMMANDS = BuildInverterDict('avionics/ground_power/q7/flags.h',
                                      'GroundPowerCommand')
INVERTER_PARAMS = BuildInverterDict('avionics/ground_power/q7/inverter_types.h',
                                    'InverterRegister')
INVERTER_SET_PARAMS = BuildSetParamDict(INVERTER_PARAMS)
GROUND_POWER_NODE = 'kAioNodeGroundPowerQ7A'
OPERATOR = 'kAioNodeOperator'
STRING_LITERAL = re.compile(r'''([A-Za-z_][A-Za-z0-9_]*)''')
MAX_STALE_TIME = 10
MAX_STALE_COUNT = 4
MAX_DC_VOLTS = 850
MIN_DC_VOLTS = 500
MAX_RETRIES = 3

INVERTER_STATUS = {
    inverter_types.kInverterStatusRunning: 'RUNNING',
    inverter_types.kInverterStatusStopped: 'READY/STOPPED',
    inverter_types.kInverterStatusEStopped: 'E-STOP',
    inverter_types.kInverterStatusFaulted: 'FAULTED',
    inverter_types.kInverterStatusInit: 'INIT'
}

# Targets are in least-positive-inverter-first order.
DEFAULTS = {
    'targets': ['Inv1', 'Inv2', 'Inv3', 'Inv4', 'Inv5', 'Inv6'],
    'start_sequence': ['Inv3', 'Inv4', 'Inv2', 'Inv5', 'Inv1', 'Inv6'],
    'stop_sequence': ['Inv6', 'Inv1', 'Inv5', 'Inv2', 'Inv4', 'Inv3']
}


def InverterNameToId(inverter):
  return int(inverter[len('Inv'):]) - 1


def InverterIdToString(inverter_id):
  return 'Inv%d' % (inverter_id + 1)


class GroundPowerClientError(cmd_client.WingClientError):
  pass


class Inverter(object):
  """Inverter object to keep track of inverter state."""

  def __init__(self, inverter_id, listener):
    self._id = inverter_id
    self._listener = listener
    self._max_retry_count = MAX_RETRIES
    self._max_command_attempts = 2
    self._stale_time_exceeded = False
    self._stale_count_exceeded = False
    self._command_delay = 1.0
    self._set_param_delay = 0.1
    self._get_param_delay = 0.1
    self._status = None

    self.state_transitions = {
        # {current_state : {target_state1: command1}, ...}
        inverter_types.kInverterStatusRunning:
        {
            inverter_types.kInverterStatusStopped:
            flags.kGroundPowerCommandStop,
        },
        inverter_types.kInverterStatusStopped:
        {
            inverter_types.kInverterStatusRunning:
            flags.kGroundPowerCommandStart,
        },
        inverter_types.kInverterStatusEStopped:
        {},
        inverter_types.kInverterStatusFaulted:
        {},
        inverter_types.kInverterStatusInit:
        {},
    }

    self._set_param_aio_client = aio.AioClient(
        ['kMessageTypeGroundPowerSetParam'], timeout=0.2)
    self._ack_param_aio_client = aio.AioClient(
        ['kMessageTypeGroundPowerAckParam'], timeout=0.2)
    self._get_param_aio_client = aio.AioClient(
        ['kMessageTypeGroundPowerGetParam'], timeout=0.2)
    self._command_aio_client = aio.AioClient(
        ['kMessageTypeGroundPowerCommand'], timeout=0.2)

  def InverterStatusToString(self, status):
    """Return appropriate string for status."""
    if status in INVERTER_STATUS.keys():
      return INVERTER_STATUS[status]
    else:
      return 'INVALID'

  def GetState(self):
    """Get current inverter state."""
    msg = self._listener.GetMsg(self._id)
    self._stale_time = time.time() - self._listener.GetMsgTime(self._id)
    self._stale_count = msg.stale_count
    self._status = msg.inverter_status
    self._stale_time_exceeded = self._stale_time > MAX_STALE_TIME
    self._stale_count_exceeded = self._stale_count > MAX_STALE_COUNT

  def SetState(self, new_state):
    """Set inverter to new state: new_state."""
    self.GetState()

    if new_state not in self.state_transitions[self._status]:
      if self._status == new_state:
        print '%s is already in state %s.' % (
            InverterIdToString(self._id),
            self.InverterStatusToString(self._status))
        return True
      raise GroundPowerClientError('Cannot transition from %s to %s.' % (
          self.InverterStatusToString(self._status),
          self.InverterStatusToString(new_state)))

    message = pack_avionics_messages.GroundPowerCommandMessage()
    allowed_transitions = self.state_transitions[self._status]
    message.command = allowed_transitions[new_state]
    message.id = self._id

    for _ in xrange(self._max_command_attempts):
      print 'Sending command to %s.' % (
          InverterIdToString(self._id))
      self._command_aio_client.Send(message,
                                    'kMessageTypeGroundPowerCommand',
                                    OPERATOR)

      for _ in xrange(self._max_retry_count):
        time.sleep(self._command_delay)

        if self._stale_time_exceeded:
          print 'No messages received from inverter in %s seconds.' % (
              MAX_STALE_TIME)
          print 'Unable to set new state.'
          return False

        if self._stale_count_exceeded:
          print 'Stale count is greater than %s.' % (MAX_STALE_COUNT)
          print 'Unable to set new state for %s.' % (
              InverterIdToString(self._id))
          return False

        self.GetState()

        if self._status == new_state:
          print '%s: success.' % (InverterIdToString(self._id))
          return True

    print '%s: Unable to set state: Command attempts exceeded.' % (
        InverterIdToString(self._id))
    print 'Use "get_status" to check if inverters are offline.'
    return False

  def TrySetParam(self, param, value):
    """Set inverter param to value."""
    message = pack_avionics_messages.GroundPowerSetParamMessage()
    message.id = self._id
    message.modbus_register = param
    message.value = value

    for _ in xrange(self._max_command_attempts):
      self._set_param_aio_client.Send(message,
                                      'kMessageTypeGroundPowerSetParam',
                                      OPERATOR)
      time.sleep(self._set_param_delay)
      for _ in xrange(self._max_retry_count):
        try:
          _, header, ack = self._ack_param_aio_client.Recv()
          if (header.type == message_type.kMessageTypeGroundPowerAckParam
              and ack.id == message.id
              and ack.value == message.value
              and ack.modbus_register == param):
            print '%s: success.' % (InverterIdToString(self._id))
            return True
        except socket.timeout:
          pass
    return False

  def TryGetParam(self, param):
    """Get value for inverter param."""
    message = pack_avionics_messages.GroundPowerGetParamMessage()
    message.id = self._id
    message.modbus_register = param

    for _ in xrange(self._max_command_attempts):
      self._get_param_aio_client.Send(message,
                                      'kMessageTypeGroundPowerGetParam',
                                      OPERATOR)
      for _ in xrange(self._max_retry_count):
        try:
          _, header, ack = self._ack_param_aio_client.Recv()
          if (header.type == message_type.kMessageTypeGroundPowerAckParam
              and ack.id == message.id and ack.modbus_register == param):
            print '%s: register: %s: value: %g.' % (
                InverterIdToString(self._id),
                ack.modbus_register,
                ack.value)
            return True
        except socket.timeout:
          pass
    return False

  def GetFaults(self):
    """Get faults from inverter."""
    self.GetState()
    if self._stale_time_exceeded:
      print '%s: No messages received in %s seconds.' % (
          InverterIdToString(self._id), MAX_STALE_TIME)
      return

    if not self._status:
      print '%s: No message data available.' % (
          InverterIdToString(self._id))

    msg = self._listener.GetMsg(self._id)
    fault_words = [msg.fault_word1, msg.fault_word2, msg.fault_word3,
                   msg.fault_word4, msg.fault_word5, msg.fault_word6,
                   msg.fault_word7, msg.fault_word8]

    faults = []
    for fault in fault_words:
      inverter_faults = []
      if fault:
        for n in range(16):
          if fault & (1 << n):
            fault_str = ground_power_faults.INVERTER_FAULTS[
                fault_words.index(fault)][n]
            inverter_faults.append(fault_str)
        faults.append(inverter_faults)
    flt_str = ''
    for fw in faults:
      for flt in fw:
        flt_str += flt + ', '

    print 'Inverter {0}: {1}'.format(InverterIdToString(self._id), flt_str)

  def PrintStatus(self):
    """Print the current status of the inverter."""
    active = True
    if self._stale_time_exceeded:
      active = False
    elif self._stale_count_exceeded:
      active = False
    elif not self._status:
      active = False
    print '%s status: %s, active: %s.' % (
        InverterIdToString(self._id),
        self.InverterStatusToString(self._status),
        active
        )


class Listener(cmd_client.AioThread):
  """Continuously listen to GroundPowerStatusMessages."""

  def __init__(self, inverters):
    self._inverters = inverters.copy()
    self._inverter_sources = {InverterNameToId(m):
                              m for m in DEFAULTS['targets']}

    t_now = time.time()
    self._ground_power_status = {InverterNameToId(m):
                                 inverter_types.kInverterStatusInit
                                 for m in DEFAULTS['targets']}

    self._ground_power_status_lock = threading.Lock()
    self._t_message = {InverterNameToId(m): t_now for m in self._inverters}
    self._t_message_lock = threading.Lock()
    self._inverter_msgs = {}
    self._inverter_msgs_lock = threading.Lock()
    self._stale = {InverterNameToId(m): True for m in self._inverters}

    super(Listener, self).__init__(['kMessageTypeGroundPowerStatus'],
                                   allowed_sources=['kAioNodeGroundPowerQ7A',],
                                   timeout=0.1)

    self.start()

  def _RunOnce(self):
    try:
      _, _, msg = self._client.Recv()
      inverter = msg.id
      t_now = time.time()

      with self._t_message_lock:
        self._t_message[inverter] = t_now
      with self._ground_power_status_lock:
        self._ground_power_status[inverter] = msg.inverter_status
      with self._inverter_msgs_lock:
        self._inverter_msgs[inverter] = msg

    except socket.timeout:
      pass

  def GetStaleCount(self, inverter_id):
    try:
      stale_count = self._inverter_msgs[inverter_id].stale_count
    except KeyError:
      raise GroundPowerClientError(
          'GetStaleCount(): No messages received from %s.' % (
              InverterIdToString(inverter_id)))
    return stale_count

  def GetStatus(self, inverter_id):
    try:
      status = self._ground_power_status[inverter_id]
    except KeyError:
      return None
    return status

  def GetMsgTime(self, inverter_id):
    msg_time = self._t_message[inverter_id]
    return msg_time

  def GetMsg(self, inverter_id):
    try:
      msg = self._inverter_msgs[inverter_id]
      return msg
    except KeyError:
      raise GroundPowerClientError(
          'GetMsg(): No messages received from %s.' % (
              InverterIdToString(inverter_id)))


class GroundPowerCommandClient(cmd_client.WingCommandClient):
  """Command line client for running ground power."""
  prompt = '(ground_power) '
  _NUM_RETRIES = 10

  def __init__(self, *args, **kwargs):
    cmd_client.WingCommandClient.__init__(self, *args, **kwargs)

    self._model = 'equinox'
    self._ground_power_listener = None
    self._SetTargets('', set_defaults=True)
    self._set_param_aio_client = aio.AioClient(
        ['kMessageTypeGroundPowerSetParam'], timeout=0.2)
    self._ack_param_aio_client = aio.AioClient(
        ['kMessageTypeGroundPowerAckParam'], timeout=0.2)
    self._get_param_aio_client = aio.AioClient(
        ['kMessageTypeGroundPowerGetParam'], timeout=0.2)
    self._command_aio_client = aio.AioClient(
        ['kMessageTypeGroundPowerCommand'], timeout=0.2)
    self._set_net_load_aio_client = aio.AioClient(
        ['kMessageTypeLoadbankSetLoad'], timeout=0.2)
    self._set_loadbank_state_aio_client = aio.AioClient(
        ['kMessageTypeLoadbankSetState'], timeout=0.2)
    self._ack_net_load_aio_client = aio.AioClient(
        ['kMessageTypeLoadbankAckParam'], timeout=0.2)
    self._ack_loadbank_state_aio_client = aio.AioClient(
        ['kMessageTypeLoadbankStateAckParam'], timeout=0.2)

  def TryStopThreads(self):
    if self._ground_power_listener:
      self._ground_power_listener.TryStop()

  def _SetTargets(self, line, **kwargs):
    """Sets inverter targets.

    Args:
      line: User supplied arguments specifying target inverters.
      **kwargs: set_defaults present if defaults should be used.

    Raises:
      GroundPowerClientError: An invalid set of targets was supplied.
    """
    if not kwargs['set_defaults']:
      if self._inverters_selected:
        self._ClearTargets()

      targets_selected, _ = cmd_client.SelectArgs(
          line.split(), DEFAULTS['targets'], require_some=True,
          require_all=True, select_all=True, require_one=False)

      self._inverters_selected = sorted(targets_selected)

      if self._inverters_selected == DEFAULTS['targets']:
        self._inverter_start_sequence = DEFAULTS['start_sequence']
        self._inverter_stop_sequence = DEFAULTS['stop_sequence']
      else:
        self._inverter_start_sequence = sorted(targets_selected)
        self._inverter_stop_sequence = sorted(targets_selected)

    else:
      self._inverters_selected = DEFAULTS['targets']
      self._inverter_start_sequence = DEFAULTS['start_sequence']
      self._inverter_stop_sequence = DEFAULTS['stop_sequence']

    if self._inverters_selected:
      print 'Inverters selected: %s.' % ', '.join(
          sorted(self._inverters_selected))
      print 'Start sequence:     %s.' % ', '.join(
          self._inverter_start_sequence)
      print 'Stop sequence:      %s.' % ', '.join(
          self._inverter_stop_sequence)
      self._ground_power_listener = Listener(set(self._inverters_selected))
      self._inverter_list = [Inverter(inverter_id, self._ground_power_listener)
                             for inverter_id
                             in xrange(inverter_types.kNumInverters)]

  @cmd_client.Command()
  def do_set_targets(self, line):  # pylint: disable=invalid-name
    """Sets inverter targets e.g. "set_targets Inv1 Inv2 Inv3"."""
    self._SetTargets(line, set_defaults=False)

  def complete_set_targets(self, text, *unused_args):  # pylint: disable=invalid-name
    return self._CompleteArg(text, sorted(DEFAULTS['targets']) + ['All'])

  @cmd_client.Command(num_args=0)
  def do_get_targets(self, line):  # pylint: disable=invalid-name
    """Displays inverter targets specified by "set_targets"."""
    if not self._inverters_selected:
      raise GroundPowerClientError('No targets selected. Use "set_targets".')

    print 'Inverters: %s.' % (
        ', '.join(self._inverters_selected))

  def _ClearTargets(self):
    """Clears selected inverter targets."""
    if not self._inverters_selected:
      raise GroundPowerClientError('No targets  selected. Use "set_targets".')

    old_inverters = set(self._inverters_selected).copy()
    self.TryStopThreads()
    self._inverters_selected = set()
    self._ground_power_listener = None
    self._ground_power_runner = None
    print 'Cleared old targets.\nOld Inverters: %s.' % (
        ', '.join(sorted(old_inverters)))

  @cmd_client.Command(num_args=0)
  def do_clear_targets(self, line):  # pylint: disable=invalid-name
    """Clears selected inverter targets."""
    self._ClearTargets()

  def _GetStatus(self, line):
    if self._inverters_selected:
      for inverter in self._inverters_selected:
        try:
          self._inverter_list[InverterNameToId(inverter)].GetState()
          self._inverter_list[InverterNameToId(inverter)].PrintStatus()
        except GroundPowerClientError:
          print 'Failed to get status from %s.' % inverter
    else:
      raise GroundPowerClientError('No targets selected. Use "set_targets".')

  @cmd_client.Command(num_args=0)
  def do_get_status(self, line):  # pylint: disable=invalid-name
    """Get the status of selected inverter targets, e.g. "get_status"."""
    self._GetStatus(line)

  @cmd_client.Command(num_args=0)
  def do_get_defaults(self, line):  # pylint: disable=invalid-name
    """Prints default start and stop sequences for inverter targets."""
    print 'Inverters:      %s.' % ', '.join(DEFAULTS['targets'])
    print 'Start sequence: %s.' % ', '.join(DEFAULTS['start_sequence'])
    print 'Stop sequence:  %s.' % ', '.join(DEFAULTS['stop_sequence'])

  @cmd_client.Command(num_args=0)
  def do_get_start_sequence(self, line):
    """Prints current start sequence for inverter targets."""
    if not self._inverters_selected:
      raise GroundPowerClientError('No targets selected. Use "set_targets".')

    if self._inverter_start_sequence:
      print 'Start sequence: %s.' % ', '.join(self._inverter_start_sequence)
    else:
      raise GroundPowerClientError(
          'No start sequence defined. Use "set_start_sequence".')

  @cmd_client.Command(num_args=0)
  def do_get_stop_sequence(self, line):
    """Prints current stop sequence for inverter targets."""
    if self._inverter_stop_sequence:
      print 'Stop sequence: %s.' % ', '.join(self._inverter_stop_sequence)
    else:
      raise GroundPowerClientError(
          'No stop sequence defined. Use "set_stop_sequence".')

  @cmd_client.Command()
  def do_set_start_sequence(self, line):  # pylint: disable=invalid-name
    """Sets start sequence for selected inverters."""
    if not self._inverters_selected:
      raise GroundPowerClientError('No targets selected. Use "set_targets".')

    targets_selected, _ = cmd_client.SelectArgs(
        line.split(), self._inverters_selected, require_some=False,
        require_all=True, select_all=False, require_one=False)

    if len(targets_selected) != len(self._inverters_selected):
      print 'Start sequence must contain all selected inverters.'
    else:
      self._inverter_start_sequence = line.split()
      print 'Setting start sequence to: %s.' % ', '.join(
          self._inverter_start_sequence)

  def complete_start_sequence(self, text, *unused_args):  # pylint: disable=invalid-name
    return self._CompleteArg(text, sorted(self._inverters_selected) + ['All'])

  @cmd_client.Command()
  def do_set_stop_sequence(self, line):  # pylint: disable=invalid-name
    """Sets stop sequence for selected inverters."""
    if not self._inverters_selected:
      raise GroundPowerClientError('No targets selected. Use "set_targets".')

    targets_selected, _ = cmd_client.SelectArgs(
        line.split(), self._inverters_selected, require_some=False,
        require_all=True, select_all=False, require_one=False)

    if len(targets_selected) != len(self._inverters_selected):
      print 'Stop sequence must contain all selected inverters.'
    else:
      self._inverter_stop_sequence = line.split()
      print 'Setting stop sequence to: %s.' % ', '.join(
          self._inverter_stop_sequence)

  complete_stop_sequence = complete_start_sequence

  @cmd_client.Command()
  def do_start(self, line):  # pylint: disable=invalid-name
    """Starts specified inverters, e.g start Inv1 Inv2 Inv3."""
    if not self._inverters_selected:
      raise GroundPowerClientError('No targets selected. Use "set_targets".')

    targets, _ = cmd_client.SelectArgs(
        line.split(), self._inverters_selected, require_some=False,
        require_all=True, select_all=True, require_one=False)

    if not targets:
      raise GroundPowerClientError('No valid arguments selected.')

    if sorted(targets) == sorted(self._inverters_selected):
      for target in self._inverter_start_sequence:
        self._inverter_list[InverterNameToId(target)].SetState(
            inverter_types.kInverterStatusRunning)
    else:
      target_order = line.split()
      for target in target_order:
        self._inverter_list[InverterNameToId(target)].SetState(
            inverter_types.kInverterStatusRunning)

  complete_start = complete_start_sequence

  @cmd_client.Command()
  def do_stop(self, line):  # pylint: disable=invalid-name
    """Stops specified inverters, e.g. "stop Inv1 Inv2 Inv3"."""
    if not self._inverters_selected:
      raise GroundPowerClientError('No targets selected. Use "set_targets".')

    targets, _ = cmd_client.SelectArgs(
        line.split(), self._inverters_selected, require_some=False,
        require_all=True, select_all=True, require_one=False)

    if not targets:
      raise GroundPowerClientError('No valid arguments selected.')

    if sorted(targets) == sorted(self._inverters_selected):
      for target in self._inverter_stop_sequence:
        try:
          self._inverter_list[InverterNameToId(target)].SetState(
              inverter_types.kInverterStatusStopped)
        except GroundPowerClientError:
          print 'Failed to stop %s.' % target
    else:
      target_order = line.split()
      for target in target_order:
        try:
          self._inverter_list[InverterNameToId(target)].SetState(
              inverter_types.kInverterStatusStopped)
        except GroundPowerClientError:
          print 'Failed to stop %s.' % target

  complete_stop = complete_start_sequence

  @cmd_client.Command()
  def do_get_faults(self, line):  # pylint: disable=invalid-name
    """Gets faults for specified inverters, e.g "get_faults Inv1 Inv2 Inv3"."""
    if not self._inverters_selected:
      raise GroundPowerClientError('No targets selected. Use "set_targets".')

    targets, _ = cmd_client.SelectArgs(
        line.split(), self._inverters_selected, require_some=False,
        require_all=True, select_all=True, require_one=False)

    if not targets:
      raise GroundPowerClientError('No valid arguments selected.')

    for target in sorted(targets):
      try:
        print self._inverter_list[InverterNameToId(target)].GetFaults()
      except GroundPowerClientError:
        print 'Failed to get faults from %s.' % target

  complete_get_faults = complete_set_targets

  def _SetInverterParam(self, inverter_id, param, value):  # pylint: disable=invalid-name
    """Sets a parameter for specified inverter."""
    if InverterIdToString(inverter_id) not in self._inverters_selected:
      raise GroundPowerClientError('%s is not a selected target.' %
                                   InverterIdToString(inverter_id))
    return self._inverter_list[inverter_id].TrySetParam(param, value)

  def _SetParam(self, line, **kwargs):  # pylint: disable=invalid-name
    """Sets a parameter for a specified inverter."""

    if not self._inverters_selected:
      raise GroundPowerClientError('No targets selected. Use "set_targets".')

    targets, args = cmd_client.SelectArgs(
        line.split(), self._inverters_selected, require_some=True,
        select_all=True)

    if kwargs:
      param, args = cmd_client.SelectArgs(
          args, ['value'], require_one=True, select_all=False)
      if args:
        try:
          value = int(args[0])
        except ValueError:
          raise GroundPowerClientError('Invalid value: ', args[0])

        if value not in range(
            int(kwargs['lower_lim']), int(kwargs['upper_lim'])):
          raise GroundPowerClientError('Supplied value out of range.')

        if any('scaling' in k for k in kwargs):
          value = int(kwargs['scaling']) * value
      else:
        value = GetStrandedArgs(line)
      modbus_reg = INVERTER_SET_PARAMS[kwargs['param']]

    else:
      param, args = cmd_client.SelectArgs(
          args, INVERTER_SET_PARAMS.keys(), require_one=True, select_all=False)

      if args:
        try:
          value = int(args[0])
        except ValueError:
          raise GroundPowerClientError('Invalid value: ', args[0])
      else:
        value = GetStrandedArgs(line)
      modbus_reg = INVERTER_SET_PARAMS[param]

    failed_targets = []

    for target in sorted(targets):
      print 'Setting %s to %g on %s.' % (param, value, target)

      success = self._inverter_list[
          InverterNameToId(target)].TrySetParam(modbus_reg, value)
      if not success:
        failed_targets.append(target)

    if failed_targets:
      raise GroundPowerClientError('Failed to verify %s from %s.'
                                   % (param, failed_targets))

  @cmd_client.Command(num_args=3)
  def do_set_param(self, line):  # pylint: disable=invalid-name
    """Sets modbus parameter for specified inverters."""
    self._SetParam(line)

  def complete_set_param(self, text, line, *unused_args):  # pylint: disable=invalid-name
    arg_number = len(line.split())
    if not text:
      arg_number += 1

    if arg_number == 2:
      return self._CompleteArg(text, sorted(self._inverters_selected) + ['All'])
    elif arg_number == 3:
      return self._CompleteArg(text, sorted(INVERTER_SET_PARAMS.keys()))
    else:
      return []

  def _GetParam(self, line):  # pylint: disable=invalid-name
    """Sets a parameter for a specified inverter."""
    if not self._inverters_selected:
      raise GroundPowerClientError('No targets selected. Use set_targets.')

    targets, args = cmd_client.SelectArgs(
        line.split(), self._inverters_selected, require_some=True,
        select_all=True)

    args, param = cmd_client.SelectArgs(
        args, INVERTER_PARAMS.keys(), require_some=True, select_all=False)

    param = list(args)[0]
    modbus_param = INVERTER_PARAMS[param]
    failed_targets = []

    for target in sorted(targets):
      print 'Getting %s from %s.' % (param, target)
      success = self._inverter_list[InverterNameToId(target)].TryGetParam(
          modbus_param)
      if not success:
        failed_targets.append(target)
    if failed_targets:
      raise GroundPowerClientError(
          'Failed to get value %s from %s.' % (param, failed_targets))

  @cmd_client.Command(num_args=2)
  def do_get_param(self, line):  # pylint: disable=invalid-name
    """Gets a parameter value for inverters, e.g. "get_param 1 SetDCVolts"."""
    self._GetParam(line)

  complete_get_param = complete_set_param

  @cmd_client.Command(num_args=3)
  def do_set_tethercompscale(self, line):  # pylint: disable=invalid-name
    """Sets tether compensation scale, e.g. set_tethercompscale 1 value 100."""
    self._SetParam(line, param='TetherCompScale', lower_lim='0',
                   upper_lim='1000')

  @cmd_client.Command(num_args=3)
  def do_set_tetherfilterval(self, line):  # pylint: disable=invalid-name
    """Sets tether filter value, e.g. set_tetherfilterval 1 value 100."""
    self._SetParam(line, param='TetherFilterVal', lower_lim='0',
                   upper_lim='1000')

  @cmd_client.Command(num_args=3)
  def do_set_tetherresistance(self, line):  # pylint: disable=invalid-name
    """Sets tether resistance in mohms, e.g. set_tetherresistance 1 value 10."""
    self._SetParam(line, param='TetherResistance', lower_lim='0',
                   upper_lim='1000')

  @cmd_client.Command(num_args=3)
  def do_set_dcvolts(self, line):  # pylint: disable=invalid-name
    """Sets DCVolts to a value, e.g. set_dcvolts 1 value 800."""
    self._SetParam(line, param='DCVolts', scaling='10', lower_lim='0',
                   upper_lim='800')

  def complete_set_dcvolts(self, text, line, *unused_args):  # pylint: disable=invalid-name
    arg_number = len(line.split())
    if not text:
      arg_number += 1

    if arg_number == 2:
      return self._CompleteArg(text, sorted(self._inverters_selected) + ['All'])
    elif arg_number == 3:
      return self._CompleteArg(text, ['value'])
    else:
      return []

  complete_set_tethercompscale = complete_set_dcvolts

  complete_set_tetherfilterval = complete_set_dcvolts

  complete_set_tetherresistance = complete_set_dcvolts

  def _SetNetLoad(self, line):  # pylint: disable=invalid-name
    """Sets total requested load from kite + loadbanks."""
    args = line.split()

    try:
      load = int(args[0])
    except ValueError:
      raise GroundPowerClientError('Please enter desired load as integer # kW.')

    if load < 0:
      raise GroundPowerClientError('Please supply positive integer load value.')

    # Send LoadbankSetLoadMessage over AIO network.
    message = pack_avionics_messages.LoadbankSetLoadMessage()
    message.desired_load_kw = load
    print 'Setting net loadbank load to %skW.' % load

    for _ in xrange(self._NUM_RETRIES):
      self._set_net_load_aio_client.Send(message,
                                         'kMessageTypeLoadbankSetLoad',
                                         OPERATOR)
      for _ in xrange(self._NUM_RETRIES):
        try:
          _, header, ack = self._ack_net_load_aio_client.Recv()
          if (header.type == message_type.kMessageTypeLoadbankAckParam
              and ack.value == message.desired_load_kw):
            print 'Loadbank load set to: %skW.' % ack.value
            return True
        except socket.timeout:
          return False
      return False

  @cmd_client.Command(num_args=1)
  def do_set_net_load_kw(self, line):  # pylint: disable=invalid-name
    """Set total requested load in kW from kite + loadbanks."""
    if not self._SetNetLoad(line):
      raise GroundPowerClientError('Failed to set loadbank load.')

  def _SetLoadbankState(self, state_bool):  # pylint: disable=invalid-name
    """Sets loadbank to on/off, thus activating or deactivating relays."""

    # Send LoadbankSetStateMessage over AIO network.
    message = pack_avionics_messages.LoadbankSetStateMessage()
    if state_bool:
      message.activate_loadbank = True
      print 'Activating loadbank.'
    else:
      message.activate_loadbank = False
      print 'Deactivating loadbank.'

    for _ in xrange(self._NUM_RETRIES):
      self._set_loadbank_state_aio_client.Send(message,
                                               'kMessageTypeLoadbankSetState',
                                               OPERATOR)
      for _ in xrange(self._NUM_RETRIES):
        try:
          _, header, ack = self._ack_loadbank_state_aio_client.Recv()
          if (header.type == message_type.kMessageTypeLoadbankStateAckParam
              and ack.value == message.activate_loadbank):
            if ack.value:
              print 'Loadbank turned on.'
            else:
              print 'Loadbank turned off.'
            return True
        except socket.timeout:
          return False
      return False

  @cmd_client.Command(num_args=0)
  def do_loadbank_turn_on(self, line):  # pylint: disable=invalid-name
    """Turn loadbank on to activate relays and start drawing power."""
    if not self._SetLoadbankState(True):
      raise GroundPowerClientError('Failed to set loadbank state.')

  @cmd_client.Command(num_args=0)
  def do_loadbank_turn_off(self, line):  # pylint: disable=invalid-name
    """Turn loadbank off to deactivate relays and set power draw to 0."""
    if not self._SetLoadbankState(False):
      raise GroundPowerClientError('Failed to set loadbank state.')

  def _FISetState(self, **kwargs):
    """Sets fault inductor state."""
    failed_targets = []
    if kwargs['toggle'] == 'enable':
      modbus_val = inverter_types.kInverterConstsFaultInductorEnable
      param = INVERTER_SET_PARAMS['SetFaultInductor']
    elif kwargs['toggle'] == 'disable':
      modbus_val = inverter_types.kInverterConstsFaultInductorDisable
      param = INVERTER_SET_PARAMS['SetFaultInductor']
    elif kwargs['toggle'] == 'reset':
      modbus_val = inverter_types.kInverterConstsFaultInductorReset
      param = INVERTER_SET_PARAMS['SetFaultInductorReset']

    for target in self._inverters_selected:
      print 'Sending fault inductor %s to %s.' % (kwargs['toggle'], target)
      success = self._SetInverterParam(InverterNameToId(target),
                                       param, modbus_val)
      if not success:
        failed_targets.append(target)

    if failed_targets:
      raise GroundPowerClientError('Failed to verify fault inductor %s from %s.'
                                   % (kwargs['toggle'], failed_targets))

  @cmd_client.Command(num_args=0)
  def do_fi_enable_all(self, line):  # pylint: disable=invalid-name
    """Enables fault inductor on all inverters."""
    self._FISetState(toggle='enable')

  @cmd_client.Command(num_args=0)
  def do_fi_disable_all(self, line):  # pylint: disable=invalid-name
    """Disables fault inductors on all inverters."""
    self._FISetState(toggle='disable')

  @cmd_client.Command(num_args=0)
  def do_source(self, line):  # pylint: disable=invalid-name
    """Source is currently unsupported in ground_power_client."""
    pass


if __name__ == '__main__':
  client = GroundPowerCommandClient()
  try:
    client.cmdloop()
  except BaseException:
    client.TryStopThreads()
    raise

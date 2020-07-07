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

"""Command line client for referencing detwist."""

import socket
import sys
import time

import gflags
from makani.avionics.common import actuator_types
from makani.avionics.common import aio
from makani.avionics.common import cmd_client
from makani.avionics.common import pack_avionics_messages

DETWIST_POS_RESOLUTION = 0.001  # [rad]
ACTUATOR_STATE_NAMES = {val: key for key, val
                        in actuator_types.__dict__.items()
                        if (key.startswith('kActuatorState')
                            and not key.startswith('kActuatorStateCommand'))}

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('ignore_state', False, 'Send command, ignoring PLC state.')
gflags.DEFINE_bool('clear_errors', False, 'Send clear errors command, then '
                   'send reference command.')
gflags.DEFINE_float('new_pos', 0, 'New value for current detwist position.')

listener = None


class Listener(cmd_client.AioThread):
  """Continuously listens to GroundStationPlcStatusMessages."""

  def __init__(self):
    self.status_msg = None
    self.status_msg_time = 0
    super(Listener, self).__init__(['kMessageTypeGroundStationPlcStatus'],
                                   timeout=0.1)
    self.start()

  def _RunOnce(self):
    try:
      _, _, msg = self._client.Recv()
      self.status_msg_time = time.time()
      self.status_msg = msg
    except socket.timeout:
      pass


def _SendClearErrors():
  """Sends 'clear errors' command to PLC AIO node."""
  msg = pack_avionics_messages.GroundStationDetwistSetStateMessage()
  msg.state_command = actuator_types.kActuatorStateCommandClearErrors
  messages = ['kMessageTypeGroundStationDetwistSetState']
  client = aio.AioClient(messages)
  client.Send(msg,
              'kMessageTypeGroundStationDetwistSetState',
              'kAioNodeOperator')


def _SendReferenceCommand(ref_position):
  """Sends 'reference' command to PLC AIO node."""
  msg = pack_avionics_messages.GroundStationPlcOperatorMessage()
  msg.command.detwist_cmd = pack_avionics_messages.kDetwistCommandReference
  msg.command.detwist_position = ref_position
  messages = ['kMessageTypeGroundStationPlcOperator']
  client = aio.AioClient(messages)
  client.Send(msg,
              'kMessageTypeGroundStationPlcOperator',
              'kAioNodeOperator')


def _PlcReady(timeout=1):
  """Returns True if PLC AIO node reports actuator state is 'ready'."""
  t_start = time.time()

  while time.time() - t_start < timeout:
    ready = (listener.status_msg and (listener.status_msg.detwist_state ==
                                      actuator_types.kActuatorStateReady))
    if ready and listener.status_msg_time >= t_start:
      return True
    time.sleep(0.01)
  if listener.status_msg_time < t_start:
    print 'No status messages from PLC.'
  else:
    print ('PLC not ready. Current state is %s.'
           % (ACTUATOR_STATE_NAMES[listener.status_msg.detwist_state]))
  return False


def _PlcReferenceSet(ref_position, timeout=1):
  """Returns True if detwist position matches the argument."""
  t_start = time.time()

  while time.time() - t_start < timeout:
    if listener.status_msg_time >= t_start:
      delta = listener.status_msg.plc.detwist_position - ref_position
      if abs(delta) < DETWIST_POS_RESOLUTION:
        return True
    time.sleep(0.01)
  if listener.status_msg_time < t_start:
    print 'No status messages from PLC.'
  else:
    print ('PLC not referenced. Current position is %f.'
           % (listener.status_msg.plc.detwist_position))
  return False


def ReferenceDetwist():
  """References the detwist and reports progress on console."""
  print 'Set current detwist position to %.2f.' % FLAGS.new_pos

  if FLAGS.clear_errors:
    print 'Clearing errors.'
    _SendClearErrors()
  if not FLAGS.ignore_state and not _PlcReady():
    print 'PLC not ready. Aborting.'
    listener.TryStop()
    sys.exit(1)

  print 'Sending reference command.'
  _SendReferenceCommand(FLAGS.new_pos)
  if not _PlcReferenceSet(FLAGS.new_pos):
    print 'Detwist may not be referenced. Check monitors.'
  else:
    print 'Detwist successfully referenced.'


if __name__ == '__main__':
  try:
    argv = FLAGS(sys.argv)
  except gflags.FlagsError, e:
    print '%s\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  listener = Listener()
  try:
    ReferenceDetwist()
  finally:
    listener.TryStop()

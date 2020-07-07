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

"""AIO message receiver."""

import collections
import copy
import logging
import threading
import time

from makani.avionics.linux.swig import aio_helper
from makani.avionics.linux.swig import aio_util
from makani.avionics.network import aio_node as aio_node_names
from makani.avionics.network import message_type as aio_message_type
from makani.avionics.network import network_config
from makani.gs.monitor2.apps.receiver import base_receiver
from makani.lib.python import c_helpers
from makani.lib.python.trackers import json_obj

_MESSAGE_TYPE_HELPER = c_helpers.EnumHelper('MessageType', aio_message_type)
_AIO_NODE_HELPER = c_helpers.EnumHelper('AioNode', aio_node_names)


class AioMessage(json_obj.JsonObj):
  """A class to temporarily store information about received AIO messages."""

  def __init__(self, buf, msg_enum, source, sequence, clock_us):
    self._buf = buf
    self._sequence = sequence
    self._timestamp = clock_us * 1.0e-6
    self._msg_enum = msg_enum
    self._source = source
    self._clock_us = clock_us
    message_type = _MESSAGE_TYPE_HELPER.Name(msg_enum)
    self._message = self._Unpack(buf, message_type)

    header = {
        'timestamp': self._timestamp,
        'sequence': self._sequence,
    }

    self._message.capture_info = header
    message_fields = (self._message._fields_)  # pylint: disable=protected-access
    if not message_fields or message_fields[-1][0] != 'capture_info':
      fields = copy.copy(message_fields)
      fields.append(('capture_info', dict))
      self._message._fields_ = fields  # pylint: disable=protected-access

  def _Unpack(self, buf, message_name):
    return aio_helper.UnpackMessage(buf, message_name)

  def Get(self, readonly):
    """Obtain a python object representation of the message.

    Args:
      readonly: True if the returned value is read-only.

    Returns:
      A nested dict about the message content.
    """
    return self._message if readonly else copy.deepcopy(self._message)

  def MessageEnum(self):
    return self._msg_enum

  def Source(self):
    return self._source

  def SequenceNum(self):
    return self._sequence

  def Timestamp(self):
    return self._timestamp

  def ClockUs(self):
    return self._clock_us


class AioReceiver(base_receiver.DataThread):
  """Class to receive and manage AIO messages."""

  def __init__(self, minimum_stale_timeout_seconds, stale_timeout_in_periods,
               receiver_idle_timeout_seconds, message_types, timeout,
               aio_loop_duration, process_duration, receiver_aio_node,
               network_yaml_file, aio_message_sequence_bits,
               populate_from_sim):
    """Initializes the AIO receiver.

    Args:
      minimum_stale_timeout_seconds: Default time period in which the buffered
          message remains live. [s]
      stale_timeout_in_periods: Time period in which the buffered message
          remains live, in number of message transfering periods.
      receiver_idle_timeout_seconds: Duration to stay live since the last
          request. [s]
          Once due, the thread exits the main loop and stops receiving messages.
      message_types: The list of message types to receive.
      timeout: The timeout when receiving messages. [s]
      aio_loop_duration: Time to continuously receive AIO messages. [s]
      process_duration: Time to process other requests. [s]
      receiver_aio_node: Name of the AIO node used to receive messages.
      network_yaml_file: The YAML file that defines nodes and messages on the
          AIO network.
      aio_message_sequence_bits: The number of bits in the AIO message sequence.
      populate_from_sim: Whether or not the receiver should populate telemtry
          from the simulator.
    """
    super(AioReceiver, self).__init__(
        minimum_stale_timeout_seconds, stale_timeout_in_periods,
        receiver_idle_timeout_seconds, message_types,
        network_yaml_file, aio_message_sequence_bits)
    self._timeout_us = long(timeout * 1e6)
    self._aio_loop_us = long(aio_loop_duration * 1e6)
    assert self._timeout_us < self._aio_loop_us
    self._process_duration = process_duration
    self._receiver_aio_node = receiver_aio_node
    self._message_types = message_types

    # _message_cache is accessed only by one thread, the AioReceiver, so
    # there is no need to guard it with locks.
    self._message_cache = collections.defaultdict(dict)

    self._populate_from_sim = populate_from_sim
    config = network_config.NetworkConfig(network_yaml_file)
    self._sources_by_message = {}
    for m in config.all_messages:
      if m.name in _MESSAGE_TYPE_HELPER:
        msg_enum = _MESSAGE_TYPE_HELPER.Value(m.name)
        self._sources_by_message[msg_enum] = [
            _AIO_NODE_HELPER.Value(sender.camel_name)
            for sender in m.all_senders]

    # The lock ensures that the AIO snapshot is produced right after
    # the AIO loop phase, without interruption from other threads.
    self._snapshot_lock = threading.Lock()
    self._snapshot = None

  def _CurrentTimeSec(self):
    """Get currrent time in seconds in the same frame as message.Timestamp()."""
    return aio_util.ClockGetUs() * 1.0e-6

  def _OnStart(self):
    if not aio_helper.Setup(self._message_types, self._receiver_aio_node):
      logging.error(
          'Unable to add multicast group.  Linux is limited to 20 multicast '
          'groups by default.  You may need to increase your limit:\n'
          '  echo <new_limit> > /proc/sys/net/ipv4/igmp_max_memberships\n')

  def _OnStop(self):
    aio_helper.TearDown()

  def GetReceivedMessageTypes(self):
    """Return the message types received so far.

    Overrides DataThread.GetReceivedMessageTypes since AioReceiver is using
    a different structure to keep track of messages.

    Returns:
      A dictionary of message type names indexed by message enum.
    """
    return {int(message_enum): _MESSAGE_TYPE_HELPER.ShortName(int(message_enum))
            for message_enum in aio_helper.ReceivedMessageCatalog()}

  def _RunOnce(self):
    if not self._should_exit:
      aio_util.AioLoop(self._timeout_us, self._aio_loop_us,
                       self._populate_from_sim)
      snapshot = super(AioReceiver, self).GetLatest(update_access_time=False)
      with self._snapshot_lock:
        self._snapshot = snapshot
    self._ExitIfAbandoned()
    # Yield to other threads.
    time.sleep(self._process_duration)

  def GetLatest(self, specified_message_enum=None):
    """Use the latest snapshot if available."""
    self._UpdateLastAccessTime()
    # Guard self._snapshot as a good lock programming practice.
    with self._snapshot_lock:
      if specified_message_enum is None:
        return self._snapshot if self._snapshot else {}
      else:
        message_type = _MESSAGE_TYPE_HELPER.ShortName(specified_message_enum)
        if self._snapshot and message_type in self._snapshot:
          return {message_type: self._snapshot[message_type]}
        else:
          return {}

  def _SourcesByMessageType(self, message_enum):
    """Iterate through all sources for a given message type.

    Yields the sources for received messages of a particular type.
    This implements DataThread._SourcesByMessageType.

    Args:
      message_enum: The message type to inspect.

    Returns:
      A generator about the sources from where the message is sent.
    """
    if self._populate_from_sim or aio_util.IsMessageTypeUpdated(message_enum):
      return self._sources_by_message[message_enum]
    return []

  def _GetMessageByTypeAndSource(self, message_enum, source):
    """Get the most recent message given type enum and source."""
    if (message_enum in self._message_cache and
        source in self._message_cache[message_enum]):
      old_message = self._message_cache[message_enum][source]
      old_clock_us = old_message.ClockUs()
    else:
      old_clock_us = None
      old_message = None

    # Attempt to get the new message. If there is one, CvtGet returns
    # (buf, sequence, and clock_us). Otherwise, it returns (sequence, clock_us).
    result = aio_util.CvtGet(source, message_enum)
    if len(result) != 3:
      # Reuse the old message if one exists.
      return old_message

    buf, sequence, clock_us = result

    if (not old_message) or old_clock_us != clock_us:
      # Regenerate the AioMessage.
      message = AioMessage(buf, message_enum, source, sequence, clock_us)
      self._message_cache[message_enum][source] = message
    else:
      # Reuse the AioMessage.
      message = old_message

    return message

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

"""Receives and organizes messages."""

import copy
import datetime
import logging
import threading

from makani.avionics.linux.swig import aio_helper
from makani.avionics.network import aio_node as aio_node_names
from makani.avionics.network import message_type as aio_message_type
from makani.avionics.network import network_config
from makani.lib.python import c_helpers
from makani.lib.python import struct_tree
import pytz

_MESSAGE_TYPE_HELPER = c_helpers.EnumHelper('MessageType', aio_message_type)
_AIO_NODE_HELPER = c_helpers.EnumHelper('AioNode', aio_node_names)


class BaseThread(threading.Thread):
  """A template to run thread with repeated tasks."""

  def __init__(self):
    super(BaseThread, self).__init__()
    self._should_exit = False

  def run(self):
    self._OnStart()
    while not self._should_exit:
      self._RunOnce()
    self._OnStop()

  def _OnStart(self):
    """Placeholder function to be called when the thread begins."""
    pass

  def _OnStop(self):
    """Placeholder function to be called when the thread ends."""
    pass

  def Exit(self):
    self._should_exit = True

  def IsDeactivated(self):
    return self._should_exit

  def TryStop(self):
    if self.is_alive():
      self.Exit()
      self.join(1)
      if self.is_alive():
        raise ValueError('Could not terminate thread.')


class DataThread(BaseThread):
  """A thread that receives data."""

  def __init__(self, minimum_stale_timeout_seconds, stale_timeout_in_periods,
               receiver_idle_timeout_seconds, message_types,
               network_yaml_file, aio_message_sequence_bits):
    """A thread that continuously receive messages.

    Args:
      minimum_stale_timeout_seconds: Default time period in which the buffered
          message remains live. [s]
      stale_timeout_in_periods: Time period in which the buffered message
          remains live, in number of message transfering periods.
      receiver_idle_timeout_seconds: Duration to stay live since the last
          request. [s]
          Once due, the thread exits the main loop and stops receiving messages.
      message_types: The list of message types to receive.
      network_yaml_file: The YAML file that defines nodes and messages on the
          AIO network.
      aio_message_sequence_bits: The number of bits in the AIO message sequence.
    """

    super(DataThread, self).__init__()
    # Data specific attributes.
    self._message_types = copy.copy(message_types)
    # The buffer time by message type name.
    self._buffer_time = {}
    config = network_config.NetworkConfig(network_yaml_file)
    message_frequencies = {
        m.name: m.frequency_hz for m in config.all_messages}

    # Determine the buffer time for each message type according to network
    # configuration. If not found, use minimum_stale_timeout_seconds.
    for message_type in _MESSAGE_TYPE_HELPER.Names():
      assert message_type in _MESSAGE_TYPE_HELPER
      short_name = _MESSAGE_TYPE_HELPER.ShortName(message_type)
      if short_name not in message_frequencies:
        self._buffer_time[message_type] = minimum_stale_timeout_seconds
      else:
        frequency = message_frequencies[short_name]
        if frequency == 0:
          self._buffer_time[message_type] = minimum_stale_timeout_seconds
        else:
          self._buffer_time[message_type] = max(
              1.0 / frequency * stale_timeout_in_periods,
              minimum_stale_timeout_seconds)
    self._idle_timeout = receiver_idle_timeout_seconds

    self._latest_access_time = self._CurrentTimeSec()

    # The overflow value of message sequence numbers.
    self._sequence_period = 2**aio_message_sequence_bits

  def _ExitIfAbandoned(self):
    """If nobody has asked for data for a while, exit the main loop."""
    current_time = self._CurrentTimeSec()
    if current_time - self._latest_access_time < self._idle_timeout:
      return False
    else:
      logging.info('Receiver is abandoned due to inactivity.')
      self.Exit()
      return True

  def GetReceivedMessageTypes(self):
    """Return the message types received so far.

    Returns:
      A dictionary of message type names indexed by message enum.
    """
    raise NotImplementedError

  def _SourcesByMessageType(self, message_enum):
    """Return the sources from which messages of a given type are ever sent."""
    raise NotImplementedError

  def _GetTimelyMessages(self, message_enum, timestamp, max_buffer_time):
    """Get the latest message by MessageTypeEnum."""
    result = {}
    for source in self._SourcesByMessageType(message_enum):
      message = self._GetMessageByTypeAndSource(message_enum, source)
      if message:
        if timestamp - message.Timestamp() > max_buffer_time:
          # Skip if the message is too old.
          continue
        try:
          aio_node = _AIO_NODE_HELPER.ShortName(source)
        except c_helpers.EnumError:
          aio_node = 'unknown'
          logging.error('Unknown AIO Node %s', source)
        result[aio_node] = message.Get(readonly=True)
    return result

  def _CurrentTimeSec(self,
                      epoch=pytz.utc.localize(datetime.datetime(1970, 1, 1))):
    """Get currrent time in seconds in the same frame as message.Timestamp()."""
    utc_datetime = datetime.datetime.now(tz=pytz.utc)
    td = utc_datetime - epoch
    return float(td.microseconds) / 1e6 + td.seconds + td.days * 24 * 3600

  def _UpdateLastAccessTime(self):
    """Refresh the usage data to keep the receiver alive."""
    self._latest_access_time = self._CurrentTimeSec()

  def GetLatest(self, specified_message_enum=None, update_access_time=True):
    """Get the latest messages from each source with the given message type."""

    current_time = self._CurrentTimeSec()
    resp = {}

    if specified_message_enum is None:
      message_type_generator = _MESSAGE_TYPE_HELPER.Names()
    else:
      message_type_generator = [
          _MESSAGE_TYPE_HELPER.Name(specified_message_enum)]
    # Get latest messages for all message types.
    for message_type in message_type_generator:
      short_message_type = _MESSAGE_TYPE_HELPER.ShortName(message_type)
      message_enum = _MESSAGE_TYPE_HELPER.Value(message_type)
      max_buffer_time = self._buffer_time[message_type]
      message_info = self._GetTimelyMessages(
          message_enum, current_time, max_buffer_time)
      if message_info:
        resp[short_message_type] = message_info

    # Tack on the filered data as a top-level key in the message snapshot.
    resp['filtered'] = aio_helper.GetFilteredData()

    dict_obj = struct_tree.StructTree(resp, fail_silently=True, readonly=True)
    if update_access_time:
      self._UpdateLastAccessTime()
    return dict_obj

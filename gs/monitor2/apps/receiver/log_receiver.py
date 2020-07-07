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

"""Retrieve messages from a log file."""

import collections
import copy
import datetime
import logging
import threading

from makani.avionics.network import message_type as aio_message_type
from makani.gs.monitor2.apps.receiver import base_receiver
from makani.lib.python import c_helpers
from makani.lib.python import struct_tree
from makani.lib.python.trackers import json_obj
import numpy
import pytz

_MESSAGE_TYPE_HELPER = c_helpers.EnumHelper('MessageType', aio_message_type)


class LogMessage(json_obj.JsonObj):
  """A message reconstructed from the log."""

  def __init__(self, payload, timestamp, seq_num, version, msg_enum, source):
    self._payload = payload
    self._timestamp = timestamp
    self._sequence = seq_num
    self._version = version
    self._msg_enum = msg_enum
    self._source = source
    self._message = None

  def Get(self, readonly):
    """Obtain a JSON representation of the message.

    Args:
      readonly: True if the returned value is read-only.

    Returns:
      A JSON object which should be readonly.
    """
    if not self._message:
      self._message = copy.copy(self._payload)
      header = {
          'timestamp': self._timestamp,
          'sequence': self._sequence,
          'aio_version': self._version,
      }
      self._message['capture_info'] = header
    if readonly:
      return self._message
    else:
      return copy.deepcopy(self._message)

  def MessageEnum(self):
    return int(self._msg_enum)

  def Source(self):
    return int(self._source)

  def SequenceNum(self):
    return self._sequence

  def Timestamp(self):
    return self._timestamp


class LogReceiver(base_receiver.DataThread):
  """Class to replay and manage recorded messages from Logs."""

  def __init__(self, minimum_stale_timeout_seconds, stale_timeout_in_periods,
               receiver_idle_timeout_seconds, log_path, message_type, source,
               network_yaml_file, aio_message_sequence_bits):
    super(LogReceiver, self).__init__(
        minimum_stale_timeout_seconds, stale_timeout_in_periods,
        receiver_idle_timeout_seconds, [message_type],
        network_yaml_file, aio_message_sequence_bits)
    log_data = struct_tree.StructTree(
        log_path, fail_silently=True, readonly=True)
    self._messages = log_data.Index(('messages', source, message_type))
    self._index = 0
    self._bound = self._messages['aio_header']['source'].size
    self._received_message_enums = set()
    # A dictionary of recently received messages.
    # {<message_type>: {<aio_node>: message}}
    self._message_snapshot = collections.defaultdict(
        lambda: collections.defaultdict(lambda: None))
    # The lock guards all read/write accesses to _message_snapshot.
    self._value_lock = threading.Lock()

  def GetReceivedMessageTypes(self):
    """Return the message types received so far.

    Returns:
      A dictionary of message type names indexed by message enum.
    """
    return {message_enum: _MESSAGE_TYPE_HELPER.ShortName(message_enum) for
            message_enum in self._received_message_enums}

  def _SourcesByMessageType(self, message_enum):
    with self._value_lock:
      return self._message_snapshot[message_enum].keys()

  def _GetMessageByTypeAndSource(self, message_enum, source):
    with self._value_lock:
      return self._message_snapshot[message_enum][source]

  def _ReceiveMessage(self, message):
    """Add a message to the message window."""
    message_enum = message.MessageEnum()
    source = message.Source()
    self._received_message_enums.add(message_enum)

    with self._value_lock:
      old_message = self._message_snapshot[message_enum][source]
      self._message_snapshot[message_enum][source] = message

    if old_message:
      self._ReportMissingMessages(message, old_message)

  def _ReportMissingMessages(self, message, old_message):
    """Detect and report if any message is missing."""
    new_sequence = message.SequenceNum()
    old_sequence = old_message.SequenceNum()
    if not self._IsNextMessageInSequence(old_sequence, new_sequence):
      message_enum = message.MessageEnum()
      source = message.Source()
      logging.warning('Unexpected message sequence (ENUM %s) from %s '
                      '(sequence gap between %s and %s). This can be caused '
                      'by missing, duplicated, or out-of-order messages.',
                      message_enum, source, old_sequence, new_sequence)

  def _IsNextMessageInSequence(self, old_sequence, new_sequence):
    return new_sequence == (old_sequence + 1 % self._sequence_period)

  def _RunOnce(self):
    timestamp = base_receiver.Timestamp(datetime.datetime.now(tz=pytz.utc))
    source = self._messages['aio_header']['source'][self._index]
    version = self._messages['aio_header']['version'][self._index]
    msg_enum = self._messages['aio_header']['type'][self._index]
    seq_num = self._messages['aio_header']['sequence'][self._index]
    payload = FrameOfDictionary(self._messages['message'], self._index)
    msg = LogMessage(payload, timestamp, seq_num, version, msg_enum, source)
    self._ReceiveMessage(msg)
    self._index += 1
    if self._index >= self._bound:
      self.Exit()
    else:
      self._ExitIfAbandoned()


def FrameOfDictionary(dict_data, index):
  """Index a (possibly nested) dictionary of lists.

  Args:
    dict_data: A dictionary of lists.
    index: The position to extract for each list.

  Returns:
    A dictionary with the indexed values.
  """
  resp = {}
  for key, value in dict_data.iteritems():
    if isinstance(value, numpy.ndarray):
      resp[key] = value[index]
    elif isinstance(value, dict):
      resp[key] = FrameOfDictionary(value, index)
    elif isinstance(value, list):
      resp[key] = [FrameOfDictionary(item, index) for item in value]
    else:
      resp[key] = value
  return resp

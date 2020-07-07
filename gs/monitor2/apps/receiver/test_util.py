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

"""Test utilities related to the receivers."""

import ctypes

from makani.avionics.common import aio
from makani.avionics.linux.swig import aio_helper
from makani.avionics.network import aio_node as aio_node_type
from makani.avionics.network import message_type as aio_message_type
from makani.avionics.network import network_config
from makani.gs.monitor2.apps.receiver import aio_receiver
from makani.gs.monitor2.project import settings

from makani.lib.python import c_helpers
from makani.lib.python import struct_tree

_MESSAGE_TYPE_HELPER = c_helpers.EnumHelper('MessageType', aio_message_type)
_MESSAGE_STRUCTS, _MCAST_GROUPS = aio.GetMessageStructsMulticastGroups(
    _MESSAGE_TYPE_HELPER.Names())


class FakeAioMessage(aio_receiver.AioMessage):
  """A class for fake AIO message."""

  def __init__(self, msg_enum, source, sequence):
    super(FakeAioMessage, self).__init__(None, msg_enum, source, sequence, 0)

  def _Unpack(self, buf, message_name):
    msg_enum = _MESSAGE_TYPE_HELPER.Value(message_name)
    message_struct = _MESSAGE_STRUCTS[msg_enum]
    if message_struct:
      buf = ctypes.create_string_buffer(c_helpers.PackSize(message_struct)).raw
      return c_helpers.Unpack(buf, message_struct)
    else:
      raise KeyError('Message type "%s" cannot be recognized.' % message_name)

  def IsValid(self):
    return self._message is not None


def SynthesizeFilteredData():
  filtered_data = aio_helper.GetFilteredData()
  filtered_data.merge_tether_down.valid = True
  for n in range(aio_node_type.kNumAioNodes):
    filtered_data.merge_tether_down.node_status_valid[n] = True
  return filtered_data


def SynthesizeMessages(message_names=None, sequence=0, fail_silently=False):
  """Synthesize fake AIO messages.

  Args:
    message_names: The set of message type names.
        If None, all messages are synthesized.
    sequence: The sequence number of all the messages.
    fail_silently: If True, operations over the returned StructTree return None
        rather than raising exceptions.

  Returns:
    A StructTree object as a nested dict of messages, indexed first by
        message types and then AIO nodes.
  """

  # TODO: Clean-up nonstandard messages.
  excluded = ['Stdio', 'ControlDebug']
  messages = {}

  message_types = network_config.NetworkConfig(
      settings.NETWORK_YAML).all_messages

  for m in message_types:
    if m.name not in _MESSAGE_TYPE_HELPER or m.name in excluded:
      continue
    if message_names and m.name not in message_names:
      continue
    msg_enum = _MESSAGE_TYPE_HELPER.Value(m.name)
    messages[m.name] = {}
    for sender in m.all_senders:
      aio_node = sender.camel_name
      message = FakeAioMessage(msg_enum, aio_node, sequence)
      if message.IsValid():
        messages[m.name][aio_node] = message.Get(readonly=True)

  messages['filtered'] = SynthesizeFilteredData()
  return struct_tree.StructTree(messages, fail_silently=fail_silently,
                                readonly=False)

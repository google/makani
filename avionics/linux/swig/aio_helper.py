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

"""High level utilities to access CVT via SWIG."""

import ctypes
from makani.avionics.common import aio
from makani.avionics.common import network_config
from makani.avionics.linux.swig import aio_util
from makani.avionics.network import aio_node as aio_node_names
from makani.avionics.network import message_type as aio_message_type
from makani.gs.monitor2.high_frequency_filters import filter_types
from makani.lib.python import c_helpers
import numpy

MESSAGE_TYPE_HELPER = c_helpers.EnumHelper('MessageType', aio_message_type)
AIO_NODE_HELPER = c_helpers.EnumHelper('AioNode', aio_node_names)

MESSAGE_STRUCTS, MCAST_GROUPS = aio.GetMessageStructsMulticastGroups(
    MESSAGE_TYPE_HELPER.Names())


def Setup(message_types, receiver_aio_node):
  """Set up the AIO network and subscribe to the given list of message types.

  Args:
    message_types: A list of message type names or short names.
    receiver_aio_node: Name of the AIO node to receive messages.

  Returns:
    True if successful.
  """

  aio_node = AIO_NODE_HELPER.Value(receiver_aio_node)
  aio_port = network_config.UDP_PORT_AIO
  result = aio_util.InitAioLoop(
      aio_node, aio_port,
      [MESSAGE_TYPE_HELPER.Value(m) for m in message_types])
  return result != -1


def TearDown():
  aio_util.AioClose()
  aio_util.TearDownAioLoop()


def ClockUsToTimestamp(clock_us, reference_clock_us, reference_timestamp):
  """Converts a reported clock measurement (in us) to a timestamp.

  Args:
    clock_us: Measured clock [us].
    reference_clock_us: Measured clock at a reference moment [us].
    reference_timestamp: Seconds after 1/1/1970 at the reference moment.

  Returns:
    Timestamp corresponding to clock_us in seconds after 1/1/1970.
  """

  return reference_timestamp + (clock_us - reference_clock_us) / 1.0e6


def UnpackMessage(swig_obj_pointer, msg_name):
  """Unpack a SWIG-wrapped memory object into an AIO message.

  Args:
    swig_obj_pointer: A SWIG-wrapped memory object pointing to the raw AIO
        message payload.
    msg_name: Name or short name of the message type.

  Returns:
    An AIO message struct.
  """

  ptr = int(swig_obj_pointer)
  c_array = ctypes.c_char * aio.GetPackMessageSize(msg_name)
  received = c_array.from_address(ptr)

  msg_type = MESSAGE_TYPE_HELPER.Value(msg_name)
  return c_helpers.Unpack(received[:], MESSAGE_STRUCTS[msg_type])


def ReceivedMessageCatalog():
  """Returns a dict of AioNode list keyed by message types received."""
  aio_updates = numpy.zeros((len(AIO_NODE_HELPER), len(MESSAGE_TYPE_HELPER)),
                            dtype=numpy.ulonglong)
  # aio_updates[source][message_type] > 0 if any message is received.
  aio_util.GetAioUpdates(aio_updates)
  received_messages = {}
  for message_type in range(len(MESSAGE_TYPE_HELPER)):
    aio_nodes = numpy.nonzero(aio_updates[:, message_type])[0]
    if aio_nodes.size:
      received_messages[message_type] = aio_nodes
  return received_messages


def GetFilteredData():
  """Returns a SWIG-wrapped pointer to the FilteredData object."""
  ptr, length = aio_util.GetFilteredData()
  c_array = ctypes.c_char * length
  buf = c_array.from_address(int(ptr))
  data = ctypes.POINTER(filter_types.FilteredData)(buf).contents
  snapshot = filter_types.FilteredData()
  ctypes.memmove(ctypes.addressof(snapshot), ctypes.addressof(data),
                 ctypes.sizeof(data))
  return snapshot

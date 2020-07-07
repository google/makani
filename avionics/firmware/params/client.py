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

"""Network client for reading (and eventually writing) parameters."""

from makani.avionics.common import aio
from makani.avionics.common import pack_avionics_messages
from makani.avionics.firmware.params import codec

SECTION_CONFIG = pack_avionics_messages.kParamSectionConfig
SECTION_CALIB = pack_avionics_messages.kParamSectionCalib
SECTION_SERIAL = pack_avionics_messages.kParamSectionSerial
SECTION_CARRIER_SERIAL = pack_avionics_messages.kParamSectionCarrierSerial


class Client(object):
  """Network client for reading (and eventually writing) parameters."""

  def __init__(self, timeout=None):
    self.aio_client = aio.AioClient(['kMessageTypeParamRequest',
                                     'kMessageTypeParamResponse'],
                                    timeout=timeout)

  def _SendBlockRequest(self, node_id, section, offset):
    """Fill out and send a ParamRequestMessage."""
    request = pack_avionics_messages.ParamRequestMessage()
    request.node_id = node_id
    request.section = section
    request.offset = offset
    self.aio_client.Send(request, 'kMessageTypeParamRequest',
                         'kAioNodeOperator')

  def _GetBlock(self, node_id, section, data, offset):
    """Query a node for a block of parameters from the specified section."""
    self._SendBlockRequest(node_id, section, offset)
    while True:
      (_, _, msg) = self.aio_client.Recv()
      if isinstance(msg, pack_avionics_messages.ParamResponseMessage):
        break
    # TODO: Verify section, offset, and length.
    if msg.length > 0:
      data[offset:offset + msg.length] = msg.data[0:msg.length]
    return msg.length

  def GetSection(self, node_id, section):
    """Obtain parameters from the specified section in the node node_id.

    Args:
      node_id: AIO node number integer.
      section: Parameter section identifier, e.g. SECTION_CALIB.

    Returns:
      A parameter object for the particular node.

    Raises:
      socket.timeout if a timeout was specified in the constructor and the
      timeout was exceeded while querying parameters.
    """
    offset = 0
    data = bytearray(64 * 1024)  # TODO:  Define a max param size.
    while offset < len(data):
      length = self._GetBlock(node_id, section, data, offset)
      offset += length
      if length < 1024:
        break
    return codec.DecodeBin(data)

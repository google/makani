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

"""Receives torque data from futek and forwards to AIO network."""

import logging
import socket
import struct

from makani.avionics.common import aio
from makani.avionics.common import pack_avionics_messages

# Multicast group and port for receiving data from torque cell.
_MCAST_GROUP = '239.0.6.222'
_MCAST_PORT = 2222


def OpenTorqueCellSocket():
  # Create client to listen to futek.
  futek_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  futek_client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  futek_client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
  futek_client.bind(('', _MCAST_PORT))
  mreq = struct.pack('4sl', socket.inet_pton(socket.AF_INET, _MCAST_GROUP),
                     socket.INADDR_ANY)
  futek_client.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
  return futek_client


def RecvTorqueSendAio(futek_client, data_struct, torque_cell_aio_client):
  """Receive data from futek, send out as AIO packet."""
  # Receive torque, angle, omega reading from futek.
  futek_message = futek_client.recv(4096)
  if len(futek_message) != data_struct.size:
    logging.warning('Expected torque message of length %d, received length %d',
                    data_struct.size, len(futek_message))
    return
  (torque, angle, omega) = data_struct.unpack(futek_message)

  # Package data into AIO packet.
  aio_torque_msg = pack_avionics_messages.TorqueCellMessage()
  aio_torque_msg.torque = torque   # N-m.
  aio_torque_msg.angle = angle     # radians.
  aio_torque_msg.omega = omega     # rad/s.

  # Send out AIO packet.
  torque_cell_aio_client.Send(aio_torque_msg, 'kMessageTypeTorqueCell',
                              'kAioNodeTorqueCell')


def Main():
  # Join multicast group to receive UDP packets from torque cell.
  futek_client = OpenTorqueCellSocket()

  # Struct format for unpacking received data.
  data_struct = struct.Struct('f f f')

  # AIO Client to send out AIO packets of torque data.
  torque_cell_aio_client = aio.AioClient(['kMessageTypeTorqueCell'])

  while True:
    RecvTorqueSendAio(futek_client, data_struct, torque_cell_aio_client)


if __name__ == '__main__':
  Main()

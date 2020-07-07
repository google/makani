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


"""Ping and measure latency to an AIO node."""

import socket
import time

from makani.avionics.common import aio
from makani.avionics.common import pack_avionics_messages
from makani.avionics.network import message_type
from makani.lib.python import c_helpers

message_type_helper = c_helpers.EnumHelper('MessageType', message_type)


def _GetUSec():
  return int(time.time() * 1000000 % (2 ** 31))


def _SubUSec(a, b):
  if a > b:
    return a - b
  else:
    return a + 2 ** 31 - b


def main():
  aio_client = aio.AioClient(['kMessageTypeLatencyProbe',
                              'kMessageTypeLatencyResponse'],
                             timeout=1)

  while True:
    probe_msg = pack_avionics_messages.LatencyProbeMessage()
    probe_msg.timestamp = _GetUSec()
    aio_client.Send(probe_msg, 'kMessageTypeLatencyProbe', 'kAioNodeOperator')

    while True:
      try:
        (addr, header, msg) = aio_client.Recv()
        now = _GetUSec()
        mtype = message_type_helper.ShortName(header.type)
        if mtype == 'LatencyResponse':
          print '%s: %dus' % (addr, _SubUSec(now, msg.timestamp))
      except socket.timeout:
        break

if __name__ == '__main__':
  main()

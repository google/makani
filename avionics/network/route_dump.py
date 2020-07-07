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

"""Dump routes from a TMS570 AIO node."""

import socket
import sys
import textwrap

from makani.avionics.common import aio
from makani.avionics.common import pack_avionics_messages
from makani.avionics.network import aio_node
from makani.avionics.network import message_type
from makani.lib.python import c_helpers

aio_node_helper = c_helpers.EnumHelper('AioNode', aio_node)
message_type_helper = c_helpers.EnumHelper('MessageType', message_type)


def _MacToString(mac):
  return '%02X:%02X:%02X:%02X:%02X:%02X' % (
      mac.a, mac.b, mac.c, mac.d, mac.e, mac.f)


def _FormatResponse(source, msg):
  source_name = aio_node_helper.ShortName(source)
  mac = _MacToString(msg.entry.ethernet_address)
  mcast = (msg.entry.ethernet_address.a & 1) == 1
  mcast_str = 'Multicast' if mcast else 'Unicast'
  port_str = ('mask 0x%02X' if mcast else 'port %d') % msg.entry.port_map
  print ('%s: %s %s on VLAN %d to %s'
         ' (valid=%d age=%d static=%d arl_con=%d priority=%d)'
         % (source_name, mcast_str, mac, msg.entry.vlan_id, port_str,
            msg.entry.valid, msg.entry.age, msg.entry.static_entry,
            msg.entry.arl_con, msg.entry.priority))


def main():
  if len(sys.argv) != 2:
    print textwrap.dedent("""
        Inspect the switch routing table of an AIO node.  Currently only access
        switches are supported.

        Usage: route_dump <node_name>"""[1:])
    sys.exit(-1)

  aio_client = aio.AioClient(['kMessageTypeDumpRoutesRequest',
                              'kMessageTypeDumpRoutesResponse'],
                             timeout=1)

  request_msg = pack_avionics_messages.DumpRoutesRequestMessage()
  request_msg.target = aio_node_helper.Value(sys.argv[1])
  aio_client.Send(request_msg, 'kMessageTypeDumpRoutesRequest',
                  'kAioNodeOperator')

  responses = []
  while True:
    try:
      (_, header, msg) = aio_client.Recv()
      mtype = message_type_helper.ShortName(header.type)
      if mtype == 'DumpRoutesResponse' and header.source == request_msg.target:
        responses.append(msg)
    except socket.timeout:
      break

  def _SortKey(msg):
    return '%3X%s'%(msg.entry.vlan_id, _MacToString(msg.entry.ethernet_address))
  for msg in sorted(responses, key=_SortKey):
    _FormatResponse(request_msg.target, msg)


if __name__ == '__main__':
  main()

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


"""Printf console for AIO nodes.

Displays printf messages from AIO nodes delivered through the
kMessageTypeStdio AIO message.  De-duplicates messages in an
intelligent way that allows for AIO node resets.
"""

import collections
import sys

from makani.avionics.common import aio
from makani.avionics.network import aio_node
from makani.avionics.network import message_type
from makani.lib.python import c_helpers

aio_node_helper = c_helpers.EnumHelper('AioNode', aio_node)
message_type_helper = c_helpers.EnumHelper('MessageType', message_type)


class AioPrintClient(aio.AioClient):

  def __init__(self, *args, **kwargs):
    super(AioPrintClient, self).__init__(*args, **kwargs)
    self._last_strings = collections.defaultdict(lambda: '')

  def IsDuplicate(self, header, payload_string, cur_time):
    """Determines if AIO message is a duplicate.

    Intelligently decides if packet is a duplicate based on sequence number,
    if the payload (string) is the same, and finally if it was received within
    a window from the suspected duplicate.

    Args:
      header: A AioHeader of received AIO packet.
      payload_string: The of received AIO packet.
      cur_time: Time of received AIO packet.

    Returns:
      A boolean specifying whether the received packet is a duplicate.
    """
    msg_key = (header.source, header.type)
    if (super(AioPrintClient, self).IsDuplicate(header, payload_string,
                                                cur_time)
        and self._last_strings[msg_key] == payload_string):
      return True
    self._last_strings[msg_key] = payload_string
    return False


def _Stdio(source_addr, string):
  octet = source_addr.split('.')[3]
  if not string.endswith('\n'): string += '\n'
  sys.stdout.write('{:s}: {:s}'.format(octet, string))
  sys.stdout.flush()


def _IsSerialParamsValid(params):
  return any(b != 0 for b in bytearray(params))


def _PrintSerialParams(source_addr, serial):
  _Stdio(source_addr, '  Hardware: %s, Rev: %ld'
         % (serial.part_name, serial.hardware_revision))
  _Stdio(source_addr, '  UPN: %s, Serial: %s'
         % (serial.part_number, serial.serial_number))


def _PrintStartupMessage(source_addr, header, message):
  """Print a startup message from a *SlowStatus or SelfTest message."""
  if message_type_helper.ShortName(header.type) == 'BootloaderSlowStatus':
    if message.bootloader_segment:
      _Stdio(source_addr, 'Bootloader')
    else:
      _Stdio(source_addr, 'Bootloader Application')
  else:
    _Stdio(source_addr,
           'AIO node: %s' % aio_node_helper.ShortName(header.source))
  _Stdio(source_addr, 'AIO version: 0x%04X' % header.version)
  _Stdio(source_addr, 'IP: %s' % source_addr)
  _Stdio(source_addr, 'Built on %s at %s' %
         (message.build_info.date, message.build_info.time))

  if _IsSerialParamsValid(message.serial_params):
    _Stdio(source_addr, 'Main Board:')
    _PrintSerialParams(source_addr, message.serial_params)
  else:
    _Stdio(source_addr, 'Unknown or invalid serialization parameters!\n')

  if (hasattr(message, 'carrier_serial_params')
      and _IsSerialParamsValid(message.carrier_serial_params)):
    _Stdio(source_addr, 'Carrier:')
    _PrintSerialParams(source_addr, message.carrier_serial_params)

  if message_type_helper.ShortName(header.type) == 'SelfTest':
    _Stdio(source_addr, message.text)


_last_sequence_nums = {}


def _HelloWing(source_addr, header, message):
  """Determine if a node recently started up."""
  key = (source_addr, header.source)
  # Test if the sequence number is less than 10 (started within the last 10
  # seconds), that the sequence number is lower than the last one we received
  # (or we hadn't received one yet), and that the sequence number was not about
  # to wrap around (near 65535).  This means that any nodes started within 10
  # seconds before starting console will report HelloWing immediately.
  if header.sequence < 10 and (
      key not in _last_sequence_nums or
      (_last_sequence_nums[key] > header.sequence and
       _last_sequence_nums[key] < (2**16 - 10))):
    _PrintStartupMessage(source_addr, header, message)
  _last_sequence_nums[key] = header.sequence


def main():
  aio_client = AioPrintClient(['kMessageTypeStdio', 'kMessageTypeSlowStatus',
                               'kMessageTypeCoreSwitchSlowStatus',
                               'kMessageTypeBootloaderSlowStatus',
                               'kMessageTypeSelfTest'])
  try:
    while True:
      (source_addr, header, message) = aio_client.Recv()
      mtype = message_type_helper.ShortName(header.type)
      if mtype == 'Stdio':
        _Stdio(source_addr, message)
      elif mtype == 'SelfTest':
        _PrintStartupMessage(source_addr, header, message)
      elif mtype == 'SlowStatus' or mtype == 'CoreSwitchSlowStatus':
        _HelloWing(source_addr, header, message)
  except KeyboardInterrupt:
    aio_client.Close()
    sys.exit()

if __name__ == '__main__':
  main()

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

"""Tests for the AIO Python interface."""

import socket
import unittest

from makani.avionics.common import aio
from makani.avionics.common import aio_version
from makani.avionics.common import network_config
from makani.avionics.common import pack_aio_header as aio_header
from makani.avionics.common import pack_avionics_messages


def SetLastSeqTime(aio_client, source, message_type, last_seq, last_time):
  """This function enables IsDuplicate() unit tests."""
  aio_client._recv_seq_nums[(source, message_type)] = last_seq
  aio_client._recv_times[(source, message_type)] = last_time


class AioTest(unittest.TestCase):

  def testStdio(self):
    source = aio.AioClient(['kMessageTypeMotorStatus',
                            'kMessageTypeStdio'])
    sink = aio.AioClient(['kMessageTypeStdio'], timeout=0.01)

    source.Send('test', 'kMessageTypeStdio', 'kAioNodeMotorSbo')
    (_, header, string) = sink.Recv()

    self.assertEqual(header.source, network_config.kAioNodeMotorSbo)
    self.assertEqual(header.type, network_config.kMessageTypeStdio)
    self.assertEqual(string, 'test')
    source.Close()
    sink.Close()

  # TODO: Test a broader set of message types.
  def testMotorStatusMessage(self):
    source = aio.AioClient(['kMessageTypeMotorStatus',
                            'kMessageTypeStdio'])
    sink = aio.AioClient(['kMessageTypeMotorStatus'], timeout=0.01)

    before = pack_avionics_messages.MotorStatusMessage()
    before.id = 3.14
    before.omega = 5.16

    source.Send(before, 'kMessageTypeMotorStatus', 'kAioNodeMotorSbo')
    (_, header, after) = sink.Recv()

    self.assertEqual(header.source, network_config.kAioNodeMotorSbo)
    self.assertEqual(header.type, network_config.kMessageTypeMotorStatus)
    self.assertEqual(before.id, after.id)
    self.assertEqual(before.omega, after.omega)
    source.Close()
    sink.Close()

  def testSourceFiltering(self):
    source = aio.AioClient(['kMessageTypeMotorStatus',
                            'kMessageTypeStdio'])
    sink = aio.AioClient(['kMessageTypeStdio'],
                         allowed_sources=['kAioNodeMotorSti'],
                         timeout=0.01)

    source.Send('test', 'kMessageTypeStdio', 'kAioNodeMotorSbo')
    with self.assertRaises(socket.timeout):
      _ = sink.Recv()

    source.Close()
    sink.Close()

  # TODO: Test IsDuplicate() with sending interface.
  def testIsDuplicate(self):
    sink = aio.AioClient(['kMessageTypeMotorStatus'], timeout=0.01)
    header = aio_header.AioHeader()
    header.version = aio_version.AIO_VERSION
    header.source = network_config.kAioNodeMotorSbo
    header.type = network_config.kMessageTypeMotorStatus
    header.sequence = 0
    header.timestamp = 0
    last_time = aio_header.AIO_EXPIRATION_TIME_US * 1e-6

    # Test first update (we have no knowledge of the initialization state).
    header.sequence = 100
    self.assertFalse(sink.IsDuplicate(header, '', last_time))
    header.sequence = (100 + aio_header.AIO_ACCEPTANCE_WINDOW) % 2**16
    self.assertFalse(sink.IsDuplicate(header, '', last_time))

    # Explicitly set last update.
    last_seq = 1
    SetLastSeqTime(sink, header.source, header.type, last_seq, last_time)

    # Test duplicate sequence number.
    header.sequence = last_seq
    self.assertTrue(sink.IsDuplicate(header, '', last_time))

    # Test previous sequence number.
    header.sequence = (last_seq + 2**16 - 1) % 2**16
    self.assertTrue(sink.IsDuplicate(header, '', last_time))

    # Test next sequence number.
    header.sequence = (last_seq + 1) % 2**16
    self.assertFalse(sink.IsDuplicate(header, '', last_time))

    # Test sequence number acceptance window.
    header.sequence = (last_seq + aio_header.AIO_ACCEPTANCE_WINDOW + 1) % 2**16
    self.assertTrue(sink.IsDuplicate(header, '', last_time))
    header.sequence = (last_seq + aio_header.AIO_ACCEPTANCE_WINDOW) % 2**16
    self.assertFalse(sink.IsDuplicate(header, '', last_time))

    # Test sequence number rollover.
    last_seq = 2**16 - 1
    SetLastSeqTime(sink, header.source, header.type, last_seq, last_time)
    header.sequence = (last_seq + 1) % 2**16
    self.assertFalse(sink.IsDuplicate(header, '', last_time))
    header.sequence = (last_seq + aio_header.AIO_ACCEPTANCE_WINDOW + 1) % 2**16
    self.assertTrue(sink.IsDuplicate(header, '', last_time))
    header.sequence = (last_seq + aio_header.AIO_ACCEPTANCE_WINDOW) % 2**16
    self.assertFalse(sink.IsDuplicate(header, '', last_time))

    # Test duplicate after expiration time.
    header.sequence = last_seq
    SetLastSeqTime(sink, header.source, header.type, last_seq, last_time)
    self.assertTrue(sink.IsDuplicate(header, '', last_time))
    cur_time = last_time + aio_header.AIO_EXPIRATION_TIME_US * 1e-6
    self.assertTrue(sink.IsDuplicate(header, '', cur_time - 1e-6))
    self.assertFalse(sink.IsDuplicate(header, '', cur_time))
    header.sequence = (last_seq + aio_header.AIO_ACCEPTANCE_WINDOW + 1) % 2**16
    self.assertTrue(sink.IsDuplicate(header, '', last_time))
    header.sequence = (last_seq + aio_header.AIO_ACCEPTANCE_WINDOW + 1) % 2**16
    self.assertTrue(sink.IsDuplicate(header, '', cur_time - 1e-6))
    self.assertFalse(sink.IsDuplicate(header, '', cur_time))

    # Clean-up.
    sink.Close()

if __name__ == '__main__':
  unittest.main()

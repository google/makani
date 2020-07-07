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

import time
import unittest

from makani.avionics.common import aio
from makani.avionics.linux.swig import aio_helper
from makani.avionics.linux.swig import aio_util
from makani.avionics.network import aio_node
from makani.avionics.network import message_type as aio_message_type
from makani.lib.python import c_helpers
import numpy

AIO_NODE_HELPER = c_helpers.EnumHelper('AioNode', aio_node)
MESSAGE_TYPE_HELPER = c_helpers.EnumHelper('MessageType', aio_message_type)


class AioHelperTest(unittest.TestCase):

  def testAioUpdate(self):
    message_type = 'kMessageTypeMotorStatus'

    # This also set the AIO updates for all messages to zero.
    self.assertTrue(aio_helper.Setup([message_type],
                                     'TelemetrySnapshot'))

    # Test whether GetAioUpdates correctly set aio_updates to all zeros.
    aio_updates = numpy.ones((len(AIO_NODE_HELPER), len(MESSAGE_TYPE_HELPER)),
                             dtype=numpy.ulonglong)
    aio_util.GetAioUpdates(aio_updates)
    self.assertEqual(numpy.amax(aio_updates), 0)

    # Mock a message.
    raw_message = '\0' * aio.GetPackMessageSize(message_type)
    source = AIO_NODE_HELPER.Value('MotorSbo')
    message_enum = MESSAGE_TYPE_HELPER.Value(message_type)

    # Put a message into CVT.
    aio_util.CvtPut(source, message_enum, 9, 1234, raw_message)

    # Get a message from the CVT and unpack it.
    result = aio_util.CvtGet(source, message_enum)
    self.assertEqual(len(result), 3)
    buf, sequence, timestamp = result
    self.assertEqual(sequence, 9)
    self.assertEqual(timestamp, 1234)
    message = aio_helper.UnpackMessage(buf, message_type)
    self.assertEqual(message.motor_status, 0)

    aio_util.AioClose()

  def testClockConversion(self):
    start_timestamp = 0.234
    start_us = aio_util.ClockGetUs()
    time.sleep(1.0)
    end_us = aio_util.ClockGetUs()
    end_timestamp = aio_helper.ClockUsToTimestamp(end_us, start_us,
                                                  start_timestamp)
    # There is some variation in how much time it really takes to run
    # time.sleep(1.0), so we check between (start_time + 1.0) and
    # (start_time + 1.0 + 0.2).
    self.assertGreaterEqual(end_timestamp, 1.234)
    self.assertLessEqual(end_timestamp, 1.234 + 0.02)

if __name__ == '__main__':
  unittest.main()

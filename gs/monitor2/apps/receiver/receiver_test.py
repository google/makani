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

"""Test modules for the receivers."""

import unittest

from makani.avionics.linux.swig import aio_util
from makani.gs.monitor2.apps.receiver import aio_receiver
from makani.gs.monitor2.apps.receiver import receiver_manager
from makani.gs.monitor2.apps.receiver import views
import mock


def _FakeAioSetup(message_types, receiver):  # pylint: disable=unused-argument
  return True


class TestReceivers(unittest.TestCase):

  @classmethod
  def setUp(cls):
    aio_util.InitFilters()

  def testAioReceier(self):
    with mock.patch('makani.avionics.linux.swig.aio_helper.Setup',
                    _FakeAioSetup):
      client_id_a = 0

      # There should be no receiver at the beginning.
      self.assertIsNone(receiver_manager.ReceiverManager.GetReceiver(
          client_id_a))

      # An Aio Receiver should be created.
      receiver_manager.ReceiverManager.CheckAndStartAioReceiver(
          client_id_a, views.CreateAioReceiver)
      current_receiver = receiver_manager.ReceiverManager.GetReceiver(
          client_id_a)
      self.assertIsInstance(current_receiver, aio_receiver.AioReceiver)

      # We should reuse the same receiver.
      client_id_b = 1
      receiver_manager.ReceiverManager.CheckAndStartAioReceiver(
          client_id_b, views.CreateAioReceiver)
      self.assertEqual(
          receiver_manager.ReceiverManager.GetReceiver(client_id_b),
          current_receiver)

      # The AIO receiver should still be available.
      receiver_manager.ReceiverManager.RetireReceiver(client_id_a)
      self.assertIsNone(receiver_manager.ReceiverManager.GetReceiver(
          client_id_a))
      self.assertEqual(
          receiver_manager.ReceiverManager.GetReceiver(client_id_b),
          current_receiver)

      # All subscribers are retired, the AIO receiver should be stopped.
      receiver_manager.ReceiverManager.RetireReceiver(client_id_b)
      self.assertIsNone(receiver_manager.ReceiverManager.GetReceiver(
          client_id_a))
      self.assertIsNone(receiver_manager.ReceiverManager.GetReceiver(
          client_id_b))


if __name__ == '__main__':
  unittest.main()

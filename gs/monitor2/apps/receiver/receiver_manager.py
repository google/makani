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

"""Utilities to manage the global receivers.

Receivers get AIO messages from either the AIO network or the log files.

The AIO receiver is shared by all clients. It is created when the first client
subscribes to receive messages from the AIO network, and is stopped when no
client subscribes to the AIO network.

The Log receiver is managed by a single client. It is created when the client
asks to replay the AIO messages from a log, and is also stopped upon client
request.
"""

import logging
import threading

from makani.gs.monitor2.apps.receiver import aio_receiver
from makani.gs.monitor2.apps.receiver import log_receiver


class ReceiverManager(object):
  """The class that keeps track of active receivers.

  It tracks what AIO or log receivers are in use and which clients are using
  them.
  """

  # _aio_receiver points to the global AIO Receiver, if one exists.
  _aio_receiver = None

  # _log_receivers is a dictionary of Log Receivers, keyed by client IDs.
  _log_receivers = {}

  # The set of client IDs subscribed to the global AIO Receiver.
  _aio_subscribers = set()

  _client_count = 0

  # The lock guards all operations start/stop the _aio_receiver.
  # We don't need locks for _log_receivers because each LogReceiver is private
  # to a client.
  _aio_receiver_lock = threading.Lock()

  @classmethod
  def GetNewClientId(cls):
    client_id = cls._client_count
    cls._client_count += 1
    return str(client_id)

  @classmethod
  def _RemoveAioSubscriber(cls, client_id):
    with cls._aio_receiver_lock:
      if client_id in cls._aio_subscribers:
        cls._aio_subscribers.remove(client_id)
      if not cls._aio_subscribers:
        cls._aio_receiver.TryStop()
        cls._aio_receiver = None

  @classmethod
  def _GetActiveReceiverOrNone(cls, receiver):
    if receiver and not receiver.IsDeactivated():
      return receiver
    else:
      return None

  @classmethod
  def GetReceiver(cls, client_id):
    """Get the receiver used by a client identified with `client_id`."""
    with cls._aio_receiver_lock:
      if client_id in cls._aio_subscribers:
        if client_id in cls._log_receivers:
          raise ValueError('One client ID can only be associated with one type '
                           'of receiver')
        if not cls._aio_receiver:
          raise ValueError('Found a client registered to a non-existing '
                           'AIO receiver.')
        return cls._GetAioReceiver()
      elif client_id in cls._log_receivers:
        return cls._GetLogReceiver(client_id)
      else:
        return None

  @classmethod
  def _GetAioReceiver(cls):
    """Get the AioReceiver or None if one does not exist.

    This function has to be protected by cls._aio_receiver_lock.

    Returns:
      The active AioReceiver, or None.
    """
    receiver = cls._GetActiveReceiverOrNone(cls._aio_receiver)
    if not receiver and cls._aio_receiver:
      # The receiver is already deactivated (likely by itself).
      cls._aio_receiver.TryStop()
      cls._aio_receiver = None
      cls._aio_subscribers.clear()
    return receiver

  @classmethod
  def _GetLogReceiver(cls, client_id):
    if client_id in cls._log_receivers:
      current_receiver = cls._log_receivers[client_id]
      receiver = cls._GetActiveReceiverOrNone(current_receiver)
      if not receiver and current_receiver:
        # The receiver is already deactivated (likely by itself).
        current_receiver.TryStop()
        del cls._log_receivers[client_id]
      return receiver
    else:
      return None

  @classmethod
  def SetLogReceiver(cls, receiver, client_id):
    if receiver is None:
      if client_id in cls._log_receivers:
        del cls._log_receivers[client_id]
    else:
      cls._log_receivers[client_id] = receiver

  @classmethod
  def RetireReceiver(cls, client_id):
    """Retire the receiver associated with a client."""

    message_receiver = cls.GetReceiver(client_id)
    if message_receiver:
      if isinstance(message_receiver, aio_receiver.AioReceiver):
        cls._RemoveAioSubscriber(client_id)
      elif isinstance(message_receiver, log_receiver.LogReceiver):
        message_receiver.TryStop()
        cls.SetLogReceiver(None, client_id)
      else:
        logging.error('Invalid receiver type "%s".', message_receiver.__class__)
        return False
      return True
    else:
      return False

  @classmethod
  def CheckAndStartAioReceiver(cls, client_id, aio_receiver_factory):
    """Start the AIO receiver for a client.

    Args:
      client_id: A identifier that distinguishes clients.
      aio_receiver_factory: A function to create the desired AioReceiver.

    Returns:
      True if a new AioReceiver is created.
    """

    message_receiver = cls.GetReceiver(client_id)
    if (message_receiver and
        not isinstance(message_receiver, aio_receiver.AioReceiver)):
      # Stop and remove active log message receivers.
      cls.RetireReceiver(client_id)

    with cls._aio_receiver_lock:
      aio_message_receiver = cls._GetAioReceiver()
      if not aio_message_receiver:
        # Start a global AIO message receiver if none exists.
        aio_message_receiver = aio_receiver_factory()
        aio_message_receiver.start()
        cls._aio_receiver = aio_message_receiver
        start_new = True
      else:
        start_new = False

      # Register the session for the AIO message receiver.
      cls._aio_subscribers.add(client_id)
    return start_new


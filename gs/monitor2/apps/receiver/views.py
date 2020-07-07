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

"""View functions to start and stop the receivers."""
import logging
import os
import string

from django.http import HttpResponse
from makani.avionics.network import message_type as aio_message_type
from makani.gs.monitor2.apps.receiver import aio_receiver
from makani.gs.monitor2.apps.receiver import log_receiver
from makani.gs.monitor2.apps.receiver import receiver_manager
from makani.gs.monitor2.project import settings
from makani.lib.python import c_helpers

_MESSAGE_TYPE_HELPER = c_helpers.EnumHelper('MessageType', aio_message_type)

# How long to listen for AIO messages each cycle, in seconds.
_AIO_RECEIVER_LISTENING_PERIOD_S = 0.005
# How long to sleep after each listening cycle, in seconds.
_AIO_RECEIVER_SLEEP_PERIOD_S = 0.015
# TIMEOUT when trying to receive the next AIO message.
_AIO_RECEIVER_AIO_TIMEOUT_S = _AIO_RECEIVER_LISTENING_PERIOD_S / 3.0
# Name of the AIO node used by the AIO receiver.
_AIO_RECEIVER_NODE = 'TelemetrySnapshot'

# Duration for a receiver to stay alive after the last client request, in
# seconds.
_RECEIVER_IDLE_TIMEOUT_S = 30
# The duration for a buffered message to be considered as not stale, or "live",
# in terms of the message's transfering period.
# E.g. if the message is transferred at 10Hz, or every 100ms, then a period
# number of 5 results in a "liveliness" duration of 500ms.
_RECEIVER_STALE_TIMEOUT_IN_PERIODS = 10
# Default time period for a buffered message to be considered as not stale.
_RECEIVER_DEFAULT_STALE_TIMEOUT_S = 0.5


def StartAioReceiver(request, client_id):  # pylint: disable=unused-argument
  """Start the AIO receiver for a client.

  Args:
    request: An HTTP request from the client.
    client_id: A identifier that distinguishes client tabs in the same session.

  Returns:
    An HttpResponse with text "receiver on".
  """
  receiver_manager.ReceiverManager.CheckAndStartAioReceiver(
      client_id, CreateAioReceiver)
  return HttpResponse('receiver_on')


def CreateAioReceiver():
  """Create an AioReceiver to receive AIO messages."""
  if settings.POPULATE_MESSAGES_FROM_SIM:
    message_type_values = [
        aio_message_type.kMessageTypeControllerCommand,
        aio_message_type.kMessageTypeControlSlowTelemetry,
        aio_message_type.kMessageTypeControlTelemetry,
        aio_message_type.kMessageTypeSimSensor,
        aio_message_type.kMessageTypeSimTetherDown,
        aio_message_type.kMessageTypeGroundStationPlcStatus,
        aio_message_type.kMessageTypeGroundEstimate,
        aio_message_type.kMessageTypeGroundEstimateSim,
    ]
    message_types = [_MESSAGE_TYPE_HELPER.Name(value)
                     for value in message_type_values]
  else:
    message_types = _MESSAGE_TYPE_HELPER.Names()

  params = {
      'minimum_stale_timeout_seconds': _RECEIVER_DEFAULT_STALE_TIMEOUT_S,
      'stale_timeout_in_periods': _RECEIVER_STALE_TIMEOUT_IN_PERIODS,
      'receiver_idle_timeout_seconds': _RECEIVER_IDLE_TIMEOUT_S,
      'message_types': message_types,
      'timeout': _AIO_RECEIVER_AIO_TIMEOUT_S,
      'aio_loop_duration': _AIO_RECEIVER_LISTENING_PERIOD_S,
      'process_duration': _AIO_RECEIVER_SLEEP_PERIOD_S,
      'receiver_aio_node': _AIO_RECEIVER_NODE,
      'network_yaml_file': settings.NETWORK_YAML,
      'aio_message_sequence_bits': settings.AIO_MESSAGE_SEQUENCE_BITS,
      'populate_from_sim': settings.POPULATE_MESSAGES_FROM_SIM,
  }
  return aio_receiver.AioReceiver(**params)


def _CreateLogReceiver(log_path, message_type, source):
  """Create a LogReceiver to replay a message from a particular source."""
  return log_receiver.LogReceiver(
      _RECEIVER_DEFAULT_STALE_TIMEOUT_S,
      _RECEIVER_STALE_TIMEOUT_IN_PERIODS,
      _RECEIVER_IDLE_TIMEOUT_S,
      log_path, message_type, source)


def StartLogReceiver(request, client_id,  # pylint: disable=unused-argument
                     message_type, source, log_path):
  """Start the log receiver for a client.

  Args:
    request: An HTTP request from the client.
    client_id: A identifier that distinguishes client tabs in the same session.
    message_type: Name of the message type.
    source: Name of the AIO node.
    log_path: Path to the log file on the server.

  Returns:
    An HttpResponse object with text "receiver_on", or an error message.
  """

  message_receiver = receiver_manager.ReceiverManager.GetReceiver(client_id)
  if message_receiver:
    receiver_manager.ReceiverManager.RetireReceiver(client_id)

  log_path_template = string.Template(log_path)
  log_abs_path = os.path.abspath(log_path_template.substitute(os.environ))

  root_log_path = '$MAKANI_HOME/logs'
  root_log_path_template = string.Template(root_log_path)
  root_log_abs_path = os.path.abspath(
      root_log_path_template.substitute(os.environ))
  if not log_abs_path.startswith(root_log_abs_path + os.sep):
    message = 'Only logs in %s can be replayed.' % root_log_path
    logging.error(message)
    return HttpResponse(message)

  log_message_receiver = _CreateLogReceiver(log_path, message_type, source)
  log_message_receiver.start()
  receiver_manager.ReceiverManager.SetLogReceiver(
      log_message_receiver, client_id)
  return HttpResponse('receiver_on')


def StopReceiver(request, client_id):  # pylint: disable=unused-argument
  """Unsubscribe the client from the receiver.

  The AIO receiver is stopped only if no client is listening to AIO messages.
  The log receiver is stopped immediately.

  Args:
    request: An HTTP request from the client.
    client_id: A identifier that distinguishes client tabs in the same session.

  Returns:
    An HttpResponse object with text "receiver_off".
  """
  receiver_manager.ReceiverManager.RetireReceiver(client_id)
  return HttpResponse('receiver_off')

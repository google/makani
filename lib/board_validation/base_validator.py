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
import datetime
import os
import time

import makani
from makani.avionics.network import message_type as aio_message_type
from makani.gs.monitor2.apps.receiver import aio_receiver
from makani.lib.python import c_helpers

_MESSAGE_TYPE_HELPER = c_helpers.EnumHelper('MessageType', aio_message_type)
_MESSAGE_TYPES = [name[0] for name in _MESSAGE_TYPE_HELPER]

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

_NETWORK_YAML = os.path.join(makani.HOME, 'avionics/network/network.yaml')
_AIO_MESSAGE_SEQUENCE_BITS = 16


class BoardValidator(object):
  """A class that continuously receives AIO messages and run validations."""

  def __init__(self, message_types, frequency, checklist):
    """Initialize the class.

    Args:
      message_types: The names of message types to listen to.
      frequency: The frequency to run the validation.
      checklist: A CheckList object.
    """

    if message_types is None:
      message_types = _MESSAGE_TYPES

    params = {
        'minimum_stale_timeout_seconds': _RECEIVER_DEFAULT_STALE_TIMEOUT_S,
        'stale_timeout_in_periods': _RECEIVER_STALE_TIMEOUT_IN_PERIODS,
        'receiver_idle_timeout_seconds': _RECEIVER_IDLE_TIMEOUT_S,
        'message_types': message_types,
        'timeout': _AIO_RECEIVER_AIO_TIMEOUT_S,
        'aio_loop_duration': _AIO_RECEIVER_LISTENING_PERIOD_S,
        'process_duration': _AIO_RECEIVER_SLEEP_PERIOD_S,
        'receiver_aio_node': _AIO_RECEIVER_NODE,
        'network_yaml_file': _NETWORK_YAML,
        'aio_message_sequence_bits': _AIO_MESSAGE_SEQUENCE_BITS,
    }
    self._aio_receiver = aio_receiver.AioReceiver(**params)
    self._aio_receiver.start()
    self._checklist = checklist
    self._period = 1.0 / frequency
    self._count = 0

  def Run(self):
    """Start the continuous validation process."""
    while True:
      try:
        current_time = datetime.datetime.now()
        self._ProcessResults(self._Validate())
        processed_time = datetime.datetime.now()
        sleep_time = (self._period -
                      (processed_time - current_time).total_seconds())
        self._count += 1
        time.sleep(max(0, sleep_time))
      except KeyboardInterrupt:
        self._aio_receiver.TryStop()
        break

  def _ProcessResults(self, check_results):
    """The callback function to post-process the check results."""
    raise NotImplementedError

  def _Validate(self):
    """Validate the currently received messages."""
    messages = self._aio_receiver.GetLatest(specified_message_enum=None)
    check_results = []
    for check_item in self._checklist.List():
      # Note a field can be a dict or list, each value will have to be
      # checked. So there may be multiple checks. The format is:
      # check_results = [{
      #     'name': ...,
      #     'value': {'stoplight': ..., 'value': ...},
      #     'std': {'stoplight': ..., 'value': ...}
      # }]
      check_item.Check(*check_item.Populate(messages))
      check_results += check_item.GetResults()
      if not check_results:
        continue
    return check_results

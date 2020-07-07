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

"""Command line client for controlling anti-collision lights."""

import os
import re
import socket

import makani
from makani.avionics.common import aio
from makani.avionics.common import cmd_client
from makani.avionics.common import pack_avionics_messages
from makani.avionics.network import aio_node
from makani.avionics.network import message_type
from makani.lib.python import c_helpers

aio_node_helper = c_helpers.EnumHelper('AioNode', aio_node)


def BuildLightParamDict():
  """Builds a dict mapping light param names to their indices."""
  # Build up parameter list.
  filename = os.path.join(makani.HOME, 'avionics/firmware/drivers/faa_light.c')
  with open(filename) as f:
    f_text = f.read()

  # Get parameter array string.
  re_string = r'static float \*g_mutable_param_addrs\[\] = {\s*^([\s\S]*)^};'
  array_string = re.search(re_string, f_text, re.MULTILINE)

  re_string = r'^ *&[\w.]+\[kLightType([\w\[\]\.]+)'
  light_param_keys = re.findall(re_string, array_string.group(0), re.MULTILINE)
  light_param_keys = [re.sub(r'\.', '_', i) for i in light_param_keys]
  light_param_keys = [re.sub(r'[\[\]]', '', i) for i in light_param_keys]

  return {key: ind for ind, key in enumerate(light_param_keys)}


# Constants.
LIGHTS = ['BaseStation', 'Port', 'Stbd', 'TailBottom', 'TailTop']
CONTROLLER = 'kAioNodeControllerA'
OPERATOR = 'kAioNodeOperator'
LIGHT_PARAMS = BuildLightParamDict()


class LightClientError(cmd_client.WingClientError):
  pass


def AioNodeNameFromLightNickname(light):
  """Returns AIO node name for the specified light node."""
  if light == 'BaseStation':
    return 'kAioNodeGpsBaseStation'
  else:
    return 'kAioNodeLight' + light


class LightCommandClient(cmd_client.WingCommandClient):
  """Command line client for running M600 anti-collision lights."""

  prompt = '(light_client) '
  _NUM_RETRIES = 10

  def __init__(self, *args, **kwargs):
    cmd_client.WingCommandClient.__init__(self, *args, **kwargs)

    self._set_param_aio_client = aio.AioClient(
        ['kMessageTypeFaaLightSetParam'], timeout=0.1)
    # The long range radio requires at least 2x160 ms for a complete command-
    # response cycle.
    self._ack_param_aio_client = aio.AioClient(
        ['kMessageTypeFaaLightAckParam'], timeout=0.35)
    self._get_param_aio_client = aio.AioClient(
        ['kMessageTypeFaaLightGetParam'], timeout=0.1)

  def _SetParam(self, line, message):  # pylint: disable=invalid-name
    """Sets a param for a specified light node."""
    targets, args = cmd_client.SelectArgs(
        line.split(), LIGHTS, require_some=True, select_all=True)
    param, args = cmd_client.SelectArgs(
        args, LIGHT_PARAMS.keys(), require_one=True, select_all=False)

    try:
      value = float(args[0])
    except ValueError:
      raise LightClientError('Invalid value: "%s".' % args[0])

    message.id = LIGHT_PARAMS[param]
    message.value = value

    failed_targets = []

    for target in targets:
      print 'Setting %s to %g on %s.' % (param, value, target)

      aio_target = AioNodeNameFromLightNickname(target)
      message.target = aio_node_helper.Value(aio_target)
      success = self._TrySetParam(
          message, 'kMessageTypeFaaLightSetParam', param, target, aio_target)

      if not success:
        failed_targets.append(target)

    if failed_targets:
      raise LightClientError('Failed to verify %s from %s.'
                             % (param, failed_targets))

  def _TrySetParam(self, message, msg_type, param, target, aio_target):
    for _ in xrange(self._NUM_RETRIES):
      self._set_param_aio_client.Send(message, msg_type, OPERATOR)
      for _ in xrange(self._NUM_RETRIES):
        try:
          _, header, ack = self._ack_param_aio_client.Recv()
          if (header.source == aio_node_helper.Value(aio_target)
              and header.type == message_type.kMessageTypeFaaLightAckParam
              and ack.id == message.id and ack.value == message.value):
            print '%s %s: %g' % (target, param, ack.value)
            return True
        except socket.timeout:
          return False
    return False

  @cmd_client.Command(num_args=3)
  def do_set_param(self, line):  # pylint: disable=invalid-name
    """Sets param for specified light node(s)."""
    message = pack_avionics_messages.FaaLightSetParamMessage()
    self._SetParam(line, message)

  def complete_set_param(self, text, line, *unused_args):  # pylint: disable=invalid-name
    arg_number = len(line.split())
    if not text:
      arg_number += 1

    if arg_number == 2:
      return self._CompleteArg(text, sorted(LIGHTS) + ['All'])
    elif arg_number == 3:
      return self._CompleteArg(text, sorted(LIGHT_PARAMS.keys()))
    else:
      return []

  complete_get_param = complete_set_param

  def _GetParam(self, line, message):
    targets, args = cmd_client.SelectArgs(
        line.split(), LIGHTS, require_some=True, select_all=True)
    param, _ = cmd_client.SelectArgs(
        args, LIGHT_PARAMS.keys(), require_one=True, select_all=False)

    message.id = LIGHT_PARAMS[param]

    failed_targets = []

    for target in targets:
      print 'Getting %s from %s...' % (param, target)
      success = True

      aio_target = AioNodeNameFromLightNickname(target)
      message.target = aio_node_helper.Value(aio_target)
      success = self._TryGetParam(
          message, 'kMessageTypeFaaLightGetParam', param, target, aio_target)

      if not success:
        failed_targets.append(target)

    if failed_targets:
      raise LightClientError('Failed to get %s from %s.'
                             % (param, failed_targets))

  def _TryGetParam(self, message, msg_type, param, target, aio_target):
    for _ in xrange(self._NUM_RETRIES):
      self._get_param_aio_client.Send(message, msg_type, OPERATOR)
      for _ in xrange(self._NUM_RETRIES):
        try:
          _, header, ack = self._ack_param_aio_client.Recv()
          if (header.source == aio_node_helper.Value(aio_target)
              and header.type == message_type.kMessageTypeFaaLightAckParam
              and ack.id == message.id):
            print '%s %s: %g' % (target, param, ack.value)
            return True
        except socket.timeout:
          return False
    return False

  @cmd_client.Command()
  def do_get_param(self, line):  # pylint: disable=invalid-name
    """Gets param from specified light node(s)."""
    message = pack_avionics_messages.FaaLightGetParamMessage()
    self._GetParam(line, message)


if __name__ == '__main__':
  client = LightCommandClient()
  try:
    client.cmdloop()
  except BaseException:
    raise

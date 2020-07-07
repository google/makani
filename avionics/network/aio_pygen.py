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

"""Programmatically generates fake AIO traffic.

This script is strictly for testing purposes. It is intended to support easy
sharing of commands that can be used to test high-level changes in other
tools, especially the monitors.

It uses exec in support of this goal. As a result, it should never ever ever
be run in a way that exposes it to external queries.

This is a cousin of aio_gen, which is focused on generating large volumes of
traffic and does not support programmatically time-varying messages.

See PrintUsage() for details.
"""

import json
import sys
import time

import gflags
from makani.avionics.common import aio
from makani.avionics.network import aio_node
from makani.avionics.network import message_type
from makani.lib.python import c_helpers

# numpy will be commonly used in the message-generation function supplied on
# the command line.
import numpy as np  # pylint: disable=unused-import

gflags.DEFINE_float('dt', 0.01,
                    'Sleep time between generation of messages.')

FLAGS = gflags.FLAGS

AIO_NODE_HELPER = c_helpers.EnumHelper('AioNode', aio_node)
MESSAGE_TYPE_HELPER = c_helpers.EnumHelper('MessageType', message_type)


def PrintUsage(argv):
  print """
Usage:
    %s [--dt <seconds>] <spec_1> [spec_2] [spec_3] ...

Each "spec" is a JSON string of the form
    { "type": <message type>,
      "node": <aio node>,
      "func": <generating function> }
"type" and "node" can be a long name, short name, or numeric value for a
MessageType or AioNode, respectively.

"func" is the body of a function with signature "def Func(msg, t)", for which
msg is a message structure corresponding to "type", and t is time since start
of this program. The function populates the data in msg as desired and need not
return explicitly.

Example:

bazel run //avionics/network:aio_pygen --dt 0.01 \
  '{"type": "ServoStatus", "node": "ServoE1",
  "func": "msg.angle_desired = t; msg.angle_bias = -t"}' \
  '{"type": "MotorStatus", "node": "MotorSbo",
  "func": "msg.bus_current = t; msg.omega = 2.0 * t"}'

To specify a multi-line function body with proper indentation, begin "func"
with a newline, e.g.

bazel run //avionics/network:aio_pygen --dt 0.01 \
  '{"type": "ServoStatus", "node": "ServoE1",
  "func": "
msg.angle_desired = t
msg.angle_bias = -t"}'
""" % argv[0]


class MessageGenerator(object):
  """Generates messages according to an input spec."""

  def __init__(self, spec):
    assert set(spec.keys()) == {'type', 'node', 'func'}

    self.type = MESSAGE_TYPE_HELPER.Name(spec['type'])
    self.node = AIO_NODE_HELPER.Name(spec['node'])

    self._struct = aio.GetAioMessageStruct(self.type)

    # Any use of exec should be met with intense skepticism. The motivation
    # behind this case is that sharing a command line to generate a test
    # scenario is much simpler than sharing a full program.
    exec('def Func(msg, t):\n%s'  # pylint: disable=exec-used
         % '\n'.join('  ' + line for line in spec['func'].splitlines()))
    self._func = Func  # pylint: disable=undefined-variable

  def GetMessage(self, t):
    message = self._struct()
    self._func(message, t)
    return message


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s' % e
    PrintUsage(argv[0])
    sys.exit(1)

  generators = []
  for spec in argv[1:]:
    # Use strict=False to allow newlines.
    spec = json.loads(spec, strict=False)
    generators.append(MessageGenerator(spec))
  client = aio.AioClient([g.type for g in generators])

  t0 = time.time()
  while True:
    t = time.time() - t0
    for g in generators:
      client.Send(g.GetMessage(t), g.type, g.node)
    time.sleep(FLAGS.dt)


if __name__ == '__main__':
  main(sys.argv)

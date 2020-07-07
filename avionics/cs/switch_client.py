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

"""Command line client for controlling core switches."""

import socket
import sys
import time

import gflags
from makani.avionics.common import aio
from makani.avionics.common import pack_avionics_messages
from makani.avionics.network import network_config
from makani.lib.python import c_helpers

gflags.DEFINE_string('disable_ports', '',
                     'List of ports to disable (default is none).')
gflags.DEFINE_string('node', None, 'AIO node to command.')
gflags.DEFINE_bool('i_am_not_breaking_the_wing', False,
                   'Specify this flag to disable ports on the wing core '
                   'switches.  Incorrect usage may disconnect you from the '
                   'wing.')
gflags.MarkFlagAsRequired('node')
FLAGS = gflags.FLAGS

select_helper = c_helpers.EnumHelper('SwitchConnectionSelect',
                                     pack_avionics_messages)


def Main(argv):
  """Set a mask to disable specific ports on a core switch."""
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    sys.stderr.write('\nError: %s\n\nUsage: %s ARGS\n%s\n'
                     % (e, argv[0], FLAGS))
    sys.exit(1)

  disable_port_mask = 0
  for port in FLAGS.disable_ports.split(','):
    if port:
      if int(port) < 0:
        raise ValueError('Invalid port: %s.' % port)
      disable_port_mask |= 1 << int(port)

  config = network_config.NetworkConfig()

  target_node = config.GetAioNode(FLAGS.node)
  if target_node.label_name != 'core_switch':
    raise ValueError('Only core switches supported in switch_client.')
  if (target_node.snake_name != 'cs_gs_a' and
      target_node.snake_name != 'cs_gs_b' and
      not FLAGS.i_am_not_breaking_the_wing):
    raise ValueError('Can only disable ports on cs_gs_a or cs_gs_b by default.'
                     '  If you know what you are doing please specify '
                     '--i_am_not_breaking_the_wing to modify the wing core '
                     'switches.  Incorrect usage can break the wing (requiring '
                     'power cycle to restore connection).')
  if (disable_port_mask & (1 << 25)) != 0:
    raise ValueError('Cannot disable the command center port.')

  client_sender = aio.AioClient(['kMessageTypeCoreSwitchConnectionSelect'],
                                allowed_sources=['kAioNodeOperator'])
  client_receiver = aio.AioClient(['kMessageTypeCoreSwitchStatus'],
                                  allowed_sources=[target_node.enum_name],
                                  timeout=1.0)

  message = pack_avionics_messages.CoreSwitchConnectionSelectMessage()
  message.target = target_node.enum_value
  message.disable_port_mask = disable_port_mask

  end_time = time.time() + 10  # Give them 10 seconds to switch.
  while time.time() < end_time:
    client_sender.Send(message, 'kMessageTypeCoreSwitchConnectionSelect',
                       'kAioNodeOperator')
    try:
      (_, _, received_message) = client_receiver.Recv()
      if received_message.disabled_port_mask == message.disable_port_mask:
        print 'Disabled port mask for %s set to 0x%x.' % (
            target_node.snake_name, received_message.disabled_port_mask)
        break
    except socket.timeout:
      pass
  else:
    print ('No response from %s.  Unable to set port mask.'
           % target_node.snake_name)


if __name__ == '__main__':
  Main(sys.argv)

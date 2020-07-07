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


"""Generate NetSendAioXXX functions to pack & send messages on the TMS570."""

import sys
import textwrap

import gflags
from makani.avionics.common import pack_avionics_messages
from makani.avionics.network import message_type
from makani.lib.python import c_helpers

gflags.DEFINE_string('source_file', None,
                     'Full path to output source file.',
                     short_name='s')
gflags.DEFINE_string('header_file', None,
                     'Full path to output header file.',
                     short_name='h')
FLAGS = gflags.FLAGS

message_type_helper = c_helpers.EnumHelper('MessageType', message_type)


def _GenStructName(message_type_name):
  """Generate C message structure for a given message type."""
  return message_type_helper.ShortName(message_type_name) + 'Message'


def _GenPackFunctionName(message_type_name):
  """Generate C pack function name for a given message type."""
  return 'Pack' + _GenStructName(message_type_name)


def _GenPackedSizeMacroName(message_type_name):
  """Generate packed size macro name for a given message type."""
  return 'PACK_' + _GenStructName(message_type_name).upper() + '_SIZE'


def _WriteNetSendAioFunction(message_type_name, f):
  """Write NetSendAio<MessageName>() function."""
  struct_name = _GenStructName(message_type_name)
  size_macro = _GenPackedSizeMacroName(message_type_name)
  pack_func = _GenPackFunctionName(message_type_name)
  pack_cast = 'PackAioMessageFunction'
  f.write(textwrap.dedent('''
      COMPILE_ASSERT({1} <= MAX_AIO_PAYLOAD_SIZE,
                     {1}_must_fit_within_MAX_AIO_PAYLOAD_SIZE);
      bool NetSendAio{0}(const {0} *msg) {{
        return NetSendAioPacked({2}, ({3}){4}, msg);
      }}
      '''.format(struct_name, size_macro, message_type_name, pack_cast,
                 pack_func)))


def _WriteNetSendAioPrototype(message_type_name, f):
  """Write NetSendAio<MessageName>() prototype."""
  struct_name = _GenStructName(message_type_name)
  f.write('bool NetSendAio{0}(const {0} *msg);\n'.format(struct_name))


def _WriteSource(messages, f):
  """Write source file."""
  f.write(textwrap.dedent('''
      #include "avionics/firmware/network/net_send.h"

      #include <stdbool.h>

      #include "avionics/common/avionics_messages.h"
      #include "avionics/common/pack_avionics_messages.h"
      #include "avionics/firmware/network/net.h"
      #include "avionics/network/message_type.h"
      #include "common/macros.h"
      ''')[1:])
  for m in messages:
    _WriteNetSendAioFunction(m, f)


def _WriteHeader(messages, f):
  """Write header file."""
  guard = 'AVIONICS_FIRMWARE_NETWORK_NET_SEND_H_'
  f.write(textwrap.dedent('''
      #ifndef {0}
      #define {0}

      #include <stdbool.h>

      #include "avionics/common/avionics_messages.h"
      #include "avionics/firmware/network/net.h"

      '''.format(guard))[1:])
  for m in messages:
    _WriteNetSendAioPrototype(m, f)
  f.write(textwrap.dedent('''
      #endif  // {0}'''.format(guard)))


def main(argv):
  """Entry point."""
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  messages = [m for (m, _) in message_type_helper
              if _GenPackFunctionName(m) in pack_avionics_messages.__dict__]
  if FLAGS.header_file:
    with open(FLAGS.header_file, 'w') as f:
      _WriteHeader(messages, f)
  if FLAGS.source_file:
    with open(FLAGS.source_file, 'w') as f:
      _WriteSource(messages, f)


if __name__ == '__main__':
  main(sys.argv)

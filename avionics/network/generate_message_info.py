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

"""Collect all message type information into one common module."""

import re
import sys
import textwrap

import gflags
import makani
from makani.avionics.network import aio_message
from makani.lib.python import c_helpers


def _GenerateSource(info_map, header_file):
  """Generate output source file as a string."""
  parts = [textwrap.dedent("""\
      #include "{header_file}"

      #include <assert.h>
      #include <stdbool.h>
      #include <stddef.h>
      #include <stdint.h>
      """).format(header_file=header_file)]

  includes = aio_message.GetHeaderFilesFromMessageInfoMap(info_map)
  includes += ['avionics/network/message_type.h']
  parts += ['#include "%s"' % f for f in sorted(includes)]

  parts.append('\nconst AioMessageInfo kAioMessageInfo[kNumMessageTypes] = {')
  for message in sorted(info_map.keys(), key=lambda m: m.name):
    info = info_map[message]
    if info:
      parts.append(c_helpers.Indent(textwrap.dedent("""\
          [{enum_name}] = {{
            .name = "{enum_name}",
            .short_name = "{short_name}",
            .pack_func = (PackAioMessageFunction){pack_func},
            .unpack_func = (UnpackAioMessageFunction){unpack_func},
            .pack_size = {pack_size},
            .unpack_size = {unpack_size}}},""").format(
                enum_name=message.enum_name,
                short_name=message.name,
                pack_func=info.pack_func,
                pack_size=info.pack_size,
                unpack_func=info.unpack_func,
                unpack_size=info.unpack_size)))
    else:
      parts.append(c_helpers.Indent(textwrap.dedent("""\
          [{enum_name}] = {{
            .name = "{enum_name}",
            .short_name = "{short_name}",
            .pack_func = NULL,
            .unpack_func = NULL,
            .pack_size = -1,
            .unpack_size = -1}},""").format(
                enum_name=message.enum_name,
                short_name=message.name)))
  parts.append('};')

  parts.append(textwrap.dedent("""
      size_t PackAioMessageData(MessageType type, const void *in,
                                 uint8_t *out) {
        if (type < kNumMessageTypes
            && kAioMessageInfo[type].pack_func != NULL) {
          return kAioMessageInfo[type].pack_func(in, 1, out);
        }
        assert(false);
        return 0;
      }

      size_t UnpackAioMessageData(MessageType type, const uint8_t *in,
                                   void *out) {
        if (type < kNumMessageTypes
            && kAioMessageInfo[type].unpack_func != NULL) {
          return kAioMessageInfo[type].unpack_func(in, 1, out);
        }
        assert(false);
        return 0;
      }
      """))
  return '\n'.join(parts)


def _GenerateHeader(info_map, header_file):
  """Generate output header file as a string."""
  guard = re.sub('[/.]', '_', header_file).upper() + '_'
  parts = [textwrap.dedent("""\
      #ifndef {guard}
      #define {guard}

      #include <stddef.h>
      #include <stdint.h>
      """).format(guard=guard)]

  includes = aio_message.GetHeaderFilesFromMessageInfoMap(info_map)
  includes += ['avionics/common/aio_header.h']
  includes += ['avionics/network/message_type.h']
  parts += ['#include "%s"' % f for f in sorted(includes)]

  parts.append(textwrap.dedent("""
      #ifdef __cplusplus
      extern "C" {
      #endif
      """))

  parts.append(textwrap.dedent("""\
      typedef size_t (* const PackAioMessageFunction)(
          const void *in, size_t num, uint8_t *out);
      typedef size_t (* const UnpackAioMessageFunction)(
          const uint8_t *in, size_t num, void *out);

      typedef struct {
        const char *name;
        const char *short_name;
        PackAioMessageFunction pack_func;
        UnpackAioMessageFunction unpack_func;
        int32_t pack_size;
        int32_t unpack_size;
      } AioMessageInfo;
      """))

  parts.append('typedef union {')
  for message in sorted(info_map.keys(), key=lambda m: m.name):
    info = info_map[message]
    if info:
      parts.append('  {struct_name} {var};'.format(
          struct_name=info.struct_name,
          var=c_helpers.CamelToSnake(message.name)))
  parts.append('} AioMessageData;')

  parts.append(textwrap.dedent("""
      typedef struct {
        AioHeader header;
        AioMessageData u;  // Union of all possible data types.
      } AioMessage;
      """))

  parts.append(textwrap.dedent("""\
      extern const AioMessageInfo kAioMessageInfo[kNumMessageTypes];

      size_t PackAioMessageData(MessageType type, const void *in,
                                 uint8_t *out);
      size_t UnpackAioMessageData(MessageType type, const uint8_t *in,
                                   void *out);

      #ifdef __cplusplus
      }}  // extern "C"
      #endif

      #endif  /* {guard} */
      """).format(guard=guard))
  return '\n'.join(parts)


def main(argv):
  """Entry point."""
  gflags.DEFINE_string('autogen_root', makani.HOME,
                       'Root of the source tree for the output files.')
  try:
    argv = gflags.FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], gflags.FLAGS)
    sys.exit(1)

  info_map = aio_message.GetMessageInfoMapByNetworkFile()
  source_file = 'avionics/network/message_info.c'
  header_file = 'avionics/network/message_info.h'

  with open(gflags.FLAGS.autogen_root + '/' + source_file, 'w') as f:
    f.write(_GenerateSource(info_map, header_file))
  with open(gflags.FLAGS.autogen_root + '/' + header_file, 'w') as f:
    f.write(_GenerateHeader(info_map, header_file))


if __name__ == '__main__':
  main(sys.argv)

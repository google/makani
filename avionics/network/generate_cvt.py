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

"""Generate cvt_entries.c source files."""

import collections
import os
import sys
import textwrap

import gflags
from makani.avionics.network import aio_message
from makani.avionics.network import network_config


def _GenerateSourceFile(cvt_entries):
  """Generate cvt_entries.c source file."""
  parts = [textwrap.dedent("""\
      #include "avionics/common/cvt_entries.h"

      #include <assert.h>
      #include <stdbool.h>
      #include <stdint.h>
      """)]

  includes = set([
      'avionics/network/aio_node.h',
      'avionics/network/message_type.h',
      'common/macros.h'])

  entries = []
  for entry in cvt_entries:
    info = aio_message.GetInfoByName(entry.message.name)
    entries.append('  CVT_ENTRY({source}, {message}, {length}),'.format(
        source=entry.source.enum_name,
        message=entry.message.enum_name,
        length=info.pack_size))
    includes.add(info.header_file)

  parts += ['#include "{}"'.format(i) for i in sorted(list(includes))]

  if entries:
    parts.append(textwrap.dedent("""
        // 2D lookup table indexed by (source, type). Each element is a pointer
        // to a CVT entry, or NULL if the (source, type) is invalid. The
        // CVT_ENTRY macro declares a CVT entry sized to hold a given message.
        // Messages are stored in an 8-byte aligned buffer.
        #define CVT_ENTRY(s, t, l)                                      \\
          [s][t] = &(CvtEntry) {                                        \\
            .len   = l,                                                 \\
            .data  = (uint64_t [(l + 7) / 8]) {0},  /* NOLINT */        \\
            .state = (CvtEntryState [1]) {{  /* NOLINT */               \\
                .sequence  = 0U,                                        \\
                .timestamp = INT32_MIN,                                 \\
                .updated   = false,                                     \\
              }}                                                        \\
          }
        """))

    parts.append('static const CvtEntry * const '
                 'g_cvt[kNumAioNodes][kNumMessageTypes] = {')
    parts.extend(sorted(entries))
    parts.append('};')
    parts.append(textwrap.dedent("""
        const CvtEntry *GetCvtEntry(AioNode source, MessageType type) {
          assert(0 <= source && source < ARRAYSIZE(g_cvt));
          assert(type < ARRAYSIZE(g_cvt[0]));
          return g_cvt[source][type];
        }
        """))
  else:
    parts.append(textwrap.dedent("""\
        const CvtEntry *GetCvtEntry(AioNode source, MessageType type) {
          (void)source;
          (void)type;
          return NULL;
        }
        """))

  return '\n'.join(parts)


def main(argv):
  gflags.DEFINE_bool('all_nodes', False,
                     'Generate CVT entries for all nodes.')
  gflags.DEFINE_bool('all_q7s', False,
                     'Generate CVT entries for all Q7 nodes.')
  gflags.DEFINE_bool('all_tms570s', False,
                     'Generate CVT entries for all TMS570 nodes.')
  gflags.DEFINE_list('aio_labels', [],
                     'Generate CVT entries for a list of AioNode labels.')
  gflags.DEFINE_list('aio_nodes', [],
                     'Generate CVT entries for a list of AioNodes.')
  gflags.DEFINE_string('output_source', 'cvt_entries.c',
                       'Output source file name.')

  flags, argv = network_config.ParseGenerationFlags(argv)
  config = network_config.NetworkConfig(flags.network_file)

  # The CVT indexes messages by source and message type.
  CvtEntry = collections.namedtuple(  # pylint: disable=invalid-name
      'CvtEntry', ['source', 'message'])
  cvt_entries = set()

  # Exclude messages not allowed in the CVT (e.g., variable length messages).
  messages = [m for m in config.aio_messages if not m.inhibit_cvt]

  # Generate CVT entries suitable for all nodes. Monitors, debugging tools,
  # and general desktop applications should use this output.
  if flags.all_nodes:
    for message in messages:
      for sender in message.all_senders:
        cvt_entries.add(CvtEntry(source=sender, message=message))

  # Generate CVT entries suitable for all Q7 nodes.
  if flags.all_q7s:
    for message in messages:
      if [r for r in message.all_receivers if r.q7_node]:
        for sender in message.all_senders:
          cvt_entries.add(CvtEntry(source=sender, message=message))

  # Generate CVT entries suitable for all TMS570 nodes.
  if flags.all_tms570s:
    for message in messages:
      if [r for r in message.all_receivers if r.tms570_node]:
        for sender in message.all_senders:
          cvt_entries.add(CvtEntry(source=sender, message=message))

  # Generate CVT entries suitable for specific labels (i.e., common application
  # binaries).
  for aio_label in flags.aio_labels:
    for aio_node in config.GetAioNodesByLabel(aio_label):
      for message in messages:
        for route in message.routes:
          if aio_node in route.receivers:
            for sender in route.senders:
              cvt_entries.add(CvtEntry(source=sender, message=message))

  # Generate CVT entries suitable for specific nodes.
  for aio_node in flags.aio_nodes:
    for message in messages:
      for route in message.routes:
        if aio_node in route.receivers:
          for sender in route.senders:
            cvt_entries.add(CvtEntry(source=sender, message=message))

  # Write source.
  source_file = os.path.join(flags.output_dir, flags.output_source)
  with open(source_file, 'w') as f:
    f.write(_GenerateSourceFile(cvt_entries))


if __name__ == '__main__':
  main(sys.argv)

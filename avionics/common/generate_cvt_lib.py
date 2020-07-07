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


"""Generate type safe CvtGetX and CvtPutX functions for accessing the CVT."""

import importlib
import os
import sys
import textwrap

import gflags
import makani
from makani.avionics.network import message_type
from makani.lib.python import c_helpers

gflags.DEFINE_string('h2py_dir', '.',
                     'Full path to H2PY_DIR containing pack modules.')
gflags.DEFINE_string('autogen_root', makani.HOME,
                     'Root of the source tree for the output files.')
FLAGS = gflags.FLAGS

message_type_helper = c_helpers.EnumHelper('MessageType', message_type)


def _GenStructName(message_type_name):
  short_name = message_type_helper.ShortName(message_type_name)
  if short_name in ['ControlTelemetry', 'ControlSlowTelemetry', 'SimTelemetry',
                    'GroundTelemetry']:
    return short_name
  else:
    return short_name + 'Message'


def _GenPackFunctionName(message_type_name):
  return 'Pack' + _GenStructName(message_type_name)


def _GenUnpackFunctionName(message_type_name):
  return 'Unpack' + _GenStructName(message_type_name)


def _GenPackedSizeMacroName(message_type_name):
  return 'PACK_' + _GenStructName(message_type_name).upper() + '_SIZE'


def _WriteCvtFunctions(message_type_name, hdr_file, src_file):
  """Write CvtGetX and CvtPutX functions."""
  struct_name = _GenStructName(message_type_name)
  size_macro = _GenPackedSizeMacroName(message_type_name)
  pack_func = _GenPackFunctionName(message_type_name)
  unpack_func = _GenUnpackFunctionName(message_type_name)
  pack_cast = 'PackCvtFunction'
  unpack_cast = 'UnpackCvtFunction'

  put_proto = ('CvtEventCodes CvtPut{0}(AioNode src, const {0} *msg, '
               'uint16_t sequence, int64_t timestamp)').format(struct_name)
  hdr_file.write(put_proto + ';\n')
  src_file.write(textwrap.dedent("""
      {0} {{
        return CvtPutPacked(src, {1}, ({2}){3}, msg, {4}, sequence, timestamp);
      }}
      """.format(put_proto, message_type_name, pack_cast, pack_func,
                 size_macro)))

  get_proto = ('bool CvtGet{0}(AioNode src, {0} *msg, uint16_t *sequence, '
               'int64_t *timestamp)').format(struct_name)
  hdr_file.write(get_proto + ';\n')
  src_file.write(textwrap.dedent("""
      {0} {{
        return CvtGetUnpacked(src, {1}, ({2}){3}, msg, sequence, timestamp);
      }}
      """.format(get_proto, message_type_name, unpack_cast, unpack_func)))


def _WriteCvtFile(dirname, pack_basename_wo_ext, messages):
  """Write cvt_xxx.c/cvt_xxx.h files for a specific packing script."""
  basename = pack_basename_wo_ext.replace('pack_', '')
  cvt_basename = os.path.join(dirname, 'cvt_' + basename)
  hdr_name = cvt_basename + '.h'
  src_name = cvt_basename + '.c'
  pack_header = os.path.join(dirname, pack_basename_wo_ext + '.h')
  message_header = os.path.join(dirname, basename + '.h')

  hdr_file = open(os.path.join(FLAGS.autogen_root, hdr_name), 'w')
  hdr_guard = hdr_name.upper().replace(os.path.sep, '_')
  hdr_guard = hdr_guard.replace(os.path.extsep, '_') + '_'
  hdr_file.write(textwrap.dedent("""
      #ifndef {0}
      #define {0}

      #include <stdbool.h>
      #include <stdint.h>

      #include "avionics/common/cvt.h"
      #include "avionics/network/aio_node.h"
      #include "{1}"

      #ifdef __cplusplus
      extern "C" {{
      #endif

      """)[1:].format(hdr_guard, message_header))

  src_file = open(os.path.join(FLAGS.autogen_root, src_name), 'w')
  src_file.write(textwrap.dedent("""
      #include "{0}"

      #include <stdbool.h>
      #include <stdint.h>

      #include "avionics/common/cvt.h"
      #include "avionics/network/aio_node.h"
      #include "{1}"
      """)[1:].format(hdr_name, pack_header))

  for m in messages:
    _WriteCvtFunctions(m, hdr_file, src_file)

  hdr_file.write(textwrap.dedent("""
      #ifdef __cplusplus
      }}  // extern "C"
      #endif

      #endif  // {0}
      """).format(hdr_guard))

  hdr_file.close()
  src_file.close()


def main(argv):
  """Entry point."""
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS [PACK_FILES...]\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  h2py_pack_files = argv[1:]
  for filename in h2py_pack_files:
    rel_path = os.path.relpath(filename, start=FLAGS.h2py_dir)
    rel_path_wo_ext = rel_path.rsplit(os.path.extsep, 1)[0]
    mod_path = rel_path_wo_ext.replace(os.path.sep, '.')
    pack_messages = importlib.import_module('makani.' + mod_path)
    messages = [m for (m, _) in message_type_helper
                if _GenUnpackFunctionName(m) in pack_messages.__dict__]

    # Not all packing functions adhere to message naming standards. Ignore
    # them here.
    if messages:
      _WriteCvtFile(os.path.dirname(rel_path_wo_ext),
                    os.path.basename(rel_path_wo_ext),
                    messages)


if __name__ == '__main__':
  main(sys.argv)

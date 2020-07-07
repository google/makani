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


"""Generates header file containing a version CRC.

This module expects a list of relevant source files to include in its
version calculation.  Specify file paths relative to makani.HOME.
"""

import binascii
import os
import re
import sys
import textwrap

import gflags
import makani

gflags.DEFINE_string('autogen_root', makani.HOME,
                     'Root of the source tree for the output files.')
gflags.DEFINE_string('output_file', None,
                     'Output header file to write.')
gflags.DEFINE_string('macro_name', 'VERSION',
                     'Macro name to define.')
gflags.DEFINE_integer('bits', 32,
                      'Number of bits to represent version.')
gflags.MarkFlagAsRequired('output_file')

FLAGS = gflags.FLAGS


def GetVersion(file_list):
  """Compute version from file_list."""
  crc = 0
  for filename in file_list:
    with open(filename, 'rb') as f:
      crc = binascii.crc32(f.read(), crc)
  return crc & ((1 << FLAGS.bits) - 1)


def GenerateHeader(input_paths, output_path):
  """Generate header file containing the version CRC."""
  with open(output_path, 'w') as f:
    output_relpath = os.path.relpath(output_path, FLAGS.autogen_root)
    guard_str = re.sub(r'[-/\.]', '_', output_relpath).upper() + '_'

    chars = (FLAGS.bits + 3) / 4
    version = '0x{{:0{}X}}'.format(chars).format(GetVersion(input_paths))
    f.write(textwrap.dedent(
        """
        #ifndef {0}
        #define {0}

        #define {1} {2}

        #endif  // {0}
        """.format(guard_str, FLAGS.macro_name, version))[1:])


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  GenerateHeader(argv[1:], FLAGS.output_file)


if __name__ == '__main__':
  main(sys.argv)

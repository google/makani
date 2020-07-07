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


"""Strip static qualifier from C functions to enable unit testing."""

import os
import re
import sys
import textwrap

import gflags
import makani

gflags.DEFINE_string('autogen_root', makani.HOME,
                     'Root of the source tree for the output files.')
gflags.DEFINE_string('input_source', None,
                     'Full path to input source file.')
gflags.MarkFlagAsRequired('input_source')
gflags.DEFINE_string('output_source', None,
                     'Full path to output source file.')
gflags.DEFINE_string('output_header', None,
                     'Full path to output header file.')
gflags.DEFINE_string('static_prefix', '',
                     'Function prefix to prepend to static functions.')
gflags.DEFINE_string('stub', '',
                     'List of functions to rename.')
gflags.DEFINE_string('stub_prefix', 'Stubbed',
                     'Function prefix to prepend to stub functions.')
FLAGS = gflags.FLAGS


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  # Read input file.
  with open(FLAGS.input_source, 'r') as input_source:
    source_data = ''
    if FLAGS.output_header:
      header_path = os.path.relpath(FLAGS.output_header,
                                    start=FLAGS.autogen_root)
      source_data += '#include "%s"\n' % header_path
    else:
      header_path = ''
    source_data += input_source.read()
    static_funcs = []

    # Rewrite stub function prototypes and definitions.
    for stub_func in FLAGS.stub.split(','):
      if stub_func:
        stub_re = re.compile(
            r'^(static\s+)?(.*\s)(%s)(\s*\([^\{;]*\))(\s*[\{;])' % stub_func,
            re.MULTILINE)
        static_funcs += [s for s in stub_re.findall(source_data) if s[0]]
        source_data = stub_re.sub(
            r'\2{0}\3\4;\n\2{0}\3\4\5'.format(FLAGS.stub_prefix), source_data)

    # Rewrite static function prototypes and definitions.
    static_re = re.compile(r'^(static\s+)(.*\s)([a-zA-Z][a-zA-Z_0-9]*)'
                           r'(\s*\([^\{;]*\))(\s*[\{;])', re.MULTILINE)
    static_funcs += static_re.findall(source_data)
    source_data = static_re.sub(
        r'\2{0}\3\4\5'.format(FLAGS.static_prefix), source_data)

    # Rewrite static function calls.
    for in_groups in static_funcs:
      in_func = in_groups[2]
      source_data = re.sub(r'(\W)(%s)(\s*\()' % in_func,
                           r'\1%s\2\3' % FLAGS.static_prefix, source_data,
                           re.MULTILINE)

    # Generate header data.
    header_guard = header_path.upper().replace(os.path.sep, '_')
    header_guard = header_guard.replace(os.path.extsep, '_') + '_'
    header_orig = os.path.relpath(FLAGS.input_source, start=makani.HOME)
    header_orig = re.sub(r'\.c.*$', '.h', header_orig)
    header_data = textwrap.dedent("""
        #ifndef {0}
        #define {0}

        #include "{1}"

    """)[1:].format(header_guard, header_orig)

    for in_groups in static_funcs:
      out_return = in_groups[1]
      out_func = FLAGS.static_prefix + in_groups[2]
      out_args = in_groups[3]
      header_data += '{}{}{};\n'.format(out_return, out_func, out_args)

    header_data += textwrap.dedent("""
        #endif  // {}
    """)[:-1].format(header_guard)

    # Write output source file.
    if FLAGS.output_source:
      with open(FLAGS.output_source, 'w') as output_source:
        output_source.write(source_data)

    # Write output header file.
    if FLAGS.output_header:
      with open(FLAGS.output_header, 'w') as output_header:
        output_header.write(header_data)


if __name__ == '__main__':
  main(sys.argv)

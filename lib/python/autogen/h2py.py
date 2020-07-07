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

"""Generates a Python wrapper for a C header.

This script is a wrapper for clang2py, customized for use in Bazel builds.
"""

from __future__ import absolute_import
from __future__ import print_function
import os
import subprocess
import sys
import tempfile
import gflags

import makani
from makani.lib.python.autogen import autogen_util

gflags.DEFINE_string('header', None, 'Input .h file.')

gflags.DEFINE_string('output', None, 'Output .py file.')

gflags.DEFINE_multistring('include_dir', None,
                          'Directory of included header files.')

gflags.DEFINE_list('defines', None, 'Compilation definitions.')

gflags.DEFINE_list('shared_libs', None,
                   'Shared libraries against which the Python wrapper module '
                   'will link.')

gflags.DEFINE_string('shared_lib_root', None,
                     'Root of the shared lib tree. This is needed to make '
                     'relative references to .so files work properly.')

FLAGS = gflags.FLAGS


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError as e:
    print('\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS))
    sys.exit(1)

  defines = FLAGS.defines if FLAGS.defines else []
  shared_libs = FLAGS.shared_libs if FLAGS.shared_libs else []
  if shared_libs and not FLAGS.shared_lib_root:
    raise ValueError('--shared_lib_root must be specified if --shared_libs is '
                     'nonempty.')

  clang_args = (['-I' + d for d in FLAGS.include_dir] +
                ['-I/usr/include/clang/7.0.1/include/'] +
                ['-D' + d for d in defines])

  with tempfile.NamedTemporaryFile(suffix='.py') as py_file:
    cmd = ([os.path.join(makani.HOME, 'lib/python/autogen/clang2py'), '-i',
            '-q', '--clang-args="%s"' % ' '.join(clang_args)]
           + ['-k', 'cdefstum']
           + ['-l' + s for s in shared_libs]
           + ['-o', py_file.name, os.path.abspath(FLAGS.header)])
    # TODO: The shell argument should not be required here.
    # Investigate why removing it fails.
    subprocess.check_call(' '.join(cmd), shell=True)
    autogen_util.FixClang2PyOutput(py_file.name, FLAGS.output)

  # This final sed command is a horrific hack around xml2py's hard-coding of .so
  # paths. It assumes that a .so will always be at the same path at which it was
  # when the Python wrapper was generated. That makes it impractical to use
  # relative paths, whereas we really ought to be referring to any .so by its
  # symlink in the runfiles directory of a Python executable.
  #
  # To work around this, we prune --shared_lib_root from the start of paths and
  # insert `makani.HOME` into the start of any CDLL calls. This works in
  # conjunction with the value of MAKANI_HOME used by a Python executable (it
  # points to the runfiles directory) to appropriately resolve the .so.
  #
  # On a related note, it would be really nice to replace h2py with SWIG.
  if shared_libs:
    cmd = ('sed -i -e'.split()
           + ['1s/^/import os\\nimport makani\\n/',
              '-e', 's|%s/||g' % FLAGS.shared_lib_root,
              '-e', r's/CDLL(\(.*\)/CDLL(os.path.join(makani.HOME, \1)/',
              FLAGS.output])
    subprocess.check_call(cmd)

  # Define helpful variables.
  found_directory = False
  with open(FLAGS.output, 'a') as f:
    for include_dir in FLAGS.include_dir:
      relpath = os.path.relpath(FLAGS.header, start=include_dir)
      if not relpath.startswith('..'):
        f.write('H2PY_HEADER_FILE = \'{}\'\n'.format(relpath))
        found_directory = True
        break
  assert found_directory


if __name__ == '__main__':
  gflags.MarkFlagAsRequired('header')
  gflags.MarkFlagAsRequired('output')
  gflags.MarkFlagAsRequired('include_dir')
  main(sys.argv)

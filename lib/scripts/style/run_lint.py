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

"""Runs lint checks."""

import itertools
import os
import re
import subprocess
import sys

import gflags
import makani
from makani.lib.bazel import bazel_util
from makani.lib.python import shell_interfaces

gflags.DEFINE_string('git_base', None,
                     'If set, only files modified since this commit will be '
                     'linted.')

gflags.DEFINE_bool('run_buildifier', True,
                   'Indicates whether buildifier should be run.')

gflags.DEFINE_bool('run_splint', False,
                   'Indicates whether splint should be run. Requires building '
                   'auto-generated files.')

gflags.DEFINE_bool('run_cpplint', True,
                   'Indicates whether cpplint should be run.')

gflags.DEFINE_bool('run_gjslint', True,
                   'Indicates whether gjslint should be run.')

gflags.DEFINE_bool('run_gpylint', True,
                   'Indicates whether gpylint should be run.')

gflags.DEFINE_bool('run_pack2_lint', True,
                   'Indicates whether pack2_lint should be run.')

gflags.DEFINE_bool('run_style_checker', True,
                   'Indicates whether the style checker should be run.')

gflags.DEFINE_bool('verbose', True,
                   'Display the "Done processing" and "Checking style in" '
                   'lines from check_google_style.sh and cpplint.py.')


FLAGS = gflags.FLAGS

# Include any files that shouldn't be linted at all here.
WHITELIST = [
    'lib/datatools/mat2csim.py',
    'lib/joystick/joystick.py',
    'lib/python/pack2/tests/test_param_pre_fmt.p2',
    'lib/scripts/style/cpplint.py',
    'sim/models/sensors/wind_sensor_test.cc',
]

# If the indentation checking is clearly wrong, but you still want to use
# cpplint.py, you can whitelist the file here.
INDENTATION_WHITELIST = ['avionics/motor/firmware/motor_thermal.c']

SPLINT_FILES = [
    'control/control_params.c',
    'control/system_params.c',
    'sim/sim_params.c',
]

_WORKSPACE_ROOT = bazel_util.GetWorkspaceRoot()


def BazelPathToLocalPath(file_name):
  assert re.match('//[^/:][^:]*:[^/:]+$', file_name), (
      'File %s does not match bazel path format' % file_name)
  file_name = file_name[2:].replace(':', '/')
  return file_name


def ReturnsZero(arg_list, **kwargs):
  """Runs a command (as an arg list) and returns True if it returns 0."""
  return subprocess.call(arg_list, **kwargs) == 0


def GatherFiles():
  """Gathers files to be linted.

  Returns:
    A dict of list of files with the following keys:
      build_files: List of BUILD files.
      c_files: List of C files (and C++ test files).
      cxx_files: List of C++ files (excluding tests).
      js_files: List of JavaScript files.
      p2_files: List of Pack2 files.
      py_files: List of Python files, excluding configs.
      py_configs: List of Python config files.
  """
  # Construct file lists. If --git_base is specified, then we only consider
  # files that have changed since the common ancestor of HEAD and --git_base.
  if FLAGS.git_base:
    common_ancestor = shell_interfaces.GetStdout(
        'git merge-base %s HEAD' % FLAGS.git_base).strip()
    filenames = shell_interfaces.GetStdout(
        'git diff %s --name-only' % common_ancestor).split()
  else:
    filenames = shell_interfaces.GetStdout(
        'git ls-tree -r --name-only --full-tree HEAD').split()

  # These are used to classify header files as C or C++ later on.
  c_and_cxx_sources = bazel_util.Query(
      r'filter("^//(?!external).*\.cc?$", kind("generated file", deps(...))'
      '+ kind("source file", deps(...)))')
  c_and_cxx_sources = {s.replace(':', '/').lstrip('/')
                       for s in c_and_cxx_sources}

  build_files = []
  c_files = []
  cxx_files = []
  js_files = []
  p2_files = []
  py_files = []
  py_configs = []

  all_headers = []

  for f in filenames:
    if f in WHITELIST or not os.path.exists(f) or os.path.islink(f):
      continue

    # C/C++ files.
    #
    # A .h file is classified as C++ if there is a corresponding .cc
    # file. Otherwise, we classify it as a C file, for which our cpplint check
    # is slightly more permissive.
    #
    # A _test.cc file is classified as a C file because most are written in a .c
    # style.
    if f.endswith('.h'):
      all_headers += [f]

      # Classify header files as C or C++, minimizing use of "file".
      if f[:-2] + '.c' in c_and_cxx_sources:
        c_files += [f]
      elif (f[:-2] + '.cc' in c_and_cxx_sources
            or 'C++ source' in shell_interfaces.GetStdout(['file', '-b', f])):
        cxx_files += [f]
      else:
        c_files += [f]

    elif f.endswith('.c'):
      c_files += [f]

    elif f.endswith('.cc'):
      if f.endswith('_test.cc'):
        c_files += [f]
      else:
        cxx_files += [f]

    # JavaScript files.
    elif f.endswith('.js'):
      js_files += [f]

    # Pack2 files.
    elif f.endswith('.p2'):
      p2_files += [f]

    # BUILD files.
    elif f.endswith('BUILD') or f.endswith('.bzl') or f == 'WORKSPACE':
      build_files += [os.path.join(_WORKSPACE_ROOT, f)]

    # Python files.
    #
    # Some lint checks are disabled for the configuration files (see below).
    elif f.endswith('.py'):
      # As of pylint 1.2, zero-length __init__.py files must be explicitly
      # excluded.
      if os.path.basename(f) == '__init__.py' and not os.stat(f).st_size:
        continue

      if (f.startswith('config/common')
          or f.startswith('config/m600')):
        py_configs += [f]
      else:
        py_files += [f]

    # Detect extensionless Python files. Note that "file" may mistakenly
    # classify some files as Python.
    elif not (f.endswith('.bzl') or f.endswith('.md')):
      file_type = shell_interfaces.GetStdout(['file', '-b', f])
      if 'Python' in file_type:
        py_files += [f]

  return {'build_files': build_files,
          'c_files': c_files,
          'cxx_files': cxx_files,
          'js_files': js_files,
          'p2_files': p2_files,
          'py_files': py_files,
          'py_configs': py_configs}


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  files = GatherFiles()

  # Track cumulative success to determine our return code.
  failures = []

  # Run buildifier.
  if FLAGS.run_buildifier:
    if FLAGS.git_base:
      if files['build_files'] and not ReturnsZero(
          ['bazel', 'run', '@com_github_bazelbuild_buildtools//buildifier',
           '--crosstool_top=@bazel_tools//tools/cpp:default-toolchain',
           '--color', 'no', '--noshow_progress', '--', '-v', '-warnings=all',
           '-mode=check', '-lint=warn']
          + files['build_files']):
        failures += ['buildifier']
    else:
      if not ReturnsZero(
          ['bazel', 'run', '@com_github_bazelbuild_buildtools//buildifier',
           '--crosstool_top=@bazel_tools//tools/cpp:default-toolchain',
           '--color', 'no', '--noshow_progress', '--', '-v', '-warnings=all',
           '-mode=check', '-lint=warn', '-r', _WORKSPACE_ROOT]):
        failures += ['buildifier']

  # Run cpplint.
  if FLAGS.run_cpplint:
    cpplint = os.path.join(makani.HOME, 'lib/scripts/style/cpplint')
    verbose = 2 if FLAGS.verbose else 1
    if files['c_files'] and not ReturnsZero(
        [cpplint, '--extensions=c,cc,h',
         '--filter=-legal/copyright,-readability/casting',
         '--verbose=%d' % verbose] + files['c_files']):
      failures += ['cpplint (C, plus C++ tests)']

    if files['cxx_files'] and not ReturnsZero(
        [cpplint, '--filter=-legal/copyright',
         '--verbose=%d' % verbose] + files['cxx_files']):
      failures += ['cpplint (C++, except tests)']

  # Run gjslint.
  # TODO(b/141990746): Remove or replace gjslint.
  if FLAGS.run_gjslint:
    # gjslint only exists on corp machines, so we skip this check if
    # the gjslint command is not found.
    gjslint_exists = False
    try:
      with open('/dev/null', 'w') as dev_null:
        gjslint_exists = ReturnsZero(['gjslint', '/dev/null'], stdout=dev_null,
                                     stderr=dev_null)
    except OSError:
      # This exception occurs if gjslint does not exist.
      pass

    if gjslint_exists:
      if files['js_files'] and not ReturnsZero(['gjslint'] + files['js_files']):
        failures += ['gjslint']
    else:
      print "Can't run gjslint -- it might not be installed."

  # Run gpylint.
  if FLAGS.run_gpylint:
    # gpylint only exists on corp machines, so we skip this check if
    # the gpylint command is not found.
    gpylint_exists = False
    try:
      with open('/dev/null', 'w') as dev_null:
        gpylint_exists = ReturnsZero(['gpylint', '--help'], stdout=dev_null,
                                     stderr=dev_null)
    except OSError:
      # This exception occurs if gpylint does not exist.
      pass

    if gpylint_exists:
      # Disable the g-unknown-interpreter warning, which complains about the
      # legitimate -u option.
      # Disable the unbalanced-tuple-packing warning, produces false positives.
      if files['py_files'] and not ReturnsZero(
          ['gpylint',
           '--disable=g-unknown-interpreter, unbalanced-tuple-unpacking']
          + files['py_files']):
        failures += ['gpylint (excluding config files)']
      # We disable the following warnings for gpylint when it is analyzing
      # configuration files because many of our legacy math-like variable
      # names don't comply with Google's style, the configuration file
      # description should go in the module docstring rather than the
      # function docstring, and gpylint has trouble determining the proper
      # order of importing with numpy:
      #
      #   C0103: Invalid variable name.
      #   C0111: Missing function doc-string.
      #   C6203: Invalid import order.
      if files['py_configs'] and not ReturnsZero(['gpylint',
                                                  '--disable=C0103,C0111,C6203']
                                                 + files['py_configs']):
        failures += ['gpylint (config files)']
    else:
      print "Can't run gpylint -- it might not be installed."

  if FLAGS.run_style_checker:
    # Run check_google_style.sh to check for indentation problems.
    indentation_files = [f for f in itertools.chain(files['c_files'],
                                                    files['cxx_files'])
                         if f not in INDENTATION_WHITELIST]
    if indentation_files:
      # TODO(b/141990064): Convert check_google_style.sh to bazel rule.
      check_style = os.path.join(_WORKSPACE_ROOT,
                                 'lib/scripts/style/check_google_style.sh')
      verbose = '--verbose' if FLAGS.verbose else '--noverbose'
      if not ReturnsZero([check_style, verbose] + indentation_files):
        failures += ['check_google_style.sh']

  # Run splint, if requested.
  if FLAGS.run_splint:
    bazel_output = bazel_util.Query(
        'filter("^//(?!external).*\\.h$", kind("generated file", deps(...)))')

    generated_headers = [BazelPathToLocalPath(p) for p in bazel_output]
    splint_build_files = SPLINT_FILES + generated_headers
    splint_ok = ReturnsZero(['bazel', 'build', '--color', 'no',
                             '--noshow_progress'] + splint_build_files)
    for f in SPLINT_FILES:
      # The only splint feature we particularly care about is detection of
      # fields that are unset by a designated initializer.
      #
      # NOTE: For a yet-unexplained reason, Ubiquity instances do not
      # warn on unused static variables or functions even without -varuse and
      # -fcnuse.
      splint_ok &= ReturnsZero(['splint', '-I', _WORKSPACE_ROOT, '-I',
                                os.path.join(_WORKSPACE_ROOT, 'bazel-genfiles'),
                                '-weak', '-unrecog', '-warnsysfiles', '-varuse',
                                '-fcnuse', os.path.join('bazel-genfiles', f)])

    if not splint_ok:
      failures += ['splint']

  # TODO(b/141991493): Convert pack2_lint.sh to bazel rule.
  if FLAGS.run_pack2_lint:
    if files['p2_files']:
      # Ensure that p2fmt is built.
      p2fmt_target = '//lib/python/pack2/tools:p2fmt'
      shell_interfaces.GetStdout(
          'bazel build --color no --noshow_progress ' + p2fmt_target)
      pack2_lint = os.path.join(_WORKSPACE_ROOT,
                                'lib/scripts/style/pack2_lint.sh')
      verbose = '--verbose' if FLAGS.verbose else '--noverbose'
      if not ReturnsZero([pack2_lint, verbose] + files['p2_files']):
        failures += ['pack2_lint.sh']

  if failures:
    print 'The following lint steps encountered errors:'
    for f in failures:
      print '  ' + f
    sys.exit(1)


if __name__ == '__main__':
  os.chdir(_WORKSPACE_ROOT)
  main(sys.argv)

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

"""Produces a coverage report using Bazel-built tests."""
# TODO: When used with gcc >= 4.7, lcov 1.10 produces errors
#     geninfo: Argument "=====" isn't numeric in numeric gt (>) at
#     /usr/bin/geninfo line 1281.
# Consider installing lcov 1.11 to eliminate them.

import os
import shutil
import subprocess
import sys

import gflags
import makani
from makani.lib.python import shell_interfaces

gflags.DEFINE_string('output_dir', None,
                     'Output directory for the coverage report.')
gflags.MarkFlagAsRequired('output_dir')

gflags.DEFINE_bool('run_tests', True,
                   'Whether to run unit tests.')
gflags.DEFINE_list('test_targets', None,
                   'If specified, only test targets indicated here (as a '
                   'comma-separated list) are built and run. Otherwise, all '
                   'C/C++ unit tests are used.')

gflags.DEFINE_bool('run_sim', False,
                   'Whether to run a simulation.')
gflags.DEFINE_float('sim_time', 500.0,
                    'If --run_sim, the simulation will run for this many '
                    '(simulated) seconds.')

FLAGS = gflags.FLAGS


def GetLcovDirectories():
  """Gets directories to use with lcov's --directory flag.

  When running lcov, we need to provide a --directory flag for each root of a
  source tree mirror that contains .gcda files. In Bazel, these directory
  paths look like
        bazel-bin/<source_tree_path>/_objs/<target_name>.
  But we can't use all directories matching this pattern because lcov dies if
  we provide one that doesn't contain data files.

  Returns:
    Sorted list of directories.
  """
  dirs = set()
  for dirname, _, files in os.walk(os.path.join(makani.HOME, 'bazel-bin')):
    for f in files:
      if f.endswith('.gcda'):
        dir_parts = dirname.split(os.sep)
        for i, p in enumerate(dir_parts):
          if p == '_objs':
            dirs.add(os.sep.join(dir_parts[:i+2]))

  return sorted(dirs)


def GetLcovDirectoryFlags():
  flags = []
  for d in GetLcovDirectories():
    flags += ['--directory', d]
  return flags


def RemoveBuildArtifacts():
  """Manually remove any existing artifacts of the lcov build.

  For some reason, when working with an unclean build, lcov will crash with an
  error of the form
      geninfo: ERROR: <some .gcno file>: reached unexpected end of file

  Running "bazel clean" would unnecessarily remove all other build artifacts.
  """
  bin_dir = shell_interfaces.GetStdout(
      'bazel info --config lcov bazel-bin').strip()
  if os.path.exists(bin_dir):
    shutil.rmtree(os.path.dirname(bin_dir))


def RunUnitTests():
  """Builds and runs unit tests to generate coverage data."""
  if FLAGS.test_targets:
    tests = FLAGS.test_targets
  else:
    tests = shell_interfaces.GetStdout(
        'bazel query kind("cc_test", ...)').split()

  # Run coverage, joining all data into one file.
  subprocess.check_call(['bazel', 'coverage', '--instrument_test_targets',
                         '--experimental_cc_coverage',
                         '--combined_report=lcov',
                         ('--coverage_report_generator=@bazel_tools//tools/tes'
                          't/CoverageOutputGenerator/java/com/google/devtools/'
                          'coverageoutputgenerator:Main')] + tests)


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  if not os.path.exists(FLAGS.output_dir):
    os.makedirs(FLAGS.output_dir)
  elif not os.path.isdir(FLAGS.output_dir):
    print ('Error: --output_dir (%s) exists and is not a directory.'
           % FLAGS.output_dir)

  RemoveBuildArtifacts()

  coverage_output = []
  if FLAGS.run_tests:
    RunUnitTests()
    coverage_output.append('bazel-out/_coverage/_coverage_report.dat')

  if FLAGS.run_sim:
    subprocess.check_call([
        'bazel', 'run', '--config', 'lcov', '//sim:run_sim', '--',
        '--comms_timeout_sec=10.0', '--time', str(FLAGS.sim_time)])

    coverage_file = os.path.join(FLAGS.output_dir, 'coverage.info')

    # The first lcov pass generates the coverage info file.
    subprocess.check_call(['lcov', '--base-directory', makani.HOME,
                           '--capture', '--output-file', coverage_file]
                          + GetLcovDirectoryFlags())

    # The second lcov pass removes data for files we don't care about.
    subprocess.check_call(['lcov', '--output-file', coverage_file,
                           '--remove', coverage_file, '/usr/include/*',
                           '--remove', coverage_file, 'gtest*',
                           '--remove', coverage_file, '*_test.cc'])

    coverage_output.append(coverage_file)

  # Generate the HTML report.
  subprocess.check_call(['genhtml', '--output-directory', FLAGS.output_dir]
                        + coverage_output)


if __name__ == '__main__':
  main(sys.argv)

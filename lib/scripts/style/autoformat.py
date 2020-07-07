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

"""Automatically lints C/C++, JS and BUILD files."""

import os
import subprocess
import sys

import gflags
from makani.lib.bazel import bazel_util
from makani.lib.python import os_util

gflags.DEFINE_bool('check_if_file_supported', False,
                   'Check whether a file is supported. Interprets the first '
                   'positional arg as a file path, and returns 0 if it is '
                   'supported, 1 otherwise.')

# Restricting the set of files considered keeps this script quick. We consider
# files modified since HEAD~ so that it can be used both to autoformat a set
# of uncommitted changes and to generate a fixup for the current HEAD.
gflags.DEFINE_bool('format_all', False,
                   'Whether to autoformat all supported files. Otherwise, '
                   'only files modified since HEAD~ will be autoformatted. '
                   'Cannot be used together with --git_base.')

gflags.DEFINE_string('git_base', None,
                     'If set, only files modified since this commit will be '
                     'autoformatted. Cannot be used together with '
                     '--format_all.')

gflags.DEFINE_bool('fixup', False,
                   'Whether to automatically amend HEAD.')

FLAGS = gflags.FLAGS

C_WHITELIST_DIRS = [
    'analysis/flight_data',
    'common',
    'control',
    'gs/aio_snapshot',
    'gs/flight_command',
    'gs/monitor',
    'gs/monitor2',
    'lib/pcap_to_hdf5',
    'sim',
    'vis',
]

JS_BLACKLIST_DIRS = [
    'analysis/log_analysis/webservice/apps/loganalyzer/static/js',
    'gs/monitor2/UI/static/scripts/visuals',
]

_WORKSPACE_ROOT = bazel_util.GetWorkspaceRoot()


def GetDeletedFiles():
  """Returns a set of all files deleted since HEAD~."""

  deleted_files = set()
  lines = subprocess.check_output(
      'git diff HEAD~ -r --name-status --diff-filter D'.split()).split('\n')
  for line in lines:
    if not line:
      continue
    parts = line.split()
    assert len(parts) == 2 and parts[0] == 'D'
    deleted_files.add(parts[1])

  return deleted_files


def IsSupportedCFile(filename):
  if not filename.endswith(('.c', '.cc', '.h')):
    return False

  for d in C_WHITELIST_DIRS:
    if not os.path.relpath(filename, start=d).startswith('..'):
      return True

  return False


def IsSupportedJsFile(filename):
  if not filename.endswith('.js'):
    return False

  for d in JS_BLACKLIST_DIRS:
    if not os.path.relpath(filename, start=d).startswith('..'):
      return False

  return True


def IsSupportedBuildFile(filename):
  return filename.endswith(('BUILD', '.bzl', 'WORKSPACE'))


def IsSupportedFile(filename):
  return (IsSupportedCFile(filename) or IsSupportedJsFile(filename) or
          IsSupportedBuildFile(filename))


# TODO(b/142070561): Refactor so that it uses the same GatherFiles from
# run_lint.py.
def GatherFiles():
  """Returns a list of files to autoformat."""
  if FLAGS.format_all:
    filenames = subprocess.check_output(
        'git ls-tree -r --name-only --full-tree HEAD'.split()).split('\n')
  elif FLAGS.git_base:
    cmd = 'git diff {} --name-only'.format(FLAGS.git_base)
    filenames = subprocess.check_output(cmd.split()).split('\n')
  else:
    filenames = subprocess.check_output(
        'git diff HEAD~ --name-only'.split()).split('\n')

  deleted_files = GetDeletedFiles()

  files = {
      'c': [],
      'js': [],
      'build': []
  }
  for f in filenames:
    if f not in deleted_files:
      if IsSupportedCFile(f):
        files['c'].append(os.path.join(_WORKSPACE_ROOT, f))
      elif IsSupportedJsFile(f):
        files['js'].append(os.path.join(_WORKSPACE_ROOT, f))
      elif IsSupportedBuildFile(f):
        files['build'].append(os.path.join(_WORKSPACE_ROOT, f))

  return files


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  if FLAGS.format_all and FLAGS.git_base:
    print '--format_all and --git_base cannot be used at the same time.'
    sys.exit(1)

  if FLAGS.fixup:
    if subprocess.check_output('git diff HEAD --name-only'.split()).strip():
      raise RuntimeError('There are uncommitted changes; --fixup is not '
                         'allowed')

  if FLAGS.check_if_file_supported:
    if IsSupportedFile(argv[1]):
      sys.exit(0)
    else:
      sys.exit(1)

  files = GatherFiles()

  for filename in files['c'] + files['js']:
    subprocess.check_call(
        ['clang-format-3.8.1', '-style=google', '-i', filename])

  if files['build']:
    subprocess.check_call([
        'bazel', 'run', '@com_github_bazelbuild_buildtools//buildifier',
        '--crosstool_top=@bazel_tools//tools/cpp:default-toolchain', '--',
        '--lint=fix', '--warnings=all'] + files['build'])

  if FLAGS.fixup:
    subprocess.check_call('git commit -a --amend --no-edit'.split())


if __name__ == '__main__':
  with os_util.ChangeDir(_WORKSPACE_ROOT):
    main(sys.argv)

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

r"""A module for finding the differences between two HDF5 log files.

Compares two log files (referred to as A and B) for differences.
Provides functions for generating a human-readable summary.  Log files
are represented as a dict whose values are either numpy.ndarray
objects or dict objects that are similarly structured.  A "path" will
refer to a sequence of keys indexing into a log file.

Can be executed at the command line to compare two files (use the
option --help for usage information).

E.g.:
  analysis/util/h5comp -t a.h5 b.h5 \
    --root=/messages/kAioNodeControllerA/kMessgaeTypeControllerCommand \
    --record=message
"""

import json
import sys

import gflags
import h5py

from makani.lib.python import algorithms
from makani.lib.python.h5_utils import h5_compare

FLAGS = gflags.FLAGS

gflags.DEFINE_integer('depth', -1, 'Limit depth for printing differences.',
                      short_name='d')
# The parser for the --help flag ignores spaces at the start of a new line,
# so we're stuck using tabs.
gflags.DEFINE_string(
    'config', '',
    'Path to a JSON configuration file. The file contains up to two nested '
    'dicts:\n'
    '\t"exclude": Has a true value at any entry that should be skipped in '
    'the comparison.\n'
    '\t"require": Has a true value at any entry that must be present in '
    'both log files.', short_name='c')
gflags.DEFINE_bool('quiet', False, 'Suppress non-error output.',
                   short_name='q')
gflags.DEFINE_bool('truncate', False,
                   'When comparing arrays with differing numbers of rows, '
                   'only compare up to the length of the shorter array.',
                   short_name='t')


def _GetMissingFields(log_file, required):
  """Returns any required fields missing from the log file."""
  missing = []

  for key_sequence, value in algorithms.TraverseNestedDict(required):
    if not value: continue

    try:
      ref = log_file
      for i, key in enumerate(key_sequence):
        ref = ref[key]
    # A group raises a KeyError, while a dataset raises a ValueError.
    except (KeyError, ValueError):
      missing.append('.'.join(key_sequence[:i]))

  return missing


def _CheckRequiredFields(data, required):
  """Fails the script if any required fields are missing."""
  missing_a = _GetMissingFields(data[0], required)
  missing_b = _GetMissingFields(data[1], required)
  if missing_a or missing_b:
    msg_parts = []
    if missing_a:
      msg_parts.append('Required fields missing from A:')
      msg_parts += ['  ' + s for s in missing_a]
    if missing_b:
      msg_parts.append('Required fields missing from B:')
      msg_parts += ['  ' + s for s in missing_b]
    raise ValueError('\n'.join(msg_parts))


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  if FLAGS.config:
    with open(FLAGS.config, 'r') as f:
      config = json.load(f)
      excludes = config.get('exclude', {})
      required = config.get('require', {})
  else:
    excludes = {}
    required = {}

  max_depth = None if FLAGS.depth < 0 else FLAGS.depth

  # Handle positional arguments.
  if len(argv) < 3:
    print 'Need at least two arguments.'
    sys.exit(-1)
  file_a = argv[1]
  file_b = argv[2]

  # Load data (may exit with IOError).
  data = [h5py.File(f, 'r') for f in [file_a, file_b]]

  _CheckRequiredFields(data, required)

  report = h5_compare.CompareLogs(data[0], data[1], FLAGS.truncate, excludes)
  if not FLAGS.quiet:
    report_string = h5_compare.GenerateReportString(report, '', max_depth)
    if report_string:
      print report_string


if __name__ == '__main__':
  main(sys.argv)

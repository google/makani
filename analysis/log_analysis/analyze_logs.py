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

"""Commandline to autocheck log data."""

import json
import sys

import gflags
from makani.analysis.checks import autocheck
from makani.analysis.log_analysis import check_util
from makani.lib.python import json_util
from makani.lib.python import struct_tree
from makani.lib.python.batch_sim import gcloud_util

gflags.DEFINE_string('directory', None, 'Directory of logs.', short_name='d')
gflags.DEFINE_string('gradebook', None, 'The JSON file of the gradebook',
                     short_name='g')
gflags.DEFINE_string('cloud_path', None, 'Cloud path of logs.', short_name='c')
gflags.DEFINE_string('input_prefix', None, 'Prefixes used to select logs.',
                     short_name='p')
gflags.DEFINE_boolean('verbose', False,
                      'True if the script should output detailed results.',
                      short_name='v')
gflags.DEFINE_string('path_to_checks', None,
                     'Path to the checks. '
                     'E.g, "makani.analysis.motor_checks.MotorChecks"',
                     short_name='l')
gflags.DEFINE_string('output_file', '', 'Output json file.',
                     short_name='o')
gflags.DEFINE_integer('min_gap', 1, 'Minimum gap between sections',
                      short_name='m')

FLAGS = gflags.FLAGS


def main(argv):

  gcloud_util.InitializeFlagsWithOAuth2(argv)

  try:
    argv = FLAGS(argv)  # Parse flags.
  except gflags.FlagsError, e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, argv, FLAGS)
    return

  if FLAGS.cloud_path and FLAGS.directory:
    print ('You can analyze logs from either a local directory or a cloud '
           'storage bucket, but not both.')
    sys.exit()

  if not (FLAGS.cloud_path or FLAGS.directory):
    print ('Please specify at least a local directory or a cloud '
           'storage bucket.')
    sys.exit()

  if FLAGS.path_to_checks:
    checks = check_util.LoadListOfChecks(FLAGS.path_to_checks)
  else:
    checks = None

  checks.SetMinGap(FLAGS.min_gap)

  if FLAGS.cloud_path:
    results = autocheck.RunFromCloud(
        FLAGS.cloud_path, FLAGS.input_prefix, checks, FLAGS.gradebook,
        FLAGS.verbose)
  else:
    results = autocheck.RunFromLocal(
        FLAGS.directory, FLAGS.input_prefix, checks, FLAGS.gradebook,
        FLAGS.verbose)

  aggregated_results = None
  if FLAGS.verbose:
    print 'Concatenate errors in all files --------------------------------'
    aggregated_results = struct_tree.StructTree(
        autocheck.GatherResultsFromMultipleFiles(results), True)
    print json.dumps(aggregated_results.Data(), indent=2,
                     cls=json_util.JsonNumpyEncoder, sort_keys=True)
  if FLAGS.output_file:
    if aggregated_results is None:
      aggregated_results = struct_tree.StructTree(
          autocheck.GatherResultsFromMultipleFiles(results), True)
    with open(FLAGS.output_file, 'w') as fp:
      json.dump(aggregated_results.Data(), fp, indent=2,
                cls=json_util.JsonNumpyEncoder, sort_keys=True)

  print 'Results by category ----------------------------------------'
  merged_results = struct_tree.StructTree(
      autocheck.MergeResultsFromMultipleFiles(
          results, ['warning', 'error', 'exception']), True)
  print json.dumps(merged_results.Data(), indent=2,
                   cls=json_util.JsonNumpyEncoder, sort_keys=True)


if __name__ == '__main__':
  main(sys.argv)

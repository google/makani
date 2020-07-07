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

"""Run the crosswind batch sim scoring functions on a log file."""

import sys

import gflags
import h5py
from makani.analysis.crosswind_batch_sims import batch_sim_params
from makani.lib.python.batch_sim import scoring_functions

gflags.DEFINE_boolean('benchmark', False, 'Time scoring function execution.')

gflags.DEFINE_boolean('control_telem_only', False,
                      'Use only controller telemetry for scoring functions.')

gflags.DEFINE_boolean('print_failure_count', False,
                      'Prints the results from GetFailureCount.')

gflags.DEFINE_boolean('print_index_of_first_failure', False,
                      'Prints the results from GetIndexOfFirstFailure.')

gflags.DEFINE_string('score_mask', '', 'Calculate only the scores which name '
                     'contains this string.')

gflags.DEFINE_boolean('output_as_csv_file', False,
                      'Flag to output to a CSV file.')

gflags.DEFINE_string('csv_filename', 'logfile_scores.csv',
                     'Output file name when output_as_csv_file is set.')

FLAGS = gflags.FLAGS


def main(argv):
  def PrintUsage(argv, error=None):
    if error:
      print '\nError: %s\n' % error
    print 'Usage: %s logfile.h5\n%s' % (argv[0], FLAGS)

  try:
    argv = FLAGS(argv)
    filename = argv[1]
  except gflags.FlagsError, e:
    PrintUsage(argv, e)
    sys.exit(1)
  except IndexError:
    PrintUsage(argv, 'Must specify filename of h5 log to process.')
    sys.exit(1)

  f = h5py.File(filename, 'r')
  flight_plan = int(f['parameters']['system_params']['flight_plan'][0])
  wing_model = int(f['parameters']['system_params']['wing_model'][0])
  offshore = bool(f['parameters']['system_params']['offshore'][0])

  params = batch_sim_params.CrosswindSweepsParameters(
      offshore=offshore, flight_plan=flight_plan,
      wing_model=wing_model)

  if FLAGS.score_mask:
    selected_scores = [sc for sc in params.scoring_functions if
                       FLAGS.score_mask.lower() in sc.name.lower()]
  else:
    selected_scores = params.scoring_functions

  scores, score_times = scoring_functions.ScoreLogFile(
      selected_scores, f, FLAGS.control_telem_only)

  if FLAGS.output_as_csv_file:
    scoring_functions.SaveScoresToCsv(scores, FLAGS.csv_filename)

  else:
    print scoring_functions.FormatScores(
        scores, print_failure_count=FLAGS.print_failure_count,
        print_index_of_first_failure=FLAGS.print_index_of_first_failure)

  if FLAGS.benchmark:
    print '\n\nScoring function execution times [s] (total time = %5.3f):' % (
        sum(score_times.values()))
    print scoring_functions.FormatScoreTimes(score_times)
  f.close()


if __name__ == '__main__':
  main(sys.argv)

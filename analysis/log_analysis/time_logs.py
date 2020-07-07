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

"""Count motor spin durations using logs in a local/cloud directory."""

import collections
import copy
import json
import logging
import os
import re
import string
import sys

from dateutil import parser as time_parser
import gflags
from makani.analysis.checks import iter_files
from makani.analysis.checks import log_util
from makani.avionics.motor.firmware import flags
from makani.avionics.network import aio_labels
from makani.lib.python import c_helpers
from makani.lib.python import json_util
from makani.lib.python import struct_tree
from makani.lib.python.batch_sim import gcloud_util
import numpy

gflags.DEFINE_string('directory', None, 'Directory of logs.', short_name='d')

gflags.DEFINE_string('cloud_path', None, 'Cloud path of logs.', short_name='c')

gflags.DEFINE_string('input_prefix', None,
                     'Prefixed regular expression used to select logs.',
                     short_name='p')

gflags.DEFINE_string('unit', 'min', 'Time unit, one of "sec", "min", "hour".',
                     short_name='u')

gflags.DEFINE_string('output', None, 'File to save results in JSON.',
                     short_name='o')

gflags.DEFINE_integer(
    'num_omega_bins', 12, 'Number of omega bins from 0 to 240 rad/s.',
    short_name='m')

gflags.DEFINE_integer(
    'num_wind_speed_bins', 6, 'Number of wind_speed bins from 0 to 25 m/s.',
    short_name='w')

gflags.DEFINE_string(
    'output_prefix', None,
    'Prefix to the files that save results for each log file.', short_name='f')

gflags.DEFINE_string(
    'selection', None,
    'Directory and time range to select files. '
    'E.g. my_dir/prefix:20151203-112233:20151211', short_name='s')

gflags.DEFINE_boolean('verbose', False, 'Verbose mode.', short_name='v')

gflags.DEFINE_boolean('group_by_tag', False, 'Group logs by their tags.',
                      short_name='g')

FLAGS = gflags.FLAGS

MOTOR_LABELS_HELPER = c_helpers.EnumHelper(
    'MotorLabel', aio_labels, prefix='kMotor')
MOTOR_STATUS_HELPER = c_helpers.EnumHelper(
    'MotorStatusFlag', flags, prefix='kMotorStatus')


def _IsForFlightTest(data):
  """Check whether the log is captured at the enterprise lot."""

  # TODO: Needs a better method once we have another ground station.
  path = 'messages.kAioNodeGpsBaseStation'
  return path in data


def _FlightCategory(data):
  """Check the flight mode in which the log is captured.

  We assume each log has a single flight category.

  Args:
    data: The message data in the form of a StructTree object.

  Returns:
    One of 'Hitl', 'MotorClient', 'Manual', 'Flight'.
  """

  message_template = string.Template(
      'messages.kAioNode$aio_node.kMessageType$message_type')
  path = message_template.substitute(
      aio_node='ControllerA',
      message_type='ControlTelemetry')
  if path not in data:
    return 'MotorClient'
  else:
    path = message_template.substitute(
        aio_node='Simulator',
        message_type='SimTelemetry')
    if path in data:
      return 'Hitl'
    elif _IsForFlightTest(data):
      return 'Flight'
    else:
      # TODO: We now assume manual mode only happens at the bird cage.
      return 'Manual'


def _GetTimestampFromString(name):
  """Get the timestamp contained within a string."""
  regex = re.compile(r'\d+-\d+')
  match = regex.search(name)
  if match:
    try:
      return time_parser.parse(match.group())
    except ValueError:
      # Parsing failed due to invalid timestamp string.
      return None
  else:
    return None


def _GetLogTag(filename):
  """Get the tag of a log."""
  regex = re.compile(r'\d+\-\d+(:?\-(?P<tag>.+))?.h5')
  match = regex.search(filename)
  if match and match.group('tag'):
    return match.group('tag')
  elif filename.endswith('.h5'):
    return filename[:-3]
  else:
    return ''


def _ParseSelectionArgs(selection_args):
  """Parse the string for the select option."""
  selection_args = selection_args.split(':')
  if len(selection_args) != 3:
    print ('Selection option should be formated as '
           '"<prefix>:<start_time>:<end_time>". E.g., '
           'my_dir/M600A:20151001-012345:20151201-235959')
    return None
  path, start_time, end_time = selection_args

  if os.path.isdir(path):
    dirname = path
    prefix = ''
  else:
    dirname = os.path.dirname(path)
    prefix = os.path.basename(path)
    if not os.path.isdir(dirname):
      print 'Directory "%s" does not exist.' % dirname
      return None

  try:
    start_time = time_parser.parse(start_time)
  except ValueError:
    print 'Invalid format for starting time: "%s".' % start_time
    return None

  try:
    end_time = time_parser.parse(end_time)
  except ValueError:
    print 'Invalid format for end time: "%s".' % end_time
    return None

  return dirname, prefix, start_time, end_time


def _GatherFromPast(dirname, prefix, start_time, end_time):
  """Load results from past analysis.

  Args:
    dirname: Path to the directory containing the logs.
    prefix: Prefix of the file.
    start_time: Starting time of the log, e.g., '20151110-162356'.
    end_time: End time of the log, e.g., '20151110-162356'.

  Returns:
    A dict of results indexed by filename.
  """
  assert os.path.isdir(dirname)
  results = {}
  for root, _, files in os.walk(dirname):
    for filename in files:
      # filename: <prefix>-20151201-112233.json
      if filename.endswith('.json') and (
          not prefix or filename.startswith(prefix)):
        timestamp = _GetTimestampFromString(filename)
        if timestamp and timestamp >= start_time and timestamp <= end_time:
          with open(os.path.join(root, filename), 'r') as fp:
            result = json.load(fp)
            # Convert lists into NumPy arrays.
            for motor_code in MOTOR_LABELS_HELPER.ShortNames():
              motor_name = 'Motor%s' % motor_code
              if motor_name in result:
                motor_result = result[motor_name]
                motor_result['histogram'] = numpy.array(
                    motor_result['histogram'])
                motor_result['omega_cdf'] = numpy.array(
                    motor_result['omega_cdf'])
                if 'wind_based_histogram' in motor_result:
                  wind_based_histogram = motor_result['wind_based_histogram']
                  for key in wind_based_histogram:
                    wind_based_histogram[key] = numpy.array(
                        wind_based_histogram[key])
            results[filename[:filename.rfind('.')]+'.h5'] = result
  return results


def _CountFlightTime(log_file, omega_range, num_omega_bins,
                     wind_speed_range, num_wind_speed_bins, unit):
  """Compute statistics about motor spins in logs.

  Duration is computed by counting the number of messages and divide it by
  the message frequency.

  Args:
    log_file: The log file to process.
    omega_range: The range of motor speeds: [lower bound, upper bound].
    num_omega_bins: The number of histogram bins for motor speeds.
    wind_speed_range: The range of wind speeds: [lower bound, upper bound].
    num_wind_speed_bins: The number of wind speed bins to characterize.
    unit: Unit of time, one of "sec", "min", and "hour".

  Returns:
    A Python dict in the form of: {
        'omega_config': {  # Bounds and bins for motor speed histograms.
            'min': min_omega, 'max': max_omega, 'num_bins': num_omega_bins},
        'wind_config': {  # Bounds and bins for wind speed breakdowns.
            'min': min_wind_speed,
            'max': max_wind_speed,
            'num_bins': num_wind_speed_bins},
        'bin_edges': Bin edges for the motor speed histogram,
        'category': The flight category,
        <motor_name>: {  # e.g., 'MotorSbo'
            'histogram': Time spent in each motor speed range,
            'omega_cdf': Time pent above certain motor speed,
            'wind_based_histogram': {
                wind speed [m/s]: Time spent in each motor speed range},
        }
    }
  """

  data = struct_tree.StructTree(log_file, True)
  message_template = string.Template(
      'messages.kAioNode$aio_node.kMessageType$message_type')
  message_name = 'MotorStatus'
  motor_names = MOTOR_LABELS_HELPER.ShortNames()
  control_telemetry_messages = ['ControlDebug', 'ControlTelemetry']
  control_telemetry_node = 'ControllerA'

  results = {
      'category': _FlightCategory(data),
      'omega_config': {
          'min': omega_range[0],
          'max': omega_range[1],
          'num_bins': num_omega_bins,
      },
      'wind_config': {
          'min': wind_speed_range[0],
          'max': wind_speed_range[1],
          'num_bins': num_wind_speed_bins
      },
  }

  if 'messages' not in data:
    logging.error('Unable to process log "%s"', log_file)
    return results

  wind_speed = None
  for control_telemetry_message in control_telemetry_messages:
    control_telemetry_path = message_template.substitute({
        'message_type': control_telemetry_message,
        'aio_node': control_telemetry_node,
    })
    wind_speed = data['%s.message.state_est.wind_g.speed_f' %
                      control_telemetry_path]
    if wind_speed is not None:
      wind_timestamp = log_util.LogTimestamp(data, control_telemetry_message,
                                             control_telemetry_node)
      break

  if wind_speed is not None:
    results['wind'] = {}
    results['wind']['avg_speed'] = numpy.average(wind_speed)
    results['wind']['max_speed'] = numpy.max(wind_speed)
    results['wind']['count'] = wind_speed.size
    wind_speed_bin_edges = numpy.linspace(
        wind_speed_range[0], wind_speed_range[1], num_wind_speed_bins)

  assert unit in ['sec', 'min', 'hour']
  if unit == 'min':
    frequency_scale = 60.0
  elif unit == 'hour':
    frequency_scale = 3600.0
  else:
    frequency_scale = 1.0

  parameters = {'message_type': message_name}
  for motor_code in motor_names:
    motor_name = 'Motor' + motor_code
    # Get the motor status and omega.
    parameters['aio_node'] = motor_name
    path = message_template.substitute(parameters)
    status = data['%s.message.%s' % (path, 'motor_status')]
    omega = data['%s.message.%s' % (path, 'omega')]
    timestamp = data['%s.capture_header.tv_sec' % path]
    if status is None:
      continue
    omega = numpy.abs(omega)

    # Select the subsequence to include for computation.
    clean_selection = log_util.MessageDedupSelection(
        data, 'MotorStatus', motor_name, message_template)

    clean_selection &= status == MOTOR_STATUS_HELPER.Value('Running')

    # Estimate timestep
    timestep = (float(timestamp[-1] - timestamp[0]) / timestamp.size /
                frequency_scale)
    # Compute the motor speed histogram in general.
    hist, _ = numpy.histogram(
        omega[clean_selection], bins=num_omega_bins,
        range=omega_range, density=False)
    # Convert message counts to duration.
    omega_hist = hist * float(timestep)
    results[motor_name] = {
        'histogram': omega_hist,
        'omega_cdf': numpy.cumsum(omega_hist[::-1])[::-1],
        'wind_based_histogram': {},
    }
    # Compute the wind-profile based motor speed histogram.
    if wind_speed is not None:
      motor_timestamp = log_util.LogTimestamp(data, message_name, motor_name)
      aligned_wind_speed = numpy.interp(motor_timestamp, wind_timestamp,
                                        wind_speed)
      for n in range(len(wind_speed_bin_edges) - 1):
        low_bar, high_bar = wind_speed_bin_edges[n:n+2]
        wind_selection = (clean_selection & (aligned_wind_speed >= low_bar) &
                          (aligned_wind_speed < high_bar))
        hist, _ = numpy.histogram(omega[wind_selection], bins=num_omega_bins,
                                  range=omega_range, density=False)
        results[motor_name]['wind_based_histogram'][
            (low_bar + high_bar) / 2.0] = hist * float(timestep)
  return results


def _GroupByCategory(results):
  """Group results by their `category` field."""
  categories = collections.defaultdict(list)
  for result in results:
    categories[result['category']].append(result)
  return categories


def _MergeResultsByCategory(results):
  """Merge results in the same category."""
  categories = _GroupByCategory(results)
  for category, result_list in categories.iteritems():
    merged_result = _MergeResults(result_list)
    if merged_result:
      categories[category] = merged_result
  merged_result = _MergeResults(categories.values())
  if merged_result:
    merged_result['category'] = 'Overall'
    categories['Overall'] = merged_result
  return categories


def _MergeResults(results):
  """Aggregate a list of results into one.

  Args:
    results: A lit of results, each is in the form of: {
        'omega_config': {  # Bounds and bins for motor speed histograms.
            'min': min_omega, 'max': max_omega, 'num_bins': num_omega_bins},
        'wind': {
            'avg_speed': average_wind_speed,
            'max_speed': max_wind_speed,
            'count': number of wind measurements,
        },
        'wind_config': {  # Bounds and bins for wind speed breakdowns.
            'min': min_wind_speed,
            'max': max_wind_speed,
            'num_bins': num_wind_speed_bins},
        'bin_edges': Bin edges for the motor speed histogram,
        'category': The flight category,
        <motor_name>: {  # e.g., 'MotorSbo'
            'histogram': Time spent in each motor speed range,
            'omega_cdf': Time spent above certain motor speed,
            'wind_based_histogram': {
                wind speed [m/s]: Time spent in each motor speed range},
        }
    }

  Returns:
    The merged results in the same form.
  """

  if not results:
    return

  # We only merge results with the same configuration as the first result.
  result = copy.deepcopy(results[0])
  wind_speed_bin_edges = numpy.linspace(
      result['wind_config']['min'],
      result['wind_config']['max'],
      result['wind_config']['num_bins'])
  wind_bin_centers = []
  for n in range(len(wind_speed_bin_edges) - 1):
    low_bar, high_bar = wind_speed_bin_edges[n:n+2]
    wind_bin_centers.append((low_bar + high_bar) / 2.0)

  for result_per_file in results[1:]:
    if result['omega_config'] != result_per_file['omega_config']:
      continue

    if 'wind' in result_per_file:
      if 'wind' not in result:
        result['wind'] = copy.deepcopy(result_per_file['wind'])
      else:
        if 'max_speed' in result_per_file['wind']:
          result['wind']['max_speed'] = max(
              result['wind']['max_speed'], result_per_file['wind']['max_speed'])
        if 'avg_speed' in result_per_file['wind']:
          count_per_file = result_per_file['wind']['count']
          result['wind']['avg_speed'] = (
              (result['wind']['avg_speed'] * result['wind']['count'] +
               result_per_file['wind']['avg_speed'] * count_per_file) /
              float(result['wind']['count'] + count_per_file))
          result['wind']['count'] += count_per_file

    for motor_code in MOTOR_LABELS_HELPER.ShortNames():
      motor_name = 'Motor' + motor_code
      if motor_name not in result_per_file:
        continue
      if motor_name not in result:
        result[motor_name] = result_per_file[motor_name]
        continue

      src = result_per_file[motor_name]
      dst = result[motor_name]
      # Add up durations.
      dst['histogram'] += src['histogram']
      dst['omega_cdf'] += src['omega_cdf']
      if result['wind_config'] == result_per_file['wind_config']:
        wind_based_histogram = dst['wind_based_histogram']
        for wind_key in wind_bin_centers:
          if wind_key in src['wind_based_histogram']:
            if wind_key not in wind_based_histogram:
              wind_based_histogram[wind_key] = (
                  src['wind_based_histogram'][wind_key])
            else:
              wind_based_histogram[wind_key] += (
                  src['wind_based_histogram'][wind_key])
  return result


def ShowTimingProfile(result, name, unit, verbose):
  """Show the timing profile from a single result."""

  if name:
    print '%s:' % name
  if 'omega_config' not in result:
    print 'Missing omega_config, skip file...'
    return
  omega_config = result['omega_config']
  bin_edges = numpy.linspace(omega_config['min'], omega_config['max'],
                             omega_config['num_bins'] + 1)
  if 'category' in result:
    print '  Mode: ' + result['category']
  if 'wind' in result:
    if 'avg_speed' in result['wind']:
      print '  Avg wind speed: %f' % result['wind']['avg_speed']
    if 'max_speed' in result['wind']:
      print '  Max wind speed: %f' % result['wind']['max_speed']

  for motor_code in MOTOR_LABELS_HELPER.ShortNames():
    motor_name = 'Motor' + motor_code
    if motor_name not in result:
      continue
    print '  [%s]:' % motor_name
    print '    Omega [rad/s]'
    print '' + ''.join(['%9.2f   ' % s for s in bin_edges] + ['  total'])
    titles = ['Duration [%s] with speed >= lower bar' % unit,
              'Duration per bin']
    for pos, key in enumerate(['omega_cdf', 'histogram']):
      stats = list(result[motor_name][key])
      if key != 'omega_cdf':
        stats += [sum(stats)]
      print '    %s:' % titles[pos]
      print '    ' + ''.join(' | %9.2f' % s for s in stats)
    wind_based_histograms = result[motor_name]['wind_based_histogram']
    if verbose:
      for key in sorted(wind_based_histograms.keys()):
        print '    Test duration when wind speed is around %s m/s' % key
        stats = list(wind_based_histograms[key])
        stats += [sum(stats)]
        print '    ' + ''.join(' | %9.2f' % s for s in stats)
    else:
      # Show a single motor.
      break


def _SetFileExt(name, ext):
  if '.' in name:
    return name[:name.rfind('.') + 1] + ext
  else:
    return name + '.' + ext


def RunFromLocal(log_dir, input_prefix, output_prefix,
                 omega_range, num_omega_bins,
                 wind_speed_range, num_wind_speed_bins, verbose, unit):
  """Run checks over local logs."""
  results_by_file = {}
  for full_name in iter_files.IterFromLocal(log_dir, input_prefix):
    filename = os.path.basename(full_name)
    if verbose:
      print 'Processing %s...' % filename
    results_by_file[filename] = _CountFlightTime(
        full_name, omega_range, num_omega_bins,
        wind_speed_range, num_wind_speed_bins, unit)
    if output_prefix:
      output_file = _SetFileExt(
          '%s%s' % (output_prefix, os.path.basename(filename)), 'json')
      with open(output_file, 'w') as fp:
        fp.write(json.dumps(results_by_file[filename], indent=2,
                            cls=json_util.JsonNumpyEncoder))
  return results_by_file


def RunFromCloud(path, input_prefix, output_prefix,
                 omega_range, num_omega_bins,
                 wind_speed_range, num_wind_speed_bins, unit):
  """Run checks over logs in the cloud."""
  results_by_file = {}
  for full_cloud_path, temp_name in iter_files.IterFilesFromCloud(
      path, input_prefix):

    results_by_file[full_cloud_path] = _CountFlightTime(
        temp_name, omega_range, num_omega_bins,
        wind_speed_range, num_wind_speed_bins, unit)

    if output_prefix:
      output_file = _SetFileExt(
          '%s%s' % (output_prefix, gcloud_util.GcsBasename(full_cloud_path)),
          'json')
      with open(output_file, 'w') as fp:
        fp.write(json.dumps(results_by_file[full_cloud_path], indent=2,
                            cls=json_util.JsonNumpyEncoder))

  return results_by_file


def main(argv):
  gcloud_util.InitializeFlagsWithOAuth2(argv)
  try:
    argv = FLAGS(argv)  # Parse flags.
  except gflags.FlagsError, e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, argv, FLAGS)
    examples = [
        'Time logs on local disk and save results to JSON:',
        '  <executable> -d logs/iron_bird -p 20150915 -o results/20150915.json',
        'Time logs in cloud, group the results by tags, and be verbose:',
        '  <executable> -c gs://m600_testing_logs/M600A -p 20150915 -g -v',
        'Aggregate time stats in the analyzed results of multiple logs:',
        '  <executable> -s <dir_name>:20151201-000000:20151201-235959',
    ]
    print '-----------------------------\nExamples:\n%s' % '\n'.join(examples)
    return

  omega_range = [0.0, 240.0]
  wind_speed_range = [0.0, 25.0]

  if not (FLAGS.cloud_path or FLAGS.directory or FLAGS.selection):
    print ('Please specify at least a local directory or a cloud '
           'storage path.')
    sys.exit()

  if sum([bool(FLAGS.cloud_path), bool(FLAGS.directory),
          bool(FLAGS.selection)]) > 1:
    print ('Please use only one of the "cloud_path (-c)", "directory (-d)", '
           ' and "select (-s)" options.')
    sys.exit()

  if FLAGS.cloud_path:
    results = RunFromCloud(
        FLAGS.cloud_path, FLAGS.input_prefix, FLAGS.output_prefix,
        omega_range, FLAGS.num_omega_bins,
        wind_speed_range, FLAGS.num_wind_speed_bins, FLAGS.unit)
  elif FLAGS.directory:
    results = RunFromLocal(
        FLAGS.directory, FLAGS.input_prefix, FLAGS.output_prefix,
        omega_range, FLAGS.num_omega_bins,
        wind_speed_range, FLAGS.num_wind_speed_bins, FLAGS.verbose, FLAGS.unit)
  else:  # selection
    selection_args = _ParseSelectionArgs(FLAGS.selection)
    if selection_args is None:
      print 'Abort: Unable to parse the "select" option.'
      sys.exit()
    else:
      results = _GatherFromPast(*selection_args)
      print 'Results are gathered from %d files.' % len(results)

  if FLAGS.verbose:
    # Print results per each file.
    filenames = sorted(results.keys())
    for filename in filenames:
      ShowTimingProfile(results[filename], filename, FLAGS.unit, True)

  if FLAGS.verbose:
    categories = ['Hitl', 'Manual', 'MotorClient', 'Flight', 'Overall']
  else:
    categories = ['Overall']

  # Print results grouped by tags.
  if FLAGS.group_by_tag:
    file_groups = collections.defaultdict(set)
    for filename in sorted(results.keys()):
      file_groups[_GetLogTag(filename)].add(filename)

    categorized_results = {}
    for tag, filenames in file_groups.iteritems():
      categorized_results[tag] = _MergeResultsByCategory(
          [results[f] for f in file_groups[tag]])
      print '\n\n************' + tag + '**********************************'
      for category in categories:
        if category in categorized_results[tag]:
          ShowTimingProfile(categorized_results[tag][category], category,
                            FLAGS.unit, FLAGS.verbose)
  else:
    # Print merged results per each category.
    categorized_results = _MergeResultsByCategory(results.values())
    for category in categories:
      if category in categorized_results:
        print '\n\n**********************************************'
        ShowTimingProfile(categorized_results[category], category,
                          FLAGS.unit, FLAGS.verbose)

  # Save merged results.
  if FLAGS.output:
    with open(FLAGS.output, 'w') as fp:
      fp.write(json.dumps(categorized_results, indent=2,
                          cls=json_util.JsonNumpyEncoder))


if __name__ == '__main__':
  main(sys.argv)

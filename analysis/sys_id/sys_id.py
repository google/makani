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

"""Produce servo transfer functions from ControlTelemetry in an h5 log file.

Example usage.  First, generate a logfile where the servos and/or
motors are commanded at a broad range of frequencies.  Second, process
the data using this program:

  ./analysis/sys_id/sys_id.py logs/last.h5

By default, output files will be created in the current directory.
Command line arguments are available to change this default.

"""

# TODO: Add plots of some time-domain data, such as the
# original timeseries, and histograms of commanded and measured
# values.

# TODO: Make it easy to display plots from multiple logs
# simultaneously (i.e. results from a simulator run along side a run
# on the real wing; or from two runs of the simulator, etc).

import os
import sys

import gflags
import h5py
from makani.analysis.sys_id import sys_id_util
from makani.lib.python import dict_util
import numpy as np
import pylab

gflags.DEFINE_string('output_dir', '',
                     'Directory into which output files will be written.')
gflags.DEFINE_boolean('save', True,
                      'Whether to save the plots into PNG and PDF files.')
gflags.DEFINE_boolean('interactive', False,
                      'Whether to display the plots interactively.')
gflags.DEFINE_integer('nfft', 1024,
                      'Number of points at which to compute the Fourier '
                      'transform.')
gflags.DEFINE_string('basename', '',
                     'Base filename for output files.  If this option is '
                     'empty, the base name of the input log file is used.')
gflags.DEFINE_float('min_coherence', 0.8,
                    'Minimum coherence required to include data point on '
                    'transfer function plot.')
gflags.DEFINE_enum('detrend', 'mean', ['none', 'mean', 'linear'],
                   'Detrending algorithm to be applied before taking the '
                   'Fourier transform.')
gflags.DEFINE_boolean('do_unwrap', True, 'Whether phase should be unwrapped.')
gflags.DEFINE_float('settling_time', 5.0,
                    'Duration [s] to skip at beginning of time series.')

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

  try:
    log = h5py.File(filename, 'r')
  except IOError, e:
    print 'Error: Could not open h5 log file "%s".\n' % filename
    raise e

  # Create the desired output directory if it doesn't already exist.
  output_dir = os.path.join(os.getcwd(), FLAGS.output_dir)
  if not os.path.isdir(output_dir):
    os.mkdir(output_dir)

  if FLAGS.basename:
    basename = FLAGS.basename
  else:
    # Name the output files after the input logfile.
    basename = os.path.basename(filename)
    # Remove the file extension (i.e. '.h5').
    basename = os.path.splitext(basename)[0]

  control_path = ['messages', 'kAioNodeControllerA',
                  'kMessageTypeControlTelemetry', 'message']

  time_path = control_path + ['time']
  flaps_cmd_path = control_path + ['control_output', 'flaps']
  flaps_val_path = control_path + ['control_input', 'flaps']
  flaps_labels = ['kFlapA1', 'kFlapA2', 'kFlapA4', 'kFlapA5', 'kFlapA7',
                  'kFlapA8', 'kFlapEle', 'kFlapRud']

  rotors_cmd_path = control_path + ['control_output', 'motor_speed_upper_limit']
  rotors_val_path = control_path + ['control_input', 'rotors']
  rotors_labels = ['kMotorSbo', 'kMotorSbi', 'kMotorPbi', 'kMotorPbo',
                   'kMotorPto', 'kMotorPti', 'kMotorSti', 'kMotorSto']

  # Check the sample rate.
  # TODO: Also check sequence numbers.
  t = dict_util.GetByPath(log, time_path)
  fs = np.round(len(t) / (t[-1] - t[0]))
  if fs != 100.0:
    print 'Warning: expected 100 Hz telemetry but found %d Hz.' % fs

  rotors_plot_filename = os.path.join(output_dir, basename + '_rotors')
  flaps_plot_filename = os.path.join(output_dir, basename + '_flaps')

  # Process the rotors data.
  print 'Processing rotor data...'
  rotors_cmd = dict_util.GetByPath(log, rotors_cmd_path)
  rotors_val = dict_util.GetByPath(log, rotors_val_path)
  sys_id_util.PlotTransferFunction(rotors_cmd, rotors_val, rotors_labels, fs,
                                   axes=None, settling_time=FLAGS.settling_time,
                                   nfft=FLAGS.nfft, detrend=FLAGS.detrend)

  if FLAGS.save:
    print 'Writing to %s.pdf.' % rotors_plot_filename
    pylab.savefig(rotors_plot_filename + '.pdf')
    print 'Writing to %s.png.' % rotors_plot_filename
    pylab.savefig(rotors_plot_filename + '.png')

  # Process the servo (flaps) data.
  print 'Processing flaps data...'
  flaps_cmd = dict_util.GetByPath(log, flaps_cmd_path)
  flaps_val = dict_util.GetByPath(log, flaps_val_path)
  sys_id_util.PlotTransferFunction(flaps_cmd, flaps_val, flaps_labels, fs,
                                   axes=None, settling_time=FLAGS.settling_time,
                                   nfft=FLAGS.nfft, detrend=FLAGS.detrend)

  if FLAGS.save:
    print 'Writing to %s.pdf.' % flaps_plot_filename
    pylab.savefig(flaps_plot_filename + '.pdf')
    print 'Writing to %s.png.' % flaps_plot_filename
    pylab.savefig(flaps_plot_filename + '.png')

  if FLAGS.save:
    with open(os.path.join(output_dir,
                           basename + '_sysid.html'), 'w') as report:
      print 'Writing report to %s.' % report.name
      report.write('<h1>Motors</h1><img src="%s"/>'
                   '<h1>Flaps</h1><img src="%s"/>'
                   % (rotors_plot_filename + '.png',
                      flaps_plot_filename + '.png'))


if __name__ == '__main__':
  main(sys.argv)

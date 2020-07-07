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

"""Write a low-pass-filtered Gaussian random timeseries to a text file."""

import sys

import gflags
import numpy as np
import scipy.signal

gflags.DEFINE_float('sample_rate', 100.0, 'Sample rate [Hz]')
gflags.DEFINE_integer('filter_order', 2, 'Order of low-pass filter')
gflags.DEFINE_float('mean', 130.0, 'Mean value of time series')
gflags.DEFINE_float('rms', 1.0, 'Standard deviation of time series')
gflags.DEFINE_float('ramp_time', 10.0, 'Ramp time [s]')
gflags.DEFINE_boolean('ramp_up', True, 'Apply ramp at beginning of waveform?')
gflags.DEFINE_boolean('ramp_down', True, 'Apply ramp at end of waveform?')
gflags.DEFINE_float('duration', 300.0, 'Time series duration [s]')
gflags.DEFINE_integer('columns', 8, 'Number of time series data columns')
gflags.DEFINE_float('cutoff_frequency', 2.0,
                    'Low-pass filter cutoff frequency [Hz]')
gflags.DEFINE_string('output_file', '', 'Output filename.')
gflags.DEFINE_boolean('use_motor_client_format', None,
                      'Produce data in a format suitable for the motor client.')

FLAGS = gflags.FLAGS


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  nyquist_frequency = FLAGS.sample_rate / 2.0
  b, a = scipy.signal.butter(FLAGS.filter_order,
                             FLAGS.cutoff_frequency / nyquist_frequency,
                             'low', analog=False)

  settling_time = 5.0 * (1.0 / FLAGS.cutoff_frequency)
  x = np.random.randn((FLAGS.duration + settling_time) * FLAGS.sample_rate)

  y = scipy.signal.lfilter(b, a, x)
  y = y[int(round(FLAGS.sample_rate * settling_time)):]
  y *= FLAGS.rms / np.std(y)
  y += FLAGS.mean
  t = np.arange(len(y)) / FLAGS.sample_rate

  # Apply ramp-up and/or ramp-down if desired.
  ramp_samples = int(FLAGS.ramp_time * FLAGS.sample_rate)
  if FLAGS.ramp_up:
    y[0:ramp_samples] = np.linspace(0.0, y[ramp_samples], ramp_samples)
  if FLAGS.ramp_down:
    y[-ramp_samples:] = np.linspace(y[-ramp_samples], 0.0, ramp_samples)

  filename = FLAGS.output_file
  if not filename:
    filename = 'blrms_%d_to_%d_mHz_rms_%.0f_mean_%.0f_order_%d.txt' % (
        0.0,
        FLAGS.cutoff_frequency * 1000,
        FLAGS.rms,
        FLAGS.mean,
        FLAGS.filter_order)

  print "Writing to '%s'..." % filename

  output = np.tile(y, (FLAGS.columns, 1))

  # Default "use_motor_client_format" to true if eight columns of data
  # were requested.
  if FLAGS.use_motor_client_format is None:
    FLAGS.use_motor_client_format = (FLAGS.columns == 8)

  # The motor client expects three blocks of eight columns each,
  # giving, respectively, the commanded torque [Nm], the lower motor
  # speed limit [rad/s], and the upper speed limit [rad/s].  To
  # achieve a speed command, we set the upper and lower speed limits
  # equal, and set the torque command (which does nothing) to zero.
  if FLAGS.use_motor_client_format:
    assert FLAGS.columns == 8
    output = np.vstack((np.zeros(output.shape), output, output))

  np.savetxt(filename, np.vstack((t, output)).T,
             fmt='%0.02f' + ' %f' * output.shape[0])

if __name__ == '__main__':
  main(sys.argv)

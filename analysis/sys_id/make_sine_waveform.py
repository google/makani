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

"""Write a sinusoidal waveform to a text file."""

import sys

import gflags
import numpy as np

gflags.DEFINE_float('sample_rate', 100.0, 'Sample rate [Hz]')
gflags.DEFINE_float('frequency', None, 'Frequency [Hz]')
gflags.DEFINE_float('mean', 0.0, 'Mean value of time series')
gflags.DEFINE_float('amplitude', 1.0, 'Amplitude of sine wave.')
gflags.DEFINE_float('duration', 0.0, 'Time series duration [s].  '
                    'By default one period of the waveform will be generated.')
gflags.DEFINE_integer('columns', 8, 'Number of time series data columns')
gflags.DEFINE_string('output_file', '', 'Output filename.')
gflags.MarkFlagAsRequired('frequency')

FLAGS = gflags.FLAGS


def main(argv):

  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  duration = FLAGS.duration
  if duration == 0.0:
    duration = 1.0 / FLAGS.frequency

  t = np.arange(0.0, duration, 1.0 / FLAGS.sample_rate)
  y = FLAGS.amplitude * np.sin(2.0 * np.pi * FLAGS.frequency * t)

  filename = FLAGS.output_file
  if not filename:
    filename = 'sine_%0.2f_Hz.txt' % FLAGS.frequency

  print "Writing to '%s'..." % filename
  np.savetxt(filename, np.vstack((t, y)).T, fmt='%0.02f %e', delimiter='\t')

if __name__ == '__main__':
  main(sys.argv)

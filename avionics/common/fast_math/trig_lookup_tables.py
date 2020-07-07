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

r"""Generates trigonometric lookup tables.

Sample usage:
  # This will generate the default tables.
  ./trig_lookup_tables.py

  # Alternatively, you can specify table properties.
  ./trig_lookup_tables.py \
    --num_atan_samples=400 \
    --num_sin_samples=513 \
    --output_files=trig_lookup_tables.h
"""

import os
import sys

import gflags
import makani
import numpy

gflags.DEFINE_integer('num_atan_samples', 400,
                      'Number of samples in the arctangent table between '
                      '0 and 1 inclusive.')
# The sine and cosine lookup function runs significantly faster when
# the number of samples is one more than a power of two.
gflags.DEFINE_integer('num_sin_samples', 513,
                      'Number of samples in the sine table between '
                      '0 and pi/2 inclusive.')
gflags.DEFINE_string('output_file',
                     os.path.join(makani.HOME,
                                  'avionics/common/fast_math/'
                                  'trig_lookup_tables.h'),
                     'File name of the header file to output.')

FLAGS = gflags.FLAGS


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  lines = [
      '#ifndef AVIONICS_TMS570_TRIG_LOOKUP_TABLES_H_',
      '#define AVIONICS_TMS570_TRIG_LOOKUP_TABLES_H_',
      '',
      'const float kAtanOctantIndexScale = %ff;' % (FLAGS.num_atan_samples - 1),
      '',
      'const float kAtanOctantTable[] = {',
      '\n'.join(['  %.10ef,' % numpy.math.atan(theta)
                 for theta in numpy.linspace(0.0, 1.0,
                                             FLAGS.num_atan_samples)]),
      '};',
      '',
      'const float kSinQuadrantIndexScale = %ff;' % ((FLAGS.num_sin_samples - 1)
                                                     / (numpy.pi / 2.0)),
      '',
      'const float kSinQuadrantTable[] = {',
      '\n'.join(['  %0.10ef,' % numpy.math.sin(theta)
                 for theta in numpy.linspace(0.0, numpy.pi / 2.0,
                                             FLAGS.num_sin_samples)]),
      '};',
      '',
      '#endif  // AVIONICS_TMS570_TRIG_LOOKUP_TABLES_H_',
      '',
  ]

  with open(FLAGS.output_file, 'w') as output_file:
    output_file.write('\n'.join(lines) + '\n')

if __name__ == '__main__':
  main(sys.argv)

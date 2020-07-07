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


"""Generates a dummy file with the wind database format."""

import sys

import gflags
import h5py
import numpy


gflags.DEFINE_string('output_file', None,
                     'Destination to write to.')

FLAGS = gflags.FLAGS


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  f = h5py.File(FLAGS.output_file, 'w')
  num_t = 10
  num_y = 4
  num_z = 2
  sz = num_t * num_y * num_z
  f.create_dataset('num_t', data=numpy.array([num_t], dtype='<i4'))
  f.create_dataset('num_y', data=numpy.array([num_y], dtype='<i4'))
  f.create_dataset('num_z', data=numpy.array([num_z], dtype='<i4'))
  f.create_dataset('mean_wind_speed', data=numpy.array([1.0]))
  f.create_dataset('duration', data=numpy.array([100.0]))
  f.create_dataset('width', data=numpy.array([15.0]))
  f.create_dataset('height', data=numpy.array([5.0]))
  f.create_dataset('u', data=numpy.array(range(0, sz), dtype='<f8'))
  f.create_dataset('v', data=sz + numpy.array(range(0, sz), dtype='<f8'))
  f.create_dataset('w', data=2*sz + numpy.array(range(0, sz), dtype='<f8'))
  f.close()


if __name__ == '__main__':
  main(sys.argv)

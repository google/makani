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

"""Script to merge pcap/pcap.gz files into one H5 file."""

import os
import subprocess
import sys

import gflags
import makani

gflags.DEFINE_string(
    'output', None, 'Name of the output HDF5 file')
gflags.MarkFlagAsRequired('output')

FLAGS = gflags.FLAGS


def MergeLogs(inputs, h5_output):
  """Post process the pcap and pcap.gz to generate a merged H5."""

  pcap_files = []
  pcap_gz_files = []
  for input_file in inputs:
    input_file = os.path.abspath(input_file)
    if input_file.endswith('.pcap.gz'):
      pcap_gz_files.append(input_file)
    elif input_file.endswith('.pcap'):
      pcap_files.append(input_file)
    else:
      raise ValueError('Cannot merge ' + input_file + '. Abort.')
  h5_output = os.path.abspath(h5_output)

  os.chdir(makani.HOME)

  if pcap_gz_files:
    print 'Unzip Pcap GZ files...'
    for gz_file in pcap_gz_files:
      try:
        subprocess.check_call(['gunzip', gz_file])
      except OSError:
        print 'Failed to process %s, aborting..' % gz_file
        return False
      pcap_files.append(gz_file[:-3])

  pcap_files.sort(key=os.path.basename)

  print 'Convert Pcap files to the merged HDF5 file...'
  pcap_to_hdf5 = '//lib/pcap_to_hdf5'
  subprocess.check_call(['bazel', 'build', pcap_to_hdf5])

  subprocess.check_call(
      ['bazel-bin/lib/pcap_to_hdf5/pcap_to_hdf5',
       '--output_file=' + h5_output] + pcap_files)
  print 'Converted to ' + h5_output

  return True


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError as e:
    print '%s\nUsage: %s ARGS input_1.h5 input_2.h5 ...\n%s' % (
        e, argv[0], FLAGS)
    sys.exit(1)

  MergeLogs(argv[1:], FLAGS.output)


if __name__ == '__main__':
  main(sys.argv)

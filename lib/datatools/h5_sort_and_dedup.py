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


r"""Sorts and dedpulicates selected messages in a log file.

Example:
  ./h5_sort_and_dedup.py \
    --input_file flight.h5 --output_file flight_dedup.h5 \
    --messages 'ControllerA.ControlDebug,FcA.FlightComputerImu
"""

import sys

import gflags
import h5py
import numpy as np

gflags.DEFINE_string('messages', 'ControllerA.ControlDebug',
                     'Comma-seperated list of node_name.message name, e.g. '
                     'ControllerA.ControlDebug.')
gflags.DEFINE_string('input_file', '', 'File to read.')
gflags.DEFINE_string('output_file', '', 'File to write.')

FLAGS = gflags.FLAGS


def _DedpulicateData(dataset, wrap=65536, window=300, wrap_window=1000):
  """Partially sorts and deduplicates a dataset.

  Locally sorts a dataset using a sliding window then removes duplicates.

  Args:
    dataset: Must contain a field aio_header with a subfield sequence.
    wrap: Sequence numbers increment modulo this value.
    window: Maximum number of samples a packet can be delayed.
    wrap_window: A sequence number in [wrap - wrap_window, wrap - 1] is
       considered to be less than one in [0, wrap_window - 1].

  Returns:
    A locally sorted/deduplicated dataset of the same dtype.
  """
  # Copy the dataset into memory as H5 datasets have stragne indexing
  # behavior.
  dataset = np.array(dataset, dtype=dataset.dtype)
  seq = np.array(dataset['aio_header']['sequence'], dtype='i8')

  ind = np.array(np.arange(seq.shape[0]))

  def _WrapLessThan(a, b):
    if a >= wrap - wrap_window and b < wrap_window:
      return True
    elif b >= wrap - wrap_window and a < wrap_window:
      return False
    else:
      return a < b

  # Find the smallest element in a sliding window of length 'window',
  # and swap it to the first position of the window.  'Smallest' here
  # is defined to include wrapping.
  print '  Sorting...'
  sorted_seq = np.array(seq)
  for start in range(seq.shape[0]):
    smallest = sorted_seq[start]
    smallest_ind = ind[start]
    smallest_i = start
    for i in range(start, np.min((start + window, seq.shape[0]))):
      if _WrapLessThan(sorted_seq[i], smallest):
        smallest = sorted_seq[i]
        smallest_ind = ind[i]
        smallest_i = i
    ind[smallest_i] = ind[start]
    sorted_seq[smallest_i] = sorted_seq[start]
    ind[start] = smallest_ind
    sorted_seq[start] = smallest
    if start % 10000 == 0:
      print '  %d / %d' % (start, seq.shape[0])

  # Sweep through sorted elements and skip duplicates.
  # If sequence numbers don't match, record index and advance.
  print '  Deduplicating...'
  dedup_ind = np.array(ind)
  idx = 1
  for i in range(1, ind.shape[0]):
    if seq[dedup_ind[idx-1]] != seq[ind[i]]:
      dedup_ind[idx] = ind[i]
      idx += 1
  dedup_ind = dedup_ind[:idx]

  return dataset[dedup_ind]


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  # Write a small deduplication routine.
  data_in = h5py.File(FLAGS.input_file, 'r')
  data_out = h5py.File(FLAGS.output_file, 'w')

  messages_in = data_in['messages']
  messages_out = data_out.create_group('messages')

  for pair in FLAGS.messages.split(','):
    node, message = pair.split('.')
    node = 'kAioNode' + node
    message = 'kMessageType' + message
    if node not in messages_in or message not in messages_in[node]:
      continue

    print 'Processing %s.%s:' % (node, message)
    dataset = _DedpulicateData(messages_in[node][message])

    if node not in messages_out:
      messages_out.create_group(node)
    messages_out[node].create_dataset(
        message, data=dataset, compression='gzip')

  data_out.close()
  data_in.close()


if __name__ == '__main__':
  main(sys.argv)

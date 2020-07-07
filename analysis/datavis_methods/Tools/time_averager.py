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

"""Utility to compute averages for entire loops."""

import numpy


def TimeAverager(loop_angles, time_data):
  """Get time average and standard deviations over all loops.

  TimeAverager will perform a time average of the inputs, after separating the
  data into loops. Recommended to use this to calculate net power in the loop.

  The following uses n as the number of samples, m as the number of fields,
  and l as the number of loops.

  Args:
    loop_angles: [n] array; the crosswind angular location [rad]
    time_data:   [n x m] array; disambiguous data type;

  Returns:
    avg_data:         struct; contains time averaged data and
                              its per-loop based standard deviation
      .average:       [m] array; mean of complete time_data
      .stddev:        [m] array; the standard deviation of complete time_data
      .loop_average:  [l x m]; time average data of each loop
      .loop_stddev:   [l x m]; standard deviation of each loop
      .loop_min:      [l x m]; min of each loop
      .loop_max:      [l x m]; max of each loop
      .Loop_<#>:      struct; contains loop-basis data
          .indx:      [int] Loop identifier
          .angles     [.. x m] angles in the loop
          .raw        [.. x m] time data in the loop
  """
  # Second order forward difference, keeping consistent vector length,
  # derivative starts at index 3
  dth_di = numpy.zeros(loop_angles.shape)
  dth_di[2:] = numpy.diff(numpy.diff(loop_angles))

  # Clear any slope changes, where the loop angle is not near 2*pi
  # reject all peaks when loop angle is less than 95% of 2*pi
  dth_di[loop_angles < 0.90 * 2 * numpy.pi] = 0.0

  # These are the first index for the data of the *current* loop
  # second order difference helps spot change in curvature of loop angle plot
  # which is more reliable for spotting peaks, irrespective of number of points.
  endloop_indices = []
  for i in xrange(dth_di.size - 2):
    if (dth_di[i + 1] < -1e-2 and dth_di[i + 1] < dth_di[i] and
        dth_di[i + 1] < dth_di[i + 2]):
      # i + 1 is the starting point of the next loop.
      endloop_indices.append(i + 1)

  # Remove any double peaks i.e. threshold peak separation
  max_overlapping = 10
  endloop_indices = numpy.array(endloop_indices)
  false_id = numpy.zeros((endloop_indices.size,), dtype=bool)
  false_id[1:] = numpy.diff(endloop_indices) < max_overlapping
  endloop_indices = endloop_indices[~false_id]

  nstruct_loops = endloop_indices.size - 1

  if isinstance(time_data, list):
    time_data = numpy.array(time_data)
    time_data = numpy.swapaxes(time_data, 0, 1)

  if nstruct_loops > 0:
    # Build the return struct
    avg_data = {
        'average': numpy.zeros(time_data.shape[1:]),
        'stddev': numpy.zeros(time_data.shape[1:]),
        'min': None,
        'max': None,
        'loop_average': [],
        'loop_stddev': [],
        'loop_min': [],
        'loop_max': [],
    }

    # Used to compile mean and standard deviation of interpolated values
    time_average = numpy.zeros(time_data.shape[1:])
    stddev = numpy.zeros(time_data.shape[1:])
    size = 0
    for i in xrange(nstruct_loops):
      # Build field names cell array
      loopname = 'Loop_%d' % i

      # Split the data into raw data bins
      jstart = endloop_indices[i]
      jend = endloop_indices[i + 1]
      length = jend - jstart
      size += length

      sum_square = numpy.sum(numpy.power(time_data[jstart:jend], 2.0), axis=0)
      loop_average = numpy.mean(time_data[jstart:jend], axis=0)
      loop_min = numpy.min(time_data[jstart:jend], axis=0)
      loop_max = numpy.max(time_data[jstart:jend], axis=0)
      avg_data['loop_average'].append(loop_average)
      avg_data['loop_stddev'].append(numpy.sqrt(
          (sum_square - loop_average * loop_average * length) /
          (length - 1)))
      avg_data['loop_min'].append(loop_min)
      avg_data['loop_max'].append(loop_max)

      # Compile your loop separated data
      avg_data[loopname] = {
          'indx': i,
          'angles': loop_angles[jstart:jend],
          'raw': time_data[jstart:jend],
      }
      time_average += loop_average * length
      stddev += sum_square

      avg_data['min'] = (
          loop_min if avg_data['min'] is None
          else numpy.minimum(avg_data['min'], loop_min))
      avg_data['max'] = (
          loop_max if avg_data['max'] is None
          else numpy.maximum(avg_data['max'], loop_max))

    # Average & stddev
    time_average /= size
    avg_data['average'] = time_average

    avg_data['loop_average'] = numpy.array(avg_data['loop_average'])
    avg_data['loop_stddev'] = numpy.array(avg_data['loop_stddev'])
    avg_data['loop_min'] = numpy.array(avg_data['loop_min'])
    avg_data['loop_max'] = numpy.array(avg_data['loop_max'])

    # Compute stddev
    avg_data['stddev'] = numpy.sqrt(
        (stddev - time_average * time_average * size) / (size - 1))
    return avg_data
  else:
    return None

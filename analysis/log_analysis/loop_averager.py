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

"""Compute average and standard deviation of flight data per average loop."""

import numbers
import numpy


def GetEndLoopIndices(loop_angles):
  """Function to get the indices corresponding to the end of a loop."""
  # Second order forward difference.
  dth_di = numpy.zeros(loop_angles.shape)
  dth_di[2:] = numpy.diff(numpy.diff(loop_angles))

  # clear any slope changes, where the loop angle is not near 2*pi
  # reject all peaks when loop angle is less than 95% of 2*pi
  dth_di[loop_angles < 0.90 * 2 * numpy.pi] = 0.0

  # These are the first index for the data of the *current* loop
  # second order difference helps spot change in curvature of loop angle plot
  # which is more reliable for spotting peaks, irrespective of number of points.
  # `max_overlapping` is actually the limit of the slope of the 1st order
  # forward difference.
  endloop_indices = []
  for i in xrange(dth_di.size - 2):
    if (dth_di[i + 1] < -1e-2 and dth_di[i + 1] < dth_di[i] and
        dth_di[i + 1] < dth_di[i + 2]):
      # i + 1 is the starting point of the next loop.
      endloop_indices.append(i + 1)
  return endloop_indices


def LoopAverager(loop_angles, time_data, bins):
  """Function to get mean and standard deviations over all loops.

  Examples:
    loop_data = LoopAverager(loop_angles, time_data, n_bins)
    loop_data = LoopAverager(loop_angles, time_data, angles_bin)

  LoopAverager will split apart the time-series data during crosswind into
  separate bins as well as compute the average and standard deviation of a
  common, interpolated angular position. Common angular positions is defined as
  the midpoint of the bin.
  If bins are specified as angle array rather than an integer, then these are
  used as common angular positions.
  Use TimeAverager.py for time based averaging e.g. net power calculations.

  Args:
    loop_angles: [n] array; the crosswind angular location [rad]
    time_data:   [m x n] array; disambiguous data type;
                                the data to be segregated on a per-loop basis
    bins:        int; no of discrete histogram angle bins for the loop data
                 [n] array; user specified common angles [rad]

  Returns:
    loop_data:        struct; contains the per-loop segregated arrays of data
                              and contains an independent array of the mean data
                              and its standard deviation
      .indx:          [int]; the integer number associated with the loop
      .common_angles: [m x n_bins] array; the common bucket angles [rad]
      .average:       [m x n_bins] array; the mean of the interpolated time_data
      .stddev:        [m x n_bins] array; the standard deviation of the
                                          interpolated time_data array
  """

  endloop_indices = GetEndLoopIndices(loop_angles)

  max_overlapping = 10
  endloop_indices = numpy.array(endloop_indices)
  false_id = numpy.zeros((endloop_indices.size,), dtype=bool)
  false_id[1:] = numpy.diff(endloop_indices) < max_overlapping
  endloop_indices = endloop_indices[~false_id]

  # Exclude partial loops.
  nstruct_loops = len(endloop_indices) - 1

  if nstruct_loops > 0:
    # Construct common angles
    if isinstance(bins, numbers.Number):
      bins = round(bins)
      del_angle = 2 * numpy.pi / bins
      # Common angles defined as midpoint of the bins
      # TODO: Descending angles?
      common_angles = numpy.linspace(
          2 * numpy.pi - del_angle / 2, del_angle/2, bins)
    else:
      common_angles = numpy.array(bins)

    # Construct independent interpolation vector
    x_ind = []
    angle_loop_id = []
    for ii in xrange(nstruct_loops):
      x_ind.append(common_angles - ii * 2 * numpy.pi)
      angle_loop_id.append(numpy.ones((len(common_angles),)) * ii)
    x_ind = numpy.concatenate(x_ind)
    angle_loop_id = numpy.concatenate(angle_loop_id)

    n_bins = len(common_angles)

    loop_angles_unwraped = loop_angles.copy()

    # Unwrap the loop angle to use interpolation in one step
    loopname = []
    for i in xrange(nstruct_loops):
      # Build field names cell array
      loopname.append('Loop_%d' % i)

      jstart = endloop_indices[i]
      jend = endloop_indices[i + 1]
      # Unwrap the loop angle to decrement values in each successive loop
      # with 2*pi, so that the loop angle array becomes a monotonically
      # decreasing time-series, making it easy to interpolate.
      loop_angles_unwraped[jstart:jend] = (
          loop_angles[jstart:jend] - i * 2 * numpy.pi)

    # Conform time_data to a [num_timeseries, timeseries_len] 2D array.
    if not isinstance(time_data, numpy.ndarray):
      time_data = numpy.array(time_data)
    if len(time_data.shape) == 1:
      time_data = numpy.reshape(time_data, (1, time_data.shape[0]))

    # Interpolate the time data
    num_timeseries = time_data.shape[0]
    time_data_interp = numpy.zeros((num_timeseries, len(x_ind)),
                                   dtype=time_data.dtype)

    loop_start_index = endloop_indices[0]
    loop_end_index = endloop_indices[-1]

    # For numpy.interp, x must be ascending.
    loop_angle_for_interp = (
        loop_angles_unwraped[loop_start_index:loop_end_index][::-1])
    time_data_for_interp = (
        time_data[:, loop_start_index:loop_end_index][:, ::-1])

    for t in xrange(num_timeseries):
      time_data_interp[t, :] = numpy.interp(
          x_ind, loop_angle_for_interp, time_data_for_interp[t, :])

    # Build the return struct
    loop_data = {}
    loop_data['common_angles'] = common_angles
    loop_data['loops'] = {}
    loops = loop_data['loops']

    # Used to compile mean and standard deviation of interpolated values
    tmp_average = numpy.zeros((num_timeseries, n_bins))
    for i in xrange(nstruct_loops):
      # Split the data into raw data bins
      jstart = endloop_indices[i]
      jend = endloop_indices[i + 1]

      # Compile your loop separated data
      loops[loopname[i]] = {}
      loops[loopname[i]]['indx'] = i
      loops[loopname[i]]['angles'] = loop_angles[jstart:jend]
      loops[loopname[i]]['raw'] = time_data[:, jstart:jend]
      loops[loopname[i]]['interp_dat'] = time_data_interp[:, angle_loop_id == i]
      tmp_average += loops[loopname[i]]['interp_dat']

    # Average & stddev
    loop_data['average'] = tmp_average / nstruct_loops

    # Compute stddev
    tmp_stddev = numpy.zeros((num_timeseries, n_bins))
    if nstruct_loops > 1:
      for i in xrange(nstruct_loops):
        tmp_stddev += (
            loops[loopname[i]]['interp_dat'] - loop_data['average']) ** 2
      loop_data['stddev'] = numpy.sqrt(tmp_stddev / nstruct_loops)
    else:
      loop_data['stddev'] = tmp_stddev
  else:
    loop_data = {}
  return loop_data

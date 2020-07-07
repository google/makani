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

import unittest
import loop_averager
import numpy


def Wrap(angles):
  return angles % (2 * numpy.pi)


class TestLoopAverager(unittest.TestCase):

  def setUp(self):
    self._points_per_loop = 1000
    num_loops = 100
    starting_loop_angle = -numpy.pi
    self._loop_angles = numpy.linspace(
        starting_loop_angle + numpy.pi * 2 * num_loops, starting_loop_angle,
        self._points_per_loop * num_loops)

    self._wrapped_loop_angles = Wrap(self._loop_angles)
    self._expected_stddev = numpy.array([0.1, 0.2, 0.3])
    self._base_functions = [
        lambda x: x,
        lambda x: x ** 2,
        lambda x: -x,
    ]
    self._functions = [
        lambda x: numpy.random.normal(x, self._expected_stddev[0]),
        lambda x: numpy.random.normal(x ** 2, self._expected_stddev[1]),
        lambda x: numpy.random.normal(-x, self._expected_stddev[2]),
    ]

    self._time_data = numpy.zeros(
        (len(self._functions), len(self._loop_angles)))
    for n in xrange(len(self._loop_angles)):
      for f in xrange(len(self._functions)):
        self._time_data[f, n] = self._functions[f](
            self._wrapped_loop_angles[n])

  def testNumBins(self):
    results = loop_averager.LoopAverager(
        self._wrapped_loop_angles, self._time_data, 10)
    self._AssertResults(results, self._functions)

  def testBinnedAngles(self):
    results = loop_averager.LoopAverager(
        self._wrapped_loop_angles, self._time_data,
        [0.5, 1.0, 2.0, 3.14, 4.0, 5.0, 6.27])
    self._AssertResults(results, self._functions)

  def testOneTimeSeries(self):
    for idx in range(len(self._functions)):
      results = loop_averager.LoopAverager(
          self._wrapped_loop_angles, self._time_data[idx, :], 10)
      self._AssertResults(results, [self._base_functions[idx]])

      results = loop_averager.LoopAverager(
          self._wrapped_loop_angles, self._time_data[idx, :],
          [0.5, 1.0, 2.0, 3.14, 4.0, 5.0, 6.27])
      self._AssertResults(results, [self._base_functions[idx]])

  def _AssertResults(self, results, functions):
    angles = results['common_angles']
    assert len(angles) > 1

    expected_average = []
    for i, angle in enumerate(angles):
      if i == len(angles) - 1:
        angle_interval = abs(angle - angles[i - 1])
      else:
        angle_interval = abs(angles[i + 1] - angle)
      sub_angles = numpy.linspace(
          angle - angle_interval * 0.5, angle + angle_interval * 0.5,
          self._points_per_loop / len(angles))
      expected_average.append([numpy.mean(f(sub_angles)) for f in functions])

    expected_average = numpy.transpose(expected_average)
    mean_diff = numpy.mean(numpy.abs(results['average'] - expected_average),
                           axis=1)
    mean_sum = numpy.mean(numpy.abs(results['average']), axis=1) + 1e-5
    error_percent = mean_diff / mean_sum
    self.assertTrue(numpy.all(error_percent < 0.05))

  def testStddev(self):
    num_bins = 30
    num_loops = 100

    # Define angles around a single loop, in descending order.
    angle_interval = numpy.pi * 2.0 / num_bins
    angles = numpy.flipud(numpy.array(range(num_bins)) * angle_interval).astype(
        float)
    # Repeat the angle for num_loop times.
    loop_angles = numpy.repeat(numpy.expand_dims(angles, 0), num_loops, axis=0)
    # Define loop data for each angle, where data values in loop[n] are all `n`.
    values = numpy.array(range(num_loops)).astype(float)
    loop_data = numpy.repeat(numpy.expand_dims(values, -1), num_bins, axis=1)
    # Expected std. dev.
    # The loop averager assumes the first and last loops are partial, so
    # they are not accounted for as full loops.
    expected_stddev = numpy.std(values[1:-1])
    expected_mean = numpy.mean(values[1:-1])

    results = loop_averager.LoopAverager(
        loop_angles.flatten(), loop_data.flatten(), num_bins)

    # Due to interpolation, there is `0.5` values on the boundary of loops.
    self.assertTrue(
        numpy.all(numpy.abs(results['average'] - expected_mean) < 0.5))
    # Same for stddev. Note the expected_stddev is around 28.29.
    self.assertTrue(
        numpy.all(numpy.abs(results['stddev'] - expected_stddev) < 0.5))


if __name__ == '__main__':
  unittest.main()

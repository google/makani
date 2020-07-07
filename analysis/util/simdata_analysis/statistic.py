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

"""Statistic class.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from makani.analysis.util.simdata_analysis import utils
import numpy as np
import pandas as pd


class Statistic(object):
  """Statistic class.
  """

  def __init__(self, resampled_data, stat_function):
    """Initializes the Statistic class.

    Args:
      resampled_data: Data obtained using bootstrap.bootstrap.
      stat_function: Statistic function to be used (for instance: np.mean).
    """
    assert isinstance(resampled_data, np.ndarray), (
        'resampled_data must be a Numpy ndarray.')
    assert len(resampled_data.shape) == 2, (
        'resampled_data must be resampled using bootstrap.bootstrap')
    assert callable(stat_function), 'stat_function must be callable'

    self.resampled_data = resampled_data.astype(float)
    self.stat_function = stat_function
    self.n_resamples = resampled_data.shape[0]
    self.n_samples = resampled_data.shape[1]
    self.resampled_stat = pd.DataFrame(resampled_data).apply(stat_function,
                                                             axis=1).values
    self.mean = self.get_mean()
    self.bounds = (self.get_percentile(2.5), self.get_percentile(97.5))

  def get_mean(self):
    """Computes the mean statistic.

    Returns:
      Mean of the statistic.
    """
    return np.mean(self.resampled_stat)

  def get_std(self):
    """Computes the standard deviation of the statistic.

    Returns:
      Standard deviation of the statistic.
    """
    return np.std(self.resampled_stat)

  def get_percentile(self, pctile):
    """Computes the percentile of the statistic.

    Args:
      pctile: Percentile to compute (between 0 and 100).

    Returns:
      Percentile of the statistic.
    """
    return np.percentile(self.resampled_stat, pctile)

  def text_histogram(self, *args):
    """Text-based histogram.

    Returns a multi-line string with a vertical histogram of
    self.resampled_stat.

    Args:
      *args: Additional arguments to be passes to utils.ascii_histogram

    Returns:
      Multi-line string.
    """
    return utils.ascii_histogram(self.resampled_stat, *args)

  def __repr__(self):
    return self.__str__()

  def __str__(self):
    lines = 'Mean: {0}\n'.format(self.mean)
    lines += 'Confidence bounds (95%): {0}\n'.format(self.bounds)
    lines += 'Histogram: \n'
    lines += self.text_histogram()
    return lines

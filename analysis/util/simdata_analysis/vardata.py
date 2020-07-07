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

"""Simulation variable data class.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from makani.analysis.util.simdata_analysis import bootstrap
from makani.analysis.util.simdata_analysis.statistic import Statistic
from makani.analysis.util.simdata_analysis.utils import ascii_histogram as hist
from makani.analysis.util.simdata_analysis.utils import ecdf
from makani.analysis.util.simdata_analysis.utils import prob_above_thr
from makani.analysis.util.simdata_analysis.utils import prob_below_thr
import matplotlib.pyplot as plt
import numpy as np


NRESAMPLES = 1000  # Default resamples.


class VariableData(object):
  """Encapsulates simulation variable data.
  """

  def __init__(self, variable_info, table_info, var_df, data):
    """Initializes the object.

    Data is resampled here so it is readily available. The default number
    of samples equals the number of samples in the data, and the number of
    resamples is the package default NRESAMPLES.

    Args:
      variable_info: Dictionary with information about the variable.
      table_info: Dictionary with information about the table.
      var_df: DataFrame containing data about the variable.
      data: Numpy array containing the numerical data.
    """
    assert isinstance(variable_info, dict)
    assert isinstance(table_info, dict)
    for field in ['name']:
      assert field in variable_info
    for field in ['title', 'index', 'num_jobs']:
      assert field in table_info

    self.variable_info = variable_info
    self.table_info = table_info
    self.var_df = var_df
    self.data = data

    self.n_samples = len(self.data)
    self.n_resamples = NRESAMPLES
    self.bootstrapped_data = bootstrap.bootstrap(self.data, self.n_samples,
                                                 self.n_resamples)
    self.n_valid_samples = len(self.data[~np.isnan(self.data)])

  def resample(self, n_samples=None, n_resamples=None):
    """Resamples the data.

    Args:
      n_samples: Number of samples. If None, the default number of
          samples is used (number of samples in original data).
      n_resamples: Number of resamples. If None, the default number of
          resamples is used (package value of NRESAMPLES).
    """
    if n_samples is None:
      n_samples = len(self.data)
    if n_resamples is None:
      n_resamples = NRESAMPLES

    self.n_samples = n_samples
    self.n_resamples = n_resamples
    self.bootstrapped_data = bootstrap.bootstrap(self.data, self.n_samples,
                                                 self.n_resamples)

  def text_histogram(self, *args):
    """Returns a multi-line string with a vertical histogram of self.data.

    Args:
      *args: Additional arguments to be passed to utils.ascii_histogram.
    """
    if self.n_valid_samples == 0:
      return '\n'
    return hist(self.data[~np.isnan(self.data)], *args)

  def mean(self):
    """Returns a Statistic object representing the mean.
    """
    return Statistic(self.bootstrapped_data, np.mean)

  def std(self):
    """Returns a Statistic object representing the standard deviation.
    """
    return Statistic(self.bootstrapped_data, np.std)

  def percentile(self, pctile):
    """Returns a Statistic object representing a percentile.

    Args:
      pctile: Percentile (between 0 and 100).
    """
    assert pctile >= 0. and pctile <= 100.

    return Statistic(self.bootstrapped_data,
                     lambda x: np.percentile(x, pctile))

  def prob_above(self, thr):
    """Probability of the variable being above a threshold.

    The probability is computed using 1 - ECDF(thr).

    Args:
      thr: Threshold value.

    Returns:
      Statistic object.
    """
    return Statistic(self.bootstrapped_data, lambda x: prob_above_thr(x, thr))

  def prob_below(self, thr):
    """Probability of the variable being below a threshold.

    The probability is computed using ECDF(thr).

    Args:
      thr: Threshold value.

    Returns:
      Statistic object.
    """
    return Statistic(self.bootstrapped_data, lambda x: prob_below_thr(x, thr))

  def hist(self, **hist_kw):
    """Returns figure with histogram.

    Args:
      **hist_kw: Keywords to be passed to matplotlib.pyplot.hist().

    Returns:
      matplotlib figure.
    """
    plt.hist(self.data[~np.isnan(self.data)], **hist_kw)
    plt.xlabel(self.variable_info['name'])
    plt.ylabel('Samples')

    return plt.gcf()

  def ecdf_data(self):
    """Computes the data for the ECDF.

    Returns:
      Tuple (xs, (mean_ecdf, lower_ecdf, upper_ecdf)):
        - xs: Values where the ECDF is computed.
        - mean_ecdf: Mean ECDF.
        - lower_ecdf: Lower bound of the ECDF (95% confidence).
        - upper_ecdf: Upper bound of the ECDF (95% confidence).
    """
    # This line returns a 3D array:
    #   resampled_ecdf_data[i,0,:] is the x values for resample i
    #   resampled_ecdf_data[i,1,:] is the CDF values for resample i
    resampled_ecdf_data = np.apply_along_axis(ecdf, 1, self.bootstrapped_data)

    # Get some datapoints.
    xs, _ = ecdf(self.data[~np.isnan(self.data)])
    xs = np.linspace(xs[0], xs[-1], 200)  # Resample evenly.

    # Interpolate the CDFs
    ecdf_mean_bounds = np.empty((3, len(xs)))
    for idx, x in enumerate(xs):
      data = np.empty(self.n_resamples)
      for i in range(self.n_resamples):
        data[i] = np.interp(x, resampled_ecdf_data[i, 0, :],
                            resampled_ecdf_data[i, 1, :], left=0, right=1.)
      ecdf_mean_bounds[0, idx] = np.mean(data)
      ecdf_mean_bounds[1, idx] = np.percentile(data, 2.5)
      ecdf_mean_bounds[2, idx] = np.percentile(data, 97.5)

    return xs, ecdf_mean_bounds

  def ecdf(self, plot_bounds=True, **plot_kws):
    """Returns figure with the ECDF.

    Args:
      plot_bounds: Flag to plot the 95% confidence bounds.
      **plot_kws: Keywords to be passed to matplotlib.pyplot.plot().

    Returns:
      matplotlib figure.
    """
    xs, ecdf_mean_bounds = self.ecdf_data()
    p = plt.plot(xs, ecdf_mean_bounds[0, :], **plot_kws)
    if plot_bounds:
      plt.plot(xs, ecdf_mean_bounds[1, :], '--', color=p[-1].get_color())
      plt.plot(xs, ecdf_mean_bounds[2, :], '--', color=p[-1].get_color())
    plt.xlabel(self.variable_info['name'])
    plt.ylabel('CDF')
    plt.yticks(np.linspace(0, 1, 11))
    plt.ylim(0, 1)
    plt.grid()

    return plt.gcf()

  def __repr__(self):
    return self.__str__()

  def __str__(self):
    lines = 'Table: {0}\n'.format(self.table_info['title'])
    lines += 'Table index: {0}\n'.format(self.table_info['index'])
    lines += 'Number of samples: {0} ({1} valid)\n'.format(
        self.n_samples, self.n_valid_samples)
    lines += 'Range (min, max): ({0}, {1})\n'.format(min(self.data),
                                                     max(self.data))
    lines += 'Histogram: \n'
    lines += self.text_histogram()
    return lines


class ScoreData(VariableData):
  """Encapsulates simulation score data.
  """

  def __init__(self, score_info, table_info, score_df):
    """Initializes the object.

    Args:
      score_info: Dictionary with information about the score.
      table_info: Dictionary with information about the table.
      score_df: DataFrame containing the columns:
          - score: score values.
          - job_id: job_ids identifying the scores.
          - folder: Folder that originated the data (useful for imports from
              multiple files, and therefore repeated job_ids).
    """
    assert set(score_df.columns.values) == set(['score', 'job_id', 'folder'])
    assert isinstance(score_info, dict)
    assert isinstance(table_info, dict)
    for field in ['name', 'index', 'units', 'severity', 'experimental']:
      assert field in score_info

    super(ScoreData, self).__init__(score_info, table_info, score_df,
                                    np.array(score_df['score']))

  def __repr__(self):
    return self.__str__()

  def __str__(self):
    lines = 'Variable: {0}\n'.format(self.variable_info['name'])
    lines += 'Variable index: {0}\n'.format(self.variable_info['index'])
    lines += 'Units: {0}\n'.format(self.variable_info['units'])
    lines += 'Severity: {0}\n'.format(self.variable_info['severity'])
    lines += 'Experimental: {0}\n'.format(self.variable_info['experimental'])
    return lines + super(ScoreData, self).__str__()


class InputData(VariableData):
  """Encapsulates simulation input data.
  """

  def __init__(self, input_info, table_info, input_df):
    """Initializes the object.

    Args:
      input_info: Dictionary with information about the input variable.
      table_info: Dictionary with information about the table.
      input_df: DataFrame containing the columns:
          - value: Input values.
          - job_id: job_ids identifying the input.
          - folder: Folder that originated the data (useful for imports from
              multiple files, and therefore repeated job_ids).
    """
    assert set(input_df.columns.values) == set(['value', 'job_id', 'folder'])
    assert isinstance(input_info, dict)
    assert isinstance(table_info, dict)
    for field in ['name']:
      assert field in input_info

    super(InputData, self).__init__(input_info, table_info, input_df,
                                    np.array(input_df['value']))

  def __repr__(self):
    return self.__str__()

  def __str__(self):
    lines = 'Input: {0}\n'.format(self.variable_info['name'])
    return lines + super(InputData, self).__str__()

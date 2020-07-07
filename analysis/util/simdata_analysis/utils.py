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

"""Utilities.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import matplotlib.pyplot as plt
import numpy as np
from scipy import stats


def ascii_histogram(data, nbins=10, height=20, character='#'):
  """Text-based vertical histogram.

  Returns a multi-line string with a vertical histogram of data.

  Args:
    data: array of data,
    nbins: Number of bins in the histogram.
    height: Height of the histogram in characters.
    character: Character to use.

  Returns:
    Multi-line string.
  """
  h = np.histogram(data, bins=nbins)
  his = """"""
  xl = ['%.2f'%n for n in h[1]]
  lxl = [len(l) for l in xl]
  bars = h[0]/np.max(h[0])*height
  his += ' '*int(np.max(bars)+2+np.max(lxl)) + (
      '%.1f %%\n'%(100.*np.max(h[0])/len(data)))
  for i, c in enumerate(bars):
    line = xl[i] + ' '*int(np.max(lxl)-lxl[i]) + ': ' + character*int(c) + '\n'
    his += line
  return his


def interp_ecdf(data, value):
  """Returns the probabilty of a random variable of being above a threshold.

  Args:
    data: Samples of the random variable.
    value: Interpolation value.
  """
  y_interp = stats.percentileofscore(data, value)/100.

  return y_interp


def prob_above_thr(data, threshold):
  """Returns the probabilty of a random variable of being above a threshold.

  Args:
    data: Samples of the random variable.
    threshold: Threshold for probabiltiy calculation.
  """
  return 1. - interp_ecdf(data, threshold)


def prob_below_thr(data, threshold):
  """Returns the probabilty of a random variable of being below a threshold.

  Args:
    data: Samples of the random variable.
    threshold: Threshold for probabiltiy calculation.
  """
  return interp_ecdf(data, threshold)


def ecdf(data):
  """Empirical CDF.

  Args:
    data: Data

  Returns:
    Tuple (xs, ys):
      - xs: Sorted data values
      - ys: ECDF value
  """
  xs = np.sort(data)
  ys = np.arange(1, len(xs)+1)/float(len(xs))
  return xs, ys


def qqplot(vardata1, vardata2):
  """Plot QQ-plot for two variables.

  Args:
    vardata1: Variable to use for X axis.
    vardata2: Variable to use for Y axis.

  Returns:
    Matplotlib figure
  """
  var1_xs, var1_ecdf_mean_bounds = vardata1.ecdf_data()
  var2_xs, var2_ecdf_mean_bounds = vardata2.ecdf_data()

  quants = np.linspace(0, 1, 1000)

  x_vals = np.interp(quants, var1_ecdf_mean_bounds[0], var1_xs)
  y_vals = np.interp(quants, var2_ecdf_mean_bounds[0], var2_xs)
  min_xvals = min(x_vals[0], y_vals[0])
  max_xvals = max(x_vals[-1], y_vals[-1])
  plt.plot(x_vals, y_vals, 'o')
  plt.plot([min_xvals, max_xvals], [min_xvals, max_xvals], 'k--')
  plt.grid()
  plt.xlabel(vardata1.variable_info['name'])
  plt.ylabel(vardata2.variable_info['name'])

  return plt.gcf()


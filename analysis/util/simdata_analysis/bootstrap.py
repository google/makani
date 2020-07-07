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

"""Bootstrap utils.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import numpy as np


def bootstrap(data, nsamples, nresamples):
  """Returns bootstrapped data.

  Args:
    data: Numpy array containing the original data.
    nsamples: Number of samples per resample.
    nresamples: Number of resamples.

  Returns:
    2D array with nsamples columns and nresamples rows.
  """
  this_data = np.array(data).squeeze()
  this_data = this_data[~np.isnan(this_data)]  # Remove NaNs

  assert len(this_data.shape) == 1

  if len(this_data) > 1:
    indexes = np.random.randint(0, len(this_data)-1, (int(nresamples),
                                                      int(nsamples)))
    return this_data[indexes]
  elif len(this_data) == 1:
    return this_data[0] * np.ones((int(nresamples), int(nsamples)))
  else:
    empty_matrix = np.empty((int(nresamples), int(nsamples)))
    empty_matrix[:] = np.nan
    return empty_matrix

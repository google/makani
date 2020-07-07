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

"""Scoring functions related to software performance."""

from makani.lib.python.batch_sim import scoring_functions
import numpy as np
import scoring_functions_util as scoring_util


class MaxSampleTimeErrorScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Identifies packet dropouts in the telemetry data."""

  def __init__(self, good_limit, bad_limit, severity):
    super(MaxSampleTimeErrorScoringFunction, self).__init__(
        'Max. Sampling Time Error', 's', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['experimental']

  def GetValue(self, output):
    return output['max_sample_time_error']

  def GetOutput(self, timeseries):
    return {'max_sample_time_error': np.max(timeseries['sample_time_error'])}

  def GetTimeSeries(self, params, sim, control):
    time = self._SelectTelemetry(sim, control, 'time')

    if scoring_util.IsSelectionValid(time):
      t_samp = scoring_util.GetTimeSamp(time)
      return {'sample_time_error': np.abs(np.diff(time) - t_samp)}
    else:
      return {'sample_time_error': np.array([float('nan')])}

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

"""Scoring functions whose goal is not to score but to indicate status."""

from makani.lib.python.batch_sim import scoring_functions


class LoopAngleScoringFunction(scoring_functions.ScoringFunction):
  """Indicate loop angles for analysis/plotting purposes."""

  def __init__(self, section_name):
    super(LoopAngleScoringFunction, self).__init__(
        'Loop Angle (%s)' % section_name, 'rad', severity=0)

  def GetSystemLabels(self):
    return ['controls']

  def GetTimeSeries(self, params, sim, control):
    flight_modes = ['kFlightModeCrosswindNormal',
                    'kFlightModeCrosswindPrepTransOut']
    loop_angle = self._SelectTelemetry(
        sim, control, ['loop_angle'],
        flight_modes=flight_modes)

    return {
        'loop_angle': loop_angle,
    }

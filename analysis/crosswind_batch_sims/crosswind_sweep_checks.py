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

"""List of checks for crosswind sweeps."""

from makani.analysis.checks.collection import score_checks
from makani.analysis.crosswind_batch_sims import batch_sim_params


class CrosswindSweepChecks(score_checks.ScoreChecks):
  """The list of checks for crosswind sweep."""

  def __init__(self, for_log, wing_model):
    sweep_parameters = batch_sim_params.CrosswindSweepsParameters(
        only_steady_flight=True, steady_flight_mode_time=0.0,
        wing_model=wing_model)
    super(CrosswindSweepChecks, self).__init__(
        for_log, sweep_parameters.scoring_functions, ['CrosswindNormal'])

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

"""Classes to perform checks on avionics sensors."""

from makani.analysis.checks import base_check
from makani.lib.python.batch_sim import scoring_functions

LOWER_BOUND = 0
UPPER_BOUND = 1
BOTH_BOUNDS = 2


class ScoreChecker(base_check.BaseCheckItem):
  """The base class for checking values of scoring function outputs."""

  @base_check.RegisterSpecs
  def __init__(self, scoring_function, field, name=None,
               bound_mode=BOTH_BOUNDS, **base_kwargs):
    """Initialize the checker for a given unit.

    Args:
      scoring_function: The score function.
      field: The field to retrieve from the scoring output.
      name: The name of the check. Default to name of the scoring function.
      bound_mode: Decide which bounds to check in the score function.
          For DoubleSidedLimitScoringFunctions, it is one of
          LOWER_BOUND/UPPER_BOUND/BOTH_BOUNDS. For
          SingleSidedLimitScoringFunctions, it can be a tuple/list of two
          elements to override the default limits in the scoring function.
      **base_kwargs: Args for the base class, including
          normal_ranges and warning_ranges.
    """
    self._scoring_function = scoring_function
    self._field = field
    self._bound_mode = bound_mode
    score_class = scoring_function.Class()

    if name is None:
      name = scoring_function.GetName()

    if issubclass(score_class,
                  scoring_functions.SingleSidedLimitScoringFunction):
      if isinstance(bound_mode, (tuple, list)):
        assert len(bound_mode) == 2
        good_limit = bound_mode[0]
        bad_limit = bound_mode[1]
      else:
        good_limit, bad_limit = scoring_function.Limits()

      assert good_limit != bad_limit
      if good_limit < bad_limit:
        normal_ranges = [[None, good_limit]]
        warning_ranges = [[None, bad_limit]]
      else:
        normal_ranges = [[good_limit, None]]
        warning_ranges = [[bad_limit, None]]

    elif issubclass(score_class,
                    scoring_functions.DoubleSidedLimitScoringFunction):
      (bad_lower_limit, good_lower_limit, good_upper_limit,
       bad_upper_limit) = scoring_function.Limits()
      if bound_mode == LOWER_BOUND:
        normal_ranges = [[good_lower_limit, None]]
        warning_ranges = [[bad_lower_limit, None]]
      elif bound_mode == UPPER_BOUND:
        normal_ranges = [[None, good_upper_limit]]
        warning_ranges = [[None, bad_upper_limit]]
      else:
        assert bound_mode == BOTH_BOUNDS
        normal_ranges = [[good_lower_limit, good_upper_limit]]
        warning_ranges = [[bad_lower_limit, bad_upper_limit]]

    else:
      # Need to manually specify the range in **base_kwargs.
      assert 'normal_ranges' in base_kwargs
      super(ScoreChecker, self).__init__(True, name=name, **base_kwargs)
      return

    super(ScoreChecker, self).__init__(
        True, name=name,
        normal_ranges=normal_ranges, warning_ranges=warning_ranges,
        **base_kwargs)

  def _RegisterInputs(self):
    """Register what fields will be used to calculate the check results."""
    return [
        self._Arg('parameters', None, None),
        self._Arg('SimTelemetry', 'Simulator', None),
        self._Arg('ControlDebug', 'ControllerA', None),
    ]

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, params, sim_telemetry, control_telemetry):
    """Calculate check results using the list of registered inputs."""
    assert self._for_log
    data = self._scoring_function.GetTimeSeries(
        params, sim_telemetry, control_telemetry)
    if self._field not in data:
      return

    data = data[self._field].flatten()
    label = self._scoring_function.GetName()
    self._CheckByRange(label, data, self._normal_ranges, self._warning_ranges)

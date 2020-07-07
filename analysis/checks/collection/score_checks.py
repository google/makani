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

"""List of checks for score functions."""
from makani.analysis.checks import base_check
from makani.analysis.checks import score_check
from makani.control import control_types
from makani.lib.python import c_helpers
from makani.lib.python.batch_sim import scoring_functions
from makani.lib.python.batch_sim.scoring_functions import aero
from makani.lib.python.batch_sim.scoring_functions import crosswind
from makani.lib.python.batch_sim.scoring_functions import kinematics
from makani.lib.python.batch_sim.scoring_functions import status
from makani.lib.python.batch_sim.scoring_functions import tether
import numpy as np

_FLIGHT_MODE_HELPER = c_helpers.EnumHelper('FlightMode', control_types)


class ScoreChecks(base_check.ListOfChecks):
  """The scoring checklist."""

  def __init__(self, for_log, scoring_function_list, flight_modes):
    """Initialize a list of checks from a list of scoring functions.

    Args:
      for_log: True if this check list is for logs (not telemetry).
      scoring_function_list: A list of scoring functions.
      flight_modes: The list of flight modes to check.

    Raises:
      ValueError: Raised if the scoring function is invalid to create a check.
    """

    # A dict of scoring function classes used to choose and
    # parameterize the checks from the list of scoring functions.
    # Example:
    #     {
    #         aero.AlphaDegScoringFunction: {
    #             'field': 'alpha',
    #             'name': None, # Default uses the name of the score function.
    #         }
    #     }
    classes_to_include = {
        aero.SurfaceSaturationScoringFunction: {
            'field': 'saturation_mask',
            # This field is actually binary (0 for no saturation, 1 for
            # saturation). Setting normal limit to 0.8 so that a warning is
            # raised when a surface saturates.
            'bounds': (0.8, 1.0),
        },
        aero.AlphaDegScoringFunction: {'field': 'alpha'},
        aero.BetaDegScoringFunction: {'field': 'beta'},
        aero.AlphaDegErrorScoringFunction: {'field': 'alpha_error'},
        aero.BetaDegErrorScoringFunction: {'field': 'beta_error'},
        kinematics.TetherSphereDeviationScoringFunction: {
            'field': 'tether_sphere_deviation'},
        tether.PitchDegScoringFunction: {'field': 'tether_pitch'},
        tether.RollDegScoringFunction: {'field': 'tether_roll'},
        aero.AirspeedMinScoringFunction: {'field': 'airspeed'},
        aero.AirspeedMaxScoringFunction: {'field': 'airspeed'},
        crosswind.AirspeedErrorScoringFunction:
            {'field': 'crosswind_airspeed_error'},
        tether.TensionMinScoringFunction: {'field': 'tension'},
        tether.TensionMaxScoringFunction: {'field': 'tension'},
        crosswind.RadiusErrorScoringFunction: {'field': 'crosswind_radius_err'},
    }

    all_flight_modes = set(_FLIGHT_MODE_HELPER.Names())
    target_flight_modes = set(
        [_FLIGHT_MODE_HELPER.Name(f) for f in flight_modes])
    sample_rate = 0.1
    self._items_to_check = []
    for scoring_func in scoring_function_list:
      scoring_class = scoring_func.Class()
      if scoring_class in classes_to_include:
        check_params = classes_to_include[scoring_class]
        field = check_params['field']

        system_labels = scoring_func.GetSystemLabels()
        scoring_flight_modes = set(system_labels) & all_flight_modes
        if scoring_flight_modes != target_flight_modes:
          continue

        name = None
        if 'name' in classes_to_include[scoring_class]:
          name = classes_to_include[scoring_class]['name']
        if name is None:
          name = scoring_func.GetName()

        scoring_func = scoring_functions.SampleFilter(
            sample_rate, scoring_func, show_rate=False)
        # Align the scoring functions to the same source to align them for
        # plotting batch-sim scoring events.
        scoring_func.SetSourcePriority(['control', 'sim'])
        if issubclass(scoring_class,
                      scoring_functions.SingleSidedLimitScoringFunction):
          bounds = check_params['bounds'] if 'bounds' in check_params else None
          self._items_to_check.append(
              score_check.ScoreChecker(
                  scoring_func, field, name=name, bound_mode=bounds))
        elif issubclass(scoring_class,
                        scoring_functions.DoubleSidedLimitScoringFunction):
          self._items_to_check += [
              score_check.ScoreChecker(
                  scoring_func, field, name='%s (Lower)' % name,
                  bound_mode=score_check.LOWER_BOUND),
              score_check.ScoreChecker(
                  scoring_func, field, name='%s (Upper)' % name,
                  bound_mode=score_check.UPPER_BOUND)]
        else:
          raise ValueError(
              'Can only check scoring functions with limits. Got %s.' %
              scoring_class.__name__)

    # TODO: Make sure this is consistent with batch_sim_params.py.
    flight_mode_time_threshold = 0.0

    self._items_to_check += [
        # Upper half.
        score_check.ScoreChecker(
            scoring_functions.SampleFilter(
                sample_rate,
                scoring_functions.CrosswindFilter(
                    status.LoopAngleScoringFunction('Upper Semicircle'),
                    flight_mode_time_threshold=flight_mode_time_threshold),
                show_rate=False),
            field='loop_angle',
            normal_ranges=[[None, np.pi], [np.pi * 2.0, None]],
            warning_ranges=[[None, np.pi], [np.pi * 2.0, None]]),

        # Lower half.
        score_check.ScoreChecker(
            scoring_functions.SampleFilter(
                sample_rate,
                scoring_functions.CrosswindFilter(
                    status.LoopAngleScoringFunction('Lower Semicircle'),
                    flight_mode_time_threshold=flight_mode_time_threshold),
                show_rate=False),
            'loop_angle',
            normal_ranges=[[None, 0.0], [np.pi, None]],
            warning_ranges=[[None, 0.0], [np.pi, None]]),
    ]

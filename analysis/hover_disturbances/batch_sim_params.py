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

"""Scoring functions used by this batch simulation."""

from makani.lib.python.batch_sim import scoring_functions
from makani.lib.python.batch_sim.scoring_functions import tether
import numpy


def _CartesianToArray(a):
  return numpy.array([a[d] for d in ('x', 'y', 'z')])


class HoverDisturbancesParameters(object):
  """Common parameters to the client and worker for this batch sim.

  Attributes:
    setup_time: Time [s] to allow position to settle before the disturbance.
    settle_time: Time [s] to allow position to settle after the disturbance.
    impulse_duration: Duration [s] of the impulse disturbance.
    scoring_functions: List of scoring functions.
  """

  def __init__(self):
    self.setup_time = 150.0
    self.impulse_duration = 3.0
    self.settle_time = 25.0

    # The flight_mode_time_threshold is used to ignore the start of
    # the flight as the wing settles.  It is distinct from the setup
    # time, which includes all the time since the beginning of the
    # flight.
    self.scoring_functions = (
        # NOTE: HoverPosition and HoverAngles scoring functions
        # previously included here were specific to flight plans that have
        # now been removed.  These scoring functions should be revisited.

        # The negative limits are set by impingement of the tether release on
        # the wing (keeping a 0.5 deg margin on ultimate). The positive limits
        # are set by the starboard bridle hitting the starboard outer pylon.
        scoring_functions.FlightModeFilter(
            'kFlightModeHoverPayOut',
            tether.SustainedPitchDegScoringFunction(-55.0, -50.0, 10.0, 17.0,
                                                    severity=1,
                                                    sustained_duration=3.0)),

        # The bad limits match up with leaving a 3 degree margin before the
        # tether angle matches the bridle angle and the other bridle goes slack.
        scoring_functions.FlightModeFilter(
            'kFlightModeHoverPayOut',
            tether.RollDegScoringFunction(-45.0, -35.0, 35.0, 45.0, severity=1),
            flight_mode_time_threshold=50.0)
    )

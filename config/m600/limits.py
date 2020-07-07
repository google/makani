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

"""Config limits, used in scoring functions, that are not defined elsewhere."""

from makani.config import mconfig
from makani.control import control_types as m
import numpy as np


@mconfig.Config(deps={
    'test_site': 'common.test_site',
})


def MakeParams(params):
  if params['test_site'] == m.kTestSiteNorway:
    # TODO: Update to more reasonable value for offshore.
    # Maybe also compute directly from active playbook.
    # Current limit is same as m600 onshore for consistency.
    min_altitude = 90.0
  else:
    # Limits derived from b/117292304.
    min_altitude = 90.0

  return {
      # Tether and bridle range of motions [rad] before interference with pylons
      # or other components, for the given flight mode and loading.
      # The bridles collide with the pylons at 17.7 deg. of pitch; using 18.0 to
      # maintain consistency with scoring functions with a -2, -1 deg offset.
      # The port bridle becomes slack at tether_roll = 53.0 deg.
      # The starboard bridle becomes slack at tether_roll = -48.4 deg.
      'tether_hover_pitch_rom': np.deg2rad([-40.0, 18.0]).tolist(),
      'tether_hover_roll_rom': np.deg2rad([-50.0, 50.0]).tolist(),
      'tether_crosswind_pitch_rom': np.deg2rad([-40.0, 18.0]).tolist(),
      'tether_crosswind_roll_rom': np.deg2rad([-50.0, 50.0]).tolist(),
      'tether_pitch_tension_threshold': 50e3,

      # Aero angle limits [rad], for scoring functions
      # - The positive "good" limit is driven by a reduction in control
      #   authority and stability.
      # - The positive "bad" limit is driven by an abrupt stall observed in
      #   CFD, and kite excursions observed in flight tests for
      #   angles-of-attack greater than 10 degrees.  A one degree margin is included.
      # - The negative "good" and "bad" limits respectively mark the onset
      #   of non-linearity and stall, as predicted by CFD for the range of
      #   flap deflections of interest, -30 to +10 degrees.
      # Used as the threshold to start scoring some trans-in functions
      # (AfterPitchForwardScoringFunction) only after the initial pitch forward
      # is complete; and as the good upper limit for the
      # MinAlphaDegScoringFunction in TransIn.
      'trans_in_pitched_forward_alpha': np.deg2rad(3.0),
      # b/118184164: The positive good and bad limits for the offshore
      # kite, kWingSerialCrosswind05, were ~3 deg lower to account for degraded
      # aerodynamic performance due to Vinyl wrap seams in an undesirable
      # locations, but scoring this config is no longer supported.
      'crosswind_alpha_limits': np.deg2rad([-16.0, -12.0, 6.0, 9.0]).tolist(),
      # The absolute sideslip limits are given by the values in
      # go/makanispec, Sec. 2; however, we reduce the sideslip limits to
      # +/- 10 deg, which should be sufficient for normal flight.
      'trans_in_beta_limits': np.deg2rad([-15.0, -7.0, 7.0, 15.0]).tolist(),
      'crosswind_beta_limits': np.deg2rad([-10.0, -7.0, 7.0, 10.0]).tolist(),
      'prep_trans_out_beta_limits': np.deg2rad([-15.0, -10.0, 10.0, 15.0]
                                              ).tolist(),

      # Scoring functions starts 10 meters lower.
      'min_altitude': min_altitude,
  }

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
import numpy as np


@mconfig.Config
def MakeParams():

  return {
      # Tether and bridle range of motions [rad] before interference with pylons
      # or other components, for the given flight mode and loading.
      # From M10096, section 5.3.1, adapted for most limitng cases.
      'tether_hover_pitch_rom': np.deg2rad([-90.0, 25.0]).tolist(),
      'tether_hover_roll_rom': np.deg2rad([-50.0, 50.0]).tolist(),
      'tether_crosswind_pitch_rom': np.deg2rad([-25.0, 25.0]).tolist(),
      'tether_crosswind_roll_rom': np.deg2rad([-50.0, 50.0]).tolist(),
      'tether_pitch_tension_threshold': 30e3,

      # Aero angle limits [rad]
      # - The positive "good" limit is driven by a reduction in control
      #   authority and stability.
      # - The positive "bad" limit is driven by an abrupt stall expected in CFD.
      #   A one degree margin is included.
      # - The negative "good" and "bad" limits respectively mark the onset
      #   of non-linearity and stall, as expected in CFD for the range of
      #   flap deflections of interest.
      # TODO: Update documentation/references.
      'trans_in_pitched_forward_alpha': np.deg2rad(3.0),
      'crosswind_alpha_limits': np.deg2rad([-15.0, -10.0, 3.0, 5.0]).tolist(),
      'trans_in_beta_limits': np.deg2rad([-10.0, -7.0, 7.0, 10.0]).tolist(),
      'crosswind_beta_limits': np.deg2rad([-10.0, -7.0, 7.0, 10.0]).tolist(),
      'prep_trans_out_beta_limits': np.deg2rad([-10.0, -7.0, 7.0, 10.0]
                                              ).tolist(),

      # Scoring functions starts 10 meters lower.
      'min_altitude': 70.0,
  }

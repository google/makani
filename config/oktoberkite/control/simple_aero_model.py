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

"""Aerodynamic properties of the system used in the controller."""

from makani.analysis.control import simple_aero
from makani.config import mconfig


@mconfig.Config
def MakeParams():
  """Returns a dict of simplified aero coefficients based on aero databases."""

  # See CalcSimpleAeroModel in analysis/control/simple_aero.py.
  # See the CL offset in config/oktoberkite/sim/aero_sim.py.
  CL_offset = 0.0  # pylint: disable=invalid-name
  simple_aero_model = {
      'CD_0': 0.12279,
      'CL_0': 1.8107,
      'CY_0': -0.07759,
      'base_flaps': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      'dCD_dalpha': 0.592732761136644,
      'dCL_dalpha': 6.323755013548282,
      'dCL_dflap': [0.1461042377583599,
                    0.2555391766283499,
                    0.4686794764170221,
                    0.5219645513641735,
                    0.3477853816444165,
                    0.2274642446669306,
                    0.7843792215340928,
                    0.0022918311805231184],
      'dCY_dbeta': -1.2019222147356845
  }

  # The 1.0E-3 is a required argument and is the relative tolerance allowed for
  # coefficient comparison.
  simple_aero.CheckSimpleAeroModel(
      'oktoberkite/big_m600_r05_aswing_baseline.json', simple_aero_model, 1e-3,
      crosswind_trimmed=True, CL_0_offset=CL_offset)

  return simple_aero_model

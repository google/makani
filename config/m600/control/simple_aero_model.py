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
  # See CalcSimpleAeroModel in analysis/control/simple_aero.py.
  # Reduce lift coefficient from the low incidence model based on information
  # from CFD and flight testing. See the CL offset in
  # config/m600/sim/aero_sim.py.
  CL_offset = -0.125
  simple_aero_model = {
      'dCL_dalpha': 7.39413425,
      'CL_0': 2.02175,
      'base_flaps': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      'dCL_dflap': [0.25382030, 0.31971044, 0.35924453, 0.38101693,
                    0.42112398, 0.37414144, 0.55175835, 0.00343774],
      'dCY_dbeta': -1.4205533,
      'CY_0': -0.08687,
      'dCD_dalpha': 0.79990,
      'CD_0': 0.10567,
  }

  # The 1.0E-3 is a required argument and is the relative tolerance allowed for
  # coefficient comparison.
  simple_aero.CheckSimpleAeroModel('m600/m600_aswing_baseline.json',
                                   simple_aero_model, 1e-3,
                                   crosswind_trimmed=True,
                                   CL_0_offset=CL_offset)

  return simple_aero_model

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

"""ODE solver options."""

from makani.config import mconfig
from makani.sim import sim_types as m


@mconfig.Config
def MakeParams():
  return {
      # ODE solver choice (see SimODESolverType in sim/sim_types.h).
      'type': m.kSimOdeSolverGslRkf45,
      # Initial time step [s].
      'initial_time_step': 1e-4,
      # Absolute tolerance [#] for time-step adaptation.
      'abs_tolerance': 1e-2,
      # Relative tolerance [#] for time-step adaptation.
      'rel_tolerance': 1e-2
  }

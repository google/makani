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

"""Ground station model."""

from makani.config import mconfig
from makani.control import system_types


@mconfig.Config(deps={'flight_plan': 'common.flight_plan'})
def MakeParams(params):
  """Make ground station model parameters."""
  # Choose the ground station model based on flight plan.
  if params['flight_plan'] in [system_types.kFlightPlanManual,
                               system_types.kFlightPlanStartDownwind]:
    # These flight plans are not yet supported with GS02.
    gs_model = system_types.kGroundStationModelTopHat
  else:
    gs_model = system_types.kGroundStationModelGSv2

  return gs_model

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

"""Flight plan."""

from makani.config import mconfig
from makani.control import system_types as m


@mconfig.Config
def MakeParams():
  # Uncomment one of the following flight plans:
  # return m.kFlightPlanDisengageEngage
  # return m.kFlightPlanHighHover
  # return m.kFlightPlanHoverInPlace
  # return m.kFlightPlanLaunchPerch
  # return m.kFlightPlanManual
  # return m.kFlightPlanStartDownwind
  return m.kFlightPlanTurnKey

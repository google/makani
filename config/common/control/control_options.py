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

"""Controller options."""

from makani.config import mconfig
from makani.control import control_types as m


@mconfig.Config(deps={
    'system': mconfig.WING_MODEL + '.system_params'
})
def MakeParams(params):
  # No control options are enabled by default.
  control_opt = 0

  # Set flight plan options.

  # If kControlOptHardCodeWind is enabled, be sure that the
  # hard_coded_wind_g in estimator.py is set appropriately.

  # control_opt |= m.kControlOptHardCodeWind

  # If kControlOptHardCodeInitialPayout is enabled, be sure that
  # hard_coded_initial_payout in estimator.py is set appropriately.

  # control_opt |= m.kControlOptHardCodeInitialPayout

  # If kControlOptHoverThrottleEStop is enabled then bringing the
  # joystick throttle below e_stop_throttle will stop the motors. This
  # should only be enabled in constrained flight.
  if params['system']['flight_plan'] in [m.kFlightPlanHoverInPlace,
                                         m.kFlightPlanLaunchPerch,
                                         m.kFlightPlanDisengageEngage]:
    control_opt |= m.kControlOptHoverThrottleEStop

  # Check that no control options are enabled during crosswind flight
  # modes.
  if params['system']['flight_plan'] in [m.kFlightPlanStartDownwind,
                                         m.kFlightPlanTurnKey]:
    assert control_opt == 0

  return control_opt

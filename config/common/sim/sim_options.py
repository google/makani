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

"""Simulator options."""

from makani.config import mconfig
from makani.sim import sim_types as m


@mconfig.Config(deps={
    'flight_plan': 'common.flight_plan',
    'test_site': 'common.test_site'})
def MakeParams(params):
  # The possible simulator flags are:
  #
  #   m.kSimOptConstraintSystem
  #   m.kSimOptFaults
  #   m.kSimOptGroundContact
  #   m.kSimOptImperfectSensors
  #   m.kSimOptPerch
  #   m.kSimOptPerchContact
  #   m.kSimOptStackedPowerSystem
  #   m.kSimOptTiedDown
  #   m.kSimOptExitOnCrash

  sim_opt = (m.kSimOptFaults
             | m.kSimOptGroundContact
             | m.kSimOptImperfectSensors
             | m.kSimOptPerch
             | m.kSimOptPerchContact
             | m.kSimOptStackedPowerSystem)

  if params['flight_plan'] == m.kFlightPlanHoverInPlace:
    sim_opt ^= m.kSimOptPerchContact

  return sim_opt

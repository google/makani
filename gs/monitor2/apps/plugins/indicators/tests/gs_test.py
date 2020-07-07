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

"""Test for ground station indicators."""

import unittest

from makani.avionics.common import actuator_types
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins.indicators import ground_station
from makani.gs.monitor2.apps.receiver import test_util


class TestIndicators(unittest.TestCase):

  def _SynthesizePlcTelemetry(self, sequence=0):
    messages = test_util.SynthesizeMessages(
        ['GroundStationPlcStatus'], sequence)
    plc = messages.Subtree('GroundStationPlcStatus.PlcTophat').Data()
    return plc, messages

  def testDetwistArmedIndicator(self):
    plc, messages = self._SynthesizePlcTelemetry()
    indicator = ground_station.DetwistArmedIndicator()

    text, stoplight = indicator.Filter(messages)
    self.assertEqual(text, 'Init')
    self.assertEqual(stoplight, stoplights.STOPLIGHT_ERROR)

    plc.detwist_state = actuator_types.kActuatorStateArmed
    text, stoplight = indicator.Filter(messages)
    self.assertEqual(text, 'Armed')
    self.assertEqual(stoplight, stoplights.STOPLIGHT_NORMAL)

    plc.detwist_state = actuator_types.kActuatorStateRunning
    text, stoplight = indicator.Filter(messages)
    self.assertEqual(text, 'Running')
    self.assertEqual(stoplight, stoplights.STOPLIGHT_NORMAL)

    plc.detwist_state = actuator_types.kActuatorStateError
    text, stoplight = indicator.Filter(messages)
    self.assertEqual(text, 'Error')
    self.assertEqual(stoplight, stoplights.STOPLIGHT_ERROR)

if __name__ == '__main__':
  unittest.main()
